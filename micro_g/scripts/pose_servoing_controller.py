# Copyright 2023, Micro-G Dev Team
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
import time

import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped
from interbotix_common_modules.angle_manipulation import angle_manipulation as ang
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from tf2_geometry_msgs import do_transform_pose


class XSArmPoseServoingController(InterbotixManipulatorXS):
    """
    Tracks a published end effector pose.
    """

    control_update_rate: float = 50

    def __init__(
        self,
        robot_model: str,
        robot_name: str,
        control_update_rate: float,
        xs_args=None,
    ):
        """
        Create the controller

        Args:
            robot_model: the robot model
            robot_name: the robot name
            control_update_rate: the rate at which the controller is updated (Hz).
            xs_args: Extra arguments to be passed to InterbotixManipulatorXS. Defaults
                to None.
        """
        # Initialize the interface to the arm
        InterbotixManipulatorXS.__init__(
            self,
            robot_model=robot_model,
            robot_name=robot_name,
            moving_time=0.2,
            accel_time=0.1,
            start_on_init=True,
            args=xs_args,
        )

        # Set the controller update rate
        self.rate = self.core.create_rate(control_update_rate)

        # Define the transforms we'll use. Notation is T_{to}{from}.
        # Frames are:
        #   - s = space frame, fixed to the robot's base link
        #   - y = yaw frame, same as s but rotated about z by the current waist angle
        #   - b = body frame, fixed to the end effector
        #   - d = desired frame for the end effector
        self.T_sy = np.identity(4)
        self.update_frames()

        # Initialize the tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Set the target frame for transformation
        self.target_frame = f"{robot_name}/base_link"

        # Create a subscription to the pose that we will track
        self.core.create_subscription(
            PoseStamped, "robot_name/desired_eef_pose", self.desired_pose_callback, 10
        )
        time.sleep(0.5)
        self.core.get_logger().info("Ready to receive servoing commands.")

    def update_frames(self):
        """
        Update the coordinate frames used by this controller.

        We need to do this manually because the y frame (base frame rotated to match
        arm yaw) is not actually published as a TF frame.
        """
        T_sb = self.arm.get_ee_pose_command()  # Current EEF pose in space frame
        rpy = ang.rotation_matrix_to_euler_angles(T_sb[:3, :3])
        self.T_sy[:2, :2] = ang.yaw_to_rotation_matrix(rpy[2])

    def desired_pose_callback(self, msg: PoseStamped):
        """
        Process a desired pose

        Args:
            msg: A PoseStamped message
        """
        # Transform the pose into the base link frame
        try:
            # Lookup the transform from the source frame to target frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, msg.header.stamp
            )

            # Transform the pose using the received transform
            transformed_pose = do_transform_pose(msg, transform)
            position = transformed_pose.pose.position
            orientation = transformed_pose.pose.orientation

            # Save the position and orientation into the homogeneous transform
            rpy = np.array([orientation.x, orientation.y, orientation.z])
            np.array([position.x, position.y, position.z])
            self.T_sd[:3, :3] = ang.euler_angles_to_rotation_matrix(rpy)
            self.T_sd[:3, 3] = position
            self.core.get_logger().info(f"T_sd:\n{self.T_sd}")

        except Exception as e:
            self.core.get_logger().error(
                f"Error transforming pose to {self.target_frame}: {e}"
            )

    def controller(self):
        """Run the controller and send commands to the robot."""
        # Convert the desired pose into the arm-aligned base frame
        T_ys = np.linalg.inv(self.T_sy)
        T_yd = np.dot(T_ys, self.T_sd)

        # Project the desired pose into the plane of the arm by zeroing the y coordinate
        T_yd[1, 3] = 0.0

        # Convert back to the fixed base frame and send to the arm
        T_sd = np.dot(self.T_sy, T_yd)
        _, success = self.arm.set_ee_pose_matrix(
            T_sd=T_sd,
            custom_guess=self.arm.get_joint_commands(),
            execute=True,
            moving_time=0.2,
            accel_time=0.1,
            blocking=False,
        )

        if not success:
            self.core.get_logger().warn(f"Desired pose not reachable!\n{T_sd}")