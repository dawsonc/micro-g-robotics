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
import argparse
import sys
import time

import numpy as np
import rclpy
import tf2_ros
import transforms3d
from geometry_msgs.msg import PoseStamped
from interbotix_common_modules.angle_manipulation import angle_manipulation as ang
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import qos_profile_system_default
from rclpy.utilities import remove_ros_args
from tf2_geometry_msgs import do_transform_pose


class XSArmPoseServoingController(InterbotixManipulatorXS):
    """Tracks a published end effector pose."""

    def __init__(
        self,
        robot_model: str,
        robot_name: str,
        control_update_rate: float = 20.0,
        moving_time: float = 0.5,
        xs_args=None,
    ):
        """
        Create the controller.

        Args:
        ----
            robot_model: the robot model
            robot_name: the robot name
            control_update_rate: the rate at which the controller is updated (Hz).
                This can also be set via a ROS parameter `control_update_rate`.
            moving_time: the duration in which all movements should take place (s).
                This can also be set via a ROS parameter `control_update_rate`.
            xs_args: Extra arguments to be passed to InterbotixManipulatorXS. Defaults
                to None.

        """
        # Initialize the interface to the arm
        # We need robot_model and robot_name to initialize the ROS node and namespace,
        # so these cannot be ROS parameters.
        super().__init__(
            robot_model=robot_model,
            robot_name=robot_name,
            start_on_init=True,
            moving_time=moving_time,
            accel_time=moving_time / 2.0,
            args=xs_args,
            node_name="pose_servoing",
        )

        # Declare ROS parameters
        control_rate_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description="Control update rate in Hz"
        )
        moving_time_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description="Moving time in seconds"
        )
        self.core.declare_parameter(
            "control_update_rate", 20.0, descriptor=control_rate_desc
        )
        self.core.declare_parameter("moving_time", 0.2, descriptor=moving_time_desc)

        # Get ROS parameters
        control_update_rate = self.core.get_parameter("control_update_rate").value
        moving_time = self.core.get_parameter("moving_time").value

        # Update the moving time (and set acceleration time to half of moving time)
        self.arm.set_trajectory_time(moving_time, moving_time / 2.0)

        # Set the controller update rate
        self.rate = self.core.create_rate(control_update_rate)

        # Define the transforms we'll use. Notation is T_{to}{from}.
        # Frames are:
        #   - s = space frame, fixed to the robot's base link
        #   - y = yaw frame, same as s but rotated about z by the current waist angle
        #   - b = end effector body frame (tool frame)
        #   - d = desired frame for the end effector
        self.T_sy = np.identity(4)
        self.T_sd = self.arm.get_ee_pose_command()
        self.update_frames()

        # Initialize the tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.core)
        # Set the target frame for transformation
        self.target_frame = "base_link"

        # Create a subscription to the pose that we will track
        self.core.create_subscription(
            PoseStamped,
            "desired_eef_pose",
            self.desired_pose_callback,
            qos_profile_system_default,
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
        Process a desired pose.

        Args:
        ----
            msg: A PoseStamped message

        """
        # Transform the pose into the base link frame
        try:
            # Lookup the transform from the source frame to target frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, msg.header.stamp
            )

            # Transform the pose using the received transform
            transformed_pose = do_transform_pose(msg.pose, transform)
            position = transformed_pose.position
            position = np.array([position.x, position.y, position.z])
            orientation = transformed_pose.orientation
            orientation = transforms3d.quaternions.quat2mat(
                [orientation.w, orientation.x, orientation.y, orientation.z]
            )

            # Save the position and orientation into the homogeneous transform
            self.T_sd[:3, :3] = orientation
            self.T_sd[:3, 3] = position
            self.core.get_logger().info(f"Received new desired pose T_sd:\n{self.T_sd}")

        except Exception as e:
            self.core.get_logger().error(
                f"Error transforming pose to {self.target_frame}: {e}"
            )
            raise (e)

    def update(self):
        """Run the controller and send commands to the robot."""
        # Convert the desired pose into the arm-aligned base frame
        T_ys = np.linalg.inv(self.T_sy)
        T_yd = np.dot(T_ys, self.T_sd)

        # Project the desired pose into the plane of the arm by zeroing the y coordinate
        T_yd[1, 3] = 0.0

        # Get the relative x, y, z, roll, pitch, yaw between the current pose and T_sd
        T_sb = self.arm.get_ee_pose_command()
        T_yb = np.dot(T_ys, T_sb)
        position_offset = T_yd[:3, 3] - T_yb[:3, 3]
        rpy_yb = np.array(ang.rotation_matrix_to_euler_angles(T_yb[:3, :3]))
        rpy_yd = np.array(ang.rotation_matrix_to_euler_angles(T_yd[:3, :3]))
        rpy_offset = rpy_yd - rpy_yb

        success = self.arm.set_ee_cartesian_trajectory(
            x=position_offset[0],
            y=0.0,
            z=position_offset[2],
            roll=rpy_offset[0],
            pitch=rpy_offset[1],
            yaw=0.0,
        )

        if not success:
            self.core.get_logger().warn(f"Desired pose not reachable!\n{self.T_sd}")

    def start_robot(self):
        """Run the controller until ROS triggers a shutdown."""
        try:
            self.start()
            while rclpy.ok():
                self.update()
                self.rate.sleep()
        except KeyboardInterrupt:
            self.shutdown()


def main():
    # Get command line arguments
    p = argparse.ArgumentParser()
    p.add_argument("--robot_model", required=True)
    p.add_argument("--robot_name", required=True)
    p.add_argument("args", nargs=argparse.REMAINDER)

    command_line_args = remove_ros_args(args=sys.argv)[1:]
    ros_args = p.parse_args(command_line_args)

    bot = XSArmPoseServoingController(ros_args.robot_model, ros_args.robot_name)
    bot.start_robot()


if __name__ == "__main__":
    main()
