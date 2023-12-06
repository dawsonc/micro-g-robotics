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

import modern_robotics as mr
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
            start_on_init=False,
            moving_time=moving_time,
            accel_time=moving_time / 2.0,
            args=xs_args,
            node_name="pose_servoing",
        )

        # Declare ROS parameters
        self.core.declare_parameters(
            namespace="",
            parameters=[
                (
                    "control_update_rate",
                    control_update_rate,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="Control update rate in Hz",
                    ),
                ),
                (
                    "moving_time",
                    moving_time,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="Moving time in seconds",
                    ),
                ),
                (
                    "kp",
                    1.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="Proportional gain for joint tracking controller",
                    ),
                ),
                (
                    "replanning_attempts",
                    5,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_INTEGER,
                        description="Number of times to attempt re-planning",
                    ),
                ),
                (
                    "timeout",
                    1.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="Wait this long after getting a target before moving home (s)",
                    ),
                ),
            ],
        )

        # Get ROS parameters
        control_update_rate = self.core.get_parameter("control_update_rate").value
        moving_time = self.core.get_parameter("moving_time").value
        self.kp = self.core.get_parameter("kp").value
        self.replanning_attempts = self.core.get_parameter("replanning_attempts").value
        self.home = [0.0, -1.494097352027893, 0.9418642520904541, 0.5338253378868103]
        self.timeout = self.core.get_parameter("timeout").value
        self.last_target_time = time.time()

        # Initialize the gripper state
        self.gripper.release(delay=0.0)

        # Update the moving time (and set acceleration time to half of moving time)
        self.arm.set_trajectory_time(moving_time, moving_time / 2.0)

        # Define the transforms we'll use. Notation is T_{to}{from}.
        # Frames are:
        #   - s = space frame, fixed to the robot's base link
        #   - y = yaw frame, same as s but rotated about z by the current waist angle
        #   - b = end effector body frame (tool frame)
        #   - d = desired frame for the end effector
        self.T_sy = np.eye(4)
        self.update_frames()

        # Define the joints to track
        self.desired_joint_positions = self.home

        # Initialize the tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.core)
        # Set the target frame for transformation
        self.target_frame = f"{robot_name}/base_link"

        # Create a subscription to the pose that we will track
        self.core.create_subscription(
            PoseStamped,
            "desired_eef_pose",
            self.desired_pose_callback,
            qos_profile_system_default,
        )
        self.core.get_logger().info("Ready to receive servoing commands.")

        # Set the controller update rate
        self.control_update_rate = control_update_rate
        self.core.create_timer(1.0 / self.control_update_rate, self.update)

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
            T_sd = np.eye(4)
            T_sd[:3, :3] = orientation
            T_sd[:3, 3] = position
            self.core.get_logger().debug(f"Received new desired pose T_sd:\n{T_sd}")

            # Project the pose into the plane of the arm by zeroing the y coordinate
            self.update_frames()
            T_ys = np.linalg.inv(self.T_sy)
            T_yd = np.dot(T_ys, T_sd)
            y_distance = T_yd[1, 3]
            T_yd[1, 3] = 0.0
            T_sd = np.dot(self.T_sy, T_yd)

            # If the target is too far away along the y axis, wait for the linear axis
            # controller to move the robot closer
            if abs(y_distance) > 0.01:
                self.core.get_logger().warn(
                    f"Target is too far away along y axis ({y_distance} m)."
                )
                self.desired_joint_positions = self.home
                return

            # Solve IK to get the joint angles that track this pose
            theta_list, success = self.get_joints_for_eef_pose(T_sd)
            if not success:
                self.core.get_logger().warn("Desired pose not reachable!")
            else:
                self.desired_joint_positions = theta_list
                self.last_target_time = time.time()

        except Exception as e:
            self.core.get_logger().error(
                f"Error transforming pose to {self.target_frame}: {e}"
            )
            raise (e)

    def get_joints_for_eef_pose(self, T_sd):
        """Use IK to get joint angles that achieve the desired end effector pose."""
        # Generate a list of initial guesses around the current joint angles
        self.arm.capture_joint_positions()
        joint_angles = np.array(self.arm.get_joint_commands())
        initial_guesses = np.random.normal(
            joint_angles,
            scale=np.pi / 3.0,
            size=(self.replanning_attempts, joint_angles.size),
        )
        # Start with the current joint angles as the first guess
        initial_guesses = np.vstack([joint_angles, initial_guesses])

        # Try each initial guess until we find one that works or run out of guesses
        for initial_guess in initial_guesses:
            # Manually do this rather than calling set_ee_pose_matrix so that we can
            # manually set the angular and linear error tolerances
            theta_list, success = mr.IKinSpace(
                Slist=self.arm.robot_des.Slist,
                M=self.arm.robot_des.M,
                T=T_sd,
                thetalist0=initial_guess,
                eomg=0.1,
                ev=0.001,
            )
            success = True

            # Check to make sure a solution was found and that no joint limits were violated
            if success:
                theta_list = self.arm._wrap_theta_list(theta_list)
                success = self.arm._check_joint_limits(theta_list)
            else:
                success = False

            # If we found a solution, return it; otherwise keep searching
            if success:
                return theta_list, success

        return None, False

    def update(self):
        """Run the controller and send commands to the robot."""
        # If we haven't received a target in a while, move home
        if time.time() - self.last_target_time > self.timeout:
            self.core.get_logger().info(
                f"No target received in {time.time() - self.last_target_time} s; moving home."
            )
            self.desired_joint_positions = self.home
            self.gripper.grasp(delay=0.0)
        else:
            self.gripper.release(delay=0.0)

        # Track the desired joint positions
        self.arm.capture_joint_positions()
        q_current = np.array(self.arm.get_joint_commands())
        q_desired = np.array(self.desired_joint_positions)
        q_setpoint = q_current + (q_desired - q_current) * self.kp
        self.arm.set_joint_positions(
            q_setpoint.tolist(),
            blocking=False,
        )

    def start_robot(self):
        """Run the controller until ROS triggers a shutdown."""
        try:
            self.start()
            rclpy.spin(self.core)
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
