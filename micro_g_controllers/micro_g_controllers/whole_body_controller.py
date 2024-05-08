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
from collections import namedtuple

import modern_robotics as mr
import numpy as np
import tf2_ros
import transforms3d
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from interbotix_common_modules.angle_manipulation import angle_manipulation as ang
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_system_default
from rclpy.utilities import remove_ros_args
from tf2_geometry_msgs import do_transform_pose
from ticlib import TicUSB

from micro_g_controllers.whole_body_controller_parameters import (
    wbc,
)
from micro_g_controllers.linear_axis_controller import (
    meters_per_second_to_microsteps_per_10k_seconds,
)

# Make a container that holds some of the command information that the controller uses
LinearStageCommand = namedtuple("Command", ["command", "stamp"])


class WholeBodyController(InterbotixManipulatorXS):
    """Tracks a published end effector pose."""

    def __init__(
        self,
        robot_model: str,
        robot_name: str,
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
            node_name="wbc",
        )

        # Setup the node parameters
        self.param_listener = wbc.ParamListener(self.core)
        self.params = self.param_listener.get_params()
        self.core.create_timer(
            1, self.update_parameters_callback, MutuallyExclusiveCallbackGroup()
        )

        self.home = [0.0, -1.494097352027893, 0.9418642520904541, 0.5338253378868103]
        self.last_target_time = time.time()

        # Initialize the gripper state
        self.gripper_open = True
        self.gripper.release(delay=0.0)

        # Update the moving time (and set acceleration time to half of moving time)
        self.arm.set_trajectory_time(
            self.params.moving_time, self.params.moving_time / 2.0
        )

        # Define the transforms we'll use. Notation is T_{to}{from}.
        # Frames are:
        #   - s = space frame, fixed to the robot's base link
        #   - y = yaw frame, same as s but rotated about z by the current waist angle
        #   - b = end effector body frame (tool frame)
        #   - d = desired frame for the end effector
        self.T_sy = np.eye(4)
        self.T_sd = None
        self.update_frames()

        # Initialize the tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.core)
        # Set the target frame for transformation
        self.target_frame = f"{robot_name}/base_link"

        # Create the interface to the motor and configure it
        self.tic = self.configure_tic()
        self.tic.set_target_velocity(0)
        self.linear_stage_command = LinearStageCommand(0, time.time())

        # Create a subscription to the pose that we will track
        self.core.create_subscription(
            PoseStamped,
            "desired_eef_pose",
            self.desired_pose_callback,
            qos_profile_system_default,
        )
        self.core.create_subscription(
            Bool,
            "grasp",
            self.grasp_object_callback,
            qos_profile_system_default,
        )
        self.core.get_logger().info("Ready to receive servoing commands.")

        # Set the controller update rate
        self.core.create_timer(1.0 / self.params.control_update_rate, self.update)

    def configure_tic(self) -> TicUSB:
        """Initialize and configure the Tic motor driver."""
        tic = TicUSB()

        tic.energize()
        tic.exit_safe_start()
        tic.set_max_speed(
            meters_per_second_to_microsteps_per_10k_seconds(
                self.params.linear_stage_limits.speed
            )
        )
        tic.halt_and_set_position(-100)

        tic.set_max_acceleration(self.params.linear_stage_limits.acceleration.max)
        tic.set_max_deceleration(self.params.linear_stage_limits.acceleration.min)

        return tic

    def update_frames(self):
        """
        Update the coordinate frames used by this controller.

        We need to do this manually because the y frame (base frame rotated to match
        arm yaw) is not actually published as a TF frame.
        """
        T_sb = self.arm.get_ee_pose_command()  # Current EEF pose in space frame
        rpy = ang.rotation_matrix_to_euler_angles(T_sb[:3, :3])
        self.T_sy[:2, :2] = ang.yaw_to_rotation_matrix(rpy[2])

    def update_parameters_callback(self):
        """Update the ROS parameters."""
        if self.param_listener.is_old(self.params):
            self.param_listener.refresh_dynamic_parameters()
            self.params = self.param_listener.get_params()

            self.arm.set_trajectory_time(
                self.params.moving_time, self.params.moving_time / 2.0
            )

    def grasp_object_callback(self, msg: Bool):
        """Callback for the grasp subscriber."""
        if msg.data:
            self.gripper.grasp(delay=0.3)
            self.gripper_open = False
        else:
            self.gripper.release(delay=0.3)
            self.gripper_open = True

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
                self.target_frame, msg.header.frame_id, rclpy.time.Time()
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
            self.T_sd = np.eye(4)
            self.T_sd[:3, :3] = orientation
            self.T_sd[:3, 3] = position
            self.core.get_logger().debug(
                f"Received new desired pose T_sd:\n{self.T_sd}"
            )
            self.last_target_time = time.time()

            self.core.get_logger().debug(f"""-----
Received pose summary

target frame: {self.target_frame}
source frame: {msg.header.frame_id}
received_pose: {msg}
transformed: {transformed_pose}
T_sd:
{self.T_sd}
""")

        except Exception as e:
            self.core.get_logger().error(
                f"Error transforming pose to {self.target_frame}: {e}"
            )
            return

    def update(self):
        """Run the controller and send commands to the robot."""
        # If we haven't received a target in a while, move home
        if (
            time.time() - self.last_target_time > self.params.timeout
            or self.T_sd is None
        ):
            self.core.get_logger().debug(
                f"No target received in {time.time() - self.last_target_time} s; moving home."
            )

            if self.gripper_open:
                self.gripper.grasp(delay=0.3)
                self.gripper_open = False

            self.arm.set_joint_positions(
                self.home,
                blocking=False,
                moving_time=0.5,
                accel_time=0.25,
            )

            self.tic.set_target_velocity(0)

            return
        else:
            if not self.gripper_open:
                self.gripper.release(delay=0.3)
                self.gripper_open = True

        # Get the current end effector pose, then get the linear and angular error
        T_sb = self.arm.get_ee_pose()
        euler_current = np.array(ang.rotation_matrix_to_euler_angles(T_sb[:3, :3]))
        position_current = T_sb[:3, 3]

        euler_desired = np.array(ang.rotation_matrix_to_euler_angles(self.T_sd[:3, :3]))
        position_desired = self.T_sd[:3, 3]

        # Box the desired position and rotation into the workspace
        position_desired[0] = np.clip(
            position_desired[0],
            self.params.workspace_limits.x.min,
            self.params.workspace_limits.x.max,
        )
        position_desired[2] = np.clip(
            position_desired[2],
            self.params.workspace_limits.z.min,
            self.params.workspace_limits.z.max,
        )
        euler_desired[-1] = np.clip(
            euler_desired[-1],
            self.params.workspace_limits.yaw.min,
            self.params.workspace_limits.yaw.max,
        )

        # Compute linear and rotational errors
        position_error = position_desired - position_current
        euler_error = euler_desired - euler_current

        # Linear stage update (track y coordinate of error)
        self.linear_stage_controller(position_error[1])

        # Arm control (don't track y error)
        position_error[1] = 0
        self.core.get_logger().debug(f"""
---------------------------------------
Diff IK summary

Current position: {position_current}
Desired position: {position_desired}
Position error: {position_error}

Current orientation: {euler_current}
Desired orientation: {euler_desired}
Orientation error: {euler_error}
""")
        self.arm_controller(position_error, euler_error)

    def arm_controller(self, position_error, euler_error):
        """Diff IK control law for arm."""
        # Compute the twist in space frame
        twist = np.zeros(6)
        twist[3:] = position_error
        twist[:3] = euler_error
        twist *= self.params.arm_kp

        # Compute the Jacobian
        # self.arm.capture_joint_positions()
        joint_angles = np.array(self.arm.get_joint_commands())
        J = mr.JacobianSpace(Slist=self.arm.robot_des.Slist, thetalist=joint_angles)

        # Get the joint velocities
        joint_velocities = np.dot(np.linalg.pinv(J), twist)

        self.core.get_logger().debug(f"""
Jacobian:
{J}

Jinv:
{np.linalg.pinv(J).round(2)}

Twist: {twist}
joint_velocities: {joint_velocities}
""")

        # Update the joint positions
        desired_joint_positions = (
            joint_angles + joint_velocities / self.params.control_update_rate
        )

        self.arm.set_joint_positions(
            desired_joint_positions,
            blocking=False,
            moving_time=0.5,
            accel_time=0.25,
        )

    def linear_stage_controller(self, linear_error):
        """Control law for linear stage to track the desired position."""
        # First, make sure the linear stage is within the boundaries
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_tag",
                "world",
                rclpy.time.Time(),
            )
        except Exception as _:
            self.core.get_logger().info("Could not get gantry transform")
            return None

        current_base_position = transform.transform.translation
        current_base_x = current_base_position.x

        if current_base_x < self.params.linear_stage_limits.x.min:
            linear_error = self.params.linear_stage_limits.x.min - current_base_x
            self.core.get_logger().warning(
                f"Current position {current_base_x} is below min position"
                f" {self.params.linear_stage_limits.x.min}"
            )
        elif current_base_x > self.params.linear_stage_limits.x.max:
            linear_error = self.params.linear_stage_limits.x.max - current_base_x
            self.core.get_logger().warning(
                f"Current position {current_base_x} is above max position"
                f" {self.params.linear_stage_limits.x.max}"
            )

        # Compute the velocity command based on the error
        speed_command = -self.params.linear_stage_kp * linear_error
        speed_command = max(
            min(speed_command, self.params.linear_stage_limits.speed),
            -self.params.linear_stage_limits.speed,
        )

        # Send the command to the motor
        speed_command_usteps_per_second = (
            meters_per_second_to_microsteps_per_10k_seconds(speed_command)
        )
        self.core.get_logger().debug(
            f"Speed command: {speed_command} ({speed_command_usteps_per_second}"
            " usteps/10ks)"
        )
        self.tic.set_target_velocity(speed_command_usteps_per_second)

    def shutdown(self):
        """De-energize the stepper motor and enter safe start"""
        self.core.get_logger().info("Shutting down linear stage.")
        self.tic.deenergize()
        self.tic.enter_safe_start()
        super().shutdown()

    def start_robot(self):
        """Run the controller until ROS triggers a shutdown."""
        try:
            self.run()
            # executor = MultiThreadedExecutor()
            # rclpy.spin(self.core, executor)
        except KeyboardInterrupt:
            self.shutdown()
        finally:
            self.shutdown()


def main():
    # Get command line arguments
    p = argparse.ArgumentParser()
    p.add_argument("--robot_model", required=True)
    p.add_argument("--robot_name", required=True)
    p.add_argument("args", nargs=argparse.REMAINDER)

    command_line_args = remove_ros_args(args=sys.argv)[1:]
    ros_args = p.parse_args(command_line_args)

    bot = WholeBodyController(ros_args.robot_model, ros_args.robot_name)
    bot.start_robot()

    bot.shutdown()


if __name__ == "__main__":
    main()
