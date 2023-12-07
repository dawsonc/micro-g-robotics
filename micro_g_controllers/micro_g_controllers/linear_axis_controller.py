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
"""Use the servo-driven 5th axis to track the object position"""

from collections import namedtuple
from time import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from ticlib import TicUSB

from micro_g_controllers.linear_axis_controller_parameters import linear_axis_controller

# Make a container that holds some of the command information that the controller uses
Command = namedtuple("Command", ["command", "stamp"])


def meters_per_second_to_microsteps_per_10k_seconds(speed: float) -> int:
    """
    Convert reasonable speed units into dumb speed units :(.

    20 teeth per rotation, 2 mm per tooth
    200 steps per rotation, 0.2 mm per step
    range 0-500,000,000
    speed is microsteps per 10,000s

    Example: speed of 200,000 corresponds to 20 microsteps per second.

    Args:
        speed: Speed in meters per second

    Returns:
        Speed in microsteps per 10,000s
    """
    meters_per_step = 0.2e-3
    steps_per_second = speed / meters_per_step
    steps_per_10k_seconds = steps_per_second * 1e4
    microsteps_per_10k_seconds = int(steps_per_10k_seconds * 2)
    return microsteps_per_10k_seconds


def meters_to_microsteps(position: float) -> int:
    """
    Convert reasonable distance units into dumb distance units :(.

    20 teeth per rotation, 2 mm per tooth
    200 steps per rotation, 0.2 mm per step
    distance is microsteps

    2 mm/tooth * 20 teeth/rotation * 1/200 rotations/step = 0.2 mm/step

    microsteps aren't 10^-6 steps, they're 2 per steps (whatever the substep setting is)

    Args:
        position: Position in meters

    Returns:
        Position in microsteps
    """
    meters_per_step = 0.2e-3
    steps = position / meters_per_step
    microsteps = int(steps * 2)
    return microsteps


class LinearAxisController(Node):
    """A controller for the linear 5th axis"""

    def __init__(self):
        super().__init__("linear_axis_node")

        # Setup the node parameters
        self.param_listener = linear_axis_controller.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.create_timer(
            1, self.update_parameters_callback, MutuallyExclusiveCallbackGroup()
        )

        # Declare a place to save the target object y coordinate
        self.target_pose_y = 0.0

        # Save the control command
        self.control_cmd = Command(0, time())

        # Declare subscriptions
        self.get_logger().debug(f"Subscribing to {self.params.input_topic}")
        self.create_subscription(
            PoseStamped,
            self.params.input_topic,
            self.target_pose_callback,
            qos_profile_system_default,
        )

        # Create the interface to the motor and configure it
        self.tic = self.configure_tic()

    def configure_tic(self) -> TicUSB:
        """Initialize and configure the Tic motor driver."""
        tic = TicUSB()

        tic.energize()
        tic.exit_safe_start()
        tic.set_max_speed(
            meters_per_second_to_microsteps_per_10k_seconds(self.params.max_speed)
        )
        tic.halt_and_set_position(-100)

        return tic

    def update_parameters_callback(self):
        """Update the ROS parameters."""
        if self.param_listener.is_old(self.params):
            self.param_listener.refresh_dynamic_parameters()
            self.params = self.param_listener.get_params()

    def target_pose_callback(self, msg: PoseStamped):
        """Callback for the target pose subscriber"""
        self.get_logger().debug(f"Received pose {msg}")
        self.control_cmd = Command(msg.pose.position.y, time())

    def update(self):
        """Update the controller"""
        time_since_last_command = time() - self.control_cmd.stamp
        if time_since_last_command > self.params.command_timeout:
            self.get_logger().warning(
                "Latest control command is stale. Stopping linear axis motor."
            )
            self.get_logger().debug(
                f"Time since last command: {time_since_last_command}"
            )
            #
            self.tic.set_target_velocity(0)
            return

        # We want to track the object so that its y coordinate in the robot frame
        # is zero. Implement P control over velocity to do this, but be sure to respect
        # the joint limits.
        current_position = self.tic.get_current_position()

        if current_position < self.params.position_limits.min:
            self.get_logger().warning(
                f"Current position {current_position} is below min position"
                f" {self.params.position_limits.min}"
            )
            deviation = (current_position - self.params.position_limits.min) / (
                self.params.position_limits.max - self.params.position_limits.min
            )
        elif current_position > self.params.position_limits.max:
            self.get_logger().warning(
                f"Current position {current_position} is above max position"
                f" {self.params.position_limits.max}"
            )
            deviation = (current_position - self.params.position_limits.max) / (
                self.params.position_limits.max - self.params.position_limits.min
            )
        else:
            deviation = self.control_cmd.command

        # TODO(evan): fix the controller
        speed_command = -self.params.kp * deviation
        speed_command = max(
            min(speed_command, self.params.max_speed), -self.params.max_speed
        )
        speed_command_usteps_per_second = (
            meters_per_second_to_microsteps_per_10k_seconds(speed_command)
        )
        self.get_logger().info(
            f"Speed command: {speed_command} ({speed_command_usteps_per_second}"
            " usteps/10ks)"
        )
        self.tic.set_target_velocity(speed_command_usteps_per_second)

    def shutdown(self):
        """De-energize the stepper motor and enter safe start"""
        self.tic.deenergize()
        self.tic.enter_safe_start()

    def start(self):
        try:
            self.create_timer(1.0 / self.params.control_frequency, self.update)
            executor = MultiThreadedExecutor()
            rclpy.spin(self, executor)
        except KeyboardInterrupt:
            self.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LinearAxisController()
    node.start()

    node.destroy_node()
    rclpy.shutdown()
