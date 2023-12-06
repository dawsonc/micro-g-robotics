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
from rcl_interfaces.msg import (
    Parameter,
    ParameterDescriptor,
    ParameterType,
    SetParametersResult,
)
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from ticlib import TicUSB

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

        # Load configurable parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "input_topic",
                    "/px100/desired_eef_pose",
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_STRING,
                        description="Name of PoseStamped topic with the object pose.",
                    ),
                ),
                (
                    "control_frequency",
                    20.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="Control rate (Hz).",
                    ),
                ),
                (
                    "kp",
                    3.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="P control gain.",
                    ),
                ),
                (
                    "max_speed",
                    0.75,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="Max speed in meters/second.",
                    ),
                ),
                (
                    "min_position",
                    0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_INTEGER,
                        description="Min position in pulses.",
                    ),
                ),
                (
                    "max_position",
                    2500,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_INTEGER,
                        description="Max position in pulses from the home position.",
                    ),
                ),
                (
                    "command_timeout",
                    1.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description=(
                            "Maximum duration before a command is considered stale and"
                            " the controller stops."
                        ),
                    ),
                ),
            ],
        )

        self.input_topic = self.get_parameter("input_topic").value
        self.kp = self.get_parameter("kp").value
        self.max_speed = self.get_parameter("max_speed").value
        self.min_position = self.get_parameter("min_position").value
        self.max_position = self.get_parameter("max_position").value
        self.control_update_rate = self.get_parameter("control_frequency").value
        self.command_timeout = self.get_parameter("command_timeout").value

        # Declare a place to save the target object y coordinate
        self.target_pose_y = 0.0

        # Save the control command
        # We store the time at which the command was received so that we can check if it
        # is stale
        self.control_cmd = Command(0, time())

        # Declare subscriptions
        self.get_logger().debug(f"Subscribing to {self.input_topic}")
        self.create_subscription(
            PoseStamped,
            self.input_topic,
            self.target_pose_callback,
            qos_profile_system_default,
        )

        # Add a callback to handle parameter updates
        self.add_on_set_parameters_callback(self.param_change_callback)

        # Create the interface to the motor and configure it
        self.tic = self.configure_tic()

    def configure_tic(self) -> TicUSB:
        """Initialize and configure the Tic motor driver."""
        tic = TicUSB()

        tic.energize()
        tic.exit_safe_start()
        tic.set_max_speed(
            meters_per_second_to_microsteps_per_10k_seconds(self.max_speed)
        )
        tic.halt_and_set_position(-100)

        return tic

    def target_pose_callback(self, msg: PoseStamped):
        """Callback for the target pose subscriber"""
        self.get_logger().debug(f"Received pose {msg}")
        self.control_cmd = Command(msg.pose.position.y, time())

    def param_change_callback(self, params: list[Parameter]) -> SetParametersResult:
        """Update the current node parameters.

        This allows us to dynamically change controller gains and other parameters while
        the robot is running.

        Args:
            params: The list of parameters to update.

        Returns:
            Whether or not the provided parameters were successfully updated. This will
            only fail if the parameter doesn't exist or doesn't support dynamic updating.
        """
        update_successful = True

        for param in params:
            match param.name:
                case "kp":
                    self.kp = param.value
                case "max_speed":
                    self.max_speed = param.value
                case "min_position":
                    self.min_position = param.value
                case "max_position":
                    self.max_position = param.value
                case "command_timeout":
                    self.command_timeout = param.value
                case _:
                    self.get_logger().warning(
                        f"Could not update {param.name}, parameter does not exist or"
                        " does not support dynamic updating."
                    )
                    update_successful = False

        return SetParametersResult(successful=update_successful)

    def update(self):
        """Update the controller"""
        time_since_last_command = time() - self.control_cmd.stamp
        if time_since_last_command > self.command_timeout:
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

        if current_position < self.min_position:
            self.get_logger().warning(
                f"Current position {current_position} is below min position"
                f" {self.min_position}"
            )
            deviation = (current_position - self.min_position) / (
                self.max_position - self.min_position
            )
        elif current_position > self.max_position:
            self.get_logger().warning(
                f"Current position {current_position} is above max position"
                f" {self.max_position}"
            )
            deviation = (current_position - self.max_position) / (
                self.max_position - self.min_position
            )
        else:
            deviation = self.control_cmd.command

        # TODO(evan): fix the controller
        speed_command = -self.kp * deviation
        speed_command = max(min(speed_command, self.max_speed), -self.max_speed)
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
            self.create_timer(1.0 / self.control_update_rate, self.update)
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LinearAxisController()
    node.start()
    rclpy.shutdown()
