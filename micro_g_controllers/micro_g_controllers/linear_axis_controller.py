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
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from ticlib import TicUSB

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
                    0.5,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="P control gain.",
                    ),
                ),
                (
                    "max_speed",
                    0.5,
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
                    1500,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_INTEGER,
                        description="Max position in pulses from the home position.",
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
        self.rate = self.create_rate(self.control_update_rate)

        # Declare a place to save the target object y coordinate
        self.target_pose_y = 0.0

        # Declare subscriptions and publications
        self.get_logger().info(f"Subscribing to {self.input_topic}")
        self.subscription = self.create_subscription(
            PoseStamped, self.input_topic, self.target_pose_callback, 10
        )

        # TODO (evan): remove when we're done debugging
        self.deviation_pub = self.create_publisher(PoseStamped, "/px100/deviation", 10)

        # Create the interface to the motor and configure it
        self.tic = self.configure_motor()

    def configure_motor(self) -> TicUSB:
        """"Configure the Tic motor driver."""
        tic = TicUSB()

        tic.energize()
        tic.exit_safe_start()
        tic.set_max_speed(self.meters_per_second_to_microsteps_per_10k_seconds(self.max_speed))
        tic.halt_and_set_position(0)

        return tic

    @staticmethod
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

    def target_pose_callback(self, msg: PoseStamped):
        """Callback for the target pose subscriber"""
        self.get_logger().info(f"Received pose {msg}")
        self.target_pose_y = msg.pose.position.y

    def update(self):
        """Update the controller"""
        # We want to track the object so that its y coordinate in the robot frame
        # is zero. Implement P control over velocity to do this, but be sure to respect
        # the joint limits.
        current_position = self.tic.get_current_position()

        if current_position < self.min_position:
            self.get_logger().warn(f"Current position {current_position} is below min position {self.min_position}")
            deviation = current_position - self.min_position
            deviation /= (self.max_position - self.min_position)
        elif current_position > self.max_position:
            self.get_logger().warn(f"Current position {current_position} is above max position {self.max_position}")
            deviation = current_position - self.max_position
            deviation /= (self.max_position - self.min_position)
        else:
            deviation = self.target_pose_y

        # TODO(evan): remove when we're done debugging
        deviation_msg = PoseStamped()
        deviation_msg.pose.position.y = deviation
        self.deviation_pub.publish(deviation_msg)

        speed_command = -self.kp * deviation
        speed_command = max(min(speed_command, self.max_speed), -self.max_speed)
        speed_command_usteps_per_second = self.meters_per_second_to_microsteps_per_10k_seconds(speed_command)
        self.get_logger().info(f"Speed command: {speed_command} ({speed_command_usteps_per_second} usteps/10ks)")
        self.tic.set_target_velocity(int(speed_command_usteps_per_second))

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
