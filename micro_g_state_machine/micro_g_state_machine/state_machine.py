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

import rclpy
from rclpy.node import Node
from statemachine import StateMachine, State
from std_msgs.msg import Empty
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_system_default
from typing import Callable
from statemachine.exceptions import TransitionNotAllowed


class FSM(StateMachine):
    """Manages states and state transitions."""

    stowed = State(initial=True)
    grasping = State()
    idle = State()

    enable = stowed.to(idle)
    grasp = idle.to(grasping)
    reset = grasping.to(idle)
    stow = grasping.to(stowed) | idle.to(stowed)

    def __init__(self, node: Node):
        """Create the state machine.

        Args:
            node: A ROS node to use for interacting with the system.
        """
        super().__init__()

        self.node = node

        self.node.create_subscription(
            Empty,
            "~/enable",
            lambda msg: self.make_transition_guard(msg, self.enable),
            qos_profile_system_default,
        )
        self.node.create_subscription(
            Empty,
            "~/shutdown",
            lambda msg: self.make_transition_guard(msg, self.shutdown),
            qos_profile_system_default,
        )
        self.node.create_subscription(
            Empty,
            "~/grasp",
            lambda msg: self.make_transition_guard(msg, self.grasp),
            qos_profile_system_default,
        )
        self.node.create_subscription(
            Empty,
            "~/reset",
            lambda msg: self.make_transition_guard(msg, self.reset),
            qos_profile_system_default,
        )

    def make_transition_guard(self, _, transition: Callable):
        """Guard the ROS node from shutdown on transition exceptions.

        This function wraps state transitions triggered by incoming ROS messages. We wrap
        the transitions to catch any transition exceptions thrown by the state machine
        (which may be accidentally caused by a user).

        Args:
            _: Signal message; this a placeholder for the incoming ROS message which we
                include to make ROS happy.
            transition: The transition function to execute when a message is received.
        """
        try:
            transition()
        except TransitionNotAllowed as e:
            self.node.get_logger().warning(f"Failed to transition between states, {e}")

    def before_enable(self):
        # TODO(evan): move to home position
        ...

    def before_grasp(self):
        # TODO(evan): start logging, arm controller, perform grasp action
        ...

    def before_reset(self):
        # TODO(evan): stop logging, move to home position, disarm controller
        ...

    def before_stow(self):
        # TODO(evan): move to stow position
        ...


def main(args: list[str] | None = None):
    rclpy.init(args=args)

    node = Node("state_machine")
    FSM(node)

    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

    node.destroy_node()
    rclpy.shutdown()
