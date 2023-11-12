import rclpy
from rclpy.node import Node
from statemachine import StateMachine, State
from std_msgs.msg import Empty


class FSM(StateMachine):
    disabled = State(initial=True)
    stowed = State()
    grasping = State()

    enable = disabled.to(stowed)
    grasp = stowed.to(grasping)
    reset = grasping.to(stowed)
    shutdown = stowed.to(disabled) | grasping.to(disabled)

    def before_enable(self):
        # TODO(evan): start the motors, move to home position
        ...

    def before_grasp(self):
        # TODO(evan): start logging, arm controller, run action
        ...

    def before_reset(self):
        # TODO(evan): stop logging, disarm controller, reset props, move to home position
        ...

    def before_shutdown(self):
        # TODO(evan): disarm motors
        ...


class FSMNode(Node):
    def __init__(self):
        super().__init__("state-machine")

        self.sm = FSM()

        self.create_subscription(Empty, "~/enable", lambda msg: self.sm.enable)
        self.create_subscription(Empty, "~/shutdown", lambda msg: self.sm.shutdown)
        self.create_subscription(Empty, "~/grasp", lambda msg: self.sm.grasp)
        self.create_subscription(Empty, "~/reset", lambda msg: self.sm.reset)


def main(args: list[str] | None = None):
    rclpy.init(args=args)
