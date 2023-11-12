import rclpy
from rclpy.node import Node
from statemachine import StateMachine, State


class GraspingFSM(StateMachine):
    disabled = State(initial=True)
    stowed = State()
    grasping = State()

    enable = disabled.to(stowed)
    disable = stowed.to(disabled) | grasping.to(disabled)
    grasp = stowed.to(grasping)
    reset = grasping.to(stowed)

    def __init__(self, node: Node):
        self.node = node

    def before_enable(self):
        # TODO(evan): enable motors and move to initial position
        ...

    def before_disable(self):
        # TODO(evan): disable motors
        ...

    def before_grasp(self):
        # TODO(evan): trigger the grasping pipeline
        ...

    def before_reset(self):
        # TODO(evan): reset joint position
        ...


class ExperimentFSM(StateMachine):
    waiting = State(initial=True)
    running = State()

    start_experiment = waiting.to(running)
    reset_experiment = running.to(waiting)

    def __init__(self, node: Node):
        super().__init__()
        self.node = node

    def before_start_experiment(self):
        # TODO(evan): start bag file recording
        ...

    def before_reset_experiment(self):
        # TODO(evan): stop bag file recording
        ...


def main(args: list[str] | None = None):
    rclpy.init(args=args)
