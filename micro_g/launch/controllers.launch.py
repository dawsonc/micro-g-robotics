from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """
    Generate a launch description to run the controllers for the micro-g system.

    Returns
    -------
        The launch description for the micro-g controllers.

    """
    return LaunchDescription(
        [
            # Launch elements for the arm controller
            DeclareLaunchArgument(
                "robot_model", default_value="px100", description="Robot model"
            ),
            DeclareLaunchArgument(
                "robot_name", default_value="px100", description="Robot name"
            ),
            Node(
                package="micro_g_controllers",
                executable="pose_servoing_controller",
                name="pose_servoing_controller_node",
                output="screen",
                arguments=[
                    "--robot_model",
                    LaunchConfiguration("robot_model"),
                    "--robot_name",
                    LaunchConfiguration("robot_name"),
                ],
            ),
        ]
    )
