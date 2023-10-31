from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription(
        [
            # Launch elements for the arm controller
            DeclareLaunchArgument(
                "robot_model", default_value="px100", description="Robot model"
            ),
            DeclareLaunchArgument(
                "use_sim",
                default_value="false",
                description="Simulate the hardware interface.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("interbotix_xsarm_control"),
                                "launch",
                                "xsarm_control.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "robot_model": LaunchConfiguration("robot_model"),
                    "use_sim": LaunchConfiguration("use_sim"),
                }.items(),
            ),
        ]
    )
