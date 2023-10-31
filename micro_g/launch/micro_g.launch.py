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

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description to run the system.

    Returns
    -------
        The launch description for the micro-g system.

    """
    args = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Launch the micro-g simulation environment.",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="px100",
            description="Robot model argument",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="px100",
            description="Robot name argument",
        ),
    ]

    use_sim = LaunchConfiguration("use_sim")
    robot_model = LaunchConfiguration("robot_model")
    robot_name = LaunchConfiguration("robot_name")

    includes = [
        # Sim environment (if needed)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("micro_g"),
                            "launch",
                            "sim.launch.py",
                        ]
                    )
                ]
            ),
            condition=IfCondition(use_sim),
        ),
        # Hardware interfaces
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("micro_g"),
                            "launch",
                            "hw_interfaces.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "robot_model": robot_model,
                "use_sim": use_sim,
            }.items(),
        ),
        # Controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("micro_g_controllers"),
                            "launch",
                            "controllers.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "robot_model": robot_model,
                "robot_name": robot_name,
            }.items(),
        ),
    ]

    return LaunchDescription(args + includes)
