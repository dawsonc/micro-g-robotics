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
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


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
            "prefix",
            default_value="",
            description=("The prefix of the model. Expected format '<prefix>/'."),
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="micro_g_description",
            description=(
                "The project description package. This package should include the"
                " description files to launch."
            ),
        ),
    ]

    use_sim = LaunchConfiguration("use_sim")
    prefix = LaunchConfiguration("prefix")
    description_package = LaunchConfiguration("description_package")

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "xacro",
                    "target",
                    "config.xacro",
                ]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_sim:=",
            use_sim,
            " ",
            "description_package:=",
            description_package,
        ]
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {"use_sim_time": use_sim, "robot_description": robot_description}
            ],
        ),
    ]

    includes = [
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
    ]

    return LaunchDescription(args + includes + nodes)
