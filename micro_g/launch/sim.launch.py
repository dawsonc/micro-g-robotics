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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description to run the Gazebo simulator.

    Returns
    -------
        The launch description for the micro-g simulator.

    """
    args = [
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
        DeclareLaunchArgument(
            "gz_world_file",
            default_value="micro_g.world",
            description="The Gazebo world file to launch.",
        ),
        DeclareLaunchArgument(
            "robot_description",
            default_value=None,
            description="The robot URDF description.",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="px100",
            description="The name of the robot model to use.",
        ),
    ]

    target_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("description_package")),
                    "xacro",
                    "target",
                    "config.xacro",
                ]
            ),
            " ",
            "prefix:=",
            LaunchConfiguration("prefix"),
            " ",
            "use_sim:=",
            "true",
            " ",
            "description_package:=",
            LaunchConfiguration("description_package"),
        ]
    )

    nodes = [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                # Clock (IGN -> ROS 2)
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
            ],
            output="both",
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                LaunchConfiguration("robot_model"),
                "-string",
                LaunchConfiguration("robot_description"),
            ],
            output="both",
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                "target",
                "-string",
                target_description,
            ],
            output="both",
        ),
    ]

    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ros_gz_sim"),
                            "launch",
                            "gz_sim.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments=[
                (
                    "gz_args",
                    [
                        "-v",
                        "4",
                        " ",
                        "-r",
                        " ",
                        LaunchConfiguration("gz_world_file"),
                    ],
                )
            ],
        ),
    ]

    return LaunchDescription(args + nodes + includes)
