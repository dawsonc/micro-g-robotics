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
from launch.conditions import IfCondition, UnlessCondition
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
                "The micro-g description package. This package should include the"
                " description files to launch."
            ),
        ),
        DeclareLaunchArgument(
            "motors_file",
            default_value=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("interbotix_xsarm_control"),
                        "config",
                        LaunchConfiguration("robot_model"),
                    ]
                ),
                ".yaml",
            ],
            description=(
                "The Interbotix motor configuration file. This should be the"
                " full path to the file."
            ),
        ),
        DeclareLaunchArgument(
            "modes_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("interbotix_xsarm_control"),
                    "config",
                    "modes.yaml",
                ]
            ),
            description=(
                "The Interbotix mode configuration file. This should be the"
                " full path to the file."
            ),
        ),
        DeclareLaunchArgument(
            "load_configs_from_eeprom",
            default_value="true",
            description=(
                "Whether or not the initial register values (under the 'motors' heading)"
                " in a motor configuration file should be written to the motors; as the"
                " values being written are stored in each motor's EEPROM (which means"
                " the values are retained even after a power cycle), this can be set to"
                " `false` after the first time using the robot. Setting to `false` also"
                " shortens the node startup time by a few seconds and preserves the life"
                " of the EEPROM."
            ),
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="px100",
            choices=["px100"],
            description="The name of the robot model to use.",
        ),
    ]

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(LaunchConfiguration("description_package")),
                    "xacro",
                    LaunchConfiguration("robot_model"),
                    "config.xacro",
                ]
            ),
            " ",
            "prefix:=",
            LaunchConfiguration("prefix"),
            " ",
            "use_sim:=",
            LaunchConfiguration("use_sim"),
            " ",
            "description_package:=",
            LaunchConfiguration("description_package"),
        ]
    )

    nodes = [
        # Robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="micro_g",
            output="both",
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim"),
                    "robot_description": robot_description,
                }
            ],
        ),
        # Interbotix SDK for hardware
        Node(
            condition=UnlessCondition(LaunchConfiguration("use_sim")),
            package="interbotix_xs_sdk",
            executable="xs_sdk",
            name="xs_sdk",
            namespace=LaunchConfiguration("robot_model"),
            arguments=[],
            parameters=[
                {
                    "motor_configs": LaunchConfiguration("motors_file"),
                    "mode_configs": LaunchConfiguration("modes_file"),
                    "load_configs": LaunchConfiguration("load_configs_from_eeprom"),
                    "robot_description": robot_description,
                    "use_sim_time": LaunchConfiguration("use_sim"),
                    "xs_driver_logging_level": "INFO",
                }
            ],
            output="both",
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("use_sim")),
            package="interbotix_xs_sdk",
            executable="xs_sdk_sim.py",
            name="xs_sdk_sim",
            namespace=LaunchConfiguration("robot_model"),
            parameters=[
                {
                    "motor_configs": LaunchConfiguration("motors_file"),
                    "mode_configs": LaunchConfiguration("modes_file"),
                    "robot_description": robot_description,
                    "use_sim_time": LaunchConfiguration("use_sim"),
                }
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
                            FindPackageShare("micro_g"),
                            "launch",
                            "sim.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "prefix": LaunchConfiguration("prefix"),
                "description_package": LaunchConfiguration("description_package"),
                "robot_description": robot_description,
            }.items(),
            condition=IfCondition(LaunchConfiguration("use_sim")),
        ),
    ]

    return LaunchDescription(args + includes + nodes)
