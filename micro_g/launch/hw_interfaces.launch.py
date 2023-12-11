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
                    "motor_configs": PathJoinSubstitution(
                        [
                            FindPackageShare("micro_g"),
                            "config",
                            "px100.yml",
                        ]
                    ),
                    "mode_configs": PathJoinSubstitution(
                        [
                            FindPackageShare("micro_g"),
                            "config",
                            "modes.yml",
                        ]
                    ),
                }.items(),
            ),
        ]
    )
