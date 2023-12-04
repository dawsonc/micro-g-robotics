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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
                output="log",
                arguments=[
                    "--robot_model",
                    LaunchConfiguration("robot_model"),
                    "--robot_name",
                    LaunchConfiguration("robot_name"),
                ],
            ),
            # Launch elements for the gantry controller
            Node(
                package="micro_g_controllers",
                executable="linear_axis_controller",
                name="linear_axis_controller_node",
                output="screen",
            ),
        ]
    )
