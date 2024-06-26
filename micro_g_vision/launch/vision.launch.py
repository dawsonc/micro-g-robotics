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
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description to run the vision system for the micro-g system.

    Returns
    -------
        The launch description for the micro-g vision system.

    """
    return LaunchDescription(
        [
            # Launch elements for the RealSense camera
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="realsense2_camera_node",
                output="log",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("micro_g_vision"),
                            "config",
                            "realsense.yml",
                        ]
                    )
                ],
            ),
            # Launch elements for the AprilTag detector
            Node(
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag",
                output="screen",
                remappings=[
                    ("image_rect", "/color/image_raw"),
                    ("camera_info", "/color/camera_info"),
                ],
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("micro_g_vision"),
                            "config",
                            "apriltag.yml",
                        ]
                    )
                ],
            ),
            # Launch the object tracker
            Node(
                package="micro_g_vision",
                executable="object_tracker",
                name="object_tracker",
                output="screen",
            ),
            # Launch the static transform from the robot gantry apriltag to the robot base
            # There doesn't seem to be a good way to put this in a config.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.108675",
                    "--z",
                    "0.0",
                    "--yaw",
                    "-1.5708",
                    "--pitch",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--frame-id",
                    "gantry_tag",
                    "--child-frame-id",
                    "world",
                ],
            ),
        ]
    )
