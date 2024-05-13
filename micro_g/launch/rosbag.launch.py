# Copyright 2024, Micro-G Dev Team
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
from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess


TOPIC_NAMES = [
    "/accel/imu_info",
    "/accel/metadata",
    "/accel/sample",
    "/gyro/imu_info",
    "/gyro/metadata",
    "/gyro/sample",
    "/imu",
    "/color/camera_info",
    "/color/image_raw",
    "/color/metadata",
    "/extrinsics/depth_to_accel",
    "/extrinsics/depth_to_color",
    "/extrinsics/depth_to_depth",
    "/extrinsics/depth_to_gyro",
    "/detections",
    "/object_pose",
    "/odometry/unfiltered",
    "/time_since_last_seen",
    "/grasp_selector/current_target_point",
    "/grasp_selector/estimated_angle_of_incidence",
    "/grasp_selector/estimated_grasp_point",
    "/grasp_selector/estimated_velocity",
    "/px100/desired_eef_pose",
    "/px100/grasp",
    "/px100/robot_description",
    "/px100/joint_states",
    "/px100/commands/joint_group",
    "/px100/commands/joint_single",
    "/px100/commands/joint_trajectory",
    "/tf",
    "/tf_static",
]


def generate_launch_description():
    # Get the current date and time in the desired format
    date_string = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    bag_name = f"src/micro-g-robotics/bags/rosbag_{date_string}"

    bag_record_node = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            bag_name,
            *TOPIC_NAMES,
        ],
        output="screen",
    )

    return LaunchDescription([bag_record_node])
