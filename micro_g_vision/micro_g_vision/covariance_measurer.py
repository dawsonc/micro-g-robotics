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
import time
from collections import deque

import numpy as np
import rclpy
import transforms3d
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
from transforms3d._gohlketransforms import quaternion_slerp


class CovarianceMeasurerNode(Node):
    """A node that estimates the covariance of a pose."""

    def __init__(self):
        super().__init__("covariance_measurer")
        self.subscription = self.create_subscription(
            PoseStamped,
            "/object_pose",
            self.callback,
            qos_profile_system_default,
        )

        self.samples = deque([], maxlen=100)
        self.create_timer(1.0, self.print_covariance)

    def callback(self, msg):
        # Get the X, Y, Z position and rotation about X, Y, Z axes
        position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )
        orientation = np.array(
            [
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ]
        )
        euler = transforms3d.euler.quat2euler(orientation)

        self.samples.append(np.concatenate([position, euler]))

    def print_covariance(self):
        samples = np.array(self.samples)
        if len(samples) < 2:
            self.get_logger().warning("Not enough samples to get covariance...")
            return
        mean = np.mean(samples, axis=0)
        centered = samples - mean
        covariance = np.dot(centered.T, centered) / (len(samples) - 1)
        self.get_logger().info(f"{mean}")
        self.get_logger().info(f"{covariance}")


def main(args=None):
    rclpy.init(args=args)
    node = CovarianceMeasurerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


"""
Covariance was measured to be
cov = np.array(
    [
        [5.249e-07, -6.370e-08, -2.438e-07, 1.623e-05, -3.177e-06, 1.601e-05],
        [-6.370e-08, 4.556e-07, 3.575e-08, 1.950e-06, 2.182e-06, 1.087e-06],
        [-2.438e-07, 3.575e-08, 1.327e-07, -1.012e-05, 1.294e-06, -8.058e-06],
        [1.623e-05, 1.950e-06, -1.012e-05, 8.972e-04, -5.394e-05, 6.080e-04],
        [-3.177e-06, 2.182e-06, 1.294e-06, -5.394e-05, 2.929e-05, -7.886e-05],
        [1.601e-05, 1.087e-06, -8.058e-06, 6.080e-04, -7.886e-05, 5.375e-04],
    ]
)
"""
