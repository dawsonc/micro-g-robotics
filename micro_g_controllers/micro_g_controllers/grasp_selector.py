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
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener


class GraspSelectorNode(Node):
    def __init__(self):
        super().__init__("grasp_selector_node")

        # Load configurable parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "target_frame",
                    "world",
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_STRING,
                        description="Name of TF frame that grasp should be expressed in.",
                    ),
                ),
                (
                    "input_topic",
                    "object_pose",
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_STRING,
                        description="Name of PoseStamped topic with the object pose.",
                    ),
                ),
                (
                    "output_topic",
                    "desired_eef_pose",
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_STRING,
                        description="Name of PoseStamped topic to publish with grasp.",
                    ),
                ),
                (
                    "x_offset",
                    0.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="X-axis offset from object_pose to desired_eef_pose.",
                    ),
                ),
                (
                    "y_offset",
                    0.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="Y-axis offset from object_pose to desired_eef_pose.",
                    ),
                ),
                (
                    "z_offset",
                    0.0,
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE,
                        description="Z-axis offset from object_pose to desired_eef_pose.",
                    ),
                ),
            ],
        )
        self.target_frame = self.get_parameter("target_frame").value
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.x_offset = self.get_parameter("x_offset").value
        self.y_offset = self.get_parameter("y_offset").value
        self.z_offset = self.get_parameter("z_offset").value

        # Declare subscriptions and publications
        self.get_logger().info(
            f"Subscribing to {self.input_topic}, publishing to {self.output_topic}"
        )
        self.subscription = self.create_subscription(
            PoseStamped, self.input_topic, self.object_pose_callback, 10
        )
        self.publisher = self.create_publisher(PoseStamped, self.output_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def object_pose_callback(self, object_pose):
        try:
            # Transform the object pose into the world frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                object_pose.header.frame_id,
                rclpy.time.Time(),
            )
            object_pose_world = do_transform_pose(object_pose.pose, transform)
            self.get_logger().debug(
                f"Object pose in {object_pose.header.frame_id}: {object_pose.pose}"
            )
            self.get_logger().debug(
                f"Object pose in {self.target_frame}: {object_pose_world}"
            )

            # Create a pose with a fixed orientation aligned with the world frame
            desired_pose = PoseStamped()
            desired_pose.header.frame_id = "world"
            desired_pose.header.stamp = object_pose.header.stamp
            desired_pose.pose.position.x = object_pose_world.position.x + self.x_offset
            desired_pose.pose.position.y = object_pose_world.position.y + self.y_offset
            desired_pose.pose.position.z = object_pose_world.position.z + self.z_offset
            self.get_logger().debug(f"Desired pose: {desired_pose}")

            # Publish the transformed pose
            self.publisher.publish(desired_pose)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().error(f"Transform lookup failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = GraspSelectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
