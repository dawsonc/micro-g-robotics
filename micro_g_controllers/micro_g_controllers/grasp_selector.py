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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener

from micro_g_controllers.grasp_selector_parameters import grasp_selector


class GraspSelectorNode(Node):
    def __init__(self):
        super().__init__("grasp_selector")

        self.param_listener = grasp_selector.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.create_timer(
            1, self.update_parameters_callback, MutuallyExclusiveCallbackGroup()
        )

        # Declare subscriptions and publications
        self.get_logger().info(
            f"Subscribing to {self.params.input_topic}, publishing to"
            f" {self.params.output_topic}"
        )
        self.subscription = self.create_subscription(
            PoseStamped, self.params.input_topic, self.object_pose_callback, 10
        )
        self.publisher = self.create_publisher(
            PoseStamped, self.params.output_topic, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def update_parameters_callback(self):
        """Update the ROS parameters."""
        if self.param_listener.is_old(self.params):
            self.param_listener.refresh_dynamic_parameters()
            self.params = self.param_listener.get_params()

    def object_pose_callback(self, object_pose):
        try:
            # Transform the object pose into the world frame
            transform = self.tf_buffer.lookup_transform(
                self.params.target_frame,
                object_pose.header.frame_id,
                rclpy.time.Time(),
            )
            object_pose_world = do_transform_pose(object_pose.pose, transform)
            self.get_logger().debug(
                f"Object pose in {object_pose.header.frame_id}: {object_pose.pose}"
            )
            self.get_logger().debug(
                f"Object pose in {self.params.target_frame}: {object_pose_world}"
            )

            # Create a pose with a fixed orientation aligned with the world frame
            desired_pose = PoseStamped()
            desired_pose.header.frame_id = "world"
            desired_pose.header.stamp = object_pose.header.stamp
            desired_pose.pose.position.x = (
                object_pose_world.position.x + self.params.offset.x
            )
            desired_pose.pose.position.y = (
                object_pose_world.position.y + self.params.offset.y
            )
            desired_pose.pose.position.z = (
                object_pose_world.position.z + self.params.offset.z
            )
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
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

    node.destroy_node()
    rclpy.shutdown()
