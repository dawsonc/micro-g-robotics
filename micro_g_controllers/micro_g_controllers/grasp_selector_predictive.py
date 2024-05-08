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
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_pose, do_transform_vector3
from tf2_ros import Buffer, TransformListener
import numpy as np

from micro_g_controllers.grasp_selector_predictive_parameters import grasp_selector


class GraspSelectorNode(Node):
    def __init__(self):
        super().__init__("grasp_selector")

        self.param_listener = grasp_selector.ParamListener(self)
        self.params = self.param_listener.get_params()
        self.create_timer(
            1, self.update_parameters_callback, MutuallyExclusiveCallbackGroup()
        )

        self.time_since_target_last_seen = self.params.max_time_since_last_seen

        # Declare subscriptions and publications
        self.get_logger().info(
            f"Subscribing to {self.params.input_topic}, publishing to"
            f" {self.params.output_pose_topic}"
        )
        self.subscription = self.create_subscription(
            Odometry, self.params.input_topic, self.object_pose_callback, 10
        )
        self.time_since_last_seen_sub = self.create_subscription(
            Float32, "/time_since_last_seen", self.time_since_last_seen_callback, 10
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped, self.params.output_pose_topic, 10
        )
        self.grasp_event_publisher = self.create_publisher(
            Bool, self.params.output_grasp_topic, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def update_parameters_callback(self):
        """Update the ROS parameters."""
        if self.param_listener.is_old(self.params):
            self.param_listener.refresh_dynamic_parameters()
            self.params = self.param_listener.get_params()

    def time_since_last_seen_callback(self, msg):
        self.time_since_target_last_seen = msg.data

    def object_pose_callback(self, msg):
        # Skip if stale
        if self.time_since_target_last_seen > self.params.max_time_since_last_seen:
            self.get_logger().warn(
                f"Skipping object pose callback, time since last seen: "
                f"{self.time_since_target_last_seen}"
            )
            return

        # Extract the current pose and velocity of the object
        object_pose = PoseWithCovarianceStamped()
        object_pose.header = msg.header
        object_pose.pose = msg.pose

        object_velocity = Vector3Stamped()
        object_velocity.vector.x = msg.twist.twist.linear.x
        object_velocity.vector.y = msg.twist.twist.linear.y
        object_velocity.vector.z = msg.twist.twist.linear.z

        try:
            # Transform the object pose and velocity into the target frame
            transform = self.tf_buffer.lookup_transform(
                self.params.target_frame,
                object_pose.header.frame_id,
                rclpy.time.Time(),
            )
            object_pose_world = do_transform_pose(object_pose.pose.pose, transform)
            self.get_logger().debug(
                f"Object pose in {object_pose.header.frame_id}: {object_pose.pose}"
            )
            self.get_logger().debug(
                f"Object pose in {self.params.target_frame}: {object_pose_world}"
            )

            # Transform the object velocity into the target frame
            object_velocity_world = do_transform_vector3(
                object_velocity, transform
            ).vector
            self.get_logger().debug(
                f"Object velocity in {object_pose.header.frame_id}: {object_velocity}"
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().error(f"Transform lookup failed: {str(e)}")

        # TODO overwrite unreliable velocity estimate
        object_velocity_world.x = 0.0
        object_velocity_world.y = 0.05
        object_velocity_world.z = 0.0

        # Get the time at which the object should reach the desired y position
        planned_grasp_time = (
            self.params.desired_grasp_y - object_pose_world.position.y
        ) / (object_velocity_world.y)
        self.get_logger().info(f"Current y {object_pose_world.position.y}, desired {self.params.desired_grasp_y}, velocity {object_velocity_world.y} planned grasp time: {planned_grasp_time}")

        # If the anticipated grasp time is within 0.2 seconds, publish a grasp event
        if planned_grasp_time < self.params.grasp_lead_time:
            self.grasp_event_publisher.publish(Bool(data=True))
        else:
            self.grasp_event_publisher.publish(Bool(data=False))

        if planned_grasp_time > 1e4:
            self.get_logger().warn(
                "Planned grasp time is too far in the future, skipping"
            )
            return
        if planned_grasp_time < 0:
            self.get_logger().warn(
                "Planned grasp time is in the past, skipping"
            )
            return

        # Anticipate the object's position at the planned grasp time
        anticipated_object_position = np.array([
            object_pose_world.position.x + object_velocity_world.x * planned_grasp_time,
            object_pose_world.position.y + object_velocity_world.y * planned_grasp_time,
            object_pose_world.position.z + object_velocity_world.z * planned_grasp_time,
        ])

        # Anticipate the angle of incidence in the x-y plane
        angle_of_incidence = np.arctan2(
            object_velocity_world.y, object_velocity_world.x
        )
        self.get_logger().info(f"vx: {object_velocity_world.x}, vy: {object_velocity_world.y}, angle of incidence: {angle_of_incidence}")
        if angle_of_incidence < 0 or angle_of_incidence > np.pi:
            self.get_logger().warn(
                "Angle of incidence is invalid, skipping"
            )
            return

        # TODO for now, fix angle of incidence
        angle_of_incidence = np.pi / 2.0

        self.get_logger().info(
            f"Anticipated object position: {anticipated_object_position}, angle of incidence: {angle_of_incidence}"
        )

        # Set the grasp pose to be the object's anticipated position in the world frame,
        # but rotated by the angle of incidence
        desired_pose = PoseStamped()
        desired_pose.header.frame_id = self.params.target_frame
        desired_pose.header.stamp = object_pose.header.stamp
        desired_pose.pose.position.x = (
            anticipated_object_position[0] + self.params.offset.x
        )
        desired_pose.pose.position.y = (
            anticipated_object_position[1] + self.params.offset.y
        )
        desired_pose.pose.position.z = (
            anticipated_object_position[2] + self.params.offset.z
        )

        # Rotate the pose by the angle of incidence (plus 180 degrees to be opposite)
        desired_pose.pose.orientation.x = 0.0
        desired_pose.pose.orientation.y = 0.0
        desired_pose.pose.orientation.z = np.sin((angle_of_incidence + np.pi) / 2)
        desired_pose.pose.orientation.w = np.cos((angle_of_incidence + np.pi) / 2)

        # Publish the transformed pose
        self.get_logger().debug(f"Desired pose: {desired_pose}")
        self.pose_publisher.publish(desired_pose)


def main(args=None):
    rclpy.init(args=args)

    node = GraspSelectorNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

    node.destroy_node()
    rclpy.shutdown()
