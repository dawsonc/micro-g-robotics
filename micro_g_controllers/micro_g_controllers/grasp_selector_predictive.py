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
from numpy.polynomial.polynomial import Polynomial as Poly

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
        self.target_position_history = []

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
        self.desired_eef_pose_publisher = self.create_publisher(
            PoseStamped, self.params.output_pose_topic, 10
        )
        self.grasp_event_publisher = self.create_publisher(
            Bool, self.params.output_grasp_topic, 10
        )
        self.estimated_velocity_publisher = self.create_publisher(
            Vector3Stamped, "/grasp_selector/estimated_velocity", 10
        )
        self.current_point_publisher = self.create_publisher(
            Vector3Stamped, "/grasp_selector/current_target_point", 10
        )
        self.estimated_grasp_point_publisher = self.create_publisher(
            Vector3Stamped, "/grasp_selector/estimated_grasp_point", 10
        )
        self.estimated_angle_of_incidence_publisher = self.create_publisher(
            Float32, "/grasp_selector/estimated_angle_of_incidence", 10
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

        # Clear history if stale
        if self.time_since_target_last_seen > self.params.max_time_since_last_seen:
            if len(self.target_position_history) > 0:
                self.get_logger().info(
                    f"Clearing history, time since last seen: "
                    f"{self.time_since_target_last_seen}"
                )
            self.target_position_history.clear()
            return

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

        # Save the object pose in the history
        current_t = (
            object_pose.header.stamp.sec + object_pose.header.stamp.nanosec * 1e-9
        )
        self.target_position_history.append(
            np.array(
                [
                    current_t,
                    object_pose_world.position.x,
                    object_pose_world.position.y,
                    object_pose_world.position.z,
                ]
            )
        )

        # Wait until the history is full before making a prediction
        if len(self.target_position_history) < self.params.window_length:
            self.get_logger().info(
                f"Waiting for history to fill {len(self.target_position_history)} / {self.params.window_length}"
            )
            return

        # If we have enough history, fit a linear model of the object's position
        data = np.array(self.target_position_history)
        ts = data[:, 0]
        ts = ts - current_t
        x, y, z = data[:, 1], data[:, 2], data[:, 3]
        x_poly = Poly.fit(ts, x, 1)
        y_poly = Poly.fit(ts, y, 1)
        z_poly = Poly.fit(ts, z, 1)

        # Predict the time when the object will reach the desired y position
        try:
            planned_grasp_time = (y_poly - self.params.desired_grasp_y).roots()[0]
            self.get_logger().debug(
                f"Current y {y[-1]}, desired {self.params.desired_grasp_y}, planned grasp time: {planned_grasp_time}"
            )
        except IndexError:
            self.get_logger().warn("No roots found, skipping")
            return

        # Anticipate the object's position at the planned grasp time
        anticipated_object_position = np.array(
            [
                x_poly(planned_grasp_time),
                y_poly(planned_grasp_time),
                z_poly(planned_grasp_time),
            ]
        )

        # If the current position is close to the anticipated position, signal the grasp
        current_target_point = np.array(
            [
                object_pose_world.position.x,
                object_pose_world.position.y,
                object_pose_world.position.z,
            ]
        )
        distance = np.linalg.norm(current_target_point - anticipated_object_position)
        if distance < self.params.grasp_radius:
            self.grasp_event_publisher.publish(Bool(data=True))
        elif distance > self.params.grasp_radius + 0.02:  # debounce
            self.grasp_event_publisher.publish(Bool(data=False))

        # Anticipate the angle of incidence in the x-y plane
        angle_of_incidence = np.arctan2(
            y_poly.deriv()(planned_grasp_time), x_poly.deriv()(planned_grasp_time)
        )
        self.get_logger().debug(
            f"vx: {x_poly.deriv()(planned_grasp_time)}, vy: {y_poly.deriv()(planned_grasp_time)}, angle of incidence: {angle_of_incidence}"
        )

        # If we have vy < 0 for some reason, flip the angle of incidence
        if angle_of_incidence > np.pi:
            angle_of_incidence -= np.pi

        if angle_of_incidence < 0 or angle_of_incidence > np.pi:
            self.get_logger().warn("Angle of incidence is invalid, skipping")
            return

        self.get_logger().debug(
            f"Anticipated object position: {anticipated_object_position}, angle of incidence: {angle_of_incidence}"
        )

        # Publish the estimated velocity, intercept point, and angle for visibility
        estimated_velocity = Vector3Stamped()
        estimated_velocity.header.stamp = self.get_clock().now().to_msg()
        estimated_velocity.header.frame_id = "base_tag"
        estimated_velocity.vector.x = x_poly.deriv()(0.0)
        estimated_velocity.vector.y = y_poly.deriv()(0.0)
        estimated_velocity.vector.z = z_poly.deriv()(0.0)
        self.estimated_velocity_publisher.publish(estimated_velocity)

        estimated_grasp_point = Vector3Stamped()
        estimated_grasp_point.header = estimated_velocity.header
        estimated_grasp_point.vector.x = anticipated_object_position[0]
        estimated_grasp_point.vector.y = anticipated_object_position[1]
        estimated_grasp_point.vector.z = anticipated_object_position[2]
        self.estimated_grasp_point_publisher.publish(estimated_grasp_point)

        current_target_point = Vector3Stamped()
        current_target_point.header = estimated_velocity.header
        current_target_point.vector.x = object_pose_world.position.x
        current_target_point.vector.y = object_pose_world.position.y
        current_target_point.vector.z = object_pose_world.position.z
        self.current_point_publisher.publish(current_target_point)

        estimated_angle_of_incidence = Float32()
        estimated_angle_of_incidence.data = angle_of_incidence
        self.estimated_angle_of_incidence_publisher.publish(
            estimated_angle_of_incidence
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
        self.desired_eef_pose_publisher.publish(desired_pose)


def main(args=None):
    rclpy.init(args=args)

    node = GraspSelectorNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor)

    node.destroy_node()
    rclpy.shutdown()
