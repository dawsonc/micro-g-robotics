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


class ObjectTrackerNode(Node):
    """A node that tracks an object covered in AprilTags."""

    def __init__(self, tag_attachments: dict[int, tuple[np.ndarray, np.ndarray]]):
        """Initialize the object tracker.

        Args:
            tag_attachments: a dictionary mapping tag IDs to offsets and RPY rotations
                (in the tag frame) between the center of the tag and the center of the
                object to which it's attached.
        """
        super().__init__("object_tracker")
        self.tag_attachments = tag_attachments

        # Initialize the moving average filter
        mwa_decay_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Decay rate of moving average filter",
        )
        self.declare_parameter("mwa_decay", 0.9, descriptor=mwa_decay_desc)
        self.mwa_decay = self.get_parameter("mwa_decay").value
        self.mwa_position = None
        self.mwa_orientation = np.array([1.0, 0.0, 0.0, 0.0])  # w x y z

        # Create the ROS plumbing: a publisher for the object position, a
        # subscription to the AprilTag detections, and a TF buffer and listener.
        # The subscription will be used to get the detections.
        # The publisher will be used to publish the object position (just 3D position)
        # The TF buffer and listener will be used to transform the object position
        self.publisher = self.create_publisher(
            PoseStamped, "/object_pose", qos_profile_system_default
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            "/detections",
            self.detection_callback,
            qos_profile_system_default,
        )

    def detection_callback(self, msg):
        """Process AprilTag detections."""
        # Select the first detection that we recognize and have a transform for
        chosen_detection = None
        for detection in msg.detections:
            # The tag is valid if it's in the tag_attachments dictionary
            # and a transform exists between it and the camera
            if detection.id not in self.tag_attachments:
                continue

            if not self.tf_buffer.can_transform(
                "camera_link", f"tag_{detection.id}", rclpy.time.Time()
            ):
                self.get_logger().warning(
                    f"Transform from camera_link to tag_{detection.id} does not exist"
                )
                continue

            chosen_detection = detection
            break

        # If no detection was found, return
        if chosen_detection is None:
            self.get_logger().warn("No valid AprilTag detections found")
            return

        # Otherwise, get the position and rotation of the object center in the tag frame
        p_tag_object, rpy_tag_object = self.tag_attachments[chosen_detection.id]
        pose_tag_object = PoseStamped()
        pose_tag_object.header.frame_id = f"tag_{chosen_detection.id}"
        pose_tag_object.header.stamp = self.get_clock().now().to_msg()
        pose_tag_object.pose.position.x = p_tag_object[0]
        pose_tag_object.pose.position.y = p_tag_object[1]
        pose_tag_object.pose.position.z = p_tag_object[2]
        (
            pose_tag_object.pose.orientation.w,
            pose_tag_object.pose.orientation.x,
            pose_tag_object.pose.orientation.y,
            pose_tag_object.pose.orientation.z,
        ) = transforms3d.euler.euler2quat(*rpy_tag_object)

        # Get the transform between camera and tag frame
        try:
            transform = self.tf_buffer.lookup_transform(
                "camera_link",
                f"tag_{chosen_detection.id}",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().error(f"Failed to lookup transform: {str(e)}")
            return

        # Transform to the camera frame
        pose_camera_object = PoseStamped()
        pose_camera_object.header.frame_id = "camera_link"
        pose_camera_object.header.stamp = self.get_clock().now().to_msg()
        pose_camera_object.pose = do_transform_pose(pose_tag_object.pose, transform)

        # Update the moving average filter
        new_position = np.array(
            [
                pose_camera_object.pose.position.x,
                pose_camera_object.pose.position.y,
                pose_camera_object.pose.position.z,
            ]
        )
        if self.mwa_position is None:
            self.mwa_position = new_position
        self.mwa_position = (
            self.mwa_decay * self.mwa_position + (1 - self.mwa_decay) * new_position
        )
        new_orientation = np.array(
            [
                pose_camera_object.pose.orientation.w,
                pose_camera_object.pose.orientation.x,
                pose_camera_object.pose.orientation.y,
                pose_camera_object.pose.orientation.z,
            ]
        )
        self.mwa_orientation = quaternion_slerp(
            self.mwa_orientation, new_orientation, 1 - self.mwa_decay
        )

        # Update the pose message with the filtered position and orientation
        (
            pose_camera_object.pose.position.x,
            pose_camera_object.pose.position.y,
            pose_camera_object.pose.position.z,
        ) = self.mwa_position
        (
            pose_camera_object.pose.orientation.w,
            pose_camera_object.pose.orientation.x,
            pose_camera_object.pose.orientation.y,
            pose_camera_object.pose.orientation.z,
        ) = self.mwa_orientation

        # Publish the object pose
        self.publisher.publish(pose_camera_object)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode(
        {
            0: (np.array([0.0, 0.0, -(0.025) / 2]), np.array([0.0, 0.0, 0.0])),
            1: (np.array([0.0, 0.0, -(0.025) / 2]), np.array([-np.pi / 2, 0.0, 0.0])),
            2: (np.array([0.0, 0.0, -(0.025) / 2]), np.array([-np.pi, 0.0, 0.0])),
            3: (
                np.array([0.0, 0.0, -(0.025) / 2]),
                np.array([-np.pi * 3 / 2, 0.0, 0.0]),
            ),
            4: (np.array([0.0, 0.0, -(0.025) / 2]), np.array([0.0, -np.pi / 2, 0.0])),
            5: (np.array([0.0, 0.0, -(0.025) / 2]), np.array([0.0, np.pi / 2, 0.0])),
        }
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
