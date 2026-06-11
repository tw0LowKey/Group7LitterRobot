#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped

from tf2_ros import Time, TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs  # registers PointStamped TF support


class PointCloudToDetectedLitterNode(Node):
    """
    Converts /cloud/centroids into detected litter topics.

    Publishes:
        /vision/detected_litter       in odom frame
        /vision/detected_litter_base  in base_link frame

    /vision/detected_litter:
        Used by the navigation node to send Nav2 goals.

    /vision/detected_litter_base:
        Used by the navigation node to check whether litter is directly
        in front of the Scout and close enough for pickup/retry.

    Subscribes:
        /cloud/centroids

    TF required:
        odom -> base_link

    This node manually broadcasts:
        base_link -> camera_link
    using the hardcoded camera transform.
    """

    def __init__(self):
        super().__init__("pointcloud_to_detected_litter_node")

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter("cloud_topic", "/cloud/centroids")

        # Odom-frame litter topic for navigation.
        self.declare_parameter("litter_topic", "/vision/detected_litter")

        # Base-link-frame litter topic for front/close checks.
        self.declare_parameter("litter_base_topic", "/vision/detected_litter_base")

        # Option A: use odom everywhere for Nav2 goal positions.
        self.declare_parameter("target_frame", "odom")

        # Match your robot odom child frame.
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_link")

        # Optional filter before publishing.
        # Keep this bigger than the all-in-one node's radius, or set to 0 to disable.
        self.declare_parameter("max_publish_radius", 3.0)

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.litter_topic = self.get_parameter("litter_topic").value
        self.litter_base_topic = self.get_parameter("litter_base_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.max_publish_radius = float(
            self.get_parameter("max_publish_radius").value
        )

        # ============================================================
        # TF setup
        # ============================================================

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.camera_to_base_transform = self.get_camera_transform()
        self.create_timer(0.1, self.publish_camera_tf)

        # ============================================================
        # Subscriber / Publishers
        # ============================================================

        self.pc_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.pc_callback,
            10,
        )

        # Odom-frame publisher for Nav2/navigation.
        self.litter_pub = self.create_publisher(
            PoseStamped,
            self.litter_topic,
            10,
        )

        # Base-link-frame publisher for front/close checks.
        self.litter_base_pub = self.create_publisher(
            PoseStamped,
            self.litter_base_topic,
            10,
        )

        self.get_logger().info("=== PointCloud to Detected Litter Node Started ===")
        self.get_logger().info(f"Cloud topic:       {self.cloud_topic}")
        self.get_logger().info(f"Litter odom topic: {self.litter_topic}")
        self.get_logger().info(f"Litter base topic: {self.litter_base_topic}")
        self.get_logger().info(f"Target frame:      {self.target_frame}")
        self.get_logger().info(f"Base frame:        {self.base_frame}")
        self.get_logger().info(f"Camera frame:      {self.camera_frame}")
        self.get_logger().info(f"Max radius:        {self.max_publish_radius:.2f} m")

    # ============================================================
    # Camera transform
    # ============================================================

    def get_camera_transform(self) -> TransformStamped:
        """
        Hardcoded base_link -> camera_link transform.

        These values are copied from your existing monitor.
        Check these carefully on the real robot.
        """

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.camera_frame

        # Camera position relative to base_link
        t.transform.translation.x = 0.29
        t.transform.translation.y = 0.14
        t.transform.translation.z = 0.648

        # Camera orientation
        t.transform.rotation.x = -0.682
        t.transform.rotation.y = 0.682
        t.transform.rotation.z = -0.191
        t.transform.rotation.w = 0.191

        return t

    def publish_camera_tf(self):
        t = self.camera_to_base_transform
        t.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(t)

    # ============================================================
    # PointCloud callback
    # ============================================================

    def pc_callback(self, msg: PointCloud2):
        num_centroids = msg.width * msg.height

        if num_centroids == 0:
            return

        if msg.point_step < 12:
            self.get_logger().warn(
                f"PointCloud point_step is {msg.point_step}, expected at least 12 bytes."
            )
            return

        published_count = 0
        published_base_count = 0

        for i in range(num_centroids):
            offset = i * msg.point_step

            try:
                x = struct.unpack_from("<f", msg.data, offset)[0]
                y = struct.unpack_from("<f", msg.data, offset + 4)[0]
                z = struct.unpack_from("<f", msg.data, offset + 8)[0]
            except Exception as e:
                self.get_logger().warn(f"Failed to unpack centroid {i}: {e}")
                continue

            # Raw centroid in camera frame.
            cam_point = PointStamped()
            cam_point.header.frame_id = (
                msg.header.frame_id if msg.header.frame_id else self.camera_frame
            )
            cam_point.header.stamp = self.get_clock().now().to_msg()
            cam_point.point.x = float(x)
            cam_point.point.y = float(y)
            cam_point.point.z = float(z)

            # ------------------------------------------------------------
            # Step 1: camera_link -> base_link using hardcoded transform
            # ------------------------------------------------------------
            try:
                point_base = tf2_geometry_msgs.do_transform_point(
                    cam_point,
                    self.camera_to_base_transform,
                )
                point_base.header.frame_id = self.base_frame
                point_base.header.stamp = Time().to_msg()
            except Exception as e:
                self.get_logger().warn(
                    f"Centroid {i}: failed camera -> base transform: {e}"
                )
                continue

            distance_2d = math.sqrt(
                point_base.point.x ** 2 + point_base.point.y ** 2
            )

            if self.max_publish_radius > 0.0 and distance_2d > self.max_publish_radius:
                self.get_logger().info(
                    f"Ignoring centroid {i}: distance {distance_2d:.2f} m "
                    f"> max radius {self.max_publish_radius:.2f} m."
                )
                continue

            # ------------------------------------------------------------
            # Publish 1: litter relative to Scout/base_link
            # ------------------------------------------------------------
            # This topic is useful for simple pickup/front checks:
            #   x > 0 means in front of robot
            #   y tells left/right offset
            #   distance = sqrt(x^2 + y^2)
            litter_pose_base = PoseStamped()
            litter_pose_base.header.frame_id = self.base_frame
            litter_pose_base.header.stamp = self.get_clock().now().to_msg()

            litter_pose_base.pose.position.x = point_base.point.x
            litter_pose_base.pose.position.y = point_base.point.y
            litter_pose_base.pose.position.z = 0.0
            litter_pose_base.pose.orientation.w = 1.0

            self.litter_base_pub.publish(litter_pose_base)
            published_base_count += 1

            # ------------------------------------------------------------
            # Step 2: base_link -> odom using TF
            # ------------------------------------------------------------
            try:
                point_odom = self.tf_buffer.transform(
                    point_base,
                    self.target_frame,
                    timeout=Duration(seconds=1.0),
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Centroid {i}: failed base -> {self.target_frame} transform: {e}"
                )
                continue

            # ------------------------------------------------------------
            # Publish 2: litter in odom frame for Nav2/navigation
            # ------------------------------------------------------------
            litter_pose = PoseStamped()
            litter_pose.header.frame_id = self.target_frame
            litter_pose.header.stamp = point_odom.header.stamp

            litter_pose.pose.position.x = point_odom.point.x
            litter_pose.pose.position.y = point_odom.point.y
            litter_pose.pose.position.z = 0.0
            litter_pose.pose.orientation.w = 1.0

            self.litter_pub.publish(litter_pose)
            published_count += 1

            self.get_logger().info(
                f"Published litter {i}: "
                f"base=({litter_pose_base.pose.position.x:.3f}, "
                f"{litter_pose_base.pose.position.y:.3f}), "
                f"odom=({litter_pose.pose.position.x:.3f}, "
                f"{litter_pose.pose.position.y:.3f}), "
                f"base_dist={distance_2d:.3f} m"
            )

        if published_base_count > 0 or published_count > 0:
            self.get_logger().info(
                f"Published base={published_base_count}/{num_centroids}, "
                f"odom={published_count}/{num_centroids} centroids to "
                f"{self.litter_base_topic} and {self.litter_topic}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToDetectedLitterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down pointcloud-to-litter node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()