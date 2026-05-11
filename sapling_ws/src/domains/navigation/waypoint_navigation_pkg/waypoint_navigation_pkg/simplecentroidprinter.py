#!/usr/bin/env python3

import struct
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster

import tf2_geometry_msgs  # needed for do_transform_point


class SimpleCentroidPrinter(Node):
    def __init__(self):
        super().__init__('simple_centroid_printer')

        self.camera_frame = 'camera_link'
        self.base_frame = 'mobile_robot_base_link'

        # Broadcaster (kept in case other nodes need the TF on the tree)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Build the static camera -> base transform once
        self.camera_to_base_transform = self.get_camera_transform()

        # publish camera TF (base_link -> camera_link)
        self.create_timer(0.1, self.publish_camera_tf)

        # subscribe to centroids
        self.create_subscription(
            PointCloud2,
            '/cloud/centroids',
            self.pc_cb,
            10
        )

        self.get_logger().info(f'Printing centroids in {self.base_frame} frame...')

    def get_camera_transform(self) -> TransformStamped:
        """Hardcoded camera_link -> mobile_robot_base_link transform,
        same style as ServerClientNode.get_camera_transform()."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.camera_frame

        t.transform.translation.x = 0.29
        t.transform.translation.y = 0.14
        t.transform.translation.z = 0.648

        t.transform.rotation.x = -0.682
        t.transform.rotation.y = 0.682
        t.transform.rotation.z = -0.191
        t.transform.rotation.w = 0.191

        return t

    def publish_camera_tf(self):
        # Refresh the timestamp and broadcast (same numbers as the stored one)
        t = self.camera_to_base_transform
        t.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(t)

    def pc_cb(self, msg):
        num_points = msg.width * msg.height

        for i in range(num_points):
            offset = i * msg.point_step

            x = struct.unpack_from('<f', msg.data, offset)[0]
            y = struct.unpack_from('<f', msg.data, offset + 4)[0]
            z = struct.unpack_from('<f', msg.data, offset + 8)[0]

            pt = PointStamped()
            pt.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.camera_frame
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x = x
            pt.point.y = y
            pt.point.z = z

            try:
                pt_base = tf2_geometry_msgs.do_transform_point(
                    pt, self.camera_to_base_transform
                )

                self.get_logger().info(
                    f"[{self.base_frame}] x={pt_base.point.x:.3f}, "
                    f"y={pt_base.point.y:.3f}, "
                    f"z={pt_base.point.z:.3f}"
                )

            except Exception as e:
                self.get_logger().warn(f"Transform failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCentroidPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()