#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener

import sensor_msgs_py.point_cloud2 as pc2


class CloudToMapNode(Node):
    def __init__(self):
        super().__init__('cloud_to_map_node')

        self.map_frame = 'map'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2,
            '/cloud/centroid',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            '/centroid_map',
            10
        )

        self.get_logger().info('PointCloud2 → map node started')

    def callback(self, msg):
        # Extract first point (centroid cloud should have 1 point)
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        if len(points) == 0:
            return

        x, y, z = points[0]

        point_cam = PointStamped()
        point_cam.header = msg.header
        point_cam.point.x = x
        point_cam.point.y = y
        point_cam.point.z = z

        try:
            point_map = self.tf_buffer.transform(
                point_cam,
                self.map_frame,
                timeout=Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {e}')
            return

        self.pub.publish(point_map)

        self.get_logger().info(
            f'Map coords: x={point_map.point.x:.3f}, '
            f'y={point_map.point.y:.3f}, '
            f'z={point_map.point.z:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CloudToMapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()