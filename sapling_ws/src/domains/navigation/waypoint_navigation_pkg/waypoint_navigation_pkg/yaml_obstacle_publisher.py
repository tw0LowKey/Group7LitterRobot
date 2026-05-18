#!/usr/bin/env python3

import struct
import yaml

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField


class YamlObstaclePublisher(Node):
    def __init__(self):
        super().__init__('yaml_obstacle_publisher')

        

        self.yaml_file = "/home/ashutosh/scout_ws_2mini/src/rm_navigation/config/fixed_obstacles.yaml"
        self.topic_name = "/obstacle_points"
        self.frame_id = "odom"
        self.publish_rate = 2.0
        self.z_height = 0.0



        self.points = self.load_points_from_yaml(self.yaml_file)

        self.publisher = self.create_publisher(
            PointCloud2,
            self.topic_name,
            10
        )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_points)

        self.get_logger().info(f"Loaded {len(self.points)} obstacle points")
        self.get_logger().info(f"YAML: {self.yaml_file}")
        self.get_logger().info(f"Topic: {self.topic_name}")
        self.get_logger().info(f"Frame: {self.frame_id}")



    def load_points_from_yaml(self, yaml_file):

        with open(yaml_file, "r") as f:
            data = yaml.safe_load(f)

        if data is None or "obstacles" not in data:
            raise RuntimeError("YAML must contain key 'obstacles'")

        loaded_points = []

        for i, obs in enumerate(data["obstacles"]):

            if "x" not in obs or "y" not in obs:
                self.get_logger().warn(
                    f"Skipping obstacle {i} (missing x or y)"
                )
                continue

            x = float(obs["x"])
            y = float(obs["y"])
            z = float(obs.get("z", self.z_height))

            loaded_points.append((x, y, z))

        return loaded_points



    def create_cloud_xyz32(self, points):

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        fields = [
            PointField(
                name="x",
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="y",
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="z",
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]

        point_step = 12

        data = b"".join(
            [struct.pack("fff", *p) for p in points]
        )

        cloud = PointCloud2()

        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = point_step
        cloud.row_step = point_step * len(points)
        cloud.data = data
        cloud.is_dense = True

        return cloud


    def publish_points(self):

        cloud_msg = self.create_cloud_xyz32(self.points)

        self.publisher.publish(cloud_msg)



def main(args=None):

    rclpy.init(args=args)

    node = YamlObstaclePublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
