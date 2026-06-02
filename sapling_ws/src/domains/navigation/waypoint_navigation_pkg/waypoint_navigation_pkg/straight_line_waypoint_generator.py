#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseArray, Pose


class StraightLineWaypointGeneratorNode(Node):
    """
    Generates 3 waypoints in a straight line, 1 metre apart, heading forward (+X).

    Publishes:
        /waypoints — PoseArray of XY waypoints (TRANSIENT_LOCAL)
    """

    def __init__(self):
        super().__init__("straight_line_waypoint_generator_node")

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter("waypoint_topic", "/waypoints")
        self.declare_parameter("goal_frame", "map")
        self.declare_parameter("num_waypoints", 3)
        self.declare_parameter("spacing", 1.0)
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)

        self.waypoint_topic = self.get_parameter("waypoint_topic").value
        self.goal_frame = self.get_parameter("goal_frame").value
        self.num_waypoints = self.get_parameter("num_waypoints").value
        self.spacing = self.get_parameter("spacing").value
        self.start_x = self.get_parameter("start_x").value
        self.start_y = self.get_parameter("start_y").value

        # ============================================================
        # QoS — latching so late subscribers get the waypoints
        # ============================================================

        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ============================================================
        # Publisher
        # ============================================================

        self.waypoint_pub = self.create_publisher(
            PoseArray,
            self.waypoint_topic,
            latching_qos,
        )

        self.get_logger().info("=== Straight Line Waypoint Generator Node Started ===")
        self.get_logger().info(f"Waypoint topic: {self.waypoint_topic}")
        self.get_logger().info(f"Goal frame:     {self.goal_frame}")
        self.get_logger().info(f"Num waypoints:  {self.num_waypoints}")
        self.get_logger().info(f"Spacing:        {self.spacing}m")
        self.get_logger().info(f"Start:          ({self.start_x}, {self.start_y})")

        # Publish once on startup; latched QoS handles late subscribers.
        self.publish_waypoints()

    # ============================================================
    # Waypoint generation
    # ============================================================

    def publish_waypoints(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = self.goal_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()

        # Heading is straight forward along +X, so yaw = 0.
        # Quaternion for yaw=0 is (z=0, w=1).
        heading = 0.0
        qz = math.sin(heading / 2.0)
        qw = math.cos(heading / 2.0)

        for i in range(self.num_waypoints):
            pose = Pose()
            pose.position.x = self.start_x + (i + 1) * self.spacing
            pose.position.y = self.start_y
            pose.position.z = 0.0
            pose.orientation.z = qz
            pose.orientation.w = qw
            pose_array.poses.append(pose)

        self.waypoint_pub.publish(pose_array)

        self.get_logger().info(
            f"Published {len(pose_array.poses)} waypoints to {self.waypoint_topic}"
        )
        for i, pose in enumerate(pose_array.poses):
            self.get_logger().info(
                f"  WP{i + 1}: XY ({pose.position.x:.4f}, {pose.position.y:.4f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = StraightLineWaypointGeneratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down straight line waypoint generator node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()