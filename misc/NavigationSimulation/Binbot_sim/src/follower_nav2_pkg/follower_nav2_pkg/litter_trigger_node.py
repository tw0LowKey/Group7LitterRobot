#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class LitterTriggerNode(Node):
    """
    Test node: publishes fake litter detections when the leader
    reaches predefined positions along its lawnmower path.

    Each trigger fires once. Litter is placed slightly off-path
    so the leader has to approach it.

    Triggers:
        L1 — Row 1 straight       (easy baseline)
        L2 — Turn 1 corner        (heading alignment test)
        L3 — Row 2 mid, reversed  (opposite direction)
        L4 — Turn 3 corner        (state reset after 3 cycles)
        L5 — Row 5 near end       (long trail, end of sweep)
    """

    def __init__(self):
        super().__init__("litter_trigger_node")

        self.declare_parameter("trigger_radius", 0.5)
        self.trigger_radius = self.get_parameter("trigger_radius").value

        # Each entry defines a test scenario:
        #   "leader" — the XY position the leader must reach to fire the trigger
        #   "litter" — where the fake litter appears (slightly off the path so
        #              the leader actually has to leave the row to collect it)
        self.triggers = [
             {
                "name": "L1 — Angled pickup",
                "leader": (2.00, 0.00),
                "litter": (2.40, 1.00),
                "fired": False,
            },
            {
                "name": "L2 — Turn 1 corner",
                "leader": (4.79, -0.60),
                "litter": (4.50, -0.90),
                "fired": False,
            },
            {
                "name": "L3 — Row 2 reversed",
                "leader": (0.80, -1.35),
                "litter": (0.50, -1.75),
                "fired": False,
            },
        ]

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.litter_pub = self.create_publisher(
            PoseStamped, "/vision/detected_litter", 10
        )

        self.get_logger().info("=== Litter Trigger Node Started ===")
        self.get_logger().info(f"Trigger radius: {self.trigger_radius}m")
        for t in self.triggers:
            self.get_logger().info(
                f"  {t['name']}: leader near {t['leader']}, "
                f"litter at {t['litter']}"
            )

    def odom_callback(self, msg):
        lx = msg.pose.pose.position.x
        ly = msg.pose.pose.position.y

        for t in self.triggers:
            if t["fired"]:
                continue

            dx = lx - t["leader"][0]
            dy = ly - t["leader"][1]
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < self.trigger_radius:
                t["fired"] = True
                self.publish_litter(t["name"], t["litter"][0], t["litter"][1])

    def publish_litter(self, name, x, y):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.litter_pub.publish(msg)
        self.get_logger().info(f"TRIGGERED {name} — litter at ({x:.2f}, {y:.2f})")

        # Log how many remain
        remaining = sum(1 for t in self.triggers if not t["fired"])
        self.get_logger().info(f"  {remaining} triggers remaining")


def main(args=None):
    rclpy.init(args=args)
    node = LitterTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()