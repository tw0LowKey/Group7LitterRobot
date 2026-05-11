import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
from rclpy.qos import QoSProfile, DurabilityPolicy

class FakeLitterNode(Node):
    def __init__(self):
        super().__init__('fake_litter_interrupt_node')

        self.publisher_ = self.create_publisher(
            PoseStamped,
            '/vision/detected_litter', QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        # Give time for subscribers to connect
        time.sleep(1)

        self.publish_litter()

    def publish_litter(self):
        coords = [
            (3.0, -0.9),  # front-right
            # (2.5, -1.0)   # further ahead
        ]

        for x, y in coords:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 0.0

            msg.pose.orientation.w = 1.0

            self.get_logger().info(f'Publishing litter at ({x}, {y})')
            self.publisher_.publish(msg)

            time.sleep(0.5)  # small gap so both get queued properly


def main(args=None):
    rclpy.init(args=args)
    node = FakeLitterNode()
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()