import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy

class LitterDetectionNode(Node):
    def __init__(self):
        super().__init__('fake_litter_interrupt_node')

        self.publisher = self.create_publisher(
            PoseStamped,
            '/vision/detected_litter',
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.get_logger().info('Litter detection node ready')
    
    def publish_litter(self, x:float, y:float):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = Time().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0 # Neutral orientation

        self.publisher.publish(msg)
        self.get_logger().info(f'Published litter at x={x: .2f}, y={y: .2f}')

def main(args=None):
    rclpy.init(args=args)
    node = LitterDetectionNode()

    # Simulate a detection 
    node.publish_litter(x=2.0, y=0.7)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()