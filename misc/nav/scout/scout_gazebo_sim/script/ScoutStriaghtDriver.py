import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ScoutStraightDriver(Node):
    def __init__(self):
        super().__init__('scout_straight_driver')
        # Create a publisher for the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/scout_mini/cmd_vel', 10)
        
        # Timer to publish at 10Hz (0.1 seconds)
        self.timer = self.create_timer(0.1, self.move_straight)
        
        self.get_logger().info('Scout Mini Straight Driver started. Press Ctrl+C to stop.')

    def move_straight(self):
        msg = Twist()
        
        # Linear velocity (m/s) - adjust as needed
        msg.linear.x = 0.5  
        
        # Ensure angular velocity is 0 to stay straight
        msg.angular.z = 0.0 
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScoutStraightDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop the robot before shutting down
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.get_logger().info('Stopping Scout Mini...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()