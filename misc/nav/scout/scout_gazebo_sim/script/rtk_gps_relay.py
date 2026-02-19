#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class RTKRelay(Node):
    def __init__(self):
        super().__init__('rtk_gps_relay')
        
        # 1. SUBSCRIBE to the "broken" simulation topic (Zero Covariance)
        self.sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',  # The topic coming from the Bridge
            self.callback,
            10)
            
        # 2. PUBLISH the "fixed" topic (High Precision Covariance)
        self.pub = self.create_publisher(NavSatFix, '/gps/fix_reliable', 10)

        # 3. DEFINE the "RTK Quality" Covariance Matrix
        # 0.0001 Variance = ~1cm accuracy. 
        # The diagonal values are [East, North, Altitude]
        self.rtk_covariance = [
            0.0001, 0.0,    0.0,
            0.0,    0.0001, 0.0,
            0.0,    0.0,    0.0001
        ]

    def callback(self, msg):
        # Create a new message to avoid modifying the original in place (good practice)
        fixed_msg = NavSatFix()
        fixed_msg.header = msg.header
        fixed_msg.status = msg.status
        fixed_msg.latitude = msg.latitude
        fixed_msg.longitude = msg.longitude
        fixed_msg.altitude = msg.altitude
        
        # INJECT the covariance manually
        fixed_msg.position_covariance = self.rtk_covariance
        fixed_msg.position_covariance_type = 2  # 2 = COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.pub.publish(fixed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RTKRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()