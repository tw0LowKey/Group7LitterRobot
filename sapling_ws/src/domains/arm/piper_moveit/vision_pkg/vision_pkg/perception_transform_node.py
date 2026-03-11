import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs 
from tf2_ros import TransformException

class PoseTransformNode(Node):
    def __init__(self):
        super().__init__('perception_transform_node')

        # 1. Broadcast the physical offset between the robot base and the camera
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # 2. Set up the TF2 Buffer and Listener to handle the math
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Set up the Publisher (to Master Node) and Subscriber (from Camera)
        self.publisher = self.create_publisher(PoseStamped, '/target_pose_base', 10)
        self.subscriber = self.create_subscription(
            PoseStamped, 
            '/camera_pose_raw', 
            self.pose_callback, 
            10
        )

        self.get_logger().info("Python Perception Translation Node Started. Waiting for camera data...")

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'   # The robot's origin point
        t.child_frame_id = 'camera_link'  # The Femto Mega's location

       
        t.transform.translation.x = 0.0093  # 9.3mm
        t.transform.translation.y = 0.014   # 14mm
        t.transform.translation.z = 0.600   # 600mm

        # Camera pitched 57.30 degrees (approx 1 radian) downwards
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.479  # Change to positive 0.479 if the camera looks up!
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.878

        self.tf_static_broadcaster.sendTransform(t)

    def pose_callback(self, msg: PoseStamped):
        try:
            
            # look at the msg.header.frame_id ('camera_link') and translate it to 'base_link'
            transformed_pose = self.tf_buffer.transform(msg, 'base_link')

            # Publish the translated coordinates for your Master Node
            self.publisher.publish(transformed_pose)
            
            # Log it so you can see it working in the terminal
            pos = transformed_pose.pose.position
            self.get_logger().info(f"Translated to base_link: [X: {pos.x:.3f}, Y: {pos.y:.3f}, Z: {pos.z:.3f}]")
            
        except TransformException as e:
            self.get_logger().warn(f"Could not transform pose: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()