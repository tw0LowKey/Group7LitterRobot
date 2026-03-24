import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf2_geometry_msgs

class LitterTransformNode(Node):
    def __init__(self):
        super().__init__('litter_transform_node')

        # 1. Pre-calculate the static transform (Camera to Scout Mini Base)
        self.camera_to_base_transform = self.get_camera_transform()

        # Broadcast it so RViz can draw the camera hovering above the Scout Mini
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster.sendTransform(self.camera_to_base_transform)

        # 2. Subscriber: Listens for a SINGLE litter coordinate from the vision system
        self.subscription = self.create_subscription(
            PoseStamped,
            '/grasp_pose',  # Update this to your vision node's output topic
            self.pose_callback,
            10
        )

        # 3. Publisher: Broadcasts the transformed coordinate to the Scout Mini
        self.publisher = self.create_publisher(
            PoseStamped,
            '/scout/target_pose',   # Update this to your navigation stack's goal topic
            10
        )

        self.get_logger().info('Litter Transform Node is running. Waiting for coordinates...')

    def get_camera_transform(self) -> TransformStamped:
        """Creates the mathematical transform from the camera to the Scout Mini base."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'   # The Scout Mini's center origin
        t.child_frame_id = 'camera_link_qais_smells'  # The Femto Mega's location
       
        # Ensure these are the actual mounted offsets on your Scout Mini
        t.transform.translation.x = 0.14  
        t.transform.translation.y = 0.14   
        t.transform.translation.z = 0.600  

        # Camera pitched 57.30 degrees downwards
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.479  
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.878
        
        return t

    def pose_callback(self, msg: PoseStamped):
        """Triggers instantly every time the camera sees a piece of litter."""
        try:
            # We use the raw '.pose' from the incoming PoseStamped to avoid the ROS 2 bug
            transformed_raw_pose = tf2_geometry_msgs.do_transform_pose(msg.pose, self.camera_to_base_transform)
            
            # Create a brand new PoseStamped message for the navigation stack
            target_msg = PoseStamped()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.header.frame_id = 'base_link'  # Officially in the robot's frame!
            target_msg.pose = transformed_raw_pose

            # Publish it to the Scout Mini
            self.publisher.publish(target_msg)
            
            # Print a clean log so you can verify the X/Y distances make sense
            self.get_logger().info(
                f"Litter Transformed -> Target is {target_msg.pose.position.x:.2f}m Forward, "
                f"{target_msg.pose.position.y:.2f}m Left/Right"
            )

        except Exception as e:
            self.get_logger().warn(f"Could not transform litter pose: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LitterTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()