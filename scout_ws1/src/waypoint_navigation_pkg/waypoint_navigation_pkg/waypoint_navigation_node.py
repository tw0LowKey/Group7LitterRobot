import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Quaternion

class WaypointNavigationNode(Node):
    def __init__(self):
        super().__init__('waypoint_navigation_node')
        self.callback_group = ReentrantCallbackGroup()
        
        # 1. Action Client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.callback_group)
        
        # 2. Service to Start
        self.start_navigation_service = self.create_service(Trigger, 'start_navigation', self.start_navigation_callback, callback_group=self.callback_group)
        
        # 3. Load Waypoints
        self.waypoints = self.load_waypoints()
        self.current_waypoint_index = 0
        self.navigation_active = False
        self.get_logger().info('Waypoint Navigation Node started. Waiting for "start_navigation" service call...')

    def load_waypoints(self):
        try:
            # --- CRITICAL CHECK: Make sure this name matches your actual package name ---
            # If your package is "rm_localization_custom", change it below!
            package_name = 'waypoint_navigation_pkg' 
            try:
                package_path = get_package_share_directory(package_name)
            except:
                self.get_logger().error(f"Could not find package '{package_name}'. Please update the code with your actual package name.")
                return []
                
            waypoints_file = os.path.join(package_path, 'config', 'waypoints.yaml') # Check if it's in a 'config' folder or root
            
            # Fallback for testing if file path is simple
            if not os.path.exists(waypoints_file):
                waypoints_file = os.path.join(package_path, 'waypoints.yaml')

            with open(waypoints_file, 'r') as f:
                waypoints_data = yaml.safe_load(f)
                return waypoints_data['waypoints']
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {str(e)}')
            return []

    async def start_navigation_callback(self, request, response):
        if not self.navigation_active:
            if not self.waypoints:
                response.success = False
                response.message = 'No waypoints loaded!'
                return response
                
            self.navigation_active = True
            self.current_waypoint_index = 0
            self.get_logger().info('Starting navigation through waypoints...')
            
            # NOTE: The service will "hang" here until all navigation is done. 
            # This is expected in this simple script design.
            await self.navigate_to_next_waypoint()
            
            response.success = True
            response.message = 'Navigation sequence finished'
        else:
            response.success = False
            response.message = 'Navigation already in progress'
        return response

    async def navigate_to_next_waypoint(self):
        if not self.navigation_active or self.current_waypoint_index >= len(self.waypoints):
            self.navigation_active = False
            self.get_logger().info('All waypoints completed!')
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_index + 1}: {waypoint["name"]}')

        goal_msg = NavigateToPose.Goal()
        
        # --- FIX 1: Frame ID MUST be 'odom' for Blind Navigation ---
        goal_msg.pose.header.frame_id = 'odom' 
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(waypoint['pose']['position']['x'])
        goal_msg.pose.pose.position.y = float(waypoint['pose']['position']['y'])
        goal_msg.pose.pose.position.z = float(waypoint['pose']['position']['z'])
        goal_msg.pose.pose.orientation.x = float(waypoint['pose']['orientation']['x'])
        goal_msg.pose.pose.orientation.y = float(waypoint['pose']['orientation']['y'])
        goal_msg.pose.pose.orientation.z = float(waypoint['pose']['orientation']['z'])
        goal_msg.pose.pose.orientation.w = float(waypoint['pose']['orientation']['w'])

        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Nav2 action server...')

        send_goal_future = await self.nav_client.send_goal_async(goal_msg)
        
        if not send_goal_future.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            self.navigation_active = False
            return

        self.get_logger().info('Goal accepted, moving...')
        get_result_future = await send_goal_future.get_result_async()

        # --- FIX 2: Correctly checking the result status ---
        if get_result_future.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached waypoint {waypoint["name"]}')
            self.current_waypoint_index += 1
            # Recursive call to next point
            await self.navigate_to_next_waypoint()
        else:
            self.get_logger().error(f'Navigation failed with status: {get_result_future.status}')
            self.navigation_active = False

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigationNode()
    # MultiThreadedExecutor is safer for reentrant callbacks
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()