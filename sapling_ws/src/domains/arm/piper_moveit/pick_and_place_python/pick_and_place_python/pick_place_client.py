#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped 
from piper_msgs.srv import MoveToPose
from piper_msgs.srv import MoveToHome
from piper_msgs.srv import SetGripWidth
from piper_msgs.srv import PickPlaceRequest
import tf2_geometry_msgs 

class ServerClientNode(Node):
    def __init__(self):
        super().__init__('server_client_node')

        self.cb_group = ReentrantCallbackGroup()

        # --- Service Server (Triggered by Vision System) ---
        self.server = self.create_service(
            PickPlaceRequest,
            'pick_place_request',
            self.server_callback,
            callback_group=self.cb_group
        )

        # --- Service Clients ---
        self.move_client = self.create_client(
            MoveToPose,
            'move_to_pose',
            callback_group=self.cb_group
        )

        self.home_client = self.create_client(
            MoveToHome,
            'move_to_home',
            callback_group=self.cb_group
        )

        self.grip_client = self.create_client(
            SetGripWidth,
            'set_grip_width',
            callback_group=self.cb_group
        )

        # Hardcoded drop-off location (e.g., the bin)
        self.place_pose = Pose()
        self.place_pose.position.x = 0.0
        self.place_pose.position.y = -0.4
        self.place_pose.position.z = -0.05
        self.place_pose.orientation.x = 0.707
        self.place_pose.orientation.y = 0.707
        self.place_pose.orientation.z = 0.0
        self.place_pose.orientation.w = 0.0

        # Pre-calculate and store the static transform once
        self.camera_to_base_transform = self.get_camera_transform()

        self._executor = None  # set in main after executor is created

        self.get_logger().info('ServerClientNode is ready and waiting for PickPlaceRequests.')

    def get_camera_transform(self) -> TransformStamped:
        """Creates and returns the mathematical transform from the camera to the robot base."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'   # The robot's origin point
        t.child_frame_id = 'camera_link'  # The Femto Mega's location
       
        t.transform.translation.x = 0.0093  # 9.3mm
        t.transform.translation.y = 0.014   # 14mm
        t.transform.translation.z = 0.600   # 600mm

        # Camera pitched 57.30 degrees downwards
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.479  
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 0.878
        
        return t

    def server_callback(self, request, response):
        """Handles the service request containing an array of potential PoseStamped targets."""
        self.get_logger().info(f'Received pick request with {len(request.poses)} potential poses.')

        transformed_poses_list = []

        # 1. Transform all Pose objects into the robot's base frame
        for target_pose_stamped in request.poses:
            try:
                # FIX 1: Extract '.pose' BEFORE passing it to the ROS 2 transform math!
                transformed_pose = tf2_geometry_msgs.do_transform_pose(target_pose_stamped.pose, self.camera_to_base_transform)
                transformed_poses_list.append(transformed_pose)
            except Exception as e:
                self.get_logger().warn(f"Could not transform pose: {e}")

        # Safety Check
        if not transformed_poses_list:
            self.get_logger().error('No valid poses remained after transformation. Aborting sequence.')
            response.success = False
            return response

        sequence_success = False

        # 2. Iterate through the transformed poses until one succeeds
        for index, test_pose in enumerate(transformed_poses_list):
            self.get_logger().info(f'--- Attempting Grasp with Pose {index + 1} of {len(transformed_poses_list)} ---')
            
            # FIX 2: test_pose is already a raw Pose now, so we removed the '.pose' here!
            if self.execute_pick_place_sequence(test_pose):
                self.get_logger().info(f'SUCCESS: Pose {index + 1} was reachable and sequence completed.')
                sequence_success = True
                break  
            else:
                self.get_logger().warn(f'FAILURE: Pose {index + 1} failed (likely a planning error). Trying next pose...')

        if not sequence_success:
            self.get_logger().error('CRITICAL: All provided poses failed. Object is unreachable.')

        response.success = sequence_success
        return response


    def execute_pick_place_sequence(self, pick_pose: Pose) -> bool:
        """The core physical movement logic for the Piper Arm."""
        results = 0

        # Open gripper then move to pick position
        self.get_logger().info('Attempting Pick')
        self.set_grip("open")
        
        # If this fails (e.g., unreachable), result is False
        result = self.send_pose_request(pick_pose)

        if not result:
            self.get_logger().error('Pick failed (Path planning rejected).')
        else:
            self.get_logger().info('Pick succeeded')
            results += 1
            self.set_grip("close")

            self.request_home()

            # Move to place position
            self.get_logger().info('Attempting Place')
            result = self.send_pose_request(self.place_pose)

            if not result:
                self.get_logger().error('Place failed.')
            else:
                self.get_logger().info('Place succeeded')
                results += 1
                self.set_grip("open")

        # Always return home (Even if the pick failed, we want to reset the arm to a safe position)
        self.get_logger().info('Returning home')
        while not self.request_home():
            self.get_logger().error('Return home failed. Retrying...')
        self.set_grip("close")

        self.get_logger().info('Returned home')
        results += 1

        return (results == 3)

    def send_pose_request(self, pose: Pose) -> bool:
        if not self.move_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('move_to_pose service not available.')
            return False

        pose_request = MoveToPose.Request()
        pose_request.pose = pose

        future = self.move_client.call_async(pose_request)
        self._executor.spin_until_future_complete(future, timeout_sec=10.0)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error(f'move_to_pose call failed: {future.exception()}')
            return False

    def request_home(self) -> bool:
        if not self.home_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('move_to_home service not available.')
            return False

        home_request = MoveToHome.Request()

        future = self.home_client.call_async(home_request)
        self._executor.spin_until_future_complete(future, timeout_sec=10.0)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error(f'move_to_home call failed: {future.exception()}')
            return False

    def set_grip(self, state: str) -> bool:
        if not self.grip_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('set_grip_width service not available.')
            return False

        grip_request = SetGripWidth.Request()
        grip_request.state = True if state == "open" else False

        future = self.grip_client.call_async(grip_request)
        self._executor.spin_until_future_complete(future, timeout_sec=10.0)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error(f'set_grip_width call failed: {future.exception()}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ServerClientNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node._executor = executor  

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()