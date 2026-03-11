#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from piper_msgs.srv import MoveToPose
from piper_msgs.srv import MoveToHome
from piper_msgs.srv import SetGripWidth
from piper_msgs.srv import PickPlaceRequest


class ServerClientNode(Node):
    def __init__(self):
        super().__init__('server_client_node')

        self.cb_group = ReentrantCallbackGroup()

        # --- Service Server ---
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

        self.place_pose = Pose()
        self.place_pose.position.x = 0.0
        self.place_pose.position.y = -0.4
        self.place_pose.position.z = -0.05
        self.place_pose.orientation.x = 0.707
        self.place_pose.orientation.y = 0.707
        self.place_pose.orientation.z = 0.0
        self.place_pose.orientation.w = 0.0

        self._executor = None  # set in main after executor is created

        self.get_logger().info('ServerClientNode is ready.')

    def server_callback(self, request, response):
        self.get_logger().info(f'Received request: {request}')

        pick_pose = request.pose

        results = 0

        # Open gripper then move to pick position
        self.get_logger().info('Attempting Pick')
        self.set_grip("open")
        result = self.send_pose_request(pick_pose)

        if not result:
            self.get_logger().error('Pick failed.')
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

        # Always return home
        self.get_logger().info('Returning home')
        while not self.request_home():
            self.get_logger().error('Return home failed.')
        self.set_grip("close")

        self.get_logger().info('Returned home')
        results += 1

        #if not result:
        #    self.get_logger().error('Return home failed.')
        #else:
        #    self.get_logger().info('Returned home')
        #    results += 1

        response.success = (results == 3)
        return response

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
    node._executor = executor  # give the node a reference to the executor

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()