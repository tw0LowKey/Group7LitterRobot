#!/usr/bin/env python3

import threading
import time
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

        self.server = self.create_service(
            PickPlaceRequest,
            'pick_place_request',
            self.server_callback,
            callback_group=self.cb_group
        )

        self.move_client = self.create_client(
            MoveToPose, 'move_to_pose',
            callback_group=self.cb_group
        )
        self.home_client = self.create_client(
            MoveToHome, 'move_to_home',
            callback_group=self.cb_group
        )
        self.grip_client = self.create_client(
            SetGripWidth, 'set_grip_width',
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

        self.camera_to_base_transform = self.get_camera_transform()

        self.get_logger().info('ServerClientNode ready.')

    def get_camera_transform(self) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'camera_link_qais_smells'
        t.transform.translation.x = 0.14
        t.transform.translation.y = 0.13
        t.transform.translation.z = 0.600
        t.transform.rotation.x = -0.678
        t.transform.rotation.y = 0.678
        t.transform.rotation.z = -0.201
        t.transform.rotation.w = 0.201
        return t

    def _wait_for_future(self, future, timeout_sec=10.0) -> bool:
        """
        Polls a future until complete or timed out.
        Safe to call from a worker thread alongside MultiThreadedExecutor.
        """
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                self.get_logger().error("Future timed out waiting for response.")
                return False
            time.sleep(0.05)
        return True

    def server_callback(self, request, response):
        self.get_logger().info(
            f'Received pick request with {len(request.poses)} poses. '
            f'Dispatching to worker thread.'
        )

        event = threading.Event()

        def run():
            response.success = self._execute_sequence(request)
            event.set()

        threading.Thread(target=run, daemon=True).start()
        event.wait()
        return response

    def _execute_sequence(self, request) -> bool:
        self.get_logger().info('Waiting for arm services to be fully ready...')
        time.sleep(2.0)

        transformed_poses_list = []

        for target_pose_stamped in request.poses:
            try:
                transformed_pose = tf2_geometry_msgs.do_transform_pose(
                    target_pose_stamped.pose, self.camera_to_base_transform
                )
                transformed_poses_list.append(transformed_pose)
                self.get_logger().info(
                    f'Transformed: x={transformed_pose.position.x:.3f} '
                    f'y={transformed_pose.position.y:.3f} '
                    f'z={transformed_pose.position.z:.3f}'
                )
            except Exception as e:
                self.get_logger().warn(f'Could not transform pose: {e}')

        if not transformed_poses_list:
            self.get_logger().error('No valid poses after transformation. Aborting.')
            return False

        for index, test_pose in enumerate(transformed_poses_list):
            self.get_logger().info(
                f'--- Attempting grasp {index + 1} / {len(transformed_poses_list)} ---'
            )
            if self.execute_pick_place_sequence(test_pose):
                self.get_logger().info(f'SUCCESS on pose {index + 1}.')
                return True
            else:
                self.get_logger().warn(f'Pose {index + 1} failed. Trying next...')

        self.get_logger().error('All poses failed.')
        return False

    def execute_pick_place_sequence(self, pick_pose: Pose) -> bool:
        results = 0

        self.get_logger().info('Opening gripper')
        self.set_grip("open")

        self.get_logger().info('Moving to pick pose')
        if not self.send_pose_request(pick_pose):
            self.get_logger().error('Pick move failed.')
        else:
            results += 1
            self.get_logger().info('Pick move succeeded. Closing gripper.')
            self.set_grip("close")

            self.request_home()

            self.get_logger().info('Moving to place pose')
            if not self.send_pose_request(self.place_pose):
                self.get_logger().error('Place move failed.')
            else:
                results += 1
                self.get_logger().info('Place move succeeded. Opening gripper.')
                self.set_grip("open")

        self.get_logger().info('Returning home')
        while not self.request_home():
            self.get_logger().error('Home failed, retrying...')
        self.set_grip("close")
        results += 1

        return (results == 3)

    def send_pose_request(self, pose: Pose) -> bool:
        if not self.move_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('move_to_pose not available.')
            return False
        req = MoveToPose.Request()
        req.pose = pose
        future = self.move_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=10.0):
            return False
        if future.result() is not None:
            return future.result().success
        self.get_logger().error(f'move_to_pose failed: {future.exception()}')
        return False

    def request_home(self) -> bool:
        if not self.home_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('move_to_home not available.')
            return False
        req = MoveToHome.Request()
        future = self.home_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=10.0):
            return False
        if future.result() is not None:
            return future.result().success
        self.get_logger().error(f'move_to_home failed: {future.exception()}')
        return False

    def set_grip(self, state: str) -> bool:
        if not self.grip_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('set_grip_width not available.')
            return False
        req = SetGripWidth.Request()
        req.state = (state == "open")
        future = self.grip_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=10.0):
            return False
        if future.result() is not None:
            return future.result().success
        self.get_logger().error(f'set_grip_width failed: {future.exception()}')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = ServerClientNode()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()