#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from piper_msgs.srv import PickPlaceRequest

# Useless comment


class ServerClientNode(Node):
    def __init__(self):
        super().__init__('server_client_node')

        self.cb_group = ReentrantCallbackGroup()

        self.pick_place_client = self.create_client(
            PickPlaceRequest,
            'pick_place_request',
            callback_group=self.cb_group
        )

        self._executor = None  # set in main before run_requests is called

    def run_requests(self):
        z_height = -0.17

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.pose.position.x = 0.4
        pose_stamped.pose.position.y = 0.0
        pose_stamped.pose.position.z = z_height
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 1.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 0.0

        while True:
            try:
                option = float(input("Pick pose: "))

                if option == 1.0:
                    pose_stamped.pose.position.x = 0.4
                    pose_stamped.pose.position.y = 0.0
                    pose_stamped.pose.position.z = z_height
                    pose_stamped.pose.orientation.x = 0.0
                    pose_stamped.pose.orientation.y = 1.0
                    pose_stamped.pose.orientation.z = 0.0
                    pose_stamped.pose.orientation.w = 0.0
                    self.send_pp_request(pose_stamped)
                elif option == 2.0:
                    pose_stamped.pose.position.x = 0.31
                    pose_stamped.pose.position.y = -0.11
                    pose_stamped.pose.position.z = z_height
                    pose_stamped.pose.orientation.x = -0.2924
                    pose_stamped.pose.orientation.y = 0.9563
                    pose_stamped.pose.orientation.z = 0.0
                    pose_stamped.pose.orientation.w = 0.0
                    self.send_pp_request(pose_stamped)
                elif option == 3.0:
                    pose_stamped.pose.position.x = 0.27
                    pose_stamped.pose.position.y = 0.19
                    pose_stamped.pose.position.z = z_height
                    pose_stamped.pose.orientation.x = -0.3987
                    pose_stamped.pose.orientation.y = 0.9171
                    pose_stamped.pose.orientation.z = 0.0
                    pose_stamped.pose.orientation.w = 0.0
                    self.send_pp_request(pose_stamped)
                elif option == 4.0:
                    pose_stamped.pose.position.x = 0.45
                    pose_stamped.pose.position.y = -0.22
                    pose_stamped.pose.position.z = z_height
                    pose_stamped.pose.orientation.x = 0.1219
                    pose_stamped.pose.orientation.y = 0.9925
                    pose_stamped.pose.orientation.z = 0.0
                    pose_stamped.pose.orientation.w = 0.0
                    self.send_pp_request(pose_stamped)
                elif option == 5.0:
                    pose_stamped.pose.position.x = 0.43
                    pose_stamped.pose.position.y = 0.15
                    pose_stamped.pose.position.z = z_height
                    pose_stamped.pose.orientation.x = 0.3256
                    pose_stamped.pose.orientation.y = 0.9455
                    pose_stamped.pose.orientation.z = 0.0
                    pose_stamped.pose.orientation.w = 0.0
                    self.send_pp_request(pose_stamped)

                elif option == 0.0:
                    print("Current z = ",z_height)
                    z_height = float(input("Enter new z height: "))         
            
            except Exception as e:
                print(e)
                break

    def send_pp_request(self, pose_stamped: PoseStamped) -> bool:
        if not self.pick_place_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('pick_place_request service not available.')
            return False

        pose_request = PickPlaceRequest.Request()
        pose_request.poses = [pose_stamped]  # array with single PoseStamped

        future = self.pick_place_client.call_async(pose_request)
        self._executor.spin_until_future_complete(future, timeout_sec=30.0)

        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error(f'pick_place_request call failed: {future.exception()}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ServerClientNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    node._executor = executor

    node.run_requests()

    try:
        executor.spin_once()
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()