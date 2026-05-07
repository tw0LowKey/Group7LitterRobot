import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger


class FrontOrientNode(Node):
    def __init__(self):
        super().__init__('front_orient_node')

        self.cb_group = ReentrantCallbackGroup()

        self.forward_distance = 4.0
        self.goal_yaw_deg = 90.0
        self.goal_frame = 'odom'

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.cb_group
        )

        self.start_service = self.create_service(
            Trigger,
            'send_front_goal',
            self.start_callback,
            callback_group=self.cb_group
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.cb_group
        )

        self.latest_odom = None
        self.goal_active = False
        self.current_goal_handle = None
        self.current_result_future = None

        self.get_logger().info('FrontOrientNode started.')
        self.get_logger().info('Call /send_front_goal to send one goal 4 m in front.')

    def odom_callback(self, msg):
        self.latest_odom = msg

    def start_callback(self, request, response):
        if self.goal_active:
            response.success = False
            response.message = 'A goal is already active.'
            return response

        if self.latest_odom is None:
            response.success = False
            response.message = 'No /odom received yet.'
            return response

        try:
            start_x = self.latest_odom.pose.pose.position.x
            start_y = self.latest_odom.pose.pose.position.y
            start_q = self.latest_odom.pose.pose.orientation

            start_yaw = self.quaternion_to_yaw(
                start_q.x, start_q.y, start_q.z, start_q.w
            )

            goal_x = start_x + self.forward_distance * math.cos(start_yaw)
            goal_y = start_y + self.forward_distance * math.sin(start_yaw)

            goal_yaw_rad = math.radians(self.goal_yaw_deg)
            goal_qx, goal_qy, goal_qz, goal_qw = self.yaw_to_quaternion(goal_yaw_rad)

            self.get_logger().info(
                f'Current pose: x={start_x:.3f}, y={start_y:.3f}, yaw={start_yaw:.3f} rad'
            )
            self.get_logger().info(
                f'Sending goal 4 m ahead: x={goal_x:.3f}, y={goal_y:.3f}'
            )
            self.get_logger().info(
                f'Final goal orientation: {self.goal_yaw_deg:.1f} deg'
            )

            self.send_goal(goal_x, goal_y, goal_qx, goal_qy, goal_qz, goal_qw)

            response.success = True
            response.message = 'Goal sent.'
            return response

        except Exception as e:
            self.get_logger().error(f'Error preparing goal: {e}')
            response.success = False
            response.message = f'Error preparing goal: {e}'
            return response

    def send_goal(self, goal_x, goal_y, qx, qy, qz, qw):
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(goal_x)
        goal_msg.pose.pose.position.y = float(goal_y)
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = float(qx)
        goal_msg.pose.pose.orientation.y = float(qy)
        goal_msg.pose.pose.orientation.z = float(qz)
        goal_msg.pose.pose.orientation.w = float(qw)

        self.goal_active = True

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Exception while sending goal: {e}')
            self.goal_active = False
            return

        if goal_handle is None:
            self.get_logger().error('Failed to get goal handle.')
            self.goal_active = False
            return

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2.')
            self.goal_active = False
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info('Goal accepted by Nav2.')

        self.current_result_future = goal_handle.get_result_async()
        self.current_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f'Exception while waiting for result: {e}')
            self.goal_active = False
            self.current_goal_handle = None
            self.current_result_future = None
            return

        self.current_goal_handle = None
        self.current_result_future = None
        self.goal_active = False

        if result is None:
            self.get_logger().error('Failed to get result from Nav2.')
            return

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Goal was canceled.')
        else:
            self.get_logger().error(f'Goal failed with status: {status}')

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_to_quaternion(yaw):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)
    node = FrontOrientNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()