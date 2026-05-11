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


class FrontBackNode(Node):
    def __init__(self):
        super().__init__('front_back_node')

        self.cb_group = ReentrantCallbackGroup()

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.cb_group
        )

        self.start_service = self.create_service(
            Trigger,
            'start_front_back',
            self.start_front_back_callback,
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
        self.sequence_active = False
        self.delay_timer = None
        self.forward_distance = 4.0  # 400 cm

        self.get_logger().info(
            'Node started. Use /start_front_back to wait 30 s and move 4.0 m forward.'
        )

    def odom_callback(self, msg):
        self.latest_odom = msg

    def start_front_back_callback(self, request, response):
        if self.sequence_active:
            response.success = False
            response.message = 'Sequence already running.'
            return response

        if self.latest_odom is None:
            response.success = False
            response.message = 'No /odom received yet.'
            return response

        self.sequence_active = True
        self.get_logger().info('Trigger received. Waiting 30 seconds...')

        self.delay_timer = self.create_timer(
            30.0,
            self.delay_done_callback,
            callback_group=self.cb_group
        )

        response.success = True
        response.message = 'Sequence armed. Navigation will start in 30 seconds.'
        return response

    def delay_done_callback(self):
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.destroy_timer(self.delay_timer)
            self.delay_timer = None

        if self.latest_odom is None:
            self.get_logger().error('No /odom available.')
            self.sequence_active = False
            return

        try:
            start_x = self.latest_odom.pose.pose.position.x
            start_y = self.latest_odom.pose.pose.position.y
            start_q = self.latest_odom.pose.pose.orientation

            start_yaw = self.quaternion_to_yaw(
                start_q.x, start_q.y, start_q.z, start_q.w
            )

            goal_x = start_x + self.forward_distance * math.cos(start_yaw)
            goal_y = start_y + self.forward_distance * math.sin(start_yaw)

            self.get_logger().info(
                f'Start pose: x={start_x:.3f}, y={start_y:.3f}, yaw={start_yaw:.3f}'
            )
            self.get_logger().info(
                f'Forward goal: x={goal_x:.3f}, y={goal_y:.3f}'
            )

            success_forward = self.send_nav_goal(goal_x, goal_y, start_q)
            if not success_forward:
                self.get_logger().error('Forward goal failed.')
                self.sequence_active = False
                return

            self.get_logger().info('Reached forward point. Demo complete.')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.sequence_active = False

    def send_nav_goal(self, x, y, q):
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available.')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = q

        self.get_logger().info(f'Sending goal: x={x:.3f}, y={y:.3f}')

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if goal_handle is None:
            self.get_logger().error('Failed to get goal handle.')
            return False

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2.')
            return False

        self.get_logger().info('Goal accepted.')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result is None:
            self.get_logger().error('Failed to get result from Nav2.')
            return False

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded.')
            return True

        self.get_logger().error(f'Goal failed with status: {result.status}')
        return False

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = FrontBackNode()
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