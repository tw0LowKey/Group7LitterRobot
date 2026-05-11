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
from std_msgs.msg import Bool


class FrontInterruptNode(Node):
    def __init__(self):
        super().__init__('front_interrupt_node')

        self.cb_group = ReentrantCallbackGroup()

        # -----------------------------
        # EASY-TO-CHANGE SETTINGS
        # -----------------------------
        self.forward_distance = 4.0
        self.delay_seconds = 10.0

        # Topic coming from the other workspace
        # Current meaning:
        #   /stop_navigation = True  -> STOP / PAUSE navigation
        #   /stop_navigation = False -> ALLOW / RESUME navigation
        self.permission_topic = '/stop_navigation'

        # Navigation is allowed only when the incoming Bool equals this value
        self.allow_when_value = False

        # Nav2 goal frame
        self.goal_frame = 'odom'

        # -----------------------------
        # ACTION CLIENT / SERVICE / SUBS
        # -----------------------------
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

        self.permission_sub = self.create_subscription(
            Bool,
            self.permission_topic,
            self.permission_callback,
            10,
            callback_group=self.cb_group
        )

        # -----------------------------
        # STATE
        # -----------------------------
        self.latest_odom = None
        self.latest_permission_msg = None

        # With allow_when_value = False:
        # msg.data = False -> navigation_allowed = True
        # msg.data = True  -> navigation_allowed = False
        self.navigation_allowed = False

        self.sequence_active = False
        self.delay_timer = None

        self.current_goal_handle = None
        self.current_result_future = None
        self.cancel_in_progress = False

        # Saved goal so it can be resent after a pause
        self.saved_goal_x = None
        self.saved_goal_y = None
        self.saved_goal_q = None

        self.waiting_for_permission_to_start = False
        self.paused_with_saved_goal = False

        self.get_logger().info('FrontInterruptNode started.')
        self.get_logger().info(f'Interrupt topic: {self.permission_topic}')
        self.get_logger().info(
            f'Navigation is allowed when topic value == {self.allow_when_value}'
        )
        self.get_logger().info(
            'Current setup: /stop_navigation=True pauses, /stop_navigation=False allows movement.'
        )
        self.get_logger().info('Call /start_front_back to arm the demo.')

    # -------------------------------------------------
    # CALLBACKS
    # -------------------------------------------------
    def odom_callback(self, msg):
        self.latest_odom = msg

    def permission_callback(self, msg: Bool):
        old_allowed = self.navigation_allowed
        self.latest_permission_msg = msg
        self.navigation_allowed = (msg.data == self.allow_when_value)

        self.get_logger().info(
            f'Interrupt update received: raw={msg.data}, '
            f'navigation_allowed={self.navigation_allowed}'
        )

        # Transition: allowed -> blocked
        if old_allowed and not self.navigation_allowed:
            self.handle_pause_request()

        # Transition: blocked -> allowed
        elif (not old_allowed) and self.navigation_allowed:
            self.handle_resume_request()

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
        self.waiting_for_permission_to_start = False
        self.paused_with_saved_goal = False

        self.get_logger().info(
            f'Trigger received. Waiting {self.delay_seconds} seconds...'
        )

        self.delay_timer = self.create_timer(
            self.delay_seconds,
            self.delay_done_callback,
            callback_group=self.cb_group
        )

        response.success = True
        response.message = (
            f'Sequence armed. Navigation will start in {self.delay_seconds} seconds.'
        )
        return response

    def delay_done_callback(self):
        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.destroy_timer(self.delay_timer)
            self.delay_timer = None

        if self.latest_odom is None:
            self.get_logger().error('No /odom available.')
            self.reset_sequence_state()
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

            self.saved_goal_x = goal_x
            self.saved_goal_y = goal_y
            self.saved_goal_q = start_q

            self.get_logger().info(
                f'Start pose: x={start_x:.3f}, y={start_y:.3f}, yaw={start_yaw:.3f}'
            )
            self.get_logger().info(
                f'Planned forward goal: x={goal_x:.3f}, y={goal_y:.3f}'
            )

            if self.navigation_allowed:
                self.get_logger().info(
                    'Navigation is currently allowed. Sending Nav2 goal now.'
                )
                self.send_saved_goal()
            else:
                self.get_logger().warn(
                    'Navigation is currently blocked. Goal is saved and will start when allowed again.'
                )
                self.waiting_for_permission_to_start = True

        except Exception as e:
            self.get_logger().error(f'Error preparing goal: {e}')
            self.reset_sequence_state()

    # -------------------------------------------------
    # PAUSE / RESUME LOGIC
    # -------------------------------------------------
    def handle_pause_request(self):
        if not self.sequence_active:
            return

        if self.current_goal_handle is not None and not self.cancel_in_progress:
            self.get_logger().warn(
                'Navigation became blocked while navigating. Cancelling current Nav2 goal...'
            )
            self.cancel_in_progress = True
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        elif self.waiting_for_permission_to_start:
            self.get_logger().warn(
                'Navigation is blocked while waiting to start. Still holding goal.'
            )
        else:
            self.get_logger().warn(
                'Navigation is blocked. No active Nav2 goal to cancel.'
            )

    def handle_resume_request(self):
        if not self.sequence_active:
            return

        if self.waiting_for_permission_to_start:
            self.get_logger().info(
                'Navigation became allowed. Starting the saved Nav2 goal.'
            )
            self.waiting_for_permission_to_start = False
            self.send_saved_goal()
            return

        if self.paused_with_saved_goal and self.current_goal_handle is None:
            self.get_logger().info(
                'Navigation became allowed. Resending saved goal to continue navigation.'
            )
            self.paused_with_saved_goal = False
            self.send_saved_goal()

    # -------------------------------------------------
    # NAV2 GOAL LOGIC
    # -------------------------------------------------
    def send_saved_goal(self):
        if self.saved_goal_x is None or self.saved_goal_y is None or self.saved_goal_q is None:
            self.get_logger().error('No saved goal exists to send.')
            self.reset_sequence_state()
            return

        if not self.navigation_allowed:
            self.get_logger().warn(
                'Tried to send goal while navigation is blocked. Holding.'
            )
            self.waiting_for_permission_to_start = True
            return

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available.')
            self.reset_sequence_state()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.saved_goal_x)
        goal_msg.pose.pose.position.y = float(self.saved_goal_y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = self.saved_goal_q

        self.get_logger().info(
            f'Sending goal: x={self.saved_goal_x:.3f}, y={self.saved_goal_y:.3f}'
        )

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Exception while sending goal: {e}')
            self.reset_sequence_state()
            return

        if goal_handle is None:
            self.get_logger().error('Failed to get goal handle.')
            self.reset_sequence_state()
            return

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2.')
            self.reset_sequence_state()
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
            self.reset_sequence_state()
            return

        self.current_goal_handle = None
        self.current_result_future = None

        if result is None:
            self.get_logger().error('Failed to get result from Nav2.')
            self.reset_sequence_state()
            return

        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded. Demo complete.')
            self.reset_sequence_state(clear_saved_goal=True)
            return

        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Goal was canceled due to interrupt condition.')
            self.paused_with_saved_goal = True
            return

        self.get_logger().error(f'Goal failed with status: {status}')
        self.reset_sequence_state()

    def cancel_done_callback(self, future):
        self.cancel_in_progress = False

        try:
            cancel_response = future.result()
        except Exception as e:
            self.get_logger().error(f'Cancel request failed: {e}')
            return

        if cancel_response is None:
            self.get_logger().error('Cancel response was None.')
            return

        self.get_logger().info('Cancel request sent successfully.')

    # -------------------------------------------------
    # HELPERS
    # -------------------------------------------------
    def reset_sequence_state(self, clear_saved_goal=False):
        self.sequence_active = False
        self.waiting_for_permission_to_start = False
        self.paused_with_saved_goal = False
        self.cancel_in_progress = False
        self.current_goal_handle = None
        self.current_result_future = None

        if self.delay_timer is not None:
            self.delay_timer.cancel()
            self.destroy_timer(self.delay_timer)
            self.delay_timer = None

        if clear_saved_goal:
            self.saved_goal_x = None
            self.saved_goal_y = None
            self.saved_goal_q = None

    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = FrontInterruptNode()
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