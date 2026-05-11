import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import rclpy.parameter
import yaml
import os
import math
from collections import deque
from enum import Enum, auto


class State(Enum):
    IDLE = auto()
    LAWNMOWER = auto()
    INTERRUPT = auto()
    RESUMING = auto()


class WaypointNavigationNode(Node):
    def __init__(self):
        super().__init__('lawnmower_litter_nav')

        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True
        )])

        self.callback_group = ReentrantCallbackGroup()

        self.waypoint_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints',
            callback_group=self.callback_group
        )
        self.navigate_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )

        self.start_service = self.create_service(
            Trigger, 'start_navigation',
            self.start_navigation_callback,
            callback_group=self.callback_group
        )

        self.current_pose = None
        self.ref_pose = None
        self.ref_lat = 49.9
        self.ref_lon = 8.9

        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            self.odom_callback, 10,
            callback_group=self.callback_group
        )

        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.stop_sub = self.create_subscription(
            Bool, '/stop_navigation',
            self.stop_navigation_callback,
            latching_qos,
            callback_group=self.callback_group
        )

        self.litter_coordinate_sub = self.create_subscription(
            PoseStamped, '/vision/detected_litter',
            self.litter_collection_callback, latching_qos,
            callback_group=self.callback_group
        )

        self.gps_print_timer = self.create_timer(
            1.0, self.print_gps_callback,
            callback_group=self.callback_group
        )

        self.state = State.IDLE
        self.goal_handle = None
        self.litter_goal_handle = None
        self.current_waypoint_index = 0
        self.saved_waypoint_index = 0
        self.waypoints = self.load_waypoints()

        self._interrupt_future: Future | None = None
        self._litter_interrupt_future: Future | None = None
        self._litter_queue: deque[PoseStamped] = deque()

        self.get_logger().info(
            f'Waypoint Navigation Node ready. Loaded {len(self.waypoints)} waypoints.'
        )

    # ------------------------------------------------------------------ #
    # Odometry / GPS                                                       #
    # ------------------------------------------------------------------ #

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.ref_pose is None:
            self.ref_pose = self.current_pose
            self.get_logger().info(
                f'Odometry anchored. Origin: Lat {self.ref_lat}, Lon {self.ref_lon}'
            )

    def xy_offsets_to_lat_lon(self, dx, dy):
        R = 6378137.0
        lat_ref_rad = math.radians(self.ref_lat)
        current_lat = self.ref_lat + math.degrees(dy / R)
        current_lon = self.ref_lon + math.degrees(dx / (R * math.cos(lat_ref_rad)))
        return current_lat, current_lon

    def print_gps_callback(self):
        if self.current_pose and self.ref_pose:
            dx = self.current_pose.position.x - self.ref_pose.position.x
            dy = self.current_pose.position.y - self.ref_pose.position.y
            lat, lon = self.xy_offsets_to_lat_lon(dx, dy)
            self.get_logger().info(
                f'[GPS] Lat: {lat:.6f}, Lon: {lon:.6f} | '
                f'X: {self.current_pose.position.x:.2f}, Y: {self.current_pose.position.y:.2f}'
            )

    # ------------------------------------------------------------------ #
    # Waypoint loading / building                                          #
    # ------------------------------------------------------------------ #

    def load_waypoints(self):
        try:
            waypoints_file = os.path.expanduser(
                '~/scout_ws_2mini/src/waypoint_navigation_pkg/waypoint_navigation_pkg/waypoints.yaml'
            )
            with open(waypoints_file, 'r') as f:
                data = yaml.safe_load(f)
                return data['waypoints']
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return []

    def build_pose_stamped(self, waypoint):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = Time().to_msg()
        pose.pose.position.x = float(waypoint['pose']['position']['x'])
        pose.pose.position.y = float(waypoint['pose']['position']['y'])
        pose.pose.position.z = float(waypoint['pose']['position']['z'])
        pose.pose.orientation.x = float(waypoint['pose']['orientation']['x'])
        pose.pose.orientation.y = float(waypoint['pose']['orientation']['y'])
        pose.pose.orientation.z = float(waypoint['pose']['orientation']['z'])
        pose.pose.orientation.w = float(waypoint['pose']['orientation']['w'])
        return pose

    # ------------------------------------------------------------------ #
    # Core navigation loop                                                 #
    # ------------------------------------------------------------------ #

    async def _run_navigation(self):
        self.waypoints = self.load_waypoints()
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded.')
            self.state = State.IDLE
            return

        # Only reset if this is a fresh run
        if self.state == State.IDLE and self.current_waypoint_index == 0:
            self.current_waypoint_index = 0

        while self.current_waypoint_index < len(self.waypoints):

            remaining = self.waypoints[self.current_waypoint_index:]
            label = 'RESUMING' if self.state == State.RESUMING else 'LAWNMOWER'
            self.state = State.LAWNMOWER if label == 'LAWNMOWER' else State.RESUMING

            self.get_logger().info(
                f'[{label}] Sending {len(remaining)} waypoints '
                f'(starting from index {self.current_waypoint_index}).'
            )

            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = [self.build_pose_stamped(wp) for wp in remaining]

            while not self.waypoint_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info(f'[{label}] Waiting for follow_waypoints server...')

            self._interrupt_future = Future()

            self.goal_handle = await self.waypoint_client.send_goal_async(
                goal_msg,
                feedback_callback=self.waypoint_feedback_callback
            )

            if not self.goal_handle.accepted:
                self.get_logger().error(f'[{label}] Goal rejected.')
                break

            result_future = self.goal_handle.get_result_async()

            def _on_result_done(f):
                if not self._interrupt_future.done():
                    self._interrupt_future.set_result('nav_done')

            result_future.add_done_callback(_on_result_done)

            trigger = await self._interrupt_future

            if trigger == 'estop':
                self.get_logger().info('Navigation halted by stop command.')
                await self.goal_handle.cancel_goal_async()
                self.goal_handle = None
                self.state = State.IDLE
                self._interrupt_future = None
                return

            if trigger == 'interrupted':
                self.get_logger().info(
                    f'[{label}] Interrupt at waypoint {self.current_waypoint_index}. '
                    'Cancelling goal...'
                )
                self.saved_waypoint_index = self.current_waypoint_index
                self.state = State.INTERRUPT

                await self.goal_handle.cancel_goal_async()
                await result_future
                self.goal_handle = None

                # Drain the entire litter queue before resuming
                completed = await self._navigate_to_litter()

                if not completed:
                    # Estopped during litter navigation — shut down cleanly
                    self.goal_handle = None
                    self.state = State.IDLE
                    self._interrupt_future = None
                    self.get_logger().info('Navigation halted during litter collection.')
                    return

                self.current_waypoint_index = self.saved_waypoint_index
                self.state = State.RESUMING
                continue

            # Nav completed normally
            result = result_future.result()
            missed = list(result.result.missed_waypoints)
            self.goal_handle = None

            if missed:
                self.get_logger().warn(f'[{label}] Missed waypoints: {missed}')
            else:
                self.get_logger().info(f'[{label}] All waypoints reached.')

            break

        self.state = State.IDLE
        self._interrupt_future = None
        self.get_logger().info('Navigation loop complete.')

    async def _navigate_to_litter(self) -> bool:
        """Navigate to all litter in the queue.
        Returns True if completed normally, False if estopped."""
        while self._litter_queue:
            detected_pose = self._litter_queue.popleft()

            self.get_logger().info(
                f'[INTERRUPT] Navigating to litter at '
                f'x={detected_pose.pose.position.x:.2f}, '
                f'y={detected_pose.pose.position.y:.2f} '
                f'({len(self._litter_queue)} remaining in queue).'
            )

            litter_goal = NavigateToPose.Goal()
            litter_goal.pose = detected_pose

            while not self.navigate_to_pose_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('[INTERRUPT] Waiting for navigate_to_pose server...')

            litter_handle = await self.navigate_to_pose_client.send_goal_async(litter_goal)
            self.litter_goal_handle = litter_handle

            if not litter_handle.accepted:
                self.get_logger().error('[INTERRUPT] Litter goal rejected — skipping.')
                self.litter_goal_handle = None
                continue

            result_future = litter_handle.get_result_async()

            estop_future = Future()

            def _on_done(f):
                if not estop_future.done():
                    estop_future.set_result('done')

            result_future.add_done_callback(_on_done)
            self._litter_interrupt_future = estop_future

            trigger = await estop_future
            self.litter_goal_handle = None
            self._litter_interrupt_future = None

            if trigger == 'estop':
                self.get_logger().info('[INTERRUPT] Estop received — aborting litter navigation.')
                return False

            self.get_logger().info(
                f'[INTERRUPT] Reached litter. {len(self._litter_queue)} remaining.'
            )

        return True

    # ------------------------------------------------------------------ #
    # Stop callback                                                        #
    # ------------------------------------------------------------------ #

    def stop_navigation_callback(self, msg: Bool):
        if not msg.data:
            return

        if self.state == State.IDLE:
            self.get_logger().warn('Stop received but no navigation is active.')
            return

        self.get_logger().info('Stop command received — cancelling navigation.')
        # removed to keep queue after stop
        # self._litter_queue.clear()

        # Cancel whichever goal is currently active
        if self.litter_goal_handle is not None:
            self.litter_goal_handle.cancel_goal_async()
            self.litter_goal_handle = None

        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

        # Signal the nav loop to stop
        if self._interrupt_future is not None and not self._interrupt_future.done():
            self._interrupt_future.set_result('estop')

        if self._litter_interrupt_future is not None and not self._litter_interrupt_future.done():
            self._litter_interrupt_future.set_result('estop')

    # ------------------------------------------------------------------ #
    # Service callback                                                     #
    # ------------------------------------------------------------------ #

    async def start_navigation_callback(self, request, response):
        if self.state != State.IDLE:
            response.success = False
            response.message = f'Navigation already active (state: {self.state.name})'
            return response

        response.success = True
        response.message = 'Navigation started.'

        await self._run_navigation()

        return response

    def waypoint_feedback_callback(self, feedback_msg):
        self.current_waypoint_index = feedback_msg.feedback.current_waypoint

    # ------------------------------------------------------------------ #
    # Litter callback                                                      #
    # ------------------------------------------------------------------ #

    def litter_collection_callback(self, detected_pose: PoseStamped):
        if self.state == State.IDLE:
            self.get_logger().info('Litter detected but navigation is idle — ignoring.')
            return

        self.get_logger().info(
            f'Litter detected at x={detected_pose.pose.position.x:.2f}, '
            f'y={detected_pose.pose.position.y:.2f} — added to queue '
            f'(queue size: {len(self._litter_queue) + 1}).'
        )
        self._litter_queue.append(detected_pose)

        # Only signal the nav loop if it isn't already handling an interrupt
        if (self._interrupt_future is not None
                and not self._interrupt_future.done()
                and self.state in (State.LAWNMOWER, State.RESUMING)):
            self._interrupt_future.set_result('interrupted')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigationNode()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
