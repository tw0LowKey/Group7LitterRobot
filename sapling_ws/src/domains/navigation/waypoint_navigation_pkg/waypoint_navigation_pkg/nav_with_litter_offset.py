import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import rclpy.parameter
import yaml
import asyncio
import os
import math
from collections import deque
from enum import Enum, auto

class State(Enum):
    IDLE = auto()
    NAVIGATING = auto()
    COLLECTING = auto()
    PAUSED = auto()

class WaypointNavigationNode(Node):
    def __init__(self):
        super().__init__('nav_with_litter_offset')

        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, False
        )])

        self.callback_group = ReentrantCallbackGroup()

        # Clients
        self.waypoint_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints',
            callback_group=self.callback_group
        )
        self.navigate_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group
        )

        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscriptions
        self.start_sub = self.create_subscription(
            Bool, '/start_navigation',
            self.start_navigation_callback,
            latching_qos,
            callback_group=self.callback_group
        )

        self.litter_coordinate_sub = self.create_subscription(
            PoseStamped, '/vision/detected_litter',
            self.litter_collection_callback, latching_qos,
            callback_group=self.callback_group
        )

        self.pause_request_sub = self.create_subscription(
            Bool, '/pause_request_with_confirm',
            self.pause_with_confirm_callback,
            latching_qos,
            callback_group=self.callback_group
        )

        # Added Odometry to get current position for math calculations
        self.odom_sub = self.create_subscription(
            Odometry, '/odom',
            self.odom_callback, 10,
            callback_group=self.callback_group
        )

        # Publishers
        self.queue_size_pub = self.create_publisher(
            Int32, '/queue_size', 10
        )
        self.call_bin_pub = self.create_publisher(
            Bool, '/call_bin', 10,
            callback_group=self.callback_group
        )
        self.pause_confirm_pub = self.create_publisher(
            Bool, '/pause_confirmed', latching_qos,
            )

        self.state = State.IDLE
        self.goal_handle = None
        self.litter_goal_handle = None
        self.current_pose = None # Tracked via odom
        self.current_waypoint_index = 0
        self.saved_waypoint_index = 0
        self.waypoints = self.load_waypoints()

        self._interrupt_future: Future | None = None
        self._litter_interrupt_future: Future | None = None
        self._current_litter_pose: PoseStamped | None = None
        self._litter_queue: deque[PoseStamped] = deque()
        self._paused_during_litter: bool = False

        self.get_logger().info(f'Node ready. Loaded {len(self.waypoints)} waypoints.')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

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

    # def get_adjusted_litter_pose(self, litter_pose: PoseStamped):
    #     """Calculates a pose 30cm away from the litter, facing it."""
    #     if self.current_pose is None:
    #         self.get_logger().warn("Current pose unknown, sending raw litter coordinates.")
    #         return litter_pose

    #     # Calculate vector from robot to litter
    #     dx = litter_pose.pose.position.x - self.current_pose.position.x
    #     dy = litter_pose.pose.position.y - self.current_pose.position.y
    #     distance = math.sqrt(dx**2 + dy**2)
        
    #     # Target angle (heading) to face the litter
    #     angle_to_litter = math.atan2(dy, dx)

    #     # If we are already closer than 30cm, stay put
    #     offset = 0.5
    #     if distance <= offset:
    #         self.get_logger().info("Robot is already within 30cm of litter.")
    #         adjusted_pose = litter_pose
    #     else:
    #         # Calculate new X and Y 30cm away from the litter along the approach line
    #         adjusted_pose = PoseStamped()
    #         adjusted_pose.header = litter_pose.header
    #         adjusted_pose.pose.position.x = litter_pose.pose.position.x - (offset * math.cos(angle_to_litter))
    #         adjusted_pose.pose.position.y = litter_pose.pose.position.y - (offset * math.sin(angle_to_litter))
    #         adjusted_pose.pose.position.z = litter_pose.pose.position.z

    #     # Set orientation to face the litter (yaw to quaternion)
    #     q = self.euler_to_quaternion(0, 0, angle_to_litter)
    #     adjusted_pose.pose.orientation = q
        
    #     return adjusted_pose

    def get_adjusted_litter_pose(self, litter_pose: PoseStamped):
        """Calculates a pose 30cm away from the litter, facing it."""
        adjusted_pose = PoseStamped()
        adjusted_pose.header.frame_id = litter_pose.header.frame_id   # 'map'
        adjusted_pose.header.stamp = Time().to_msg()                  # zero stamp

        if self.current_pose is None:
            self.get_logger().warn("Current pose unknown, sending raw litter coordinates.")
            adjusted_pose.pose = litter_pose.pose
            return adjusted_pose

        dx = litter_pose.pose.position.x - self.current_pose.position.x
        dy = litter_pose.pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_litter = math.atan2(dy, dx)

        offset = 0.3
        if distance <= offset:
            self.get_logger().info("Robot is already within 30cm of litter.")
            adjusted_pose.pose.position.x = litter_pose.pose.position.x
            adjusted_pose.pose.position.y = litter_pose.pose.position.y
            adjusted_pose.pose.position.z = litter_pose.pose.position.z
        else:
            adjusted_pose.pose.position.x = litter_pose.pose.position.x - (offset * math.cos(angle_to_litter))
            adjusted_pose.pose.position.y = litter_pose.pose.position.y - (offset * math.sin(angle_to_litter))
            adjusted_pose.pose.position.z = litter_pose.pose.position.z

        adjusted_pose.pose.orientation = self.euler_to_quaternion(0, 0, angle_to_litter)
        return adjusted_pose

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def build_pose_stamped(self, waypoint):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        # pose.header.stamp = self.get_clock().now().to_msg()
        # Timestamp shouldn't be necessary for FollowWaypoints, and can cause issues if too old, so leaving it out for now
        pose.header.stamp = Time().to_msg()
        pose.pose.position.x = float(waypoint['pose']['position']['x'])
        pose.pose.position.y = float(waypoint['pose']['position']['y'])
        pose.pose.position.z = float(waypoint['pose']['position']['z'])
        pose.pose.orientation.x = float(waypoint['pose']['orientation']['x'])
        pose.pose.orientation.y = float(waypoint['pose']['orientation']['y'])
        pose.pose.orientation.z = float(waypoint['pose']['orientation']['z'])
        pose.pose.orientation.w = float(waypoint['pose']['orientation']['w'])
        return pose

    async def _run_navigation(self):
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded.')
            self.state = State.IDLE
            return

        if self._paused_during_litter and self._litter_queue:
            self.get_logger().info(f'Resuming litter collection — {len(self._litter_queue)} item(s) in queue.')
            self.state = State.COLLECTING
            completed = await self._navigate_to_litter()
            if not completed:
                self.saved_waypoint_index = self.current_waypoint_index
                self._paused_during_litter = True
                self.state = State.PAUSED
                return
            self._paused_during_litter = False

        self.state = State.NAVIGATING

        while self.current_waypoint_index < len(self.waypoints):
            if self._litter_queue:
                self.state = State.COLLECTING
                completed = await self._navigate_to_litter()
                if not completed:
                    self._paused_during_litter = True
                    self.state = State.PAUSED
                    return
                self.state = State.NAVIGATING

            remaining = self.waypoints[self.current_waypoint_index:]
            self.get_logger().info(f'[{self.state.name}] Sending waypoints from index {self.current_waypoint_index}.')

            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = [self.build_pose_stamped(wp) for wp in remaining]

            while not self.waypoint_client.wait_for_server(timeout_sec=0.0):
                self.get_logger().info(f'[{self.state.name}] Waiting for server...')
                await asyncio.sleep(1.0)

            self._interrupt_future = Future()
            if self._litter_queue and not self._interrupt_future.done():
                self._interrupt_future.set_result('interrupted')

            self.goal_handle = await self.waypoint_client.send_goal_async(
                goal_msg, feedback_callback=self.waypoint_feedback_callback
            )

            if not self.goal_handle.accepted:
                self.get_logger().error(f'[{self.state.name}] Goal rejected.')
                break

            result_future = self.goal_handle.get_result_async()

            def _on_nav_done(f):
                if self._interrupt_future and not self._interrupt_future.done():
                    self._interrupt_future.set_result('nav_done')

            result_future.add_done_callback(_on_nav_done)
            trigger = await self._interrupt_future

            # Race condition guard, in case litter is missed:
            if trigger != 'paused' and self._litter_queue:
                trigger = 'interrupted'

            if trigger == 'paused':
                self.get_logger().info(f'[{self.state.name}] Paused. Saving progress.')
                self.saved_waypoint_index = self.current_waypoint_index
                if self.goal_handle:
                    await self.goal_handle.cancel_goal_async()
                self.goal_handle = None
                self.state = State.PAUSED
                return

            if trigger == 'interrupted':
                self.get_logger().info(f'[{self.state.name}] Interrupting for litter...')
                self.saved_waypoint_index = self.current_waypoint_index
                self.state = State.COLLECTING

                if self.goal_handle:
                    await self.goal_handle.cancel_goal_async()
                    await result_future
                self.goal_handle = None

                completed = await self._navigate_to_litter()
                if not completed:
                    self._paused_during_litter = True
                    self.state = State.PAUSED
                    return

                self.current_waypoint_index = self.saved_waypoint_index
                self.state = State.NAVIGATING
                continue

            result = result_future.result()
            self.goal_handle = None
            self.get_logger().info(f'[{self.state.name}] Batch complete.')
            break

        self.state = State.IDLE
        self.get_logger().info('Navigation loop complete.')

    async def _navigate_to_litter(self) -> bool:
        while self._litter_queue:
            detected_pose = self._litter_queue.popleft()
            self._current_litter_pose = detected_pose

            # ADJUSTED GOAL LOGIC: 30cm away facing target
            adjusted_goal_pose = self.get_adjusted_litter_pose(detected_pose)

            self.get_logger().info(
                f'[COLLECTING] Navigating to ARM REACH POSE (30cm away) at '
                f'x={adjusted_goal_pose.pose.position.x:.2f}, y={adjusted_goal_pose.pose.position.y:.2f}'
            )

            litter_goal = NavigateToPose.Goal()
            litter_goal.pose = adjusted_goal_pose

            while not self.navigate_to_pose_client.wait_for_server(timeout_sec=0.0):
                self.get_logger().info('[COLLECTING] Waiting for server...')
                await asyncio.sleep(1.0)

            litter_handle = await self.navigate_to_pose_client.send_goal_async(litter_goal)
            self.litter_goal_handle = litter_handle

            if not litter_handle.accepted:
                self.get_logger().error('[COLLECTING] Goal rejected.')
                continue

            result_future = litter_handle.get_result_async()
            self._litter_interrupt_future = Future()

            def _on_litter_done(f):
                if self._litter_interrupt_future and not self._litter_interrupt_future.done():
                    self._litter_interrupt_future.set_result('done')

            result_future.add_done_callback(_on_litter_done)
            trigger = await self._litter_interrupt_future
            self.litter_goal_handle = None
            
            if trigger == 'paused':
                self.get_logger().info('[COLLECTING] Pause received.')
                return False
            
            # Fix to potential issue of re-queueing of stale pose
            self._current_litter_pose = None

            self.get_logger().info(f'[COLLECTING] Reached arm distance. Triggering collection and PAUSING.')
            msg = Bool()
            msg.data = True
            self.call_bin_pub.publish(msg)

            self.state = State.PAUSED
            return False 

        return True
    
    async def pause_with_confirm_callback(self, msg: Bool):
        if msg.data:
            await self._do_pause(send_confirmation=True)

    async def start_navigation_callback(self, msg: Bool):
        if msg.data:
            if self.state == State.PAUSED:
                self.get_logger().info(f'Resuming at index {self.saved_waypoint_index}.')
                self.state = State.NAVIGATING
                await self._run_navigation()
            elif self.state == State.IDLE:
                self.get_logger().info('Starting new run.')
                self.current_waypoint_index = 0
                self.saved_waypoint_index = 0
                self._paused_during_litter = False
                self.state = State.NAVIGATING
                await self._run_navigation()
        else:
            await self._do_pause()

    async def _do_pause(self, send_confirmation: bool = False):
        if self.state in (State.IDLE, State.PAUSED):
            return
        self.get_logger().info('Pause command received.')

        # 1. Handle litter navigation cancellation
        if self.litter_goal_handle and self._current_litter_pose is not None:
            # Re-queue the litter so it's not lost
            self._litter_queue.appendleft(self._current_litter_pose)
            self._current_litter_pose = None # prevent double re-queue
            await self.litter_goal_handle.cancel_goal_async()
            # clear the goal handle (this might not be needed or might be wrong)
            self.litter_goal_handle = None
        elif self.litter_goal_handle:
            # Handle exists but pose already consumed - just cancel
            await self.litter_goal_handle.cancel_goal_async()
            self.litter_goal_handle = None
            

        # 2. Handle waypoint navigation cancellation
        if self.goal_handle:
            # Await the cancellation of the lawnmower algorithm
            await self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        
        # 3. Trigger the internal futures to break the async loops
        if self._interrupt_future and not self._interrupt_future.done():
            self._interrupt_future.set_result('paused')
        if self._litter_interrupt_future and not self._litter_interrupt_future.done():
            self._litter_interrupt_future.set_result('paused')

        # 4. Now send confirmation of the pause to the vision system
        # Only if the vision system sent the pause
        if send_confirmation:
            confirm_msg = Bool()
            confirm_msg.data = True
            self.pause_confirm_pub.publish(confirm_msg)

        self.state = State.PAUSED
        self.get_logger().info('Navigation loop halted and pause confirmed.')

    def waypoint_feedback_callback(self, feedback_msg):
        # Indexing logic remains as previously requested
        self.current_waypoint_index = (
            self.saved_waypoint_index + feedback_msg.feedback.current_waypoint
        )

    def litter_collection_callback(self, detected_pose: PoseStamped):
        if self.state == State.IDLE:
            return

        self.get_logger().info(f'Litter detected. Queue size: {len(self._litter_queue) + 1}')
        self._litter_queue.append(detected_pose)

        # Publish the litter queue size to the monitor node
        size_msg = Int32()
        size_msg.data = len(self._litter_queue)
        self.queue_size_pub.publish(size_msg)

        # Auto-resume if paused
        # This should be changed to only resume on a litter detection pause
        if self.state == State.PAUSED:
            self.get_logger().info('Litter pose received - resuming navigation.')
            self.state = State.NAVIGATING
            self.executor.create_task(self._run_navigation())
            return # added to stop the code from going through the interrupt logic 

        if (self._interrupt_future and not self._interrupt_future.done()
            and self.state == State.NAVIGATING):
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
