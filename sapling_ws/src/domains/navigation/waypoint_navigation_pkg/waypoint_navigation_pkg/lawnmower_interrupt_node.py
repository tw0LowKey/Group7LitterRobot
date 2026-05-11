#!/usr/bin/env python3

import os
import yaml
from collections import deque
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from nav2_msgs.action import FollowWaypoints, NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped


class State(Enum):
    IDLE = auto()
    LAWNMOWER = auto()
    INTERRUPT = auto()
    RESUMING = auto()


class LawnmowerInterruptNode(Node):
    def __init__(self):
        super().__init__("lawnmower_interrupt_node")

        self.callback_group = ReentrantCallbackGroup()

        self.waypoint_client = ActionClient(
            self,
            FollowWaypoints,
            "/follow_waypoints",
            callback_group=self.callback_group
        )

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=self.callback_group
        )

        self.start_service = self.create_service(
            Trigger,
            "/start_navigation",
            self.start_navigation_callback,
            callback_group=self.callback_group
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_callback,
            10,
            callback_group=self.callback_group
        )

        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.stop_sub = self.create_subscription(
            Bool,
            "/stop_navigation",
            self.stop_navigation_callback,
            latching_qos,
            callback_group=self.callback_group
        )

        self.litter_sub = self.create_subscription(
            PoseStamped,
            "/vision/detected_litter",
            self.litter_callback,
            latching_qos,
            callback_group=self.callback_group
        )

        self.print_timer = self.create_timer(
            1.0,
            self.print_gps_xy,
            callback_group=self.callback_group
        )

        self.odom = None
        self.gps = None

        self.state = State.IDLE
        self.goal_handle = None
        self.litter_goal_handle = None

        self.current_waypoint_index = 0
        self.saved_waypoint_index = 0

        self._interrupt_future: Future | None = None
        self._litter_interrupt_future: Future | None = None
        self._litter_queue: deque[PoseStamped] = deque()

        self.waypoints = self.load_waypoints()

        self.get_logger().info(
            f"lawnmower_interrupt_node ready. Loaded {len(self.waypoints)} waypoints."
        )

    # ------------------------------------------------------------
    # Odom / GPS printing only
    # ------------------------------------------------------------

    def odom_callback(self, msg):
        self.odom = msg

    def gps_callback(self, msg):
        self.gps = msg

    def print_gps_xy(self):
        if self.odom is None or self.gps is None:
            return

        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        lat = self.gps.latitude
        lon = self.gps.longitude

        self.get_logger().info(
            f"GPS lat={lat:.8f}, lon={lon:.8f} | local x={x:.2f}, y={y:.2f}"
        )

    # ------------------------------------------------------------
    # Waypoints
    # ------------------------------------------------------------

    def load_waypoints(self):
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            waypoints_file = os.path.join(script_dir, "waypoints.yaml")

            with open(waypoints_file, "r") as f:
                data = yaml.safe_load(f)

            self.get_logger().info(f"Loaded waypoints from: {waypoints_file}")

            return data["waypoints"]

        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {e}")
            return []

    def build_pose_stamped(self, waypoint):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0

        pose.pose.position.x = float(waypoint["pose"]["position"]["x"])
        pose.pose.position.y = float(waypoint["pose"]["position"]["y"])
        pose.pose.position.z = float(waypoint["pose"]["position"]["z"])

        pose.pose.orientation.x = float(waypoint["pose"]["orientation"]["x"])
        pose.pose.orientation.y = float(waypoint["pose"]["orientation"]["y"])
        pose.pose.orientation.z = float(waypoint["pose"]["orientation"]["z"])
        pose.pose.orientation.w = float(waypoint["pose"]["orientation"]["w"])

        return pose

    # ------------------------------------------------------------
    # Main navigation
    # ------------------------------------------------------------

    async def _run_navigation(self):
        self.waypoints = self.load_waypoints()

        if not self.waypoints:
            self.state = State.IDLE
            return

        while self.current_waypoint_index < len(self.waypoints):

            remaining = self.waypoints[self.current_waypoint_index:]

            if self.state == State.RESUMING:
                self.state = State.RESUMING
            else:
                self.state = State.LAWNMOWER

            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = [self.build_pose_stamped(wp) for wp in remaining]

            while not self.waypoint_client.wait_for_server(timeout_sec=1.0):
                pass

            self._interrupt_future = Future()

            self.goal_handle = await self.waypoint_client.send_goal_async(
                goal_msg,
                feedback_callback=self.waypoint_feedback_callback
            )

            if not self.goal_handle.accepted:
                break

            result_future = self.goal_handle.get_result_async()

            def on_result_done(_):
                if not self._interrupt_future.done():
                    self._interrupt_future.set_result("nav_done")

            result_future.add_done_callback(on_result_done)

            trigger = await self._interrupt_future

            if trigger == "estop":
                await self.goal_handle.cancel_goal_async()
                self.goal_handle = None
                self.state = State.IDLE
                self._interrupt_future = None
                return

            if trigger == "interrupted":
                self.saved_waypoint_index = self.current_waypoint_index
                self.state = State.INTERRUPT

                await self.goal_handle.cancel_goal_async()
                await result_future
                self.goal_handle = None

                completed = await self._navigate_to_litter()

                if not completed:
                    self.state = State.IDLE
                    self._interrupt_future = None
                    return

                self.current_waypoint_index = self.saved_waypoint_index
                self.state = State.RESUMING
                continue

            break

        self.state = State.IDLE
        self.goal_handle = None
        self._interrupt_future = None

    async def _navigate_to_litter(self) -> bool:
        while self._litter_queue:
            detected_pose = self._litter_queue.popleft()

            while not self.navigate_client.wait_for_server(timeout_sec=1.0):
                pass

            litter_goal = NavigateToPose.Goal()
            litter_goal.pose = detected_pose

            self.litter_goal_handle = await self.navigate_client.send_goal_async(
                litter_goal
            )

            if not self.litter_goal_handle.accepted:
                self.litter_goal_handle = None
                continue

            result_future = self.litter_goal_handle.get_result_async()

            estop_future = Future()

            def on_litter_done(_):
                if not estop_future.done():
                    estop_future.set_result("done")

            result_future.add_done_callback(on_litter_done)

            self._litter_interrupt_future = estop_future
            trigger = await estop_future

            self.litter_goal_handle = None
            self._litter_interrupt_future = None

            if trigger == "estop":
                return False

        return True

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------

    async def start_navigation_callback(self, request, response):
        if self.state != State.IDLE:
            response.success = False
            response.message = "Navigation already active."
            return response

        response.success = True
        response.message = "Navigation started."

        await self._run_navigation()

        return response

    def waypoint_feedback_callback(self, feedback_msg):
        self.current_waypoint_index = feedback_msg.feedback.current_waypoint

    def stop_navigation_callback(self, msg):
        if not msg.data:
            return

        if self.litter_goal_handle is not None:
            self.litter_goal_handle.cancel_goal_async()
            self.litter_goal_handle = None

        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

        if self._interrupt_future is not None and not self._interrupt_future.done():
            self._interrupt_future.set_result("estop")

        if self._litter_interrupt_future is not None and not self._litter_interrupt_future.done():
            self._litter_interrupt_future.set_result("estop")

    def litter_callback(self, detected_pose):
        if self.state == State.IDLE:
            return

        self._litter_queue.append(detected_pose)

        if (
            self._interrupt_future is not None
            and not self._interrupt_future.done()
            and self.state in (State.LAWNMOWER, State.RESUMING)
        ):
            self._interrupt_future.set_result("interrupted")


def main(args=None):
    rclpy.init(args=args)

    node = LawnmowerInterruptNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()