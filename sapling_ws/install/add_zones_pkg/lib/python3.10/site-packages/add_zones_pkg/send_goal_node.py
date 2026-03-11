import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import JointConstraint, Constraints
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from moveit_msgs.msg import PositionIKRequest, RobotState
from geometry_msgs.msg import PoseStamped

class PiperPickPlace(Node):
    def __init__(self):
        super().__init__('piper_pick_place')
        # Action client for MoveGroup
        self._client = ActionClient(self, MoveGroup, 'move_action')
        # Service client for compute_ik
        self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self._ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik service...')

    def move_to_pose(self, target_pose: Pose):
        # Step 1: Compute IK for link6
        ik_request = PositionIKRequest()
        ik_request.group_name = "arm"
        ik_request.ik_link_name = "link6"
        ik_request.pose_stamped = PoseStamped()
        ik_request.pose_stamped.header.frame_id = "world"
        ik_request.pose_stamped.pose = target_pose
        ik_request.timeout.sec = 2

        future_ik = self._ik_client.call_async(GetPositionIK.Request(ik_request=ik_request))
        rclpy.spin_until_future_complete(self, future_ik)
        ik_response = future_ik.result()

        if ik_response.error_code.val != 1:
            self.get_logger().error("IK failed. Pose may be unreachable.")
            return None

        # Step 2: Build MoveGroup goal using joint constraints
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"
        goal_msg.request.allowed_planning_time = 15.0
        goal_msg.request.num_planning_attempts = 50
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        joint_constraints = []
        for name, pos in zip(ik_response.solution.joint_state.name,
                             ik_response.solution.joint_state.position):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)

        constraints = Constraints()
        constraints.joint_constraints = joint_constraints
        goal_msg.request.goal_constraints.append(constraints)

        # Step 3: Send goal to MoveGroup
        self._client.wait_for_server()
        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()

def main(args=None):
    rclpy.init(args=args)
    node = PiperPickPlace()

    # Example reachable pose
    pose = Pose()
    pose.position.x = 0.25
    pose.position.y = 0.0
    pose.position.z = 0.15
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0

    node.move_to_pose(pose)


if __name__ == "__main__":
    main()