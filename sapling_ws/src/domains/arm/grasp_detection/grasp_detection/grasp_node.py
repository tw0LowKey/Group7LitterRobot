import os
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray
from piper_msgs.srv import PickPlaceRequest
from ament_index_python.packages import get_package_share_directory

# ── Gripper parameters ───────────────────────────────
GRIPPER_WIDTH_MIN = 0.001
GRIPPER_WIDTH_MAX = 0.07
GRIPPER_FINGER_LENGTH = 0.074
GRIPPER_FINGER_THICK = 0.04
GRIPPER_PALM_THICK = 0.024
GRIPPER_WIDTH_Y = 0.02
GRIPPER_BODY_DEPTH = 0.145
GRIPPER_TARGET_PENETRATION = GRIPPER_FINGER_LENGTH * (1/10)

GRIPPER_SHAFT_DEPTH = GRIPPER_BODY_DEPTH - GRIPPER_PALM_THICK
GRIPPER_PALM_WIDTH_Y = 0.09
GRIPPER_SHAFT_RADIUS = 0.04
GRASP_CONTACT_DEPTH = 0.01

GRIPPER_STANDOFF = GRIPPER_FINGER_LENGTH - GRIPPER_TARGET_PENETRATION

GROUND_PLANE_OFFSET = 0.01
N_ANCHORS = 500
NORMAL_K = 50
REACHABILITY_RADIUS = 0.6

_CAMERA_TRANSLATION = np.array([0.14, 0.13, 0.600])
_CAMERA_ROTATION    = np.array([0.0, 0.479, 0.0, 0.878])  # [x, y, z, w]

W_BODY        = 0.33
W_SYM         = 0.33
W_FRICTION    = 0.34

TOP_K = 1
PROCESS_EVERY_N = 3


def _local_to_world_pos(local_offset: np.ndarray, grasp_pos: np.ndarray, grasp_rot: np.ndarray) -> np.ndarray:
    return grasp_pos + grasp_rot @ local_offset


def _make_cube_marker(header, ns, marker_id, world_pos, quat, scale_xyz, rgba):
    from visualization_msgs.msg import Marker
    m = Marker()
    m.header = header
    m.ns = ns
    m.id = marker_id
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = float(world_pos[0])
    m.pose.position.y = float(world_pos[1])
    m.pose.position.z = float(world_pos[2])
    m.pose.orientation.x = float(quat[0])
    m.pose.orientation.y = float(quat[1])
    m.pose.orientation.z = float(quat[2])
    m.pose.orientation.w = float(quat[3])
    m.scale.x = float(scale_xyz[0])
    m.scale.y = float(scale_xyz[1])
    m.scale.z = float(scale_xyz[2])
    m.color.r = float(rgba[0])
    m.color.g = float(rgba[1])
    m.color.b = float(rgba[2])
    m.color.a = float(rgba[3])
    return m


class GraspNode(Node):
    def __init__(self):
        super().__init__('grasp_node')

        self._msg_count = 0

        _pitch_rot = R.from_euler('x', -57.0, degrees=True).as_matrix()
        self._base_pos_in_camera = _pitch_rot @ _CAMERA_TRANSLATION
        self.get_logger().info(
            f"Base link in camera frame: "
            f"[{self._base_pos_in_camera[0]:.3f}, "
            f"{self._base_pos_in_camera[1]:.3f}, "
            f"{self._base_pos_in_camera[2]:.3f}]"
        )

        self._ground_plane = None
        self._up = np.array([0.0, 0.0, 1.0])
        plane_path = os.path.join(get_package_share_directory('computer_vision'), 'ground_plane.npy')
        if os.path.exists(plane_path):
            self._ground_plane = np.load(plane_path)
            self._ground_plane[3] += GROUND_PLANE_OFFSET
            a, b, c, _ = self._ground_plane
            self._up = np.array([a, b, c], dtype=float)
            self._up /= np.linalg.norm(self._up)
            self.get_logger().info(
                f"Loaded ground plane (offset {GROUND_PLANE_OFFSET*100:.1f}cm into ground): "
                f"[a={self._ground_plane[0]:.4f}, b={self._ground_plane[1]:.4f}, "
                f"c={self._ground_plane[2]:.4f}, d={self._ground_plane[3]:.4f}]"
            )
        else:
            self.get_logger().warn(
                "No ground plane file found — ground collision disabled, "
                "using world Z as up vector."
            )

        self.pc_sub = self.create_subscription(
            PointCloud2, '/selected_mask', self.pc_callback, 1
        )
        self.grasp_pub  = self.create_publisher(PoseStamped, '/grasp_pose', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/grasp_markers', 10)

        self._startup_count = 0
        self._startup_timer = self.create_timer(0.5, self._startup_publish)

        self.pick_client   = self.create_client(PickPlaceRequest, 'pick_place_request')
        self.robot_is_busy = False

        self.get_logger().info("GraspNode started")

    def _startup_publish(self):
        self._startup_count += 1
        if self._startup_count > 10:
            self._startup_timer.cancel()
            return

    def _feasibility_reason(self, points, grasp_pos, grasp_rot):
        pts_local = (points - grasp_pos) @ grasp_rot
        x, y, z = pts_local[:, 0], pts_local[:, 1], pts_local[:, 2]
        half_w = GRIPPER_WIDTH_MAX / 2.0
        outer_x = half_w + GRIPPER_FINGER_THICK

        in_env = (z >= 0.0) & (z <= GRIPPER_FINGER_LENGTH)
        if pts_local[in_env].shape[0] == 0:
            return 'no_points_in_envelope'

        left_finger  = (x >= half_w)  & (x <= outer_x)
        right_finger = (x <= -half_w) & (x >= -outer_x)
        in_finger_y  = np.abs(y) <= GRIPPER_WIDTH_Y / 2.0
        in_finger_z  = (z >= 0.0) & (z <= GRIPPER_FINGER_LENGTH)
        if np.any((left_finger | right_finger) & in_finger_y & in_finger_z):
            return 'finger_collision'

        in_palm_x = np.abs(x) <= outer_x
        in_palm_y = np.abs(y) <= GRIPPER_PALM_WIDTH_Y / 2.0
        in_palm_z = (z < 0.0) & (z >= -GRIPPER_PALM_THICK)
        if np.any(in_palm_x & in_palm_y & in_palm_z):
            return 'palm_collision'

        in_shaft_z = (z < -GRIPPER_PALM_THICK) & (z >= -GRIPPER_BODY_DEPTH)
        radial_dist = np.sqrt(x**2 + y**2)
        in_shaft_radial = radial_dist <= GRIPPER_SHAFT_RADIUS
        if np.any(in_shaft_z & in_shaft_radial):
            return 'shaft_collision'

        if self._ground_plane is not None:
            a, b, c, d = self._ground_plane
            corners_local = np.array([
                [ outer_x,  GRIPPER_WIDTH_Y/2,       GRIPPER_FINGER_LENGTH],
                [ outer_x, -GRIPPER_WIDTH_Y/2,       GRIPPER_FINGER_LENGTH],
                [-outer_x,  GRIPPER_WIDTH_Y/2,       GRIPPER_FINGER_LENGTH],
                [-outer_x, -GRIPPER_WIDTH_Y/2,       GRIPPER_FINGER_LENGTH],
                [ outer_x,  GRIPPER_PALM_WIDTH_Y/2,  0.0],
                [ outer_x, -GRIPPER_PALM_WIDTH_Y/2,  0.0],
                [-outer_x,  GRIPPER_PALM_WIDTH_Y/2,  0.0],
                [-outer_x, -GRIPPER_PALM_WIDTH_Y/2,  0.0],
                [ outer_x,  GRIPPER_PALM_WIDTH_Y/2, -GRIPPER_PALM_THICK],
                [ outer_x, -GRIPPER_PALM_WIDTH_Y/2, -GRIPPER_PALM_THICK],
                [-outer_x,  GRIPPER_PALM_WIDTH_Y/2, -GRIPPER_PALM_THICK],
                [-outer_x, -GRIPPER_PALM_WIDTH_Y/2, -GRIPPER_PALM_THICK],
                [ outer_x,  GRIPPER_PALM_WIDTH_Y/2, -GRIPPER_BODY_DEPTH],
                [ outer_x, -GRIPPER_PALM_WIDTH_Y/2, -GRIPPER_BODY_DEPTH],
                [-outer_x,  GRIPPER_PALM_WIDTH_Y/2, -GRIPPER_BODY_DEPTH],
                [-outer_x, -GRIPPER_PALM_WIDTH_Y/2, -GRIPPER_BODY_DEPTH],
            ])
            corners_world = grasp_pos + corners_local @ grasp_rot.T
            signed_dists = corners_world @ np.array([a, b, c]) + d
            if np.any(signed_dists < 0.0):
                return 'ground_collision'

        return None

    def score_grasp(self, points, grasp_pos, grasp_rot):
        pts_local = (points - grasp_pos) @ grasp_rot
        x, y, z = pts_local[:, 0], pts_local[:, 1], pts_local[:, 2]
        half_w = GRIPPER_WIDTH_MAX / 2.0

        in_env = (z >= 0.0) & (z <= GRIPPER_FINGER_LENGTH)
        in_volume = in_env & (x >= -half_w) & (x <= half_w)
        enclosed = pts_local[in_volume]
        n_enclosed = enclosed.shape[0]

        in_contact_y = np.abs(enclosed[:, 1]) <= GRIPPER_WIDTH_Y / 2.0
        in_contact_z = (enclosed[:, 2] >= 0.0) & (enclosed[:, 2] <= GRIPPER_FINGER_LENGTH)
        in_contact_yz = in_contact_y & in_contact_z

        left_contact = (enclosed[:, 0] >= half_w - GRASP_CONTACT_DEPTH) & (enclosed[:, 0] <= half_w)
        right_contact = (enclosed[:, 0] <= -half_w + GRASP_CONTACT_DEPTH) & (enclosed[:, 0] >= -half_w)

        n_left = float(np.sum(left_contact & in_contact_yz))
        n_right = float(np.sum(right_contact & in_contact_yz))
        n_contact = n_left + n_right

        body_score = n_contact / (n_enclosed + 1e-6)
        sym_score = 1.0 - abs(n_left - n_right) / (n_contact + 1e-6)
        approach_axis = grasp_rot[:, 2]
        friction_score = max(0.0, float(-np.dot(approach_axis, self._up)))

        return (W_BODY * body_score + W_SYM * sym_score + W_FRICTION * friction_score)

    def _estimate_normal(self, anchor: np.ndarray, points: np.ndarray) -> np.ndarray:
        dists = np.linalg.norm(points - anchor, axis=1)
        k = min(NORMAL_K, points.shape[0])
        neighbours = points[np.argsort(dists)[:k]]

        centred = neighbours - neighbours.mean(axis=0)
        _, _, Vt = np.linalg.svd(centred, full_matrices=False)
        normal = Vt[-1]

        if np.dot(normal, self._up) < 0:
            normal = -normal

        return normal / np.linalg.norm(normal)

    def generate_candidates(self, points, n_anchors=N_ANCHORS, rolls_per_point=10):
        candidates = []
        if points.shape[0] == 0:
            return candidates

        n = min(n_anchors, points.shape[0])
        indices = np.random.choice(points.shape[0], size=n, replace=False)
        anchors = points[indices]

        for anchor in anchors:
            z_axis = self._estimate_normal(anchor, points)
            z_axis = -z_axis

            grasp_pos = anchor - z_axis * GRIPPER_STANDOFF

            tmp = np.array([1.0, 0.0, 0.0])
            if abs(np.dot(tmp, z_axis)) > 0.9:
                tmp = np.array([0.0, 1.0, 0.0])
            x_ref = np.cross(tmp, z_axis)
            x_ref /= np.linalg.norm(x_ref)

            for k in range(rolls_per_point):
                roll = (k / rolls_per_point) * np.pi
                x_axis = (
                    x_ref * np.cos(roll)
                    + np.cross(z_axis, x_ref) * np.sin(roll)
                    + z_axis * np.dot(z_axis, x_ref) * (1 - np.cos(roll))
                )
                x_axis /= np.linalg.norm(x_axis)
                y_axis = np.cross(z_axis, x_axis)
                y_axis /= np.linalg.norm(y_axis)

                grasp_rot = np.stack([x_axis, y_axis, z_axis], axis=1)
                candidates.append((grasp_pos, grasp_rot))

        return candidates

    def _build_gripper_markers(self, header, idx, pos, rot, quat):
        from visualization_msgs.msg import Marker
        markers = []
        half_w = GRIPPER_WIDTH_MAX / 2.0
        half_ft = GRIPPER_FINGER_THICK / 2.0

        palm_local = np.array([0.0, 0.0, -GRIPPER_PALM_THICK / 2.0])
        palm_world = _local_to_world_pos(palm_local, pos, rot)
        markers.append(_make_cube_marker(
            header, "gripper_palm", idx * 10 + 0,
            palm_world, quat,
            scale_xyz=[GRIPPER_WIDTH_MAX + 2 * GRIPPER_FINGER_THICK, GRIPPER_PALM_WIDTH_Y, GRIPPER_PALM_THICK],
            rgba=[0.0, 1.0, 0.0, 0.5],
        ))

        lf_local = np.array([half_w + half_ft, 0.0, GRIPPER_FINGER_LENGTH / 2.0])
        lf_world = _local_to_world_pos(lf_local, pos, rot)
        markers.append(_make_cube_marker(
            header, "gripper_left_finger", idx * 10 + 1,
            lf_world, quat,
            scale_xyz=[GRIPPER_FINGER_THICK, GRIPPER_WIDTH_Y, GRIPPER_FINGER_LENGTH],
            rgba=[0.0, 0.5, 0.0, 0.5],
        ))

        rf_local = np.array([-(half_w + half_ft), 0.0, GRIPPER_FINGER_LENGTH / 2.0])
        rf_world = _local_to_world_pos(rf_local, pos, rot)
        markers.append(_make_cube_marker(
            header, "gripper_right_finger", idx * 10 + 2,
            rf_world, quat,
            scale_xyz=[GRIPPER_FINGER_THICK, GRIPPER_WIDTH_Y, GRIPPER_FINGER_LENGTH],
            rgba=[0.0, 0.5, 0.0, 0.5],
        ))

        shaft_local = np.array([0.0, 0.0, -GRIPPER_PALM_THICK - GRIPPER_SHAFT_DEPTH / 2.0])
        shaft_world = _local_to_world_pos(shaft_local, pos, rot)
        shaft_m = Marker()
        shaft_m.header = header
        shaft_m.ns = "gripper_body"
        shaft_m.id = idx * 10 + 3
        shaft_m.type = Marker.CYLINDER
        shaft_m.action = Marker.ADD
        shaft_m.pose.position.x = float(shaft_world[0])
        shaft_m.pose.position.y = float(shaft_world[1])
        shaft_m.pose.position.z = float(shaft_world[2])
        shaft_m.pose.orientation.x = float(quat[0])
        shaft_m.pose.orientation.y = float(quat[1])
        shaft_m.pose.orientation.z = float(quat[2])
        shaft_m.pose.orientation.w = float(quat[3])
        shaft_m.scale.x = GRIPPER_SHAFT_RADIUS * 2
        shaft_m.scale.y = GRIPPER_SHAFT_RADIUS * 2
        shaft_m.scale.z = float(GRIPPER_SHAFT_DEPTH)
        shaft_m.color.r = 0.5
        shaft_m.color.g = 0.0
        shaft_m.color.b = 0.0
        shaft_m.color.a = 0.3
        markers.append(shaft_m)

        return markers

    def pc_callback(self, pc_msg):
        callback_start = time.time()
        self._msg_count += 1
        if self._msg_count % PROCESS_EVERY_N != 0:
            return

        if self.robot_is_busy:
            self.get_logger().info(
                "Arm is currently moving. Skipping vision request...",
                throttle_duration_sec=2.0
            )
            return

        points_list = list(pc2.read_points(pc_msg, field_names=("x", "y", "z", "instance_id"), skip_nans=True))
        if not points_list:
            return

        all_points = np.array([[p[0], p[1], p[2], p[3]] for p in points_list], dtype=np.float32)
        instance_ids = all_points[:, 3]
        unique_ids = np.unique(instance_ids)
        if unique_ids.size == 0:
            return

        points = all_points[:, :3]

        gen_start = time.time()
        candidates = self.generate_candidates(points)
        gen_time = time.time() - gen_start

        reject_counts = {
            'no_points_in_envelope': 0,
            'finger_collision': 0,
            'palm_collision': 0,
            'shaft_collision': 0,
            'ground_collision': 0,
        }
        scored_candidates = []
        for grasp_pos, grasp_rot in candidates:
            reason = self._feasibility_reason(points, grasp_pos, grasp_rot)
            if reason is not None:
                reject_counts[reason] += 1
                continue
            score = self.score_grasp(points, grasp_pos, grasp_rot)
            scored_candidates.append((score, grasp_pos, grasp_rot))
        score_time = time.time() - gen_start

        self.get_logger().info(
            f"Candidate rejection breakdown (total={len(candidates)}): "
            + ", ".join(f"{k}={v}" for k, v in reject_counts.items())
        )

        if not scored_candidates:
            self.get_logger().info("No feasible grasp found")
            return

        scored_candidates.sort(key=lambda x: x[0], reverse=True)
        top_candidates = scored_candidates[:TOP_K]

        marker_array = MarkerArray()
        pose_array_to_send = []

        for idx, (score, pos, rot) in enumerate(top_candidates):
            rot_swapped = rot @ R.from_euler('z', 90, degrees=True).as_matrix()
            quat = R.from_matrix(rot_swapped).as_quat()
            quat_marker = R.from_matrix(rot).as_quat()

            pose_msg = PoseStamped()
            pose_msg.header = pc_msg.header
            pose_msg.pose.position.x = float(pos[0])
            pose_msg.pose.position.y = float(pos[1])
            pose_msg.pose.position.z = float(pos[2])
            pose_msg.pose.orientation.x = float(quat[0])
            pose_msg.pose.orientation.y = float(quat[1])
            pose_msg.pose.orientation.z = float(quat[2])
            pose_msg.pose.orientation.w = float(quat[3])
            self.grasp_pub.publish(pose_msg)
            pose_array_to_send.append(pose_msg)

            for m in self._build_gripper_markers(pc_msg.header, idx, pos, rot, quat_marker):
                marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)

        if pose_array_to_send:
            self.get_logger().info(
                f"Sending {len(pose_array_to_send)} grasp poses to Master Node..."
            )

            if not self.pick_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    "Master node service not available. Skipping this cycle."
                )
                return

            self.get_logger().info("DEBUG: Service is available. Setting busy and calling async...")
            self.robot_is_busy = True

            req = PickPlaceRequest.Request()
            req.poses = pose_array_to_send
            future = self.pick_client.call_async(req)
            self.get_logger().info(f"DEBUG: call_async fired. Future: {future}")
            future.add_done_callback(self.grasp_response_callback)
            self.get_logger().info("DEBUG: Done callback registered.")

        callback_elapsed = time.time() - callback_start
        self.get_logger().info(
            f"Grasp callback total time: {callback_elapsed:.3f}s | "
            f"Candidate generation: {gen_time:.3f}s | "
            f"Scoring/filtering: {score_time:.3f}s | "
            f"Published {len(top_candidates)} poses and markers"
        )

    def grasp_response_callback(self, future):
        self.get_logger().info("DEBUG: grasp_response_callback entered.")
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("SUCCESS: Master Node completed the pick-and-place sequence!")
            else:
                self.get_logger().warn("FAILURE: Master Node could not reach any of the provided poses.")
        except Exception as e:
            self.get_logger().error(f"Service call to Master Node failed: {e}")
        finally:
            self.get_logger().info("Unlocking vision system for next scan.")
            self.robot_is_busy = False


def main(args=None):
    rclpy.init(args=args)
    node = GraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()