import os
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from piper_msgs.srv import PickPlaceRequest

# Gripper parameters
GRIPPER_WIDTH_MIN = 0.001
GRIPPER_WIDTH_MAX = 0.07
GRIPPER_FINGER_LENGTH = 0.07
GRIPPER_FINGER_THICK = 0.04
GRIPPER_PALM_THICK = 0.024
GRIPPER_WIDTH_Y = 0.035
GRIPPER_BODY_DEPTH = 0.145
GRIPPER_MIN_PENETRATION = 0.001
GRIPPER_TARGET_PENETRATION = GRIPPER_FINGER_LENGTH / 2.0

GRIPPER_SHAFT_DEPTH = GRIPPER_BODY_DEPTH - GRIPPER_PALM_THICK  # 0.121
GRIPPER_PALM_WIDTH_Y = 0.09
GRIPPER_SHAFT_RADIUS = 0.04

GRASP_CONTACT_DEPTH = 0.01

# Scoring weights
W_BODY        = 0.5
W_SYM         = 0.2
W_PENETRATION = 0.2
W_FRICTION    = 0.1

TOP_K = 10
PROCESS_EVERY_N = 3


def _local_to_world_pos(local_offset: np.ndarray, grasp_pos: np.ndarray, grasp_rot: np.ndarray) -> np.ndarray:
    return grasp_pos + grasp_rot @ local_offset


def _make_cube_marker(header, ns, marker_id, world_pos, quat, scale_xyz, rgba):
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


class RobustGraspNode(Node):
    def __init__(self):
        super().__init__('robust_grasp_node')

        self._msg_count = 0

        self._ground_plane = None
        plane_path = os.path.expanduser('~/ground_plane.npy')
        if os.path.exists(plane_path):
            self._ground_plane = np.load(plane_path)
            self.get_logger().info(
                f"Loaded ground plane: [a={self._ground_plane[0]:.4f}, "
                f"b={self._ground_plane[1]:.4f}, c={self._ground_plane[2]:.4f}, "
                f"d={self._ground_plane[3]:.4f}]"
            )
        else:
            self.get_logger().warn(
                "No ground plane file found — ground collision disabled."
            )

        self.pc_sub = self.create_subscription(
            PointCloud2, '/cloud/masks_clean', self.pc_callback, 10
        )
        self.grasp_pub = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/grasp_markers', 10)
        self.ground_marker_pub = self.create_publisher(Marker, '/ground_marker', 10)

        self.pick_client = self.create_client(PickPlaceRequest, 'pick_place_request')
        self.robot_is_busy = False

        self.get_logger().info("RobustGraspNode started")

    # -------------------------
    # Feasibility
    # -------------------------
    def _feasibility_reason(self, points, grasp_pos, grasp_rot):
        pts_local = (points - grasp_pos) @ grasp_rot
        x, y, z = pts_local[:, 0], pts_local[:, 1], pts_local[:, 2]
        half_w  = GRIPPER_WIDTH_MAX / 2.0
        half_ft = GRIPPER_FINGER_THICK / 2.0
        outer_x = half_w + GRIPPER_FINGER_THICK

        in_env = (z >= 0.0) & (z <= GRIPPER_FINGER_LENGTH)
        if pts_local[in_env].shape[0] == 0:
            return 'no_points_in_envelope'

        in_jaw = (np.abs(x) <= half_w) & (np.abs(y) <= GRIPPER_WIDTH_Y / 2.0)
        pts_in_jaw = pts_local[in_jaw]
        pts_in_jaw_env = pts_in_jaw[
            (pts_in_jaw[:, 2] >= 0.0) & (pts_in_jaw[:, 2] <= GRIPPER_FINGER_LENGTH)
        ]
        if pts_in_jaw_env.shape[0] == 0:
            return 'insufficient_penetration'

        left_finger  = (x >= half_w)   & (x <= outer_x)
        right_finger = (x <= -half_w)  & (x >= -outer_x)
        in_finger_y  = np.abs(y) <= GRIPPER_WIDTH_Y / 2.0
        in_finger_z  = (z >= 0.0) & (z <= GRIPPER_FINGER_LENGTH)
        if np.any((left_finger | right_finger) & in_finger_y & in_finger_z):
            return 'finger_collision'

        in_palm_x = np.abs(x) <= outer_x
        in_palm_y = np.abs(y) <= GRIPPER_PALM_WIDTH_Y / 2.0
        in_palm_z = (z < 0.0) & (z >= -GRIPPER_PALM_THICK)
        if np.any(in_palm_x & in_palm_y & in_palm_z):
            return 'palm_collision'

        in_shaft_z      = (z < -GRIPPER_PALM_THICK) & (z >= -GRIPPER_BODY_DEPTH)
        radial_dist     = np.sqrt(x**2 + y**2)
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
            signed_dists  = corners_world @ np.array([a, b, c]) + d
            if np.any(signed_dists < 0.0):
                return 'ground_collision'

        return None

    # -------------------------
    # Scoring
    # -------------------------
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

        left_contact  = (enclosed[:, 0] >= half_w - GRASP_CONTACT_DEPTH) & (enclosed[:, 0] <= half_w)
        right_contact = (enclosed[:, 0] <= -half_w + GRASP_CONTACT_DEPTH) & (enclosed[:, 0] >= -half_w)

        n_left  = float(np.sum(left_contact  & in_contact_yz))
        n_right = float(np.sum(right_contact & in_contact_yz))
        n_contact = n_left + n_right

        body_score = n_contact / (n_enclosed + 1e-6)
        sym_score = 1.0 - abs(n_left - n_right) / (n_contact + 1e-6)

        in_jaw = in_volume & (np.abs(y) <= GRIPPER_WIDTH_Y / 2.0)
        pts_in_jaw = pts_local[in_jaw]
        actual_pen = float(pts_in_jaw[:, 2].max()) if pts_in_jaw.shape[0] > 0 else 0.0
        object_extent = float(pts_local[:, 2].max() - pts_local[:, 2].min()) if pts_local.shape[0] > 0 else GRIPPER_FINGER_LENGTH
        adaptive_target = min(GRIPPER_TARGET_PENETRATION, object_extent / 2.0)
        adaptive_target = max(adaptive_target, GRIPPER_MIN_PENETRATION)
        penetration_score = max(0.0, 1.0 - abs(actual_pen - adaptive_target) / adaptive_target)

        approach_axis = grasp_rot[:, 2]
        clearance = self._ground_clearance(points)
        threshold = GRIPPER_WIDTH_Y / 2.0
        lateral_fraction = float(np.clip(1.0 - clearance / threshold, 0.0, 1.0))
        downward_score = max(0.0, float(approach_axis[2]))
        lateral_score  = max(0.0, 1.0 - abs(float(approach_axis[2])))
        friction_score = (1.0 - lateral_fraction) * downward_score + lateral_fraction * lateral_score

        return (W_BODY * body_score + W_SYM * sym_score
                + W_PENETRATION * penetration_score + W_FRICTION * friction_score)

    def _ground_clearance(self, points: np.ndarray) -> float:
        if self._ground_plane is None:
            return 1.0
        a, b, c, d = self._ground_plane
        normal = np.array([a, b, c])
        dists = points @ normal + d
        return float(np.min(dists))

    def _sample_approach_directions(self, n_samples: int, lateral_fraction: float) -> np.ndarray:
        if self._ground_plane is not None:
            a, b, c, _ = self._ground_plane
            up = np.array([a, b, c], dtype=float)
            up /= np.linalg.norm(up)
        else:
            up = np.array([0.0, 0.0, 1.0])

        n_lateral = int(round(n_samples * lateral_fraction))
        n_hemi    = n_samples - n_lateral
        directions = []

        while len(directions) < n_hemi:
            vec = np.random.randn(3)
            if np.dot(vec, up) < 0:
                vec -= 2.0 * np.dot(vec, up) * up
            norm = np.linalg.norm(vec)
            if norm < 1e-6:
                continue
            directions.append(vec / norm)

        while len(directions) < n_samples:
            vec = np.random.randn(3)
            vec -= np.dot(vec, up) * up
            norm = np.linalg.norm(vec)
            if norm < 1e-6:
                continue
            vec /= norm
            tilt = np.random.uniform(0.0, np.deg2rad(15.0))
            vec = vec * np.cos(tilt) - up * np.sin(tilt)
            vec /= np.linalg.norm(vec)
            directions.append(vec)

        return np.array(directions)

    def generate_candidates(self, points, n_samples=60, rolls_per_point=6):
        candidates = []
        if points.shape[0] == 0:
            return candidates

        centroid = points.mean(axis=0)
        standoff = GRIPPER_PALM_THICK / 2 + 0.005

        clearance = self._ground_clearance(points)
        threshold = GRIPPER_WIDTH_Y / 2.0
        lateral_fraction = float(np.clip(1.0 - clearance / threshold, 0.0, 1.0))
        self.get_logger().info(
            f"Ground clearance: {clearance*100:.1f} cm  ->  lateral fraction: {lateral_fraction:.2f}"
        )

        approach_dirs = self._sample_approach_directions(n_samples, lateral_fraction)

        for z_axis in approach_dirs:
            projections = (points - centroid) @ z_axis
            near_surface = points[np.argmin(projections)]
            grasp_pos = near_surface - z_axis * standoff

            tmp = np.array([1.0, 0.0, 0.0])
            if abs(np.dot(tmp, z_axis)) > 0.9:
                tmp = np.array([0.0, 1.0, 0.0])
            x_ref = np.cross(tmp, z_axis)
            x_ref /= np.linalg.norm(x_ref)

            for k in range(rolls_per_point):
                roll = (k / rolls_per_point) * np.pi
                x_axis = (x_ref * np.cos(roll) + np.cross(z_axis, x_ref) * np.sin(roll)
                          + z_axis * np.dot(z_axis, x_ref) * (1 - np.cos(roll)))
                x_axis /= np.linalg.norm(x_axis)
                y_axis = np.cross(z_axis, x_axis)
                y_axis /= np.linalg.norm(y_axis)

                grasp_rot = np.stack([x_axis, y_axis, z_axis], axis=1)
                candidates.append((grasp_pos, grasp_rot))
        return candidates

    # -------------------------
    # Gripper marker helpers
    # -------------------------
    def _build_gripper_markers(self, header, idx, pos, rot, quat):
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

    # -------------------------
    # Callback
    # -------------------------
    def pc_callback(self, pc_msg):
        if self.robot_is_busy:
            self.get_logger().info(
                "Arm is currently moving. Skipping vision request...",
                throttle_duration_sec=2.0
            )
            return

        callback_start = time.time()
        self._msg_count += 1
        if self._msg_count % PROCESS_EVERY_N != 0:
            return

        points_list = list(pc2.read_points(pc_msg, field_names=("x","y","z","instance_id"), skip_nans=True))
        if not points_list:
            return

        all_points = np.array([[p[0],p[1],p[2],p[3]] for p in points_list], dtype=np.float32)
        instance_ids = all_points[:, 3]
        unique_ids = np.unique(instance_ids)
        if unique_ids.size == 0:
            return
        first_id = unique_ids[0]
        points = all_points[instance_ids == first_id, :3]
        points = points[points[:, 2] < 1.0]
        if points.shape[0] == 0:
            return

        gen_start = time.time()
        candidates = self.generate_candidates(points, n_samples=500, rolls_per_point=10)
        gen_time = time.time() - gen_start

        reject_counts = {
            'no_points_in_envelope':    0,
            'insufficient_penetration': 0,
            'finger_collision':         0,
            'palm_collision':           0,
            'shaft_collision':          0,
            'ground_collision':         0,
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

        # Publish ground plane marker
        if self._ground_plane is not None:
            a, b, c, d = self._ground_plane
            normal = np.array([a, b, c])
            origin = -d * normal

            marker = Marker()
            marker.header = pc_msg.header
            marker.ns = "ground_plane"
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(origin[0])
            marker.pose.position.y = float(origin[1])
            marker.pose.position.z = float(origin[2])

            world_z = np.array([0.0, 0.0, 1.0])
            axis = np.cross(world_z, normal)
            axis_norm = np.linalg.norm(axis)
            if axis_norm < 1e-6:
                if np.dot(world_z, normal) > 0:
                    qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
                else:
                    qx, qy, qz, qw = 1.0, 0.0, 0.0, 0.0
            else:
                axis = axis / axis_norm
                angle = float(np.arccos(np.clip(np.dot(world_z, normal), -1.0, 1.0)))
                s = np.sin(angle / 2.0)
                qx, qy, qz, qw = float(axis[0]*s), float(axis[1]*s), float(axis[2]*s), float(np.cos(angle/2.0))

            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = 2.0
            marker.scale.y = 2.0
            marker.scale.z = 0.002
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.4

            self.ground_marker_pub.publish(marker)

        # Final busy check before sending service call
        if self.robot_is_busy:
            self.get_logger().info(
                "Arm is currently moving. Skipping vision request...",
                throttle_duration_sec=2.0
            )
            return

        marker_array = MarkerArray()
        pose_array_to_send = []

        for idx, (score, pos, rot) in enumerate(top_candidates):
            quat = R.from_matrix(rot).as_quat()

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

            for m in self._build_gripper_markers(pc_msg.header, idx, pos, rot, quat):
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

            self.get_logger().info(
                "DEBUG: Service is available. Setting busy and calling async..."
            )
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
                self.get_logger().info(
                    "SUCCESS: Master Node completed the pick-and-place sequence!"
                )
            else:
                self.get_logger().warn(
                    "FAILURE: Master Node could not reach any of the provided poses."
                )
        except Exception as e:
            self.get_logger().error(f"Service call to Master Node failed: {e}")
        finally:
            self.get_logger().info("Unlocking vision system for next scan.")
            self.robot_is_busy = False


def main(args=None):
    rclpy.init(args=args)
    node = RobustGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()