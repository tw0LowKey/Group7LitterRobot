import os
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

# Gripper parameters
GRIPPER_WIDTH_MIN = 0.001
GRIPPER_WIDTH_MAX = 0.07
GRIPPER_FINGER_LENGTH = 0.07
GRIPPER_FINGER_THICK = 0.025
GRIPPER_PALM_THICK = 0.024
GRIPPER_WIDTH_Y = 0.035
GRIPPER_BODY_DEPTH = 0.145
GRIPPER_MIN_PENETRATION = 0.001
GRIPPER_TARGET_PENETRATION = GRIPPER_FINGER_LENGTH / 2.0

# Scoring weights
W_BODY = 0.5
W_SYM = 0.2
W_PENETRATION = 0.2
W_FRICTION = 0.1

TOP_K = 50
PROCESS_EVERY_N = 5


class RobustGraspNode(Node):
    def __init__(self):
        super().__init__('robust_grasp_node')

        self._msg_count = 0

        # Load ground plane
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
        self.get_logger().info("RobustGraspNode started")

    # Feasibility
    def _feasibility_reason(self, points, grasp_pos, grasp_rot):
        pts_local = (points - grasp_pos) @ grasp_rot
        x, y, z = pts_local[:, 0], pts_local[:, 1], pts_local[:, 2]
        half_w = GRIPPER_WIDTH_MAX / 2.0

        # Finger envelope
        in_env = (z >= 0.0) & (z <= GRIPPER_FINGER_LENGTH)
        pts_env = pts_local[in_env]
        if pts_env.shape[0] == 0:
            return 'no_points_in_envelope'

        # Jaw footprint in X-Y
        in_jaw = (np.abs(x) <= half_w) & (np.abs(y) <= GRIPPER_WIDTH_Y / 2.0)
        pts_in_jaw = pts_local[in_jaw]

        pts_in_finger_envelope = pts_in_jaw[(pts_in_jaw[:, 2] >= 0.0) & (pts_in_jaw[:, 2] <= GRIPPER_FINGER_LENGTH)]
        if pts_in_finger_envelope.shape[0] == 0:
            return 'insufficient_penetration'

        # Body collision
        behind = z < 0.0
        if np.any(behind & (np.abs(x) <= half_w) & (np.abs(y) <= GRIPPER_WIDTH_Y / 2.0)):
            return 'body_collision'

        # Side collision
        side = behind & (np.abs(y) > GRIPPER_WIDTH_Y / 2.0)
        if np.any(side & (z >= -GRIPPER_BODY_DEPTH)):
            return 'side_collision'

        # Ground collision
        if self._ground_plane is not None:
            a, b, c, d = self._ground_plane
            corners_local = np.array([
                [ half_w,  GRIPPER_WIDTH_Y/2, -GRIPPER_BODY_DEPTH],
                [ half_w, -GRIPPER_WIDTH_Y/2, -GRIPPER_BODY_DEPTH],
                [-half_w,  GRIPPER_WIDTH_Y/2, -GRIPPER_BODY_DEPTH],
                [-half_w, -GRIPPER_WIDTH_Y/2, -GRIPPER_BODY_DEPTH],
                [ half_w,  GRIPPER_WIDTH_Y/2,  GRIPPER_FINGER_LENGTH],
                [ half_w, -GRIPPER_WIDTH_Y/2,  GRIPPER_FINGER_LENGTH],
                [-half_w,  GRIPPER_WIDTH_Y/2,  GRIPPER_FINGER_LENGTH],
                [-half_w, -GRIPPER_WIDTH_Y/2,  GRIPPER_FINGER_LENGTH],
            ])
            corners_world = grasp_pos + corners_local @ grasp_rot.T
            signed_dists = corners_world @ np.array([a, b, c]) + d

            if np.any(signed_dists < 0.0):
                return 'ground_collision'

        return None

    # Scoring
    def score_grasp(self, points, grasp_pos, grasp_rot):
        pts_local = (points - grasp_pos) @ grasp_rot
        x, y, z = pts_local[:, 0], pts_local[:, 1], pts_local[:, 2]
        half_w = GRIPPER_WIDTH_MAX / 2.0

        in_env = (z >= 0.0) & (z <= GRIPPER_FINGER_LENGTH)
        in_volume = in_env & (x >= -half_w) & (x <= half_w)
        enclosed = pts_local[in_volume]
        body_score = float(enclosed.shape[0])

        # Symmetry
        n_pos = float(np.sum(enclosed[:, 0] >= 0)) if enclosed.shape[0] > 0 else 0.0
        n_neg = float(enclosed.shape[0] - n_pos)
        sym_score = 1.0 - abs(n_pos - n_neg) / (enclosed.shape[0] + 1e-6)

        # Penetration
        in_jaw = in_volume & (np.abs(y) <= GRIPPER_WIDTH_Y / 2.0)
        pts_in_jaw = pts_local[in_jaw]
        actual_pen = float(pts_in_jaw[:, 2].max()) if pts_in_jaw.shape[0] > 0 else 0.0
        penetration_score = max(0.0, 1.0 - abs(actual_pen - GRIPPER_TARGET_PENETRATION) / GRIPPER_TARGET_PENETRATION)

        # Friction cone score (approx)
        approach_axis = grasp_rot[:, 2]
        friction_score = max(0.0, approach_axis[2])

        return W_BODY * body_score + W_SYM * sym_score + W_PENETRATION * penetration_score + W_FRICTION * friction_score

    # Candidate generation
    def generate_candidates(self, points, n_samples=60, rolls_per_point=6):
        candidates = []
        if points.shape[0] == 0:
            return candidates

        centroid = points.mean(axis=0)
        standoff = GRIPPER_PALM_THICK / 2 + 0.005

        for _ in range(n_samples):
            vec = np.random.randn(3)
            vec[2] = abs(vec[2])
            norm = np.linalg.norm(vec)
            if norm < 1e-6:
                continue
            z_axis = vec / norm

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

    # Callback
    def pc_callback(self, pc_msg):
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
            'no_points_in_envelope': 0,
            'width_out_of_range': 0,
            'body_collision': 0,
            'too_deep': 0,
            'side_collision': 0,
            'insufficient_penetration': 0,
            'ground_collision': 0
        }

        score_start = time.time()
        scored_candidates = []
        for grasp_pos, grasp_rot in candidates:
            reason = self._feasibility_reason(points, grasp_pos, grasp_rot)
            if reason is not None:
                reject_counts[reason] += 1
                continue
            score = self.score_grasp(points, grasp_pos, grasp_rot)
            scored_candidates.append((score, grasp_pos, grasp_rot))
        score_time = time.time() - score_start

        self.get_logger().info(
            f"Candidate rejection breakdown (total={len(candidates)}): "
            + ", ".join(f"{k}={v}" for k, v in reject_counts.items())
        )

        if not scored_candidates:
            self.get_logger().info("No feasible grasp found")
            return

        scored_candidates.sort(key=lambda x: x[0], reverse=True)
        top_candidates = scored_candidates[:TOP_K]

        for score, pos, rot in top_candidates:
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

        callback_elapsed = time.time() - callback_start
        self.get_logger().info(
            f"Grasp callback total time: {callback_elapsed:.3f}s | "
            f"Candidate generation: {gen_time:.3f}s | "
            f"Scoring/filtering: {score_time:.3f}s | "
            f"Published {len(top_candidates)} poses"
        )


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