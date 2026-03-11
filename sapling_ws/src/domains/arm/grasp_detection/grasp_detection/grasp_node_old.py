import os
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

import sensor_msgs_py.point_cloud2 as pc2

# ------------------------
# Gripper physical parameters
# ------------------------
GRIPPER_WIDTH_MIN = 0.001       # min finger gap (m)
GRIPPER_WIDTH_MAX = 0.07        # max finger gap (m)
GRIPPER_FINGER_LENGTH = 0.07    # finger length along approach (m)
GRIPPER_FINGER_THICK = 0.025    # finger cross-section thickness (m)
GRIPPER_PALM_THICK = 0.024      # palm slab thickness (m)
GRIPPER_WIDTH_Y = 0.055         # finger side thickness (m)
GRIPPER_BODY_DEPTH = 0.145      # full depth of gripper body behind palm face (m)
GRIPPER_MIN_PENETRATION = 0.01  # minimum finger engagement depth into object (m)
GRIPPER_TARGET_PENETRATION = GRIPPER_FINGER_LENGTH / 2.0  # ideal penetration depth (m)

# Scoring weights — must sum to 1.0
W_BODY = 0.5          # weight for body enclosure score
W_SYM  = 0.2          # weight for closing-axis symmetry
W_PENETRATION = 0.3   # weight for finger penetration depth

# Marker lifetime in seconds
MARKER_LIFETIME_S = 1.0

# Callback throttle
PROCESS_EVERY_N = 5


class RobustGraspNode(Node):
    def __init__(self):
        super().__init__('robust_grasp_node')

        self._msg_count = 0

        # Load saved ground plane if available
        self._ground_plane = None
        plane_path = os.path.expanduser('~/ground_plane.npy')
        if os.path.exists(plane_path):
            self._ground_plane = np.load(plane_path)
            self.get_logger().info(
                f"Loaded ground plane: "
                f"[a={self._ground_plane[0]:.4f}, b={self._ground_plane[1]:.4f}, "
                f"c={self._ground_plane[2]:.4f}, d={self._ground_plane[3]:.4f}]"
            )
        else:
            self.get_logger().warn(
                "No ground plane file found at ~/ground_plane.npy — "
                "ground collision checking disabled. "
                "Run ground_plane_node to calibrate."
            )

        self.pc_sub = self.create_subscription(
            PointCloud2, '/cloud/masks_clean', self.pc_callback, 10
        )
        self.grasp_pub = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/grasp_markers', 10)
        self.ground_marker_pub = self.create_publisher(Marker, '/ground_plane/marker', 10)

        # Publish ground plane marker at 1 Hz so it's always visible in RViz2
        # regardless of whether a grasp is being detected
        if self._ground_plane is not None:
            self.create_timer(1.0, self._publish_ground_marker)

        self.get_logger().info("RobustGraspNode started")

    # ----------------------------------------------------------------
    # Ground plane marker
    # ----------------------------------------------------------------
    def _publish_ground_marker(self):
        """
        Publish a semi-transparent yellow slab aligned with the ground plane.
        Called at 1 Hz via timer so it stays visible in RViz2 continuously.
        """
        if self._ground_plane is None:
            return

        normal = self._ground_plane[:3]
        d = self._ground_plane[3]
        plane_normal = normal.copy()

        # Position anchor: project a point 1 m along +Z (forward into the scene
        # in camera_color_optical_frame) onto the plane.
        scene_centre = np.array([0.0, 0.0, 1.0])
        t = -(np.dot(normal, scene_centre) + d) / (np.dot(normal, normal) + 1e-9)
        origin = scene_centre + t * normal

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'camera_color_optical_frame'
        marker.ns = 'ground_plane'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.lifetime = rclpy.duration.Duration(seconds=1.5).to_msg()

        marker.pose.position.x = float(origin[0])
        marker.pose.position.y = float(origin[1])
        marker.pose.position.z = float(origin[2])

        # The plane normal points upward, which in camera_color_frame is
        # approximately -Y. We want the slab face (marker XY plane) to lie
        # flat on the ground, so the slab thickness axis (local Z) must align
        # with the plane normal.
        #
        # Build rotation matrix with:
        #   local Z = plane normal (upward, ≈ -Y in camera frame)
        #   local X = camera +X (right), projected onto the plane
        #   local Y = local Z × local X
        # In camera_color_optical_frame: X right, Y down, Z forward.
        # Use camera +X (right) as the reference in-plane axis.
        cam_x = np.array([1.0, 0.0, 0.0])
        local_z = plane_normal  # plane normal ≈ -Y (upward in optical frame)

        local_x = cam_x - np.dot(cam_x, local_z) * local_z
        local_x_norm = np.linalg.norm(local_x)
        if local_x_norm < 1e-6:
            cam_z = np.array([0.0, 0.0, 1.0])
            local_x = cam_z - np.dot(cam_z, local_z) * local_z
            local_x /= np.linalg.norm(local_x)
        else:
            local_x /= local_x_norm

        local_y = np.cross(local_z, local_x)
        local_y /= np.linalg.norm(local_y)

        # Columns of rotation matrix are local axes expressed in world frame
        rot_matrix = np.stack([local_x, local_y, local_z], axis=1)
        quat = R.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]

        marker.pose.orientation.x = float(quat[0])
        marker.pose.orientation.y = float(quat[1])
        marker.pose.orientation.z = float(quat[2])
        marker.pose.orientation.w = float(quat[3])

        marker.scale.x = 2.0    # 2 m wide
        marker.scale.y = 2.0    # 2 m deep
        marker.scale.z = 0.002  # 2 mm thin slab

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.4

        self.ground_marker_pub.publish(marker)

    # ----------------------------------------------------------------
    # Feasibility check
    # ----------------------------------------------------------------
    def _feasibility_reason(self, points, grasp_pos, grasp_rot):
        """
        Returns a rejection reason string, or None if the candidate is feasible.

        Grasp frame convention
        ----------------------
          X : closing / finger-separation axis
          Y : finger side-thickness axis
          Z : approach axis — fingers extend in +Z from the palm face

        Checks
        ------
        1. At least some points fall inside the finger-length envelope (Z in [0, FL])
        2. The cross-section of points within ±half_w along X fits the jaw
           (only the slice between the fingers matters, not the whole object)
        3. No points collide with the gripper body behind the palm face (Z < 0)
           within the body footprint (X within jaw width, Y within side thickness)
        4. No points extend deeper than the finger length (Z > FL)
        5. No side collision behind the palm along Y
        """
        pts_local = (points - grasp_pos) @ grasp_rot  # (N, 3)
        z = pts_local[:, 2]
        y = pts_local[:, 1]
        x = pts_local[:, 0]
        half_w = GRIPPER_WIDTH_MAX / 2.0

        # Check 1: points in finger envelope
        in_env = (z >= 0.0) & (z <= GRIPPER_FINGER_LENGTH)
        pts_env = pts_local[in_env]
        if pts_env.shape[0] == 0:
            return 'no_points_in_envelope'

        # Check 2: cross-section within jaw width fits
        pts_jaw = pts_env[np.abs(pts_env[:, 0]) <= half_w]
        if pts_jaw.shape[0] == 0:
            return 'no_points_in_envelope'
        span = pts_jaw[:, 0].max() - pts_jaw[:, 0].min()
        if span < GRIPPER_WIDTH_MIN:
            return 'width_out_of_range'

        # Check 3: body collision
        behind = z < 0.0
        if np.any(behind & (np.abs(x) <= half_w) & (np.abs(y) <= GRIPPER_WIDTH_Y / 2.0)):
            return 'body_collision'

        # Check 4: points within the jaw footprint must not exceed finger length
        in_jaw_footprint = (np.abs(x) <= half_w) & (np.abs(y) <= GRIPPER_WIDTH_Y / 2.0)
        pts_in_jaw = pts_local[in_jaw_footprint]
        if pts_in_jaw.shape[0] > 0 and pts_in_jaw[:, 2].max() > GRIPPER_FINGER_LENGTH:
            return 'too_deep'

        # Check 5a: fingers must penetrate far enough to actually engage the object
        if pts_in_jaw.shape[0] == 0 or pts_in_jaw[:, 2].max() < GRIPPER_MIN_PENETRATION:
            return 'insufficient_penetration'

        # Check 5: side collision
        side = behind & (np.abs(y) > GRIPPER_WIDTH_Y / 2.0)
        if np.any(side & (z >= -GRIPPER_BODY_DEPTH)):
            return 'side_collision'

        # Check 6: no gripper corner intersects the ground plane (if calibrated)
        if self._ground_plane is not None:
            a, b, c, d = self._ground_plane
            half_y = GRIPPER_WIDTH_Y / 2.0
            # Eight corners spanning the full gripper volume in local frame
            corners_local = np.array([
                [ half_w,  half_y, -GRIPPER_BODY_DEPTH],
                [ half_w, -half_y, -GRIPPER_BODY_DEPTH],
                [-half_w,  half_y, -GRIPPER_BODY_DEPTH],
                [-half_w, -half_y, -GRIPPER_BODY_DEPTH],
                [ half_w,  half_y,  GRIPPER_FINGER_LENGTH],
                [ half_w, -half_y,  GRIPPER_FINGER_LENGTH],
                [-half_w,  half_y,  GRIPPER_FINGER_LENGTH],
                [-half_w, -half_y,  GRIPPER_FINGER_LENGTH],
            ])
            # Rotate corners into world frame and offset by grasp position
            corners_world = grasp_pos + corners_local @ grasp_rot.T
            # Positive signed distance means below the ground plane
            # (plane normal points upward, so ax+by+cz+d > 0 is below ground)
            signed_dists = corners_world @ np.array([a, b, c]) + d
            if np.any(signed_dists > 0.0):
                return 'ground_collision'

        return None

    # ----------------------------------------------------------------
    # Scoring function
    # ----------------------------------------------------------------
    def score_grasp(self, points, grasp_pos, grasp_rot, mu=0.5):
        pts_local = (points - grasp_pos) @ grasp_rot  # transform to grasp frame
        x = pts_local[:, 0]  # closing axis
        y = pts_local[:, 1]
        z = pts_local[:, 2]
        half_w = GRIPPER_WIDTH_MAX / 2.0

        # Points inside the finger envelope
        in_env = (z >= 0.0) & (z <= GRIPPER_FINGER_LENGTH) & (np.abs(x) <= half_w)
        pts_env = pts_local[in_env]
        body_score = float(pts_env.shape[0])

        if body_score == 0:
            return 0.0  # discard empty candidate

        # Symmetry across X (closing) axis
        n_pos = float(np.sum(pts_env[:, 0] >= 0))
        n_neg = float(pts_env.shape[0] - n_pos)
        sym_score = 1.0 - abs(n_pos - n_neg) / (pts_env.shape[0] + 1e-6)

        # Penetration depth along Z
        in_jaw = in_env & (np.abs(y) <= GRIPPER_WIDTH_Y / 2.0)
        pts_in_jaw = pts_local[in_jaw]
        actual_penetration = float(pts_in_jaw[:, 2].max()) if pts_in_jaw.shape[0] > 0 else 0.0
        penetration_score = max(0.0, 1.0 - abs(actual_penetration - GRIPPER_TARGET_PENETRATION)
                                / GRIPPER_TARGET_PENETRATION)

        # ---------------------------
        # Friction cone / slip approximation
        # ---------------------------
        # Approximate local normals from nearest neighbors
        # Here we use a simple PCA approximation for each point in jaw
        friction_score = 0.0
        if pts_in_jaw.shape[0] > 3:
            cov = np.cov(pts_in_jaw.T)
            _, _, vh = np.linalg.svd(cov)
            normal_est = vh[-1]  # estimated surface normal along least variance
            # Angle between closing axis (X) and surface normal
            angle = np.arccos(np.clip(np.abs(normal_est[0]), 0.0, 1.0))  # normal aligned with X
            # Max angle allowed by friction cone
            max_angle = np.arctan(mu)
            friction_score = max(0.0, 1.0 - angle / (max_angle + 1e-6))

        # ---------------------------
        # Coverage / visible balance (penalize one-sided grasps)
        # ---------------------------
        left_pts = pts_env[pts_env[:,0] > 0].shape[0]
        right_pts = pts_env[pts_env[:,0] < 0].shape[0]
        coverage_score = 1.0 - abs(left_pts - right_pts) / (left_pts + right_pts + 1e-6)

        # ---------------------------
        # Weighted sum
        # ---------------------------
        W_BODY = 0.4
        W_SYM = 0.15
        W_PEN = 0.2
        W_FRIC = 0.15
        W_COV = 0.1

        score = (W_BODY * body_score +
                W_SYM * sym_score +
                W_PEN * penetration_score +
                W_FRIC * friction_score +
                W_COV * coverage_score)
        return score

    # ----------------------------------------------------------------
    # Gripper visualisation — includes body depth marker
    # ----------------------------------------------------------------
    def create_gripper_marker(self, grasp_pos, grasp_rot, frame_id, stamp):
        """
        Build a MarkerArray visualising the full gripper geometry:

          0  — Palm slab (green)         : front face at Z=0
          1  — Left finger (red)         : +X side, extending in +Z
          2  — Right finger (red)        : -X side, extending in +Z
          3  — Gripper body (blue, 30%)  : extends -Z behind the palm by
                                           GRIPPER_BODY_DEPTH, showing the
                                           volume checked for body collision
        """
        markers = MarkerArray()
        quat = R.from_matrix(grasp_rot).as_quat()
        lifetime = rclpy.duration.Duration(seconds=MARKER_LIFETIME_S).to_msg()

        def make_marker(marker_id, pos_world, sx, sy, sz, r, g, b, a=0.8):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.ns = 'gripper'
            m.id = marker_id
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.lifetime = lifetime
            m.pose.position.x = float(pos_world[0])
            m.pose.position.y = float(pos_world[1])
            m.pose.position.z = float(pos_world[2])
            m.pose.orientation.x = float(quat[0])
            m.pose.orientation.y = float(quat[1])
            m.pose.orientation.z = float(quat[2])
            m.pose.orientation.w = float(quat[3])
            m.scale.x = float(sx)
            m.scale.y = float(sy)
            m.scale.z = float(sz)
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = float(a)
            return m

        # Palm — centred at grasp_pos, thin slab in Z
        markers.markers.append(make_marker(
            marker_id=0, pos_world=grasp_pos,
            sx=GRIPPER_WIDTH_MAX, sy=GRIPPER_WIDTH_Y, sz=GRIPPER_PALM_THICK,
            r=0.0, g=1.0, b=0.0, a=0.5,
        ))

        # Fingers — flush with front face of palm, extending in +Z
        palm_face_z = GRIPPER_PALM_THICK / 2.0
        finger_z_offset = palm_face_z + GRIPPER_FINGER_LENGTH / 2.0
        for marker_id, local_x in [
            (1,  GRIPPER_WIDTH_MAX / 2.0 + GRIPPER_FINGER_THICK / 2.0),
            (2, -(GRIPPER_WIDTH_MAX / 2.0 + GRIPPER_FINGER_THICK / 2.0)),
        ]:
            world_pos = grasp_pos + grasp_rot @ np.array([local_x, 0.0, finger_z_offset])
            markers.markers.append(make_marker(
                marker_id=marker_id, pos_world=world_pos,
                sx=GRIPPER_FINGER_THICK, sy=GRIPPER_WIDTH_Y, sz=GRIPPER_FINGER_LENGTH,
                r=1.0, g=0.0, b=0.0,
            ))

        # Gripper body — extends behind the palm face by GRIPPER_BODY_DEPTH.
        # Centred at palm_pos - z_axis * (GRIPPER_BODY_DEPTH / 2) in local frame.
        body_z_offset = -(GRIPPER_BODY_DEPTH / 2.0)
        body_world_pos = grasp_pos + grasp_rot @ np.array([0.0, 0.0, body_z_offset])
        markers.markers.append(make_marker(
            marker_id=3, pos_world=body_world_pos,
            sx=GRIPPER_WIDTH_MAX, sy=GRIPPER_WIDTH_Y, sz=GRIPPER_BODY_DEPTH,
            r=0.0, g=0.0, b=1.0, a=0.3,
        ))

        return markers

    # ----------------------------------------------------------------
    # Candidate generation
    # ----------------------------------------------------------------
    def generate_candidates(self, points, n_samples=60, rolls_per_point=6):
        """
        Generate 6-DOF grasp candidates anchored at the object surface entry point.

        For each sampled approach direction the near-side surface point is found
        (the point with the most negative projection onto the approach axis,
        i.e. the first surface the gripper encounters). The palm is placed just
        behind that point. This avoids the body_collision failures that occur
        when the palm is placed at the centroid, which is typically inside the object.

        Approach directions are sampled over the full unit sphere — side approaches
        with small Z components are essential for an upright cup.
        """
        candidates = []
        if points.shape[0] == 0:
            return candidates

        centroid = points.mean(axis=0).astype(np.float64)
        standoff = GRIPPER_PALM_THICK / 2.0 + 0.005

        for _ in range(n_samples):
            # Restrict to +Z hemisphere — the robot approaches from the camera side.
            # Approaches from behind the camera (negative Z) place the palm on the
            # wrong side of the object.
            vec = np.random.randn(3)
            vec[2] = abs(vec[2])  # force +Z component
            norm = np.linalg.norm(vec)
            if norm < 1e-6:
                continue
            z_axis = vec / norm

            # Near-side surface point: furthest in the -z_axis direction
            projections = (points - centroid) @ z_axis
            near_surface = points[np.argmin(projections)].astype(np.float64)
            grasp_pos = near_surface - z_axis * standoff

            tmp = np.array([1.0, 0.0, 0.0])
            if np.abs(np.dot(tmp, z_axis)) > 0.9:
                tmp = np.array([0.0, 1.0, 0.0])
            x_ref = np.cross(tmp, z_axis)
            x_ref /= np.linalg.norm(x_ref)

            for k in range(rolls_per_point):
                roll = (k / rolls_per_point) * np.pi
                x_axis = (x_ref * np.cos(roll)
                          + np.cross(z_axis, x_ref) * np.sin(roll)
                          + z_axis * np.dot(z_axis, x_ref) * (1 - np.cos(roll)))
                x_axis /= np.linalg.norm(x_axis)
                y_axis = np.cross(z_axis, x_axis)
                y_axis /= np.linalg.norm(y_axis)

                grasp_rot = np.stack([x_axis, y_axis, z_axis], axis=1)
                candidates.append((grasp_pos, grasp_rot))

        return candidates

    # ----------------------------------------------------------------
    # Main callback
    # ----------------------------------------------------------------
    def pc_callback(self, pc_msg):
        self._msg_count += 1
        if self._msg_count % PROCESS_EVERY_N != 0:
            return

        points_list = list(pc2.read_points(
            pc_msg, field_names=("x", "y", "z", "instance_id"), skip_nans=True
        ))
        raw_pts = len(points_list)

        if not points_list:
            self.get_logger().info(f"Callback: raw pts: {raw_pts} | clean pts: 0")
            return

        all_points = np.array(
            [[p[0], p[1], p[2], p[3]] for p in points_list], dtype=np.float32
        )

        instance_ids = all_points[:, 3]
        unique_ids = np.unique(instance_ids)
        if unique_ids.size == 0:
            self.get_logger().info("No instances found in point cloud")
            return
        first_id = unique_ids[0]

        points = all_points[instance_ids == first_id, :3]
        points = points[points[:, 2] < 1.0]

        if points.shape[0] == 0:
            self.get_logger().info("No points after filtering")
            return

        self.get_logger().info(
            f"Callback: raw pts: {raw_pts} | instance {int(first_id)} pts: {points.shape[0]}"
        )
        self.get_logger().info(
            f"Cloud extents — "
            f"X: [{points[:,0].min():.3f}, {points[:,0].max():.3f}]  "
            f"Y: [{points[:,1].min():.3f}, {points[:,1].max():.3f}]  "
            f"Z: [{points[:,2].min():.3f}, {points[:,2].max():.3f}]"
        )

        candidates = self.generate_candidates(points, n_samples=30, rolls_per_point=6)
        if not candidates:
            self.get_logger().info("No grasp candidates generated")
            return

        best_score = -np.inf
        best_pos, best_rot = None, None

        reject_counts = {
            'no_points_in_envelope':    0,
            'width_out_of_range':       0,
            'body_collision':           0,
            'too_deep':                 0,
            'side_collision':           0,
            'insufficient_penetration': 0,
            'ground_collision':         0,
        }

        for grasp_pos, grasp_rot in candidates:
            reason = self._feasibility_reason(points, grasp_pos, grasp_rot)
            if reason is not None:
                reject_counts[reason] += 1
                continue
            score = self.score_grasp(points, grasp_pos, grasp_rot)
            if score > best_score:
                best_score = score
                best_pos = grasp_pos
                best_rot = grasp_rot

        self.get_logger().info(
            f"Candidate rejection breakdown (total={len(candidates)}): "
            + ", ".join(f"{k}={v}" for k, v in reject_counts.items())
        )

        if best_pos is None:
            self.get_logger().info("No feasible grasp found")
            return

        self.get_logger().info(
            f"Selected grasp at {best_pos} | score: {best_score:.3f}"
        )

        quat = R.from_matrix(best_rot).as_quat()

        pose_msg = PoseStamped()
        pose_msg.header = pc_msg.header
        pose_msg.pose.position.x = float(best_pos[0])
        pose_msg.pose.position.y = float(best_pos[1])
        pose_msg.pose.position.z = float(best_pos[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])

        self.grasp_pub.publish(pose_msg)

        markers = self.create_gripper_marker(
            best_pos, best_rot, pc_msg.header.frame_id, pc_msg.header.stamp
        )
        self.marker_pub.publish(markers)


# ------------------------
# Main
# ------------------------
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