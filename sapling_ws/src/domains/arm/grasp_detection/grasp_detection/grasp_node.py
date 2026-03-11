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

# GRIPPER_BODY_DEPTH = GRIPPER_PALM_THICK + shaft_depth
# => shaft_depth = GRIPPER_BODY_DEPTH - GRIPPER_PALM_THICK = 0.121
GRIPPER_SHAFT_DEPTH = GRIPPER_BODY_DEPTH - GRIPPER_PALM_THICK  # 0.121

# Palm extends further in Y than the fingers do.
GRIPPER_PALM_WIDTH_Y  = 0.09   # 9 cm — full palm depth in the Y axis

# Shaft is a cylinder whose axis runs along local Z.
# The closing axis (X) gives the diameter: 6 cm -> radius = 3 cm.
GRIPPER_SHAFT_RADIUS  = 0.04   # 3 cm — radius of the cylindrical body

# Thickness of the contact slab just inside each finger's inner face used by
# score_grasp.  Points within this slab on either side contribute to body_score.
# A value of ~1/7 of the jaw half-width works well — wide enough to catch
# surface points on a curved object without reaching the jaw centre.
GRASP_CONTACT_DEPTH = 0.01  # 1 cm

# Scoring weights
W_BODY        = 0.5
W_SYM         = 0.2
W_PENETRATION = 0.2
W_FRICTION    = 0.1

TOP_K = 1
PROCESS_EVERY_N = 3


def _local_to_world_pos(local_offset: np.ndarray, grasp_pos: np.ndarray, grasp_rot: np.ndarray) -> np.ndarray:
    """Transform a local-frame offset into a world-frame position."""
    return grasp_pos + grasp_rot @ local_offset


def _make_cube_marker(header, ns, marker_id, world_pos, quat, scale_xyz, rgba):
    """Helper to build a CUBE Marker."""
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

        # Subscribers / Publishers
        self.pc_sub = self.create_subscription(
            PointCloud2, '/cloud/masks_clean', self.pc_callback, 10
        )
        self.grasp_pub = self.create_publisher(PoseStamped, '/grasp_pose', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/grasp_markers', 10)
        self.ground_marker_pub = self.create_publisher(Marker, '/ground_marker', 10)

        self.get_logger().info("RobustGraspNode started")

    # -------------------------
    # Feasibility
    # -------------------------
    def _feasibility_reason(self, points, grasp_pos, grasp_rot):
        pts_local = (points - grasp_pos) @ grasp_rot
        x, y, z = pts_local[:, 0], pts_local[:, 1], pts_local[:, 2]
        half_w  = GRIPPER_WIDTH_MAX / 2.0
        half_ft = GRIPPER_FINGER_THICK / 2.0
        outer_x = half_w + GRIPPER_FINGER_THICK   # outer X edge of finger / palm

        # ── Reachability checks ───────────────────────────────────────────────
        # These are not collision tests — they verify the object is positioned
        # such that the fingers can engage it at all.

        # 1. At least one point must lie within the finger envelope (z axis).
        in_env = (z >= 0.0) & (z <= GRIPPER_FINGER_LENGTH)
        if pts_local[in_env].shape[0] == 0:
            return 'no_points_in_envelope'

        # 2. At least one point must lie inside the jaw gap (between the fingers).
        in_jaw = (np.abs(x) <= half_w) & (np.abs(y) <= GRIPPER_WIDTH_Y / 2.0)
        pts_in_jaw = pts_local[in_jaw]
        pts_in_jaw_env = pts_in_jaw[
            (pts_in_jaw[:, 2] >= 0.0) & (pts_in_jaw[:, 2] <= GRIPPER_FINGER_LENGTH)
        ]
        if pts_in_jaw_env.shape[0] == 0:
            return 'insufficient_penetration'

        # ── Collision checks ─────────────────────────────────────────────────
        # Each check corresponds directly to a physical gripper part and tests
        # whether any object point lies inside that part's volume.

        # 3. Finger collision — solid volume of either finger block.
        #    X: from inner face (±half_w) to outer face (±outer_x)
        #    Y: finger depth (GRIPPER_WIDTH_Y)
        #    Z: finger length
        left_finger  = (x >= half_w)   & (x <= outer_x)
        right_finger = (x <= -half_w)  & (x >= -outer_x)
        in_finger_y  = np.abs(y) <= GRIPPER_WIDTH_Y / 2.0
        in_finger_z  = (z >= 0.0) & (z <= GRIPPER_FINGER_LENGTH)
        if np.any((left_finger | right_finger) & in_finger_y & in_finger_z):
            return 'finger_collision'

        # 4. Palm collision — rectangular plate behind the finger roots.
        #    X: full outer width (±outer_x)
        #    Y: palm is wider than fingers — uses GRIPPER_PALM_WIDTH_Y
        #    Z: from back of fingers (0) to back of palm (-PALM_THICK)
        in_palm_x = np.abs(x) <= outer_x
        in_palm_y = np.abs(y) <= GRIPPER_PALM_WIDTH_Y / 2.0
        in_palm_z = (z < 0.0) & (z >= -GRIPPER_PALM_THICK)
        if np.any(in_palm_x & in_palm_y & in_palm_z):
            return 'palm_collision'

        # 5. Shaft collision — cylindrical body behind the palm.
        #    The shaft axis runs along local Z; its cross-section is circular
        #    with radius GRIPPER_SHAFT_RADIUS in the X-Y plane.
        #    Z: from back of palm (-PALM_THICK) to back of body (-BODY_DEPTH)
        in_shaft_z      = (z < -GRIPPER_PALM_THICK) & (z >= -GRIPPER_BODY_DEPTH)
        radial_dist     = np.sqrt(x**2 + y**2)
        in_shaft_radial = radial_dist <= GRIPPER_SHAFT_RADIUS
        if np.any(in_shaft_z & in_shaft_radial):
            return 'shaft_collision'

        # 6. Ground collision — bounding box corners of every gripper part
        #    transformed into world space and tested against the floor plane.
        #
        #    Three sets of corners are needed because each part has a different
        #    Y extent:
        #      - Finger tips: Y = GRIPPER_WIDTH_Y        (narrowest)
        #      - Palm faces:  Y = GRIPPER_PALM_WIDTH_Y   (widest — previously missing)
        #      - Shaft back:  Y = GRIPPER_PALM_WIDTH_Y   (bounded by palm width since
        #                                                  the cylinder fits inside it)
        if self._ground_plane is not None:
            a, b, c, d = self._ground_plane
            corners_local = np.array([
                # Finger tip corners
                [ outer_x,  GRIPPER_WIDTH_Y/2,       GRIPPER_FINGER_LENGTH],
                [ outer_x, -GRIPPER_WIDTH_Y/2,       GRIPPER_FINGER_LENGTH],
                [-outer_x,  GRIPPER_WIDTH_Y/2,       GRIPPER_FINGER_LENGTH],
                [-outer_x, -GRIPPER_WIDTH_Y/2,       GRIPPER_FINGER_LENGTH],
                # Palm front corners (z = 0, widest Y extent)
                [ outer_x,  GRIPPER_PALM_WIDTH_Y/2,  0.0],
                [ outer_x, -GRIPPER_PALM_WIDTH_Y/2,  0.0],
                [-outer_x,  GRIPPER_PALM_WIDTH_Y/2,  0.0],
                [-outer_x, -GRIPPER_PALM_WIDTH_Y/2,  0.0],
                # Palm back corners (z = -PALM_THICK, widest Y extent)
                [ outer_x,  GRIPPER_PALM_WIDTH_Y/2, -GRIPPER_PALM_THICK],
                [ outer_x, -GRIPPER_PALM_WIDTH_Y/2, -GRIPPER_PALM_THICK],
                [-outer_x,  GRIPPER_PALM_WIDTH_Y/2, -GRIPPER_PALM_THICK],
                [-outer_x, -GRIPPER_PALM_WIDTH_Y/2, -GRIPPER_PALM_THICK],
                # Shaft back corners (z = -BODY_DEPTH, Y bounded by palm width
                # since the cylinder cross-section fits within the palm footprint)
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

        # Contact score — points near each finger's inner face rather than
        # the full jaw volume.  This discriminates against end-on approaches
        # (e.g. a flat bottle base) where points cluster at x ≈ 0, far from
        # either finger face, and rewards side grasps where the object surface
        # curves toward ±half_w.
        #
        # A thin contact slab of GRASP_CONTACT_DEPTH is defined just inside
        # each finger face.  The score is the fraction of enclosed points that
        # fall in either slab, normalised to [0, 1].  Using a fraction rather
        # than a raw count keeps the score comparable to the other 0-1 terms
        # so the weights remain meaningful regardless of point cloud density.
        in_contact_y = np.abs(enclosed[:, 1]) <= GRIPPER_WIDTH_Y / 2.0
        in_contact_z = (enclosed[:, 2] >= 0.0) & (enclosed[:, 2] <= GRIPPER_FINGER_LENGTH)
        in_contact_yz = in_contact_y & in_contact_z

        left_contact  = (enclosed[:, 0] >= half_w - GRASP_CONTACT_DEPTH) & (enclosed[:, 0] <= half_w)
        right_contact = (enclosed[:, 0] <= -half_w + GRASP_CONTACT_DEPTH) & (enclosed[:, 0] >= -half_w)

        n_left  = float(np.sum(left_contact  & in_contact_yz))
        n_right = float(np.sum(right_contact & in_contact_yz))
        n_contact = n_left + n_right

        # Normalise by enclosed count so sparse and dense clouds are treated
        # equally.  Falls back to 0 if nothing is enclosed.
        body_score = n_contact / (n_enclosed + 1e-6)

        # Symmetry — balance of contact points between the two finger faces.
        # Uses contact counts rather than enclosed counts so it is consistent
        # with the new body score: a grasp with good contact on both sides
        # scores higher than one where only one finger has contact.
        sym_score = 1.0 - abs(n_left - n_right) / (n_contact + 1e-6)

        # Penetration
        # The target penetration is capped at half the object's own extent along
        # the approach axis so short objects are not penalised for failing to
        # reach the default GRIPPER_TARGET_PENETRATION (= FINGER_LENGTH / 2).
        in_jaw = in_volume & (np.abs(y) <= GRIPPER_WIDTH_Y / 2.0)
        pts_in_jaw = pts_local[in_jaw]
        actual_pen = float(pts_in_jaw[:, 2].max()) if pts_in_jaw.shape[0] > 0 else 0.0
        object_extent = float(pts_local[:, 2].max() - pts_local[:, 2].min()) if pts_local.shape[0] > 0 else GRIPPER_FINGER_LENGTH
        adaptive_target = min(GRIPPER_TARGET_PENETRATION, object_extent / 2.0)
        adaptive_target = max(adaptive_target, GRIPPER_MIN_PENETRATION)  # never below minimum
        penetration_score = max(0.0, 1.0 - abs(actual_pen - adaptive_target) / adaptive_target)

        # Friction / approach score.
        # For tall objects a downward approach is preferred (approach_axis[2] high).
        # For short objects near the ground the useful approaches are lateral
        # (approach_axis[2] ~ 0).  We blend between the two based on clearance
        # so lateral grasps are not penalised when they are the only viable option.
        approach_axis = grasp_rot[:, 2]
        clearance = self._ground_clearance(points)
        threshold = GRIPPER_WIDTH_Y / 2.0   # same threshold as candidate generation
        lateral_fraction = float(np.clip(1.0 - clearance / threshold, 0.0, 1.0))
        downward_score = max(0.0, float(approach_axis[2]))
        lateral_score  = max(0.0, 1.0 - abs(float(approach_axis[2])))
        friction_score = (1.0 - lateral_fraction) * downward_score + lateral_fraction * lateral_score

        return (W_BODY * body_score + W_SYM * sym_score
                + W_PENETRATION * penetration_score + W_FRICTION * friction_score)

    def _ground_clearance(self, points: np.ndarray) -> float:
        """
        Return the signed distance of the lowest object point above the ground
        plane — i.e. how much vertical room exists beneath the object.

        This is used to decide how aggressively to bias sampling toward lateral
        approach directions.  The relevant limit is GRIPPER_WIDTH_Y/2: if the
        object sits closer to the ground than half the gripper depth (0.0275 m),
        a lateral approach will have its lower finger edge intersect the ground.
        GRIPPER_BODY_DEPTH is NOT the limiting dimension here — the body extends
        horizontally away from the object for lateral grasps and never approaches
        the ground.

        Returns 1.0 if no ground plane is loaded.
        """
        if self._ground_plane is None:
            return 1.0
        a, b, c, d = self._ground_plane
        normal = np.array([a, b, c])
        dists = points @ normal + d
        return float(np.min(dists))

    def _sample_approach_directions(self, n_samples: int, lateral_fraction: float) -> np.ndarray:
        """
        Sample `n_samples` unit approach vectors.

        `lateral_fraction` in [0, 1] controls how many come from the lateral
        (ground-parallel) band vs the full upper hemisphere:
          0.0  -> all from upper hemisphere  (tall objects, no ground risk)
          1.0  -> all lateral                (very flat objects on the ground)

        Lateral directions lie in the plane perpendicular to the ground
        normal and are given a small random downward tilt (0-15 deg) so the
        gripper approaches the side of the object rather than sliding beneath.
        """
        if self._ground_plane is not None:
            a, b, c, _ = self._ground_plane
            up = np.array([a, b, c], dtype=float)
            up /= np.linalg.norm(up)
        else:
            up = np.array([0.0, 0.0, 1.0])

        n_lateral = int(round(n_samples * lateral_fraction))
        n_hemi    = n_samples - n_lateral
        directions = []

        # Upper-hemisphere directions
        while len(directions) < n_hemi:
            vec = np.random.randn(3)
            if np.dot(vec, up) < 0:
                vec -= 2.0 * np.dot(vec, up) * up   # reflect into upper hemisphere
            norm = np.linalg.norm(vec)
            if norm < 1e-6:
                continue
            directions.append(vec / norm)

        # Lateral directions (parallel to ground + small downward tilt)
        while len(directions) < n_samples:
            vec = np.random.randn(3)
            vec -= np.dot(vec, up) * up              # project onto ground plane
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

        # Ramp lateral sampling based on ground clearance vs finger half-width.
        #
        # For a top-down grasp the fingers point downward to pinch the object
        # from the sides — the body extends upward toward the robot and never
        # threatens the ground.  The problem for short objects is simply that
        # they aren't tall enough to provide a good grip in the jaw.
        #
        # For a lateral grasp the approach axis is horizontal, so GRIPPER_WIDTH_Y
        # spans vertically.  The lower edge of the finger block sits
        # GRIPPER_WIDTH_Y/2 = 0.0275 m below the grasp origin.  Once the object's
        # lowest point is closer to the ground than that, a lateral grasp also
        # clips the floor — so we bias toward lateral approaches while the
        # feasibility checker handles the remaining floor-clipping cases.
        clearance = self._ground_clearance(points)
        threshold = GRIPPER_WIDTH_Y / 2.0   # 0.0275 m
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
        """
        Build the four CUBE markers that visualise a gripper pose.

        Gripper local frame (columns of `rot`):
          local X = rot[:,0]  — finger-spread axis
          local Y = rot[:,1]  — lateral axis
          local Z = rot[:,2]  — approach axis (fingers point in +Z)

        Geometry (all in local frame, origin = grasp_pos):
        ┌──────────────────────────────────────────────────────────┐
        │  local Z                                                 │
        │  ↑                                                       │
        │  GRIPPER_FINGER_LENGTH (0.07)  ← finger tip             │
        │  │                                                       │
        │  │   [left finger]   [right finger]                     │
        │  │   at +X_half       at -X_half                        │
        │  │   centred in Y     centred in Y                      │
        │  │                                                       │
        │  0  ← grasp_pos (origin)                                │
        │  │                                                       │
        │  -GRIPPER_PALM_THICK (−0.024)  ← back of palm          │
        │  │                                                       │
        │  -GRIPPER_BODY_DEPTH (−0.145) ← back of shaft          │
        └──────────────────────────────────────────────────────────┘

        The palm spans the full gripper width in X and GRIPPER_PALM_WIDTH_Y
        (0.09 m) in Y, sitting in Z ∈ [−GRIPPER_PALM_THICK, 0].

        Each finger spans GRIPPER_FINGER_THICK in X (outward from ±X_half),
        GRIPPER_WIDTH_Y in Y, and GRIPPER_FINGER_LENGTH in Z.

        The shaft (body) is a CYLINDER of radius GRIPPER_SHAFT_RADIUS (0.03 m)
        whose axis runs along local Z, spanning Z ∈ [−GRIPPER_BODY_DEPTH, −GRIPPER_PALM_THICK].
        """
        markers = []
        half_w = GRIPPER_WIDTH_MAX / 2.0          # ±X extent of palm / body
        half_ft = GRIPPER_FINGER_THICK / 2.0      # half-thickness of one finger in X

        # ------------------------------------------------------------------
        # Palm  —  local centre at (0, 0, −GRIPPER_PALM_THICK/2)
        # ------------------------------------------------------------------
        palm_local = np.array([0.0, 0.0, -GRIPPER_PALM_THICK / 2.0])
        palm_world = _local_to_world_pos(palm_local, pos, rot)
        markers.append(_make_cube_marker(
            header, "gripper_palm", idx * 10 + 0,
            palm_world, quat,
            scale_xyz=[GRIPPER_WIDTH_MAX + 2 * GRIPPER_FINGER_THICK, GRIPPER_PALM_WIDTH_Y, GRIPPER_PALM_THICK],
            rgba=[0.0, 1.0, 0.0, 0.5],
        ))

        # ------------------------------------------------------------------
        # Left finger  — sits OUTSIDE the 7 cm jaw opening on the +X side.
        #
        # The jaw gap is GRIPPER_WIDTH_MAX (0.07 m), so the inner face of
        # each finger is at ±half_w.  The finger centre in X is therefore:
        #   centre_x = half_w + half_ft   (inner edge at half_w, outer at half_w + FINGER_THICK)
        # It covers Z ∈ [0, GRIPPER_FINGER_LENGTH]:
        #   centre_z = GRIPPER_FINGER_LENGTH / 2
        # ------------------------------------------------------------------
        lf_local = np.array([half_w + half_ft, 0.0, GRIPPER_FINGER_LENGTH / 2.0])
        lf_world = _local_to_world_pos(lf_local, pos, rot)
        markers.append(_make_cube_marker(
            header, "gripper_left_finger", idx * 10 + 1,
            lf_world, quat,
            scale_xyz=[GRIPPER_FINGER_THICK, GRIPPER_WIDTH_Y, GRIPPER_FINGER_LENGTH],
            rgba=[0.0, 0.5, 0.0, 0.5],
        ))

        # ------------------------------------------------------------------
        # Right finger — mirror of left in X
        # ------------------------------------------------------------------
        rf_local = np.array([-(half_w + half_ft), 0.0, GRIPPER_FINGER_LENGTH / 2.0])
        rf_world = _local_to_world_pos(rf_local, pos, rot)
        markers.append(_make_cube_marker(
            header, "gripper_right_finger", idx * 10 + 2,
            rf_world, quat,
            scale_xyz=[GRIPPER_FINGER_THICK, GRIPPER_WIDTH_Y, GRIPPER_FINGER_LENGTH],
            rgba=[0.0, 0.5, 0.0, 0.5],
        ))

        # ------------------------------------------------------------------
        # Shaft (body) — cylindrical, axis along local Z.
        #   centre at (0, 0, −PALM_THICK − SHAFT_DEPTH/2)
        #   RViz CYLINDER scale: x = diameter, y = diameter, z = length
        #   A CYLINDER in RViz is oriented along its local Z axis by default,
        #   so the quaternion from the grasp pose aligns it correctly.
        # ------------------------------------------------------------------
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
        shaft_m.scale.x = GRIPPER_SHAFT_RADIUS * 2   # diameter in X
        shaft_m.scale.y = GRIPPER_SHAFT_RADIUS * 2   # diameter in Y
        shaft_m.scale.z = float(GRIPPER_SHAFT_DEPTH) # length along Z
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
        callback_start = time.time()
        self._msg_count += 1
        if self._msg_count % PROCESS_EVERY_N != 0:
            return

        # Read points
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

        # Generate candidates
        gen_start = time.time()
        candidates = self.generate_candidates(points, n_samples=500, rolls_per_point=10)
        gen_time = time.time() - gen_start

        # Feasibility & scoring
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

        # -------------------------
        # Publish ground plane marker
        # -------------------------
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

            # Orient cube so local +Z = plane normal
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

        # -------------------------
        # Publish gripper markers & poses
        # -------------------------
        marker_array = MarkerArray()
        for idx, (score, pos, rot) in enumerate(top_candidates):
            quat = R.from_matrix(rot).as_quat()  # [x, y, z, w]

            # Pose
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

            # Markers (palm + two fingers + shaft)
            for m in self._build_gripper_markers(pc_msg.header, idx, pos, rot, quat):
                marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)

        callback_elapsed = time.time() - callback_start
        self.get_logger().info(
            f"Grasp callback total time: {callback_elapsed:.3f}s | "
            f"Candidate generation: {gen_time:.3f}s | "
            f"Scoring/filtering: {score_time:.3f}s | "
            f"Published {len(top_candidates)} poses and markers"
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