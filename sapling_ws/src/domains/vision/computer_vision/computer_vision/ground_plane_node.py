import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PointStamped
import sensor_msgs_py.point_cloud2 as pc2
from ament_index_python.packages import get_package_share_directory

# THIS HAS BEEN UPDATED, BUT NOT RUN TO TEST DUE TO PRE-EXISTENCE OF A GOOD GROUND PLANE, RUN FOR TESTING IF NEEDED.

PLANE_SAVE_PATH = os.path.join(get_package_share_directory('computer_vision'), 'ground_plane.npy')

RANSAC_ITERATIONS = 100
RANSAC_INLIER_THRESHOLD = 0.01
RANSAC_MIN_INLIER_RATIO = 0.3
GROUND_Y_FRACTION = 0.4
MIN_POINTS = 100
FRAMES_TO_AVERAGE = 5

class GroundPlaneNode(Node):
    def __init__(self):
        super().__init__('ground_plane_node')

        self._plane_coeffs = None
        self._candidate_planes = []
        self._calibrated = False

        # Publishers
        self.plane_pub = self.create_publisher(Float64MultiArray, '/ground_plane/coefficients', 10)
        self.normal_pub = self.create_publisher(PointStamped, '/ground_plane/normal_origin', 10)

        # Load saved plane if exists
        if os.path.exists(PLANE_SAVE_PATH):
            self._plane_coeffs = np.load(PLANE_SAVE_PATH)
            self._calibrated = True
            self.get_logger().info(
                f"Loaded ground plane: [a={self._plane_coeffs[0]:.4f}, "
                f"b={self._plane_coeffs[1]:.4f}, c={self._plane_coeffs[2]:.4f}, "
                f"d={self._plane_coeffs[3]:.4f}]"
            )
        else:
            self.get_logger().info("No saved ground plane found — calibrating from /camera/depth/points")

        self.pc_sub = self.create_subscription(PointCloud2, '/camera/depth/points', self.pc_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_plane)

    # --------------------------
    # RANSAC plane fit
    # --------------------------
    def _fit_plane_ransac(self, points):
        best_inliers = None
        best_count = 0
        n = points.shape[0]
        if n < 3:
            return None

        for _ in range(RANSAC_ITERATIONS):
            idx = np.random.choice(n, 3, replace=False)
            p0, p1, p2 = points[idx[0]], points[idx[1]], points[idx[2]]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal /= norm
            d = -np.dot(normal, p0)
            dists = np.abs(points @ normal + d)
            inliers = dists < RANSAC_INLIER_THRESHOLD
            count = np.sum(inliers)

            if count > best_count:
                best_count = count
                best_inliers = inliers

        if best_inliers is None or best_count / n < RANSAC_MIN_INLIER_RATIO:
            return None

        # Refit using PCA
        inlier_pts = points[best_inliers]
        centroid = inlier_pts.mean(axis=0)
        cov = (inlier_pts - centroid).T @ (inlier_pts - centroid)
        _, _, Vt = np.linalg.svd(cov)
        normal = Vt[-1]

        # Flip normal to point upward (optical frame)
        if normal[1] > 0:
            normal = -normal

        d = -np.dot(normal, centroid)
        return normal, d, best_inliers

    # --------------------------
    # Point cloud callback
    # --------------------------
    def pc_callback(self, pc_msg):
        if self._calibrated and len(self._candidate_planes) == 0:
            return

        points_list = list(pc2.read_points(pc_msg, field_names=("x","y","z"), skip_nans=True))
        if len(points_list) < MIN_POINTS:
            return

        points = np.array([[p[0],p[1],p[2]] for p in points_list], dtype=np.float64)
        y_threshold = np.percentile(points[:, 1], (1.0 - GROUND_Y_FRACTION) * 100)
        ground_candidates = points[points[:, 1] >= y_threshold]
        if ground_candidates.shape[0] < 3:
            return

        result = self._fit_plane_ransac(ground_candidates)
        if result is None:
            return

        normal, d, _ = result
        self._candidate_planes.append(np.array([normal[0], normal[1], normal[2], d]))
        self.get_logger().info(f"Collected plane sample {len(self._candidate_planes)}/{FRAMES_TO_AVERAGE}")

        if len(self._candidate_planes) >= FRAMES_TO_AVERAGE:
            avg_coeffs = np.mean(self._candidate_planes, axis=0)
            avg_coeffs[:3] /= np.linalg.norm(avg_coeffs[:3])
            self._plane_coeffs = avg_coeffs
            self._calibrated = True
            self._candidate_planes = []
            np.save(PLANE_SAVE_PATH, self._plane_coeffs)

            self.get_logger().info(
                f"Ground plane calibrated and saved: [a={avg_coeffs[0]:.4f}, b={avg_coeffs[1]:.4f}, "
                f"c={avg_coeffs[2]:.4f}, d={avg_coeffs[3]:.4f}]"
            )
            cam_up = np.array([0.0,-1.0,0.0])
            tilt_deg = float(np.degrees(np.arccos(np.clip(np.dot(avg_coeffs[:3], cam_up),-1.0,1.0))))
            self.get_logger().info(f"Camera tilt from horizon: {tilt_deg:.2f} degrees")

    # --------------------------
    # Publish plane
    # --------------------------
    def publish_plane(self):
        if self._plane_coeffs is None:
            return

        msg = Float64MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label='plane_coefficients', size=4, stride=4))
        msg.data = self._plane_coeffs.tolist()
        self.plane_pub.publish(msg)

        normal = self._plane_coeffs[:3]
        d = self._plane_coeffs[3]
        origin = -d*normal
        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'camera_color_optical_frame'
        ps.point.x = float(origin[0])
        ps.point.y = float(origin[1])
        ps.point.z = float(origin[2])
        self.normal_pub.publish(ps)


def main(args=None):
    rclpy.init(args=args)
    node = GroundPlaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()