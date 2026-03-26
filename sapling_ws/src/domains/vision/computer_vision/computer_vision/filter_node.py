import os
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory
import sensor_msgs_py.point_cloud2 as pc2


REACHABILITY_RADIUS = 0.6
GROUND_PLANE_OFFSET = 0.01
_CAMERA_TRANSLATION = np.array([0.14, 0.13, 0.600])

PROCESS_EVERY_N = 3


def build_selected_cloud_msg(header, points, instance_ids):
    cloud_dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('instance_id', np.int32)
    ])

    cloud_array = np.zeros(points.shape[0], dtype=cloud_dtype)
    cloud_array['x'] = points[:, 0]
    cloud_array['y'] = points[:, 1]
    cloud_array['z'] = points[:, 2]
    cloud_array['instance_id'] = instance_ids

    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='instance_id', offset=12, datatype=PointField.INT32, count=1),
    ]

    return pc2.create_cloud(header, fields, cloud_array)


def build_centroid_cloud_msg(header, centroids, instance_ids):
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='instance_id', offset=12, datatype=PointField.INT32, count=1),
    ]

    points = [
        (
            float(c[0]),
            float(c[1]),
            float(c[2]),
            int(uid)
        )
        for c, uid in zip(centroids, instance_ids)
    ]

    return pc2.create_cloud(header, fields, points)


class FilterNode(Node):
    def __init__(self):
        super().__init__('filter_node')

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

        plane_path = os.path.join(
            get_package_share_directory('computer_vision'),
            'ground_plane.npy'
        )

        if os.path.exists(plane_path):
            self._ground_plane = np.load(plane_path)
            self._ground_plane[3] += GROUND_PLANE_OFFSET

            a, b, c, _ = self._ground_plane
            self._up = np.array([a, b, c], dtype=float)
            self._up /= np.linalg.norm(self._up)

            self.get_logger().info(
                f"Loaded ground plane: {self._ground_plane}"
            )

        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/cloud/masks_clean',
            self.pc_callback,
            1
        )

        self.selected_mask_pub = self.create_publisher(PointCloud2, '/selected_mask', 1)
        self.centroids_pub = self.create_publisher(PointCloud2, '/cloud/centroids', 10)
        self.sphere_marker_pub = self.create_publisher(Marker, '/reachability_sphere', 10)
        self.base_link_markers_pub = self.create_publisher(MarkerArray, '/base_link_debug', 10)
        self.ground_marker_pub = self.create_publisher(Marker, '/ground_marker', 10)

        self._startup_count = 0
        self._startup_timer = self.create_timer(0.5, self._startup_publish)

        self.get_logger().info("FilterNode started")

    def _startup_publish(self):
        self._startup_count += 1

        if self._startup_count > 10:
            self._startup_timer.cancel()
            return

        header = self._make_static_header()

        self.sphere_marker_pub.publish(self._build_sphere_marker(header))
        self.base_link_markers_pub.publish(self._build_base_link_debug_markers(header))

        if self._ground_plane is not None:
            self.ground_marker_pub.publish(self._build_ground_plane_marker(header))

    def _make_static_header(self):
        from std_msgs.msg import Header

        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'camera_color_optical_frame'
        return h

    def _build_ground_plane_marker(self, header):
        a, b, c, d = self._ground_plane
        normal = np.array([a, b, c])
        origin = -d * normal

        marker = Marker()
        marker.header = header
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
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            axis /= axis_norm
            angle = np.arccos(np.clip(np.dot(world_z, normal), -1.0, 1.0))
            s = np.sin(angle / 2.0)

            qx = axis[0] * s
            qy = axis[1] * s
            qz = axis[2] * s
            qw = np.cos(angle / 2.0)

        marker.pose.orientation.x = float(qx)
        marker.pose.orientation.y = float(qy)
        marker.pose.orientation.z = float(qz)
        marker.pose.orientation.w = float(qw)

        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 0.002

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.4

        return marker

    def _build_base_link_debug_markers(self, header):
        markers = []

        up = self._up
        cam_z = np.array([0.0, 0.0, 1.0])

        horiz_forward = cam_z - np.dot(cam_z, up) * up
        norm = np.linalg.norm(horiz_forward)

        if norm > 1e-6:
            horiz_forward /= norm

        p0 = np.array([0.0, 0.0, 0.0])
        p1 = p0 + (-up) * 0.6
        p2 = p1 + (-horiz_forward) * 0.14
        p3 = p2 + np.array([0.14, 0.0, 0.0])

        self._base_pos_in_camera = p3.copy()

        def line_marker(mid, p_start, p_end, r, g, b):
            m = Marker()
            m.header = header
            m.ns = "base_link_debug"
            m.id = mid
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD

            m.scale.x = 0.005

            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = 1.0

            pt1 = Point()
            pt1.x = float(p_start[0])
            pt1.y = float(p_start[1])
            pt1.z = float(p_start[2])

            pt2 = Point()
            pt2.x = float(p_end[0])
            pt2.y = float(p_end[1])
            pt2.z = float(p_end[2])

            m.points = [pt1, pt2]
            return m

        markers.append(line_marker(0, p0, p1, 0.0, 0.0, 1.0))
        markers.append(line_marker(1, p1, p2, 1.0, 0.0, 0.0))
        markers.append(line_marker(2, p2, p3, 0.0, 1.0, 0.0))

        ma = MarkerArray()
        ma.markers = markers
        return ma

    def _build_sphere_marker(self, header):
        m = Marker()
        m.header = header
        m.ns = "reachability_sphere"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position.x = float(self._base_pos_in_camera[0])
        m.pose.position.y = float(self._base_pos_in_camera[1])
        m.pose.position.z = float(self._base_pos_in_camera[2])

        m.pose.orientation.w = 1.0

        m.scale.x = REACHABILITY_RADIUS * 2.0
        m.scale.y = REACHABILITY_RADIUS * 2.0
        m.scale.z = REACHABILITY_RADIUS * 2.0

        m.color.r = 0.0
        m.color.g = 0.6
        m.color.b = 1.0
        m.color.a = 0.1

        return m

    def pc_callback(self, pc_msg):
        self._msg_count += 1

        if self._msg_count % PROCESS_EVERY_N != 0:
            return

        self.sphere_marker_pub.publish(self._build_sphere_marker(pc_msg.header))
        self.base_link_markers_pub.publish(self._build_base_link_debug_markers(pc_msg.header))

        if self._ground_plane is not None:
            self.ground_marker_pub.publish(self._build_ground_plane_marker(pc_msg.header))

        points_list = list(pc2.read_points(
            pc_msg,
            field_names=("x", "y", "z", "instance_id"),
            skip_nans=True
        ))

        if not points_list:
            return

        xyz = np.array([[p[0], p[1], p[2]] for p in points_list], dtype=np.float32)
        raw_ids = np.array([p[3] for p in points_list], dtype=np.float32)
        instance_ids = np.round(raw_ids).astype(np.int32)

        unique_ids = np.unique(instance_ids)

        self.get_logger().info(f"Rounded unique ids: {unique_ids}")

        selected_points = None
        selected_id = None

        outside_centroids = []
        outside_ids = []

        for uid in unique_ids:
            candidate_pts = xyz[instance_ids == uid]
            # candidate_pts = candidate_pts[candidate_pts[:, 2] < 1.0]

            if candidate_pts.shape[0] == 0:
                continue

            centroid = candidate_pts.mean(axis=0)
            dist = np.linalg.norm(centroid - self._base_pos_in_camera)

            if dist <= REACHABILITY_RADIUS:
                if selected_points is None:
                    selected_points = candidate_pts
                    selected_id = uid

                    self.get_logger().info(
                        f"Instance {uid} selected | dist={dist:.3f}"
                    )
            else:
                outside_centroids.append(centroid)
                outside_ids.append(uid)

                self.get_logger().info(
                    f"Instance {uid} outside | dist={dist:.3f}"
                )

        if selected_points is not None:
            ids_array = np.full(
                selected_points.shape[0],
                selected_id,
                dtype=np.int32
            )

            self.selected_mask_pub.publish(
                build_selected_cloud_msg(
                    pc_msg.header,
                    selected_points,
                    ids_array
                )
            )

        if outside_centroids:
            centroid_array = np.array(outside_centroids, dtype=np.float32)
            id_array = np.array(outside_ids, dtype=np.int32)

            self.centroids_pub.publish(
                build_centroid_cloud_msg(
                    pc_msg.header,
                    centroid_array,
                    id_array
                )
            )

            self.get_logger().info(
                f"Published {len(outside_centroids)} centroids"
            )


def main(args=None):
    rclpy.init(args=args)

    node = FilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()