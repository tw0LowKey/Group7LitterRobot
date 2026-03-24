import os
os.environ["OPEN3D_CPU_RENDERING"] = "true"
os.environ["DISPLAY"] = ""
import time
import cv2
import numpy as np
import rclpy
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import torch
from geometry_msgs.msg import PoseStamped

torch.cuda.empty_cache()


def build_raw_cloud_msg(header, points, instance_ids, colors_rgba):
    """Full cloud with x, y, z, rgba, instance_id — used for /cloud/masks_raw."""
    cloud_dtype = np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('rgba', np.uint32), ('instance_id', np.int32)
    ])
    cloud_array = np.zeros(points.shape[0], dtype=cloud_dtype)
    cloud_array['x'] = points[:, 0]
    cloud_array['y'] = points[:, 1]
    cloud_array['z'] = points[:, 2]
    cloud_array['instance_id'] = instance_ids

    r = colors_rgba[:, 0].astype(np.uint32)
    g = colors_rgba[:, 1].astype(np.uint32)
    b = colors_rgba[:, 2].astype(np.uint32)
    a = np.full(len(r), 255, dtype=np.uint32)
    cloud_array['rgba'] = (a << 24) | (r << 16) | (g << 8) | b

    fields = [
        PointField(name='x',           offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',           offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',           offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='rgba',        offset=12, datatype=PointField.UINT32,  count=1),
        PointField(name='instance_id', offset=16, datatype=PointField.INT32,   count=1),
    ]
    return pc2.create_cloud(header, fields, cloud_array)


def build_clean_cloud_msg(header, points, instance_ids):
    """Minimal cloud with x, y, z, instance_id only — used for /cloud/masks_clean."""
    cloud_dtype = np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('instance_id', np.int32)
    ])
    cloud_array = np.zeros(points.shape[0], dtype=cloud_dtype)
    cloud_array['x'] = points[:, 0]
    cloud_array['y'] = points[:, 1]
    cloud_array['z'] = points[:, 2]
    cloud_array['instance_id'] = instance_ids

    fields = [
        PointField(name='x',           offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',           offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',           offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='instance_id', offset=12, datatype=PointField.INT32,   count=1),
    ]
    return pc2.create_cloud(header, fields, cloud_array)


def clean_point_cloud_o3d(points_xyz, nb_neighbors=10, std_ratio=2.0,
                           voxel_size=0.005, min_points=1000):
    if points_xyz.shape[0] == 0:
        return points_xyz

    pc_o3d = o3d.geometry.PointCloud()
    pc_o3d.points = o3d.utility.Vector3dVector(points_xyz)

    # Statistical outlier removal
    pc_clean, _ = pc_o3d.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio
    )

    # Adaptive voxel size — if the fixed voxel size would leave fewer than
    # min_points, halve the voxel size until the target is met or a floor
    # of 0.001 m (1 mm) is reached.  This preserves detail on small objects
    # without changing behaviour for large ones.
    pts_after_outlier = len(pc_clean.points)

    actual_neighbors = min(nb_neighbors, max(1, pts_after_outlier // 5))
    pc_clean, _ = pc_o3d.remove_statistical_outlier(
        nb_neighbors=actual_neighbors, std_ratio=std_ratio
    )

    adaptive_voxel = voxel_size
    while adaptive_voxel > 0.001:
        pc_down = pc_clean.voxel_down_sample(adaptive_voxel)
        if len(pc_down.points) >= min_points or adaptive_voxel <= 0.001:
            break
        adaptive_voxel /= 2.0
    
    pc_clean, _ = pc_o3d.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio
    )

    return np.asarray(pc_down.points)

class RGBDepthNode(Node):
    def __init__(self):
        super().__init__('rgb_depth_node')

        self.bridge = CvBridge()
        model_path = os.path.join(get_package_share_directory('computer_vision'), 'models', 'Model_1_x2.engine')
        self.model = YOLO(model_path, task='segment')
        self.rgb_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')
        self.info_sub = Subscriber(self, CameraInfo, '/camera/color/camera_info')

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub], queue_size=10, slop=0.03
        )
        self.sync.registerCallback(self.synced_callback)

        self.pc_pub_raw = self.create_publisher(PointCloud2, '/cloud/masks_raw', 10)
        self.pc_pub_clean = self.create_publisher(PointCloud2, '/cloud/masks_clean', 10)
        self.contour_mask_pub = self.create_publisher(Image, '/camera/masks/contours', 10)

        # NEW: publisher for the Scout Mini
        self.centroid_pub = self.create_publisher(PoseStamped, '/camera/litter_pose', 10)

        self.last_callback_time = None

    def _is_transparent(self, mask, depth, fx, fy, cx, cy, pts_raw, expand_px=15, z_threshold=0.04):
        instance_mean_z = float(np.mean(pts_raw[:, 2]))

        mask_uint8 = mask.astype(np.uint8)
        kernel = np.ones((expand_px * 2 + 1, expand_px * 2 + 1), np.uint8)
        dilated = cv2.dilate(mask_uint8, kernel)
        ring = (dilated - mask_uint8).astype(bool)

        ys, xs = np.nonzero(ring)
        bg_z = depth[ys, xs]
        bg_z = bg_z[(bg_z > 0) & (bg_z > instance_mean_z - 0.05)]
        if bg_z.size < 10:
            return False

        bg_X = (xs - cx) * bg_z / fx
        bg_Y = (ys - cy) * bg_z / fy
        bg_pts = np.stack([bg_X, bg_Y, bg_z], axis=1)

        best_inlier_mean_z = None
        best_count = 0
        n = bg_pts.shape[0]
        for _ in range(50):
            if n < 3:
                break
            idx = np.random.choice(n, 3, replace=False)
            p0, p1, p2 = bg_pts[idx[0]], bg_pts[idx[1]], bg_pts[idx[2]]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal /= norm
            d = -np.dot(normal, p0)
            dists = np.abs(bg_pts @ normal + d)
            inliers = dists < 0.01
            count = np.sum(inliers)
            if count > best_count:
                best_count = count
                best_inlier_mean_z = float(np.mean(bg_pts[inliers, 2]))

        if best_inlier_mean_z is None:
            return False

        return abs(instance_mean_z - best_inlier_mean_z) < z_threshold
    
    def synced_callback(self, rgb_msg, depth_msg, info_msg):
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth_msg.encoding == '16UC1':
            depth = depth / 1000.0

        fx, fy, cx, cy = info_msg.k[0], info_msg.k[4], info_msg.k[2], info_msg.k[5]

        rgb_for_color = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

        results = self.model.track(
            rgb, conf=0.5, verbose=False, imgsz=1280, retina_masks=True, batch=1, persist=True
        )
        masks_full = results[0].masks.data.cpu().numpy().astype(bool) if results[0].masks else []

        ids = results[0].boxes.id
        if ids is None:
            ids = range(1, len(masks_full) + 1)

        instance_mask_contour = np.zeros(depth.shape, dtype=np.uint16)
        raw_points_list, raw_ids_list, raw_colors_list = [], [], []
        clean_points_list, clean_ids_list = [], []

        for mask, track_id in zip(masks_full, ids):
            i = int(track_id)

            ys, xs = np.nonzero(mask)
            Z = depth[ys, xs]
            valid = Z > 0
            if np.count_nonzero(valid) == 0:
                continue

            xs, ys, Z = xs[valid], ys[valid], Z[valid]
            X = (xs - cx) * Z / fx
            Y = (ys - cy) * Z / fy
            pts_raw = np.stack((X, Y, Z), axis=-1)
            colors_raw = rgb_for_color[ys, xs]

            is_transparent = self._is_transparent(mask, depth, fx, fy, cx, cy, pts_raw)
            self.get_logger().info(f"Instance {i}: {'TRANSPARENT' if is_transparent else 'opaque'} | mean_z={np.mean(pts_raw[:,2]):.3f}m")

            raw_points_list.append(pts_raw)
            raw_ids_list.append(np.full(pts_raw.shape[0], i, dtype=np.int32))
            raw_colors_list.append(colors_raw)

            pts_clean = clean_point_cloud_o3d(pts_raw)
            if pts_clean.shape[0] > 0:
                clean_points_list.append(pts_clean)
                clean_ids_list.append(np.full(pts_clean.shape[0], i, dtype=np.int32))

                # --- NEW: CALCULATE AND PUBLISH THE CENTROID FOR THE SCOUT MINI ---
                centroid = np.mean(pts_clean, axis=0) # Calculates the exact middle [X, Y, Z]

                pose_msg = PoseStamped()
                pose_msg.header = rgb_msg.header      # Automatically gets 'camera_color_frame' and exact timestamp
                
                pose_msg.pose.position.x = float(centroid[0])
                pose_msg.pose.position.y = float(centroid[1])
                pose_msg.pose.position.z = float(centroid[2])
                
                # Quaternions must be valid. w=1.0 means "no rotation relative to camera"
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
                pose_msg.pose.orientation.w = 1.0
                
                self.centroid_pub.publish(pose_msg)
                # -----------------------------------------------------------------

            # 2D contours for visualization
            mask_uint8 = mask.astype(np.uint8)
            contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                cv2.fillPoly(instance_mask_contour, [largest_contour], color=i)

        # Publish RAW cloud (retains colour)
        if raw_points_list:
            raw_pts = np.concatenate(raw_points_list, axis=0)
            raw_ids = np.concatenate(raw_ids_list, axis=0)
            raw_colors = np.concatenate(raw_colors_list, axis=0)
            self.pc_pub_raw.publish(build_raw_cloud_msg(rgb_msg.header, raw_pts, raw_ids, raw_colors))

        # Publish CLEAN cloud (x, y, z, instance_id only)
        if clean_points_list:
            clean_pts = np.concatenate(clean_points_list, axis=0)
            clean_ids = np.concatenate(clean_ids_list, axis=0)
            self.pc_pub_clean.publish(build_clean_cloud_msg(rgb_msg.header, clean_pts, clean_ids))

        # Publish contour mask
        mask_msg = self.bridge.cv2_to_imgmsg(instance_mask_contour, encoding='mono16')
        mask_msg.header = rgb_msg.header
        self.contour_mask_pub.publish(mask_msg)

        # Logging
        current_time = time.time()
        raw_n = int(sum(arr.shape[0] for arr in raw_points_list)) if raw_points_list else 0
        clean_n = int(sum(arr.shape[0] for arr in clean_points_list)) if clean_points_list else 0
        if self.last_callback_time is not None:
            dt = current_time - self.last_callback_time
            self.get_logger().info(f"Callback: {dt:.4f}s | raw pts: {raw_n} | clean pts: {clean_n}")
        else:
            self.get_logger().info(f"First callback | raw pts: {raw_n} | clean pts: {clean_n}")
        self.last_callback_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = RGBDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
