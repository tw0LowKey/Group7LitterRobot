import os
import time
import cv2
import numpy as np
import rclpy
import open3d as o3d
import sensor_msgs_py.point_cloud2 as pc2
import torch
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException

os.environ["OPEN3D_CPU_RENDERING"] = "true"
os.environ["DISPLAY"] = ""
torch.cuda.empty_cache()
os.environ["PYTORCH_NO_CUDA_MEMORY_CACHING"] = "1"

CONTOUR_PALETTE = [
    (255,  64,  64), ( 64, 255,  64), ( 64,  64, 255),
    (255, 255,   0), (  0, 255, 255), (255,   0, 255),
    (255, 165,   0), (128,   0, 255),
]


def build_raw_cloud_msg(header, points, instance_ids, colors_rgba):
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


def clean_point_cloud_o3d(points_xyz, nb_neighbors=10, std_ratio=2.0, voxel_size=0.005, min_points=1000):
    if points_xyz.shape[0] == 0:
        return points_xyz
    pc_o3d = o3d.geometry.PointCloud()
    pc_o3d.points = o3d.utility.Vector3dVector(points_xyz)
    pc_clean, _ = pc_o3d.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    pts_after = len(pc_clean.points)
    actual_neighbors = min(nb_neighbors, max(1, pts_after // 5))
    pc_clean, _ = pc_o3d.remove_statistical_outlier(nb_neighbors=actual_neighbors, std_ratio=std_ratio)

    adaptive_voxel = voxel_size
    while adaptive_voxel > 0.001:
        pc_down = pc_clean.voxel_down_sample(adaptive_voxel)
        if len(pc_down.points) >= min_points:
            break
        adaptive_voxel /= 2.0
    return np.asarray(pc_down.points)


class RGBDepthNode(Node):
    def __init__(self):
        super().__init__('rgb_depth_node')
        self.bridge = CvBridge()
        self.enable_3d = True  # Toggle to True to re-enable 3D pipeline

        model_path = os.path.join(
            get_package_share_directory('computer_vision'),
            'models', 'Model_1_engine_2.engine'
        )
        self.model = YOLO(model_path, task='segment')

        self.rgb_sub   = Subscriber(self, Image,      '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image,      '/camera/depth/image_raw')
        self.info_sub  = Subscriber(self, CameraInfo, '/camera/color/camera_info')

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.info_sub], queue_size=3, slop=0.2
        )
        self.sync.registerCallback(self.synced_callback)

        self.pc_pub_raw       = self.create_publisher(PointCloud2, '/cloud/masks_raw',       10)
        self.pc_pub_clean     = self.create_publisher(PointCloud2, '/cloud/masks_clean',     10)
        self.contour_mask_pub = self.create_publisher(Image,       '/camera/masks/contours', 10)
        self.centroid_pub     = self.create_publisher(PoseStamped, '/camera/litter_pose',    10)

        self.last_callback_time = None
        self._score_stats = {}  # track_id -> {'sum': float, 'count': int}

    def synced_callback(self, rgb_msg, depth_msg, info_msg):
        self.get_logger().info("Entered RGB Callback")
        rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth_msg.encoding == '16UC1':
            depth = depth / 1000.0

        fx, fy = info_msg.k[0], info_msg.k[4]
        cx, cy = info_msg.k[2], info_msg.k[5]
        rgb_for_color = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

        results    = self.model.track(rgb, conf=0.5, verbose=False, imgsz=1280,
                                      retina_masks=True, batch=1, persist=True)
        masks_full = results[0].masks.data.cpu().numpy().astype(bool) if results[0].masks else []
        ids        = (results[0].boxes.id
                      if results[0].boxes.id is not None
                      else range(1, len(masks_full) + 1))
        scores     = (results[0].boxes.conf.cpu().numpy()
                      if results[0].boxes.conf is not None
                      else np.ones(len(masks_full)))

        instance_mask_contour = rgb_for_color.copy()
        raw_pts_l, raw_ids_l, raw_cols_l = [], [], []
        clean_pts_l, clean_ids_l         = [], []

        self.get_logger().info("Mask Loop")

        for mask, track_id, score in zip(masks_full, ids, scores):
            i = int(track_id)

            # --- 2D contour overlay (always runs) ---
            contours, _ = cv2.findContours(
                mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if contours:
                colour = CONTOUR_PALETTE[i % len(CONTOUR_PALETTE)]
                cv2.drawContours(
                    instance_mask_contour,
                    [max(contours, key=cv2.contourArea)],
                    contourIdx=-1, color=colour, thickness=3
                )

            # --- 3D pipeline (gated) ---
            if self.enable_3d:
                ys, xs = np.nonzero(mask)
                Z      = depth[ys, xs]
                valid  = Z > 0
                if np.any(valid):
                    xs, ys, Z = xs[valid], ys[valid], Z[valid]
                    pts_raw   = np.stack(
                        ((xs - cx) * Z / fx, (ys - cy) * Z / fy, Z), axis=-1
                    )

                    raw_pts_l.append(pts_raw)
                    raw_ids_l.append(np.full(pts_raw.shape[0], i, dtype=np.int32))
                    raw_cols_l.append(rgb_for_color[ys, xs])

                    pts_clean = clean_point_cloud_o3d(pts_raw)
                    if pts_clean.shape[0] > 0:
                        clean_pts_l.append(pts_clean)
                        clean_ids_l.append(np.full(pts_clean.shape[0], i, dtype=np.int32))

                        centroid = np.mean(pts_clean, axis=0)
                        pose_msg = PoseStamped()
                        pose_msg.header          = rgb_msg.header
                        pose_msg.pose.position.x = float(centroid[0])
                        pose_msg.pose.position.y = float(centroid[1])
                        pose_msg.pose.position.z = float(centroid[2])
                        pose_msg.pose.orientation.w = 1.0
                        self.centroid_pub.publish(pose_msg)

            # --- Score stats (always runs) ---
            stats = self._score_stats.setdefault(i, {'sum': 0.0, 'count': 0})
            stats['sum']   += float(score)
            stats['count'] += 1
            if stats['count'] % 30 == 0:
                avg = stats['sum'] / stats['count']
                self.get_logger().info(
                    f"Instance {i} | avg_score ({stats['count']} frames): {avg:.3f}"
                )

        # Reset stats for instances no longer tracked
        active_ids = {int(t) for t in ids}
        for stale_id in list(self._score_stats.keys()):
            if stale_id not in active_ids:
                del self._score_stats[stale_id]

        # --- Cloud publishes (gated) ---
        if self.enable_3d and raw_pts_l:
            self.pc_pub_raw.publish(build_raw_cloud_msg(
                rgb_msg.header,
                np.concatenate(raw_pts_l),
                np.concatenate(raw_ids_l),
                np.concatenate(raw_cols_l)
            ))
        if self.enable_3d and clean_pts_l:
            self.pc_pub_clean.publish(build_clean_cloud_msg(
                rgb_msg.header,
                np.concatenate(clean_pts_l),
                np.concatenate(clean_ids_l)
            ))

        # --- Contour image publish (always runs) ---
        mask_msg        = self.bridge.cv2_to_imgmsg(instance_mask_contour, encoding='rgb8')
        mask_msg.header = rgb_msg.header
        self.contour_mask_pub.publish(mask_msg)

        # --- Timing log ---
        torch.cuda.empty_cache()
        current_time = time.time()
        if self.last_callback_time is not None:
            dt = current_time - self.last_callback_time
            self.get_logger().info(f"Callback: {dt:.4f}s | 3D: {'on' if self.enable_3d else 'off'}")
        else:
            self.get_logger().info(f"First callback | 3D: {'on' if self.enable_3d else 'off'}")
        self.last_callback_time = current_time
        self.get_logger().info("Mask End")

def main(args=None):
    rclpy.init(args=args)
    node = RGBDepthNode()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()