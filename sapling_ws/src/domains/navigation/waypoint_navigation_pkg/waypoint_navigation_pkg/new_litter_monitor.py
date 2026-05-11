#!/usr/bin/env python3

import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from tf2_ros import Time, TransformBroadcaster, Buffer, TransformListener

import tf2_geometry_msgs  # noqa: F401  # registers PointStamped/PoseStamped TF support

# This node should compare the number of litter items currently in the queue with the number of centroids being published
# If centroids > queue size: trigger a pause request with confirm
# When the confirm is received, the node should translate the centroids currently being published into map co-ordinates
# (only those within pick_radius of base_link) and publish these to /vision/detected_litter


class PointCloudMonitor(Node):
    def __init__(self):
        super().__init__('new_litter_monitor')

        # Frames
        self.base_frame = 'mobile_robot_base_link'
        self.camera_frame = 'camera_link'
        self.map_frame = 'map'

        # TF2 setup (still needed for the dynamic base_link -> map hop)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Build the static camera -> base transform once (used directly for math)
        self.camera_to_base_transform = self.get_camera_transform()

        # Publish the camera TF on a timer so other nodes can see it on the tree
        self.create_timer(0.1, self.publish_camera_tf)

        # State tracking
        self.latest_pc = None
        self.known_queue_size = 0
        self._awaiting_confirm = False

        latching_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Subscriptions
        self.queue_sub = self.create_subscription(
            Int32, '/queue_size', self.queue_cb, 10
        )
        self.pc_sub = self.create_subscription(
            PointCloud2, '/cloud/centroids', self.pc_cb, 10
        )
        self.confirm_sub = self.create_subscription(
            Bool, '/pause_confirmed', self.confirm_cb, latching_qos
        )

        # Publishers
        self.pause_request_pub = self.create_publisher(
            Bool, '/pause_request_with_confirm', latching_qos
        )
        self.nav_control_pub = self.create_publisher(
            Bool, '/start_navigation', latching_qos
        )
        self.litter_pub = self.create_publisher(
            PoseStamped, '/vision/detected_litter', latching_qos
        )

        self.get_logger().info('Litter monitor started, listening to /cloud/centroids')

    def get_camera_transform(self) -> TransformStamped:
        """Hardcoded camera_link -> mobile_robot_base_link transform.
        Same values as SimpleCentroidPrinter."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame    # mobile_robot_base_link
        t.child_frame_id = self.camera_frame   # camera_link

        # Camera position relative to base_link
        t.transform.translation.x = 0.29
        t.transform.translation.y = 0.14
        t.transform.translation.z = 0.648

        # Camera orientation
        t.transform.rotation.x = -0.682
        t.transform.rotation.y = 0.682
        t.transform.rotation.z = -0.191
        t.transform.rotation.w = 0.191

        return t

    def publish_camera_tf(self):
        # Refresh timestamp and rebroadcast the same hardcoded transform
        t = self.camera_to_base_transform
        t.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(t)

    def queue_cb(self, msg):
        self.known_queue_size = msg.data

    def pc_cb(self, msg):
        self.latest_pc = msg
        num_centroids = msg.width * msg.height

        if num_centroids > self.known_queue_size and not self._awaiting_confirm:
            self.get_logger().warn(
                f'Sensed {num_centroids} objects but queue only has '
                f'{self.known_queue_size}. PAUSING.'
            )

            self._awaiting_confirm = True
            pause_cmd = Bool()
            pause_cmd.data = True
            self.pause_request_pub.publish(pause_cmd)

    def confirm_cb(self, msg):
        if msg.data and self.latest_pc and self._awaiting_confirm:
            try:
                self.process_and_publish_centroids()
            finally:
                self._awaiting_confirm = False
                self.pause_request_pub.publish(Bool(data=False))

    def process_and_publish_centroids(self):
        msg = self.latest_pc
        num_centroids = msg.width * msg.height
        published_count = 0

        # Data layout assumed: x(4b), y(4b), z(4b), [instance_id(4b)] - little endian floats
        for i in range(num_centroids):
            offset = i * msg.point_step

            x = struct.unpack_from('<f', msg.data, offset)[0]
            y = struct.unpack_from('<f', msg.data, offset + 4)[0]
            z = struct.unpack_from('<f', msg.data, offset + 8)[0]

            # Wrap raw centroid as a PointStamped in the camera frame
            cam_point = PointStamped()
            cam_point.header.frame_id = (
                msg.header.frame_id if msg.header.frame_id else self.camera_frame
            )
            cam_point.header.stamp = self.get_clock().now().to_msg()
            cam_point.point.x = float(x)
            cam_point.point.y = float(y)
            cam_point.point.z = float(z)

            # Step 1: transform camera_link -> mobile_robot_base_link
            # using the hardcoded transform (no TF lookup, no time issues).
            try:
                point_base = tf2_geometry_msgs.do_transform_point(
                    cam_point, self.camera_to_base_transform
                )
                # Make sure the stamped header reflects the new frame for the next hop
                point_base.header.frame_id = self.base_frame
                point_base.header.stamp = Time().to_msg()  # latest available for next lookup
            except Exception as e:
                self.get_logger().warn(
                    f'Centroid {i}: failed to transform to {self.base_frame}: {e}'
                )
                continue

            distance_2d = math.sqrt(
                point_base.point.x ** 2 + point_base.point.y ** 2
            )

            self.get_logger().info(
                f'Centroid {i} in {self.base_frame}: '
                f'x={point_base.point.x:.3f}, y={point_base.point.y:.3f}, '
                f'z={point_base.point.z:.3f}, dist={distance_2d:.3f} m'
            )

            # Step 2: transform mobile_robot_base_link -> map
            # This one MUST go through TF because base->map is dynamic (localization).
            try:
                point_map = self.tf_buffer.transform(
                    point_base,
                    self.map_frame,
                    timeout=Duration(seconds=1.0),
                )
            except Exception as e:
                self.get_logger().warn(
                    f'Centroid {i}: failed to transform to {self.map_frame}: {e}'
                )
                continue

            map_pose = PoseStamped()
            map_pose.header.frame_id = self.map_frame
            map_pose.header.stamp = point_map.header.stamp
            map_pose.pose.position.x = point_map.point.x
            map_pose.pose.position.y = point_map.point.y
            map_pose.pose.position.z = point_map.point.z
            map_pose.pose.orientation.w = 1.0

            self.litter_pub.publish(map_pose)
            published_count += 1
            self.get_logger().info(
                f'Published litter at map coords: '
                f'x={point_map.point.x:.3f}, y={point_map.point.y:.3f}, '
                f'z={point_map.point.z:.3f}'
            )

        # All poses sent - tell nav to resume
        self.nav_control_pub.publish(Bool(data=True))
        self.get_logger().info(
            f'Published {published_count}/{num_centroids} centroids, sent resume command'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
# import math
# import struct
# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration
# from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# from sensor_msgs.msg import PointCloud2
# from std_msgs.msg import Bool, Int32
# from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
# from tf2_ros import Time, TransformBroadcaster, Buffer, TransformListener

# import tf2_geometry_msgs  # noqa: F401  # registers PointStamped/PoseStamped TF support

# # This node should compare the number of litter items currently in the queue with the number of centroids being published
# # If centroids > queue size: trigger a pause request with confirm
# # When the confirm is received, the node should translate the centroids currently being published into map co-ordinates
# # (only those within pick_radius of base_link) and publish these to /vision/detected_litter


# class PointCloudMonitor(Node):
#     def __init__(self):
#         super().__init__('new_litter_monitor')

#         # Frames
#         self.base_frame = 'mobile_robot_base_link'
#         self.camera_frame = 'camera_link'
#         self.map_frame = 'map'

#         # TF2 setup
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Publish static-ish camera TF on a timer (base_link -> camera_link)
#         self.create_timer(0.1, self.publish_camera_tf)

#         # State tracking
#         self.latest_pc = None
#         self.known_queue_size = 0
#         self._awaiting_confirm = False

#         latching_qos = QoSProfile(
#             depth=1,
#             durability=DurabilityPolicy.TRANSIENT_LOCAL,
#             reliability=ReliabilityPolicy.RELIABLE,
#         )

#         # Subscriptions
#         self.queue_sub = self.create_subscription(
#             Int32, '/queue_size', self.queue_cb, 10
#         )
#         self.pc_sub = self.create_subscription(
#             PointCloud2, '/cloud/centroids', self.pc_cb, 10
#         )
#         self.confirm_sub = self.create_subscription(
#             Bool, '/pause_confirmed', self.confirm_cb, latching_qos
#         )

#         # Publishers
#         self.pause_request_pub = self.create_publisher(
#             Bool, '/pause_request_with_confirm', latching_qos
#         )
#         self.nav_control_pub = self.create_publisher(
#             Bool, '/start_navigation', latching_qos
#         )
#         self.litter_pub = self.create_publisher(
#             PoseStamped, '/vision/detected_litter', latching_qos
#         )

#         self.get_logger().info('Litter monitor started, listening to /cloud/centroids')

#     def publish_camera_tf(self):
#         t = TransformStamped()

#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = self.base_frame      # mobile_robot_base_link
#         t.child_frame_id = self.camera_frame     # camera_link

#         # Camera position relative to base_link
#         t.transform.translation.x = 0.29
#         # t.transform.translation.y = 0.14 adjusted based off simplecentroidprinter feedback
#         t.transform.translation.y = 0.89
#         t.transform.translation.z = 0.648

#         # Camera orientation
#         t.transform.rotation.x = -0.682
#         t.transform.rotation.y = 0.682
#         t.transform.rotation.z = -0.191
#         t.transform.rotation.w = 0.191

#         self.tf_broadcaster.sendTransform(t)

#     def queue_cb(self, msg):
#         self.known_queue_size = msg.data

#     def pc_cb(self, msg):
#         self.latest_pc = msg
#         num_centroids = msg.width * msg.height

#         if num_centroids > self.known_queue_size and not self._awaiting_confirm:
#             self.get_logger().warn(
#                 f'Sensed {num_centroids} objects but queue only has '
#                 f'{self.known_queue_size}. PAUSING.'
#             )

#             self._awaiting_confirm = True
#             pause_cmd = Bool()
#             pause_cmd.data = True
#             self.pause_request_pub.publish(pause_cmd)

#     def confirm_cb(self, msg):
#         if msg.data and self.latest_pc and self._awaiting_confirm:
#             try:
#                 self.process_and_publish_centroids()
#             finally:
#                 self._awaiting_confirm = False
#                 self.pause_request_pub.publish(Bool(data=False))

#     def process_and_publish_centroids(self):
#         msg = self.latest_pc
#         num_centroids = msg.width * msg.height
#         published_count = 0

#         # Data layout assumed: x(4b), y(4b), z(4b), [instance_id(4b)] - little endian floats
#         for i in range(num_centroids):
#             offset = i * msg.point_step

#             x = struct.unpack_from('<f', msg.data, offset)[0]
#             y = struct.unpack_from('<f', msg.data, offset + 4)[0]
#             z = struct.unpack_from('<f', msg.data, offset + 8)[0]

#             # Wrap raw centroid as a PointStamped in the camera frame so we can
#             # use tf_buffer.transform() (handles time interpolation properly).
#             cam_point = PointStamped()
#             cam_point.header.frame_id = (
#                 msg.header.frame_id if msg.header.frame_id else self.camera_frame
#             )
#             # Use current time so the broadcast camera TF is available
#             # cam_point.header.stamp = self.get_clock().now().to_msg() removed because translating in the future message
#             # cam_point.header.stamp = msg.header.stamp
#             cam_point.header.stamp = Time().to_msg()  # zero timestamp means "latest available"
#             cam_point.point.x = float(x)
#             cam_point.point.y = float(y)
#             cam_point.point.z = float(z)

#             # Step 1: transform into base_link to apply the radius filter
#             try:
#                 point_base = self.tf_buffer.transform(
#                     cam_point,
#                     self.base_frame,
#                     timeout=Duration(seconds=1.0),
#                 )
#             except Exception as e:
#                 self.get_logger().warn(
#                     f'Centroid {i}: failed to transform to {self.base_frame}: {e}'
#                 )
#                 continue

#             distance_2d = math.sqrt(
#                 point_base.point.x ** 2 + point_base.point.y ** 2
#             )

#             self.get_logger().info(
#                 f'Centroid {i} in {self.base_frame}: '
#                 f'x={point_base.point.x:.3f}, y={point_base.point.y:.3f}, '
#                 f'z={point_base.point.z:.3f}, dist={distance_2d:.3f} m'
#             )

#             # Step 2: transform into map and publish as a PoseStamped
#             # tf2 will chain: camera_link -> mobile_robot_base_link -> odom -> map
#             try:
#                 point_map = self.tf_buffer.transform(
#                     cam_point,
#                     self.map_frame,
#                     timeout=Duration(seconds=1.0),
#                 )
#             except Exception as e:
#                 self.get_logger().warn(
#                     f'Centroid {i}: failed to transform to {self.map_frame}: {e}'
#                 )
#                 continue

#             map_pose = PoseStamped()
#             map_pose.header.frame_id = self.map_frame
#             map_pose.header.stamp = point_map.header.stamp
#             map_pose.pose.position.x = point_map.point.x
#             map_pose.pose.position.y = point_map.point.y
#             map_pose.pose.position.z = point_map.point.z
#             map_pose.pose.orientation.w = 1.0

#             self.litter_pub.publish(map_pose)
#             published_count += 1
#             self.get_logger().info(
#                 f'Published litter at map coords: '
#                 f'x={point_map.point.x:.3f}, y={point_map.point.y:.3f}, '
#                 f'z={point_map.point.z:.3f}'
#             )

#         # All poses sent - tell nav to resume
#         self.nav_control_pub.publish(Bool(data=True))
#         self.get_logger().info(
#             f'Published {published_count}/{num_centroids} centroids, sent resume command'
#         )


# def main(args=None):
#     rclpy.init(args=args)
#     node = PointCloudMonitor()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()