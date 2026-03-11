import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


class BannedZonePublisher(Node):

    def __init__(self):
        super().__init__('banned_zone_publisher')

        self.publisher_ = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        self.timer = self.create_timer(2.0, self.add_banned_zone)

    def add_banned_zone(self):
        collision_object = CollisionObject()

        collision_object.id = "banned_zone"
        collision_object.header = Header()
        collision_object.header.frame_id = "world"

        # Create a box
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.4, 0.4, 0.4]  # size (x,y,z)

        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.0
        pose.position.z = 0.2
        pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD

        planning_scene = PlanningScene()
        planning_scene.world.collision_objects.append(collision_object)
        planning_scene.is_diff = True

        self.publisher_.publish(planning_scene)

        self.get_logger().info("Banned zone added.")
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = BannedZonePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
