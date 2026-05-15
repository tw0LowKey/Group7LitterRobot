import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header


class BannedZonePublisher(Node):

    def __init__(self):
        super().__init__('banned_zone_publisher')

        # 2D Array list of collision boxes with the following format:
        #self.collision_object_list = [[Object_ID, x_dim, y_dim, z_dim, x_pos, y_pos, z_pos, w], [...]]
        #self.collision_object_list = [[string, double, double,double,double,double,double,double,]]
        # Dimensions and positions in metres relative to arm base frame

        self.collision_object_list = [["scout_mini_box", 0.612, 0.58, 0.275, -0.13, 0.0, -0.138, 1.0],
                                      ["camera_pole_v", 0.05, 0.05, 0.65, -0.04, 0.135, 0.325,1.0],
                                      ["camera_pole_h", 0.342, 0.05, 0.05, -0.120, 0.15, 0.635, 1.0],
                                      ["camera", 0.17, 0.14, 0.17, 0.1, 0.15, 0.635,1.0],
                                      ["floor", 2.0, 2.0, 0.01, 0.0, 0.0, -0.280, 1.0],
                                      ["LiDAR", 0.13, 0.09, 0.08, 0.115, 0.0, 0.04, 1.0],
                                      ["GPS_antenna", 0.05, 0.05, 0.17, -0.275, 0.15, 0.735, 1.0],
                                      ["payload", 0.245 ,0.38, 0.12, -0.22, 0.0, 0.03, 1.0]]

        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        # Wait for move_group's service to come up before doing anything else
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /apply_planning_scene service...')

        self.add_banned_zone()

    def add_banned_zone(self):

        planning_scene = PlanningScene()

        # Add each object in turn
        for object in self.collision_object_list:

            collision_object = CollisionObject()

            # Create scout mini collision object
            collision_object.id = object[0]
            collision_object.header = Header()
            collision_object.header.frame_id = "arm_base_link"

            # Create a box shape
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [object[1], object[2], object[3]]  # size (x,y,z)

            # Set pose
            pose = Pose()
            pose.position.x = object[4]
            pose.position.y = object[5]
            pose.position.z = object[6]
            pose.orientation.w = object[7]

            # Apply pose and shape to scout mini object
            collision_object.primitives.append(box)
            collision_object.primitive_poses.append(pose)
            collision_object.operation = CollisionObject.ADD

            # Add object to planning scene
            planning_scene.world.collision_objects.append(collision_object)

            self.get_logger().info(object[0]+ " added successfully")

        planning_scene.is_diff = True

        request = ApplyPlanningScene.Request()
        request.scene = planning_scene

        future = self.client.call_async(request)
        future.add_done_callback(self.scene_response_callback)

    def scene_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Planning scene updated successfully.')
            else:
                self.get_logger().error('Failed to apply planning scene.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):

    try:
        rclpy.init(args=args)
        node = BannedZonePublisher()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()