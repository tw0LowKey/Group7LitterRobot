import random


import rclpy
from rclpy.task import Future
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from piper_msgs.srv import PickPlaceRequest




class WhatIsThePointServiceServerNode(Node):
    """A ROS2 Node with a Service Server for WhatIsThePoint."""

    def __init__(self):
        super().__init__('what_is_the_point_service_server')

        self.service_server = self.create_service(
            srv_type=PickPlaceRequest,
            srv_name='/pick_place_request',
            callback=self.what_is_the_point_service_callback)

        self.service_server_call_count: int = 0

    def what_is_the_point_service_callback(self,
                                           request: PickPlaceRequest.Request,
                                           response: PickPlaceRequest.Response
                                           ) -> PickPlaceRequest.Response:
        """Analyses an AmazingQuote and returns what is the point.
           If the quote contains 'life', it returns a point whose sum of coordinates is 42.
           Otherwise, it returns a random point whose sum of coordinates is not 42.
        """




        # Increase the call count
        self.service_server_call_count = self.service_server_call_count + 1

        self.get_logger().info((f"""
            This is the call number {self.service_server_call_count} to this Service Server.
            The analysis of the AmazingQuote below is complete.
            
                    {request.poses[0].pose.position.x}, {request.poses[0].pose.position.y}, {request.poses[0].pose.position.z}
            
            -- {request.poses[0].pose.orientation.x}, {request.poses[0].pose.orientation.y}, {request.poses[0].pose.orientation.z}, {request.poses[0].pose.orientation.w}
            
            The point has been sent back to the client.
        """))

        return True


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        what_is_the_point_service_server_node = WhatIsThePointServiceServerNode()

        rclpy.spin(what_is_the_point_service_server_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()