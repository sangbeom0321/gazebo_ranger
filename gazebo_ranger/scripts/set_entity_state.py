#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion
import tf_transformations

class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(SetEntityState, '/world/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service to become available...')
        self.req = SetEntityState.Request()

    def set_pose(self, x, y, z, roll, pitch, yaw):
        quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        self.req.state.name = "_robot"
        self.req.state.pose.position = Point(x=x, y=y, z=z)
        self.req.state.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Service call succeeded')
        else:
            self.get_logger().error('Service call failed')


def main(args=None):
    rclpy.init(args=args)

    x = -0.14
    y = -1.76
    z = 0.5
    roll = 0.0
    pitch = 0.0
    yaw = 1.57

    service_client = ServiceClient()
    service_client.set_pose(x, y, z, roll, pitch, yaw)
    service_client.send_request()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
