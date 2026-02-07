#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import LinkStates
from tf_transformations import quaternion_matrix
import numpy as np

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/ranger/cmd_vel', 10)
        self.sub = self.create_subscription(
            LinkStates,
            '/world/link_states_plug',
            self.listener_callback,
            10
        )
        self.index_flag = True
        self.base_link_index = None

    def calculate_cmd_vel(self, pose, twist):
        cmd_vel = Twist()

        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]  # [x, y, z, w]

        # Convert quaternion to rotation matrix
        R = quaternion_matrix(quaternion)[:3, :3]

        # Transform linear velocity in base_link frame
        cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z = (np.dot([twist.linear.x,twist.linear.y,twist.linear.z],R ))

        # Copy angular velocity
        cmd_vel.angular = twist.angular
        return cmd_vel

    def listener_callback(self, msg):
        if (self.index_flag == True) or (self.base_link_index is None):
            base_link_name = "_robot::base_link"
            self.base_link_index = next((i for i, name in enumerate(msg.name) if name == base_link_name), None)
            self.index_flag = False
        # Calculate cmd_vel based on received pose and twist
        # print(msg.pose[1].orientation)
        if self.base_link_index is not None :
            cmd_vel = self.calculate_cmd_vel(msg.pose[self.base_link_index], msg.twist[self.base_link_index])
            self.publisher_.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_publisher = CmdVelPublisher()
    rclpy.spin(cmd_vel_publisher)
    cmd_vel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
