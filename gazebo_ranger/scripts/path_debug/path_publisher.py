#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path 
import utm
import math
import numpy as np
import ament_index_python

class Path_publisher(Node):
    def __init__(self):
        super().__init__('Path_publisher_node')
        # map 프레임의 원점 UTM 좌표
        self.utm_x_zero = 346638.99532500084
        self.utm_y_zero = 4070388.235393337
        # Publish heading
        self.publisher_straight = self.create_publisher(
            Path,
            '/straight',
            10
        )
        self.publisher_circle = self.create_publisher(
            Path,
            '/circle',
            10
        )
        self.publisher_u_turn = self.create_publisher(
            Path,
            '/u_turn',
            10
        )
        self.publisher_gym = self.create_publisher(
            Path,
            '/gym',
            10
        )

        self.publisher_straight_viz = self.create_publisher(
            Path,
            '/straight_viz',
            10
        )
        self.publisher_circle_viz = self.create_publisher(
            Path,
            '/circle_viz',
            10
        )
        self.publisher_u_turn_viz = self.create_publisher(
            Path,
            '/u_turn_viz',
            10
        )
        self.publisher_gym_viz = self.create_publisher(
            Path,
            '/gym_viz',
            10
        )
        
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.msg_straight, self.msg_straight_viz = self.read_file('path_debug/straight_line.txt')
        self.msg_circle, self.msg_circle_viz = self.read_file('path_debug/circle_line.txt')
        self.msg_u_turn, self.msg_u_turn_viz = self.read_file('path_debug/U_turn.txt')
        self.msg_gym, self.msg_gym_viz = self.read_file('path_debug/gym_path.txt')
        
    def timer_callback(self):
        self.publisher_straight.publish(self.msg_straight)
        self.publisher_circle.publish(self.msg_circle)
        self.publisher_u_turn.publish(self.msg_u_turn)
        self.publisher_gym.publish(self.msg_gym)
        
        self.publisher_straight_viz.publish(self.msg_straight_viz)
        self.publisher_circle_viz.publish(self.msg_circle_viz)
        self.publisher_u_turn_viz.publish(self.msg_u_turn_viz)
        self.publisher_gym_viz.publish(self.msg_gym_viz)
        
    def read_file(self,file_name):
        package_path = ament_index_python.get_package_share_directory('gazebo_ranger')
        file_path = os.path.join(package_path, 'scripts', file_name)
        with open(file_path, 'r') as file:
            # Read all lines from the file
            lines = file.readlines()

        # Process the data
        x_values = []
        y_values = []
        poses = []
        poses_viz = []
        msg = Path()
        msg_viz = Path()

        for line in lines:
            # Split each line into two values
            values = line.strip().split(',')

            # Convert values to float
            x_values.append(float(values[0]))
            y_values.append(float(values[1]))
            msg_temp = PoseStamped()
            msg_temp_viz = PoseStamped()
            msg_temp.pose.position.x = float(values[0])
            msg_temp.pose.position.y = float(values[1])
            msg_temp_viz.pose.position.x = float(values[0])-self.utm_x_zero
            msg_temp_viz.pose.position.y = float(values[1])-self.utm_y_zero
            poses.append(msg_temp)
            poses_viz.append(msg_temp_viz)
            
        msg.poses = poses
        msg_viz.poses = poses_viz
        msg.header.frame_id = 'map'
        msg_viz.header.frame_id = 'map'
        return msg, msg_viz
    
def main(args=None):
    rclpy.init(args=args)
    path_publisher = Path_publisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()