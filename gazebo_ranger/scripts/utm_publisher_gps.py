#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray, Float64
import utm
import math
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_transformations
from geometry_msgs.msg import Quaternion

class GPSToUTMNode(Node):
    def __init__(self):
        super().__init__('gps_to_utm_node')
        self.cur_pos=[0, 0]
        self.utm_msgs = []
        self.msg_stamps = []
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 10Hz
        self.heading = 0
        # Subscription for /gps/fix
        self.subscription_fix = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback_fix,
            10)
        self.publisher_fix = self.create_publisher(
            Float64MultiArray,
            '/utm_coordinates1',
            10)

        # Subscription for /gps/fix2
        self.subscription_fix2 = self.create_subscription(
            NavSatFix,
            '/gps/fix2',
            self.gps_callback_fix2,
            10)
        self.publisher_fix2 = self.create_publisher(
            Float64MultiArray,
            '/utm_coordinates2',
            10)
        
        # Publish heading
        self.publisher_heading = self.create_publisher(
            Float64,
            '/Local/heading',
            10
        )

        # Publish heading
        self.publisher_utm_mean = self.create_publisher(
            Float64MultiArray,
            '/Local/utm',
            10
        )
    
    def convert_yaw_to_quaternion(self, yaw):
        # Yaw 각도를 쿼터니언으로 변환
        quaternion = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

    def timer_callback(self):
        t = TransformStamped()

        # 현재 시간 설정
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # map 프레임의 원점 UTM 좌표
        utm_x_zero = 346638.99532500084
        utm_y_zero = 4070388.235393337

        # 위치 설정
        t.transform.translation.x = self.cur_pos[0] - utm_x_zero
        t.transform.translation.y = self.cur_pos[1] - utm_y_zero
        t.transform.translation.z = 0.0

        # 회전 설정 (여기서는 회전 없음)
        qut = self.convert_yaw_to_quaternion(self.heading)
        t.transform.rotation.x = qut.x
        t.transform.rotation.y = qut.y
        t.transform.rotation.z = qut.z
        t.transform.rotation.w = qut.w

        # 변환 브로드캐스트
        self.br.sendTransform(t)

    def gps_callback_fix(self, msg):
        nanosec_temp, utm_temp = self.publish_utm_coordinates(msg, self.publisher_fix)
        self.utm_msgs.append(nanosec_temp); self.utm_msgs.append(utm_temp[0]); self.utm_msgs.append(utm_temp[1])
        if(len(self.utm_msgs) > 4):
            self.publish_pos_heading(self.utm_msgs)
        
    def gps_callback_fix2(self, msg):
        nanosec_temp, utm_temp = self.publish_utm_coordinates(msg, self.publisher_fix2)
        self.utm_msgs.append(nanosec_temp); self.utm_msgs.append(utm_temp[0]); self.utm_msgs.append(utm_temp[1])
        print(self.utm_msgs)
        if(len(self.utm_msgs) > 4):
            self.publish_pos_heading(self.utm_msgs)

    def publish_utm_coordinates(self, msg, publisher):
        lat = msg.latitude
        lon = msg.longitude

        # Convert latitude and longitude to UTM
        utm_coords = utm.from_latlon(lat, lon)

        # Create and publish UTM coordinates message
        utm_msg = Float64MultiArray()
        utm_msg.data = [utm_coords[0], utm_coords[1]]
        publisher.publish(utm_msg)
        return msg.header.stamp.nanosec, utm_msg.data
    
    def publish_pos_heading(self, utm_msgs):
        if(abs(self.utm_msgs[0]-self.utm_msgs[3]) < 0.005):
            angle_rad = math.atan2(self.utm_msgs[5]-self.utm_msgs[2],self.utm_msgs[4]-self.utm_msgs[1])
            self.cur_pos = [(self.utm_msgs[4]+self.utm_msgs[1])/2, (self.utm_msgs[5]+self.utm_msgs[2])/2]
            utm_mean_msg = Float64MultiArray()
            utm_mean_msg.data = self.cur_pos
            heading_msg = Float64()
            heading_msg.data = angle_rad
            self.publisher_heading.publish(heading_msg)
            self.publisher_utm_mean.publish(utm_mean_msg)
            self.utm_msgs = []
            self.heading = angle_rad
        else:
            self.utm_msgs = []
    
def main(args=None):
    rclpy.init(args=args)
    gps_to_utm_node = GPSToUTMNode()
    rclpy.spin(gps_to_utm_node)
    gps_to_utm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()