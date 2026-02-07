#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
import utm
import math
import numpy as np

class GPSToUTMPath(Node):
    def __init__(self):
        super().__init__('gps_to_utm_node')

        self.utm_msgs = []
        # Subscription for /gps/fix
        self.subscription_fix = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback_fix,
            10)

        # Subscription for /gps/fix2
        self.subscription_fix2 = self.create_subscription(
            NavSatFix,
            '/gps/fix2',
            self.gps_callback_fix2,
            10)
        
        # Publish heading
        self.publisher_locale = self.create_publisher(
            Float64MultiArray,
            '/locale',
            10
        )
        
    def gps_callback_fix(self, msg):
        utm_temp = self.publish_utm_coordinates(msg)
        self.utm_msgs.append(utm_temp[0]); self.utm_msgs.append(utm_temp[1])
        if(len(self.utm_msgs) > 4):
            self.publish_path(self.utm_msgs)
        
    def gps_callback_fix2(self, msg):
        utm_temp = self.publish_utm_coordinates(msg)
        self.utm_msgs.append(utm_temp[0]); self.utm_msgs.append(utm_temp[1])
        print(self.utm_msgs)
        if(len(self.utm_msgs) > 3):
            self.publish_path(self.utm_msgs)

    def publish_utm_coordinates(self, msg):
        lat = msg.latitude
        lon = msg.longitude

        # Convert latitude and longitude to UTM
        utm_coords = utm.from_latlon(lat, lon)

        # Create and publish UTM coordinates message
        utm_msg = Float64MultiArray()
        utm_msg.data = [utm_coords[0], utm_coords[1]]
        return utm_msg.data
    
    def publish_path(self, utm_msgs):
        if(abs(self.utm_msgs[0]-self.utm_msgs[2]) > 0.000005):
            path = [(self.utm_msgs[3]+self.utm_msgs[1])/2,(self.utm_msgs[2]+self.utm_msgs[0])/2]
            
            path_msg = Float64MultiArray()
            path_msg.data = path
            self.publisher_locale.publish(path_msg)
            self.utm_msgs = []
        else:
            self.utm_msgs = []
    
def main(args=None):
    rclpy.init(args=args)
    gps_to_utm_node = GPSToUTMPath()
    rclpy.spin(gps_to_utm_node)
    gps_to_utm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()