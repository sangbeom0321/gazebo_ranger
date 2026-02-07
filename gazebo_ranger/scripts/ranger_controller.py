#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray,Float32MultiArray

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02
        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pos = np.array([0,0,0,0], float)
        self.vel = np.array([0,0,0,0], float)
        self.sub = self.create_subscription(Float32MultiArray, '/Control/ranger_data', self.cmd_callback, qos_profile=10)
        

    def cmd_callback(self, msg):
        self.vel[0] = msg.data[0]
        self.vel[1] = msg.data[1]
        self.vel[2] = msg.data[2]
        self.vel[3] = msg.data[3]

        self.pos[0] = msg.data[4]
        self.pos[1] = msg.data[5]
        self.pos[2] = msg.data[6]
        self.pos[3] = msg.data[7]
        
    def timer_callback(self):
        pos_array = Float64MultiArray(data=self.pos) 
        vel_array = Float64MultiArray(data=self.vel) 
        self.pub_pos.publish(pos_array)
        self.pub_vel.publish(vel_array)
        self.pos[:] = 0
        self.vel[:] = 0

def main(args=None):
    rclpy.init(args=args)
    
    commander = Commander()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()
    
if __name__ == '__main__':
    main()

