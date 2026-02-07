#!/usr/bin/python3
import math
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

vel_msg = Twist()  # robot velocity
mode_selection = 0 # 1:opposite phase, 2:in-phase, 3:pivot turn 4: none

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        timer_period = 0.02
        self.wheel_seperation = 0.37
        self.wheel_base = 0.5
        self.wheel_radius = 0.1
        self.wheel_steering_y_offset = 0.00
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset
        self.pi = 3.14159265359
        self.max_speed = 1*self.pi

        self.pos = np.array([0,0,0,0], float)
        self.vel = np.array([0,0,0,0], float) #left_front, right_front, left_rear, right_rear

        self.pub_pos = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.pub_vel = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.sub = self.create_subscription(Twist, '/ranger_mini/cmd_vel', self.cmd_callback, qos_profile=10)
        

    def cmd_callback(self, msg):
        vel_msg.linear.x = msg.linear.x *3.0
        vel_msg.linear.y = msg.linear.y *3.0
        vel_msg.angular.z = -msg.angular.z
        
    def timer_callback(self):
        if(vel_msg.linear.x == 0.0 and vel_msg.linear.y == 0.0 and vel_msg.angular.z == 0.0):
            mode_selection = 1000
        elif(vel_msg.linear.x != 0.0 and vel_msg.angular.z != 0.0 and vel_msg.linear.y == 0.0):
            mode_selection = 1
        elif(vel_msg.linear.x == 0.0 and vel_msg.angular.z != 0.0 and vel_msg.linear.y == 0.0):
            mode_selection = 3
        else:
            mode_selection = 2
        # opposite phase
        if(mode_selection == 1):
            
            vel_steerring_offset = vel_msg.angular.z * self.wheel_steering_y_offset
            sign = np.sign(vel_msg.linear.x)

            self.vel[0] = sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) - vel_steerring_offset
            self.vel[1] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) + vel_steerring_offset
            self.vel[2] = sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) - vel_steerring_offset
            self.vel[3] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) + vel_steerring_offset

            self.pos[0] = -math.atan(vel_msg.angular.z*self.wheel_base/(2*vel_msg.linear.x + vel_msg.angular.z*self.steering_track))
            self.pos[1] = -math.atan(vel_msg.angular.z*self.wheel_base/(2*vel_msg.linear.x - vel_msg.angular.z*self.steering_track))
            self.pos[2] = -self.pos[0]
            self.pos[3] = -self.pos[1]

        # in-phase
        elif(mode_selection == 2):

            V = math.hypot(vel_msg.linear.x, vel_msg.linear.y)
            sign = np.sign(vel_msg.linear.x)
            

                
            if(vel_msg.linear.x != 0.0):
                ang = vel_msg.linear.y / vel_msg.linear.x
            else:
                ang = 0
            
            self.pos[0] = math.atan(ang)
            self.pos[1] = math.atan(ang)
            self.pos[2] = self.pos[0]
            self.pos[3] = self.pos[1]
            
            if(vel_msg.linear.x == 0.0):
                if(vel_msg.linear.y > 0.0):
                    self.pos[0] = self.pi / 2.0
                    self.pos[1] = self.pi / 2.0
                    self.pos[2] = self.pos[0]
                    self.pos[3] = self.pos[1]
                    
                    self.vel[:] = self.max_speed
                else:
                    self.pos[0] = -self.pi / 2.0
                    self.pos[1] = -self.pi / 2.0
                    self.pos[2] = self.pos[0]
                    self.pos[3] = self.pos[1]
                    
                    self.vel[:] = self.max_speed
            else:
                self.vel[:] = sign*V
            
            
        # pivot turn
        elif(mode_selection == 3):

            self.pos[0] = -math.atan(self.steering_track/self.wheel_base)
            self.pos[1] = math.atan(self.steering_track/self.wheel_base)
            self.pos[2] = math.atan(self.steering_track/self.wheel_base)
            self.pos[3] = -math.atan(self.steering_track/self.wheel_base)
            
            self.vel[0] = vel_msg.angular.z 
            self.vel[1] = -vel_msg.angular.z 
            self.vel[2] = self.vel[0]
            self.vel[3] = self.vel[1]

        else:

            self.pos[:] = 0
            self.vel[:] = 0

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