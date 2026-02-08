#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math
import numpy as np


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        
        # Robot parameters (from URDF)
        self.wheel_radius = 0.16459  # m
        self.wheelbase = 0.498  # m
        self.track = 0.58306  # m
        
        # Odometry state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Previous time
        self.last_time = None
        
        # Joint names - all wheel joints and steering joints
        self.front_left_wheel_joint = 'fl_wheel'
        self.front_right_wheel_joint = 'fr_wheel'
        self.rear_left_wheel_joint = 'rl_wheel'
        self.rear_right_wheel_joint = 'rr_wheel'
        
        self.front_left_steering_joint = 'fl_steering_wheel'
        self.front_right_steering_joint = 'fr_steering_wheel'
        self.rear_left_steering_joint = 'rl_steering_wheel'
        self.rear_right_steering_joint = 'rr_steering_wheel'
        
        # Current wheel velocities from joint states (rad/s)
        self.front_left_wheel_velocity = 0.0
        self.front_right_wheel_velocity = 0.0
        self.rear_left_wheel_velocity = 0.0
        self.rear_right_wheel_velocity = 0.0
        
        # Current steering angles from joint states (rad)
        self.front_left_steering_angle = 0.0
        self.front_right_steering_angle = 0.0
        self.rear_left_steering_angle = 0.0
        self.rear_right_steering_angle = 0.0
        
        # Subscriber: joint_states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher: odom
        self.odom_pub = self.create_publisher(
            Odometry,
            '/wheel_odom',
            10
        )
        
        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Timer to periodically publish odometry
        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(timer_period, self.publish_odometry)
        
        self.get_logger().info('Wheel Odometry node started.')
    
    def joint_state_callback(self, msg):
        """Read actual wheel velocities and steering angles from joint states"""
        try:
            # Read all wheel velocities from joint states
            fl_idx = msg.name.index(self.front_left_wheel_joint)
            self.front_left_wheel_velocity = msg.velocity[fl_idx]
            
            fr_idx = msg.name.index(self.front_right_wheel_joint)
            self.front_right_wheel_velocity = msg.velocity[fr_idx]
            
            rl_idx = msg.name.index(self.rear_left_wheel_joint)
            self.rear_left_wheel_velocity = msg.velocity[rl_idx]
            
            rr_idx = msg.name.index(self.rear_right_wheel_joint)
            self.rear_right_wheel_velocity = msg.velocity[rr_idx]
            
            # Read all steering angles from joint states
            fl_steer_idx = msg.name.index(self.front_left_steering_joint)
            self.front_left_steering_angle = msg.position[fl_steer_idx]
            
            fr_steer_idx = msg.name.index(self.front_right_steering_joint)
            self.front_right_steering_angle = msg.position[fr_steer_idx]
            
            rl_steer_idx = msg.name.index(self.rear_left_steering_joint)
            self.rear_left_steering_angle = msg.position[rl_steer_idx]
            
            rr_steer_idx = msg.name.index(self.rear_right_steering_joint)
            self.rear_right_steering_angle = msg.position[rr_steer_idx]
            
        except ValueError as e:
            # Joint not found (may be during initialization)
            pass
    
    def calculate_odometry(self, dt):
        """Calculate odometry from actual wheel velocities and steering angles in joint states"""
        if dt <= 0.0:
            return
        
        # Calculate wheel linear velocities (m/s) from joint states (rad/s)
        fl_linear = self.front_left_wheel_velocity * self.wheel_radius
        fr_linear = self.front_right_wheel_velocity * self.wheel_radius
        rl_linear = self.rear_left_wheel_velocity * self.wheel_radius
        rr_linear = self.rear_right_wheel_velocity * self.wheel_radius
        
        # Use rear wheels for linear velocity (rear wheels have minimal steering angle)
        linear_velocity = (rl_linear + rr_linear) / 2.0
        
        # Check if pivot turn (in-place rotation)
        if abs(linear_velocity) < 0.01:
            # Pivot turn: use differential drive model
            left_avg_velocity = (fl_linear + rl_linear) / 2.0
            right_avg_velocity = (fr_linear + rr_linear) / 2.0
            angular_velocity = (left_avg_velocity - right_avg_velocity) / self.track
            angular_velocity = -angular_velocity  # Invert sign for pivot turn
        else:
            # Ackermann steering: use actual steering angles from joint states
            # Average steering angles (for 4-wheel steering)
            avg_front_steering = (self.front_left_steering_angle + self.front_right_steering_angle) / 2.0
            avg_rear_steering = (self.rear_left_steering_angle + self.rear_right_steering_angle) / 2.0
            
            # Use front steering angle for Ackermann calculation
            # Angular velocity = linear_velocity * tan(steering_angle) / wheelbase
            if abs(avg_front_steering) > 0.001:  # If steering angle is significant
                # Ackermann steering model
                angular_velocity = linear_velocity * math.tan(avg_front_steering) / self.wheelbase
            else:
                # Straight motion or very small steering angle
                # Fall back to differential drive model
                left_avg_velocity = (fl_linear + rl_linear) / 2.0
                right_avg_velocity = (fr_linear + rr_linear) / 2.0
                angular_velocity = (left_avg_velocity - right_avg_velocity) / self.track
        
        # Update position (integration)
        self.x += linear_velocity * math.cos(self.theta) * dt
        self.y += linear_velocity * math.sin(self.theta) * dt
        self.theta += angular_velocity * dt
        
        # Normalize theta to [-pi, pi] range
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
    
    def publish_odometry(self):
        """Publish odometry message"""
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calculate time difference
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Calculate odometry
        self.calculate_odometry(dt)
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion)
        odom_msg.pose.pose.orientation = self.quaternion_from_yaw(self.theta)
        
        # Calculate wheel linear velocities (m/s) from joint states (rad/s)
        fl_linear = self.front_left_wheel_velocity * self.wheel_radius
        fr_linear = self.front_right_wheel_velocity * self.wheel_radius
        rl_linear = self.rear_left_wheel_velocity * self.wheel_radius
        rr_linear = self.rear_right_wheel_velocity * self.wheel_radius
        
        # Use rear wheels for linear velocity (rear wheels have minimal steering angle)
        linear_vel = (rl_linear + rr_linear) / 2.0
        
        # Calculate angular velocity using same method as calculate_odometry
        if abs(linear_vel) < 0.01:
            # Pivot turn
            left_avg = (fl_linear + rl_linear) / 2.0
            right_avg = (fr_linear + rr_linear) / 2.0
            angular_vel = (left_avg - right_avg) / self.track
            angular_vel = -angular_vel
        else:
            # Ackermann steering
            avg_front_steering = (self.front_left_steering_angle + self.front_right_steering_angle) / 2.0
            if abs(avg_front_steering) > 0.001:
                angular_vel = linear_vel * math.tan(avg_front_steering) / self.wheelbase
            else:
                left_avg = (fl_linear + rl_linear) / 2.0
                right_avg = (fr_linear + rr_linear) / 2.0
                angular_vel = (left_avg - right_avg) / self.track
        
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel
        
        # Covariance (simple value setting)
        odom_msg.pose.covariance[0] = 0.1  # x
        odom_msg.pose.covariance[7] = 0.1  # y
        odom_msg.pose.covariance[35] = 0.1  # yaw
        
        odom_msg.twist.covariance[0] = 0.1  # vx
        odom_msg.twist.covariance[35] = 0.1  # vyaw
        
        # Publish
        self.odom_pub.publish(odom_msg)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.quaternion_from_yaw(self.theta)
        
        self.tf_broadcaster.sendTransform(t)
        
        self.last_time = current_time
    
    def quaternion_from_yaw(self, yaw):
        """Create quaternion from yaw angle"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    wheel_odometry = WheelOdometry()
    rclpy.spin(wheel_odometry)
    wheel_odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
