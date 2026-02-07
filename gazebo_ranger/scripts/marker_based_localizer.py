#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformException
import numpy as np
import math


class MarkerBasedLocalizer(Node):
    def __init__(self):
        super().__init__('marker_based_localizer')
        
        # Parameters
        self.declare_parameter('marker_world_x', 3.0)  # ArUco box spawned at x=3.0
        self.declare_parameter('marker_world_y', 0.0)
        self.declare_parameter('marker_world_z', 1.0)
        self.declare_parameter('marker_world_yaw', 3.14159)  # -X face faces toward robot (180 deg)
        self.declare_parameter('camera_frame', 'kinect_camera_optical')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('publish_tf', True)
        
        self.marker_world_x = self.get_parameter('marker_world_x').get_parameter_value().double_value
        self.marker_world_y = self.get_parameter('marker_world_y').get_parameter_value().double_value
        self.marker_world_z = self.get_parameter('marker_world_z').get_parameter_value().double_value
        self.marker_world_yaw = self.get_parameter('marker_world_yaw').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.publish_tf_flag = self.get_parameter('publish_tf').get_parameter_value().bool_value
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # TF broadcaster (optional)
        if self.publish_tf_flag:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Last known robot pose
        self.last_robot_pose = None
        self.last_update_time = None
        
        # Timer for continuous publishing
        self.declare_parameter('publish_rate', 10.0)  # Hz
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.publish_timer_callback)
        
        # Subscribers (multiple cameras)
        self.marker_pose_sub_front = self.create_subscription(
            PoseStamped,
            '/aruco_marker_pose_front',
            self.marker_pose_callback,
            10
        )
        
        self.marker_pose_sub_upward = self.create_subscription(
            PoseStamped,
            '/aruco_marker_pose_upward',
            self.marker_pose_callback,
            10
        )
        
        # Publishers
        self.robot_pose_pub = self.create_publisher(
            PoseStamped,
            '/robot_pose_from_aruco',
            10
        )
        
        self.robot_odom_pub = self.create_publisher(
            Odometry,
            '/aruco_localization',
            10
        )
        
        # Publisher for relative pose (robot pose relative to marker)
        self.relative_pose_pub = self.create_publisher(
            PoseStamped,
            '/aruco_relative_pose',
            10
        )
        
        self.get_logger().info('Marker-Based Localizer started')
        self.get_logger().info(f'  Marker world pose: x={self.marker_world_x}, y={self.marker_world_y}, '
                              f'z={self.marker_world_z}, yaw={self.marker_world_yaw}')
        self.get_logger().info(f'  Camera frame: {self.camera_frame}')
        self.get_logger().info(f'  Base frame: {self.base_frame}')
        self.get_logger().info(f'  World frame: {self.world_frame}')
        
    def marker_pose_callback(self, msg):
        """Calculate robot pose from marker detection"""
        try:
            # Get transform from camera to base_link
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            except TransformException as ex:
                self.get_logger().warn(f'Could not transform {msg.header.frame_id} to {self.base_frame}: {ex}',
                                      throttle_duration_sec=5.0)
                return
            
            # Marker pose in camera frame
            marker_in_camera = self.pose_to_matrix(msg.pose)
            
            # Camera to base_link transform
            camera_to_base = self.transform_to_matrix(transform.transform)
            
            # Marker pose in base_link frame
            marker_in_base = camera_to_base @ marker_in_camera
            
            # Extract relative pose (base_link relative to marker)
            rel_x, rel_y, rel_z, rel_roll, rel_pitch, rel_yaw = self.matrix_to_pose(marker_in_base)
            
            # Publish relative pose (marker-centric view)
            self.publish_relative_pose(msg.header.stamp, rel_x, rel_y, rel_z, rel_roll, rel_pitch, rel_yaw)
            
            # Marker pose in world frame (known)
            marker_in_world = self.create_transform_matrix(
                self.marker_world_x,
                self.marker_world_y,
                self.marker_world_z,
                0, 0, self.marker_world_yaw
            )
            
            # Calculate robot pose: world_T_robot = world_T_marker @ marker_T_base
            # Since we have base_T_marker, we need marker_T_base = inv(base_T_marker)
            base_in_marker = np.linalg.inv(marker_in_base)
            robot_in_world = marker_in_world @ base_in_marker
            
            # Extract position and orientation
            x, y, z, roll, pitch, yaw = self.matrix_to_pose(robot_in_world)
            
            # Store last known pose
            self.last_robot_pose = (x, y, z, roll, pitch, yaw)
            self.last_update_time = self.get_clock().now()
            
            # Publish robot pose
            self.publish_robot_pose(msg.header.stamp, x, y, z, roll, pitch, yaw)
            
            self.get_logger().info(
                f'Relative to marker: x={rel_x:.3f}, y={rel_y:.3f}, z={rel_z:.3f}, yaw={rel_yaw:.3f}',
                throttle_duration_sec=0.5
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in localization: {e}')
    
    def publish_timer_callback(self):
        """Continuously publish last known pose"""
        if self.last_robot_pose is not None:
            x, y, z, roll, pitch, yaw = self.last_robot_pose
            current_time = self.get_clock().now()
            
            # Publish with current timestamp
            self.publish_robot_pose(current_time.to_msg(), x, y, z, roll, pitch, yaw)
            
            # Check if data is stale
            if self.last_update_time is not None:
                age = (current_time - self.last_update_time).nanoseconds / 1e9
                if age > 2.0:  # Warn if no update for 2 seconds
                    self.get_logger().warn(
                        f'No marker detection for {age:.1f}s, publishing last known pose',
                        throttle_duration_sec=5.0
                    )
    
    def publish_relative_pose(self, timestamp, x, y, z, roll, pitch, yaw):
        """Publish robot pose relative to marker"""
        rel_pose_msg = PoseStamped()
        rel_pose_msg.header.stamp = timestamp
        rel_pose_msg.header.frame_id = 'aruco_marker'  # Marker frame
        
        rel_pose_msg.pose.position.x = x
        rel_pose_msg.pose.position.y = y
        rel_pose_msg.pose.position.z = z
        
        quat = self.euler_to_quaternion(roll, pitch, yaw)
        rel_pose_msg.pose.orientation.x = quat[0]
        rel_pose_msg.pose.orientation.y = quat[1]
        rel_pose_msg.pose.orientation.z = quat[2]
        rel_pose_msg.pose.orientation.w = quat[3]
        
        self.relative_pose_pub.publish(rel_pose_msg)
    
    def publish_robot_pose(self, timestamp, x, y, z, roll, pitch, yaw):
        """Publish robot pose as PoseStamped and Odometry"""
        # PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = self.world_frame
        
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        
        quat = self.euler_to_quaternion(roll, pitch, yaw)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.robot_pose_pub.publish(pose_msg)
        
        # Odometry
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = pose_msg.pose
        
        self.robot_odom_pub.publish(odom_msg)
        
        # Optional TF broadcast
        if self.publish_tf_flag:
            tf_msg = TransformStamped()
            tf_msg.header = pose_msg.header
            tf_msg.child_frame_id = self.base_frame + '_aruco'
            
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = z
            tf_msg.transform.rotation = pose_msg.pose.orientation
            
            self.tf_broadcaster.sendTransform(tf_msg)
    
    def pose_to_matrix(self, pose):
        """Convert geometry_msgs/Pose to 4x4 transformation matrix"""
        q = pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w])
        
        # Quaternion to rotation matrix
        R = self.quaternion_to_rotation_matrix(quat)
        
        # Build transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        
        return T
    
    def transform_to_matrix(self, transform):
        """Convert geometry_msgs/Transform to 4x4 transformation matrix"""
        q = transform.rotation
        quat = np.array([q.x, q.y, q.z, q.w])
        
        R = self.quaternion_to_rotation_matrix(quat)
        
        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = transform.translation.x
        T[1, 3] = transform.translation.y
        T[2, 3] = transform.translation.z
        
        return T
    
    def create_transform_matrix(self, x, y, z, roll, pitch, yaw):
        """Create 4x4 transformation matrix from position and Euler angles"""
        R = self.euler_to_rotation_matrix(roll, pitch, yaw)
        
        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        
        return T
    
    def matrix_to_pose(self, T):
        """Extract position and Euler angles from 4x4 transformation matrix"""
        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]
        
        R = T[:3, :3]
        roll, pitch, yaw = self.rotation_matrix_to_euler(R)
        
        return x, y, z, roll, pitch, yaw
    
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion [x, y, z, w] to 3x3 rotation matrix"""
        x, y, z, w = q
        
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])
        
        return R
    
    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        """Convert Euler angles to 3x3 rotation matrix (ZYX convention)"""
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        
        return R
    
    def rotation_matrix_to_euler(self, R):
        """Convert 3x3 rotation matrix to Euler angles (ZYX convention)"""
        sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
        
        singular = sy < 1e-6
        
        if not singular:
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            roll = math.atan2(-R[1, 2], R[1, 1])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = 0
        
        return roll, pitch, yaw
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion [x, y, z, w]"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return np.array([qx, qy, qz, qw])


def main(args=None):
    rclpy.init(args=args)
    node = MarkerBasedLocalizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

