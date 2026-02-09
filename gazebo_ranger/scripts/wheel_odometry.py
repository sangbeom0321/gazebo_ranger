#!/usr/bin/env python3
"""
4-Wheel Steer + 4-Wheel Drive (Swerve) Odometry

수학적 모델:
- 각 바퀴는 자기 방향으로만 굴러간다 (횡방향 미끄럼 없음 가정)
- 차체 twist (v_x, v_y, ω)와 바퀴 속도 v_i의 관계:
  cos(α_i) * v_x + sin(α_i) * v_y + ω * (-y_i*cos(α_i) + x_i*sin(α_i)) = v_i
  
- 4개 바퀴 → 4개 방정식, 미지수 3개 → 과잉 결정
- 최소제곱법 (QR/SVD)으로 해를 구함
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from rosgraph_msgs.msg import Clock
import math
import numpy as np


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        
        # Robot parameters (from URDF)
        self.wheel_radius = 0.16459  # m
        
        # Wheel positions in base_link frame (from URDF)
        # 순서: FL, FR, RL, RR (각 바퀴의 조향각과 매칭)
        # fl_steering_wheel: xyz="0.23 0.206 -0.1"
        # fr_steering_wheel: xyz="0.23 -0.206 -0.1"
        # rl_steering_wheel: xyz="-0.23 0.206 -0.1"
        # rr_steering_wheel: xyz="-0.23 -0.206 -0.1"
        self.wheel_positions = np.array([
            [0.23, 0.206],   # FL: front-left
            [0.23, -0.206],  # FR: front-right
            [-0.23, 0.206],  # RL: rear-left
            [-0.23, -0.206]  # RR: rear-right
        ])  # Shape: (4, 2) - [x_i, y_i] for each wheel
        
        # Odometry state variables (world frame)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Simulation time from /clock topic
        self.current_sim_time = None
        self.last_time = None
        
        # Joint names (joint_states의 실제 순서)
        # name 순서: fr_wheel, rl_wheel, rr_wheel, fl_wheel, 
        #            fl_steering_wheel, fr_steering_wheel, rl_steering_wheel, rr_steering_wheel
        # velocity: 앞 4개가 wheel 속도 (fr, rl, rr, fl 순서)
        # position: 마지막 4개가 steering 각도 (fl, fr, rl, rr 순서)
        # 
        # 매핑: wheel_velocities[0]=fr, [1]=rl, [2]=rr, [3]=fl
        #       steering_angles[0]=fl, [1]=fr, [2]=rl, [3]=rr
        #       wheel_positions[0]=fl, [1]=fr, [2]=rl, [3]=rr
        self.wheel_joints = ['fr_wheel', 'rl_wheel', 'rr_wheel', 'fl_wheel']
        self.steering_joints = ['fl_steering_wheel', 'fr_steering_wheel', 
                                'rl_steering_wheel', 'rr_steering_wheel']
        
        # Current wheel velocities and steering angles
        # wheel_velocities: [fr, rl, rr, fl] 순서로 저장
        # steering_angles: [fl, fr, rl, rr] 순서로 저장
        self.wheel_velocities = np.zeros(4)  # rad/s
        self.steering_angles = np.zeros(4)  # rad
        
        # Subscriber: /clock topic for simulation time
        # Gazebo publishes /clock with BEST_EFFORT reliability - must match
        clock_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            clock_qos
        )
        
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
        
        # Timer to periodically publish odometry
        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(timer_period, self.publish_odometry)
        
        self.get_logger().info('Wheel Odometry node started (4WS+4WD Swerve model)')
        self.get_logger().info('Subscribing to /clock topic for simulation time')
    
    def clock_callback(self, msg):
        """Store simulation time from /clock topic"""
        self.current_sim_time = msg.clock
    
    def joint_state_callback(self, msg):
        """Read wheel velocities and steering angles from joint states"""
        try:
            # Read wheel velocities (rad/s)
            for i, joint_name in enumerate(self.wheel_joints):
                idx = msg.name.index(joint_name)
                self.wheel_velocities[i] = msg.velocity[idx]
            
            # Read steering angles (rad)
            for i, joint_name in enumerate(self.steering_joints):
                idx = msg.name.index(joint_name)
                self.steering_angles[i] = msg.position[idx]
                
        except ValueError as e:
            # Joint not found (may be during initialization)
            pass
    
    def solve_body_twist(self):
        """
        Solve for body twist (v_x, v_y, ω) using SVD-based least squares
        
        Uses SVD pseudo-inverse for robust handling of rank-deficient cases.
        
        Returns:
            np.array([v_x, v_y, ω]) or None if solution fails
        """
        # 매핑: 
        # wheel_velocities: [fr, rl, rr, fl] 순서
        # steering_angles: [fl, fr, rl, rr] 순서  
        # wheel_positions: [fl, fr, rl, rr] 순서
        # 
        # 각 바퀴를 FL, FR, RL, RR 순서로 재배열
        wheel_velocities_reordered = np.array([
            self.wheel_velocities[3],  # FL (wheel_velocities[3])
            self.wheel_velocities[0],  # FR (wheel_velocities[0])
            self.wheel_velocities[1],  # RL (wheel_velocities[1])
            self.wheel_velocities[2]   # RR (wheel_velocities[2])
        ])
        
        # steering_angles는 이미 [fl, fr, rl, rr] 순서이므로 그대로 사용
        
        # Calculate wheel linear velocities (m/s)
        wheel_linear_velocities = wheel_velocities_reordered * self.wheel_radius
        
        # 정지 상태: 모든 바퀴 속도가 매우 작으면 twist = 0 (노이즈로 인한 v_y 폭발 방지)
        max_wheel_linear = np.max(np.abs(wheel_linear_velocities))
        if max_wheel_linear < 0.05:  # ~0.05 m/s 미만이면 정지로 간주
            return np.array([0.0, 0.0, 0.0])
        
        # Build constraint matrix A (4x3)
        # Each row: [cos(α_i), sin(α_i), -y_i*cos(α_i) + x_i*sin(α_i)]
        A = np.zeros((4, 3))
        
        for i in range(4):
            alpha_i = self.steering_angles[i]  # 이미 [fl, fr, rl, rr] 순서
            x_i, y_i = self.wheel_positions[i]  # 이미 [fl, fr, rl, rr] 순서
            
            cos_alpha = math.cos(alpha_i)
            sin_alpha = math.sin(alpha_i)
            
            A[i, 0] = cos_alpha
            A[i, 1] = sin_alpha
            A[i, 2] = -y_i * cos_alpha + x_i * sin_alpha
        
        # Right-hand side vector b (4x1)
        b = wheel_linear_velocities
        
        # Solve using SVD pseudo-inverse (more robust than QR)
        # This handles rank-deficient cases gracefully
        try:
            # Use SVD to compute pseudo-inverse
            U, s, Vt = np.linalg.svd(A, full_matrices=False)
            
            # Set tolerance for singular values (relative to largest)
            tol = max(A.shape) * np.finfo(s.dtype).eps
            s_inv = np.zeros_like(s)
            s_inv[s > tol] = 1.0 / s[s > tol]
            
            # Compute pseudo-inverse: A^+ = V * S^+ * U^T
            A_pinv = Vt.T @ np.diag(s_inv) @ U.T
            
            # Solve: x = A^+ * b
            x = A_pinv @ b
            
            # 노이즈로 인한 비정상 값 제한 (정지 시 v_y 폭발 등)
            x = self._clamp_twist(float(x[0]), float(x[1]), float(x[2]))
            
            # Check effective rank
            effective_rank = np.sum(s > tol)
            
            # Handle different rank cases
            if effective_rank < 2:
                # Rank 0 or 1: insufficient information
                # Fall back to simple heuristics
                avg_vel = np.mean(wheel_linear_velocities)
                avg_steering = np.mean(self.steering_angles)
                
                if abs(avg_vel) < 0.001:
                    # All wheels stopped
                    return np.array([0.0, 0.0, 0.0])
                elif abs(avg_steering) < 0.001:
                    # All wheels straight - pure translation
                    return np.array([avg_vel, 0.0, 0.0])
                else:
                    # All wheels have same steering angle
                    v_x = avg_vel * math.cos(avg_steering)
                    v_y = avg_vel * math.sin(avg_steering)
                    omega = 0.0  # Cannot determine from same steering angles
                    return np.array([v_x, v_y, omega])
            
            elif effective_rank == 2:
                # Rank 2: 조향각이 거의 같을 때 (예: 전부 0) v_y, omega가 불안정해질 수 있음
                # 조향각이 모두 작으면 직진으로 간주: v_y=0, omega=0, v_x만 평균 바퀴속도
                max_steering = np.max(np.abs(self.steering_angles))
                if max_steering < 0.02:  # ~1도 미만
                    avg_linear = np.mean(wheel_linear_velocities)
                    return np.array([avg_linear, 0.0, 0.0])
                return x
            
            else:
                # Rank 3: full rank, normal case
                return x
            
        except np.linalg.LinAlgError as e:
            self.get_logger().warn(f'Linear algebra error: {e}')
            # Fallback: return zero twist
            return np.array([0.0, 0.0, 0.0])
    
    def _clamp_twist(self, v_x, v_y, omega, max_linear=2.0, max_angular=2.0):
        """노이즈로 인한 비정상 twist 제한 (m/s, rad/s)"""
        v_x = max(-max_linear, min(max_linear, v_x))
        v_y = max(-max_linear, min(max_linear, v_y))
        omega = max(-max_angular, min(max_angular, omega))
        return np.array([v_x, v_y, omega])
    
    def calculate_odometry(self, dt):
        """
        Calculate odometry from body twist
        
        Args:
            dt: Time difference in seconds
        """
        if dt <= 0.0:
            return
        
        # Solve for body twist
        body_twist = self.solve_body_twist()
        
        if body_twist is None:
            return
        
        v_x, v_y, omega = body_twist
        
        # Mid-point integration for better accuracy
        # Δx_b, Δy_b, Δθ in body frame
        delta_x_b = v_x * dt
        delta_y_b = v_y * dt
        delta_theta = omega * dt
        
        # Update orientation first (mid-point)
        theta_mid = self.theta + delta_theta / 2.0
        
        # Transform body frame displacement to world frame
        cos_theta_mid = math.cos(theta_mid)
        sin_theta_mid = math.sin(theta_mid)
        
        # World frame displacement
        delta_x_w = cos_theta_mid * delta_x_b - sin_theta_mid * delta_y_b
        delta_y_w = sin_theta_mid * delta_x_b + cos_theta_mid * delta_y_b
        
        # Update position and orientation
        self.x += delta_x_w
        self.y += delta_y_w
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi] range
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
    
    def publish_odometry(self):
        """Publish odometry message"""
        # Use simulation time from /clock topic
        if self.current_sim_time is None:
            # Wait for first clock message
            return
        
        current_time = self.current_sim_time
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calculate time difference (in seconds)
        # Convert Time messages to seconds
        current_sec = current_time.sec + current_time.nanosec / 1e9
        last_sec = self.last_time.sec + self.last_time.nanosec / 1e9
        dt = current_sec - last_sec
        
        # Skip if time difference is invalid or too large (likely a time reset)
        if dt <= 0.0 or dt > 1.0:
            self.last_time = current_time
            return
        
        # Calculate odometry
        self.calculate_odometry(dt)
        
        # Solve for current body twist (for twist message)
        body_twist = self.solve_body_twist()
        
        # solve_body_twist always returns a valid array (never None)
        v_x, v_y, omega = body_twist
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time  # Use Time message directly
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation (quaternion)
        odom_msg.pose.pose.orientation = self.quaternion_from_yaw(self.theta)
        
        # Twist (body frame velocities)
        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.linear.y = v_y
        odom_msg.twist.twist.angular.z = omega
        
        # Covariance matrices
        # Pose covariance (6x6 matrix in row-major order)
        pose_cov = np.zeros(36)
        pose_cov[0] = 0.1   # x variance
        pose_cov[7] = 0.1   # y variance
        pose_cov[35] = 0.1  # yaw variance
        odom_msg.pose.covariance = pose_cov.tolist()
        
        # Twist covariance
        twist_cov = np.zeros(36)
        twist_cov[0] = 0.1   # v_x variance
        twist_cov[7] = 0.1   # v_y variance
        twist_cov[35] = 0.1  # ω variance
        odom_msg.twist.covariance = twist_cov.tolist()
        
        # Publish
        self.odom_pub.publish(odom_msg)
        
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
