#!/usr/bin/env python3
"""Keyboard teleop for aruco_box in Gazebo (discrete planar motion)."""

import math
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from gazebo_msgs.srv import SetEntityState, GetEntityState

CANDIDATE_SERVICES = [
    '/gazebo/set_entity_state',
    '/world/set_entity_state',
    '/world/world/set_entity_state'
]

HELP_TEXT = """
Box Keyboard Controller
---------------------------
  w / s : move +X / -X (step)
  a / d : move +Y / -Y (step)
  q / e : rotate yaw left / right (step)
  space : reset pose
  k     : stop (no movement)
  h     : print this help
  Ctrl+C: exit
"""


def clamp(value, limit):
    if limit <= 0:
        return value
    return max(min(value, limit), -limit)


class BoxKeyboardController(Node):
    def __init__(self):
        super().__init__('box_keyboard_controller')

        self.entity_base_name = self.declare_parameter('entity_name', 'aruco_box').get_parameter_value().string_value
        self.initial_x = self.declare_parameter('initial_x', 3.0).get_parameter_value().double_value
        self.initial_y = self.declare_parameter('initial_y', 0.0).get_parameter_value().double_value
        self.initial_z = self.declare_parameter('initial_z', 1.5).get_parameter_value().double_value
        self.step = self.declare_parameter('step', 0.1).get_parameter_value().double_value
        self.yaw_step = self.declare_parameter('yaw_step', 0.1).get_parameter_value().double_value
        self.position_limit = self.declare_parameter('position_limit', 0.0).get_parameter_value().double_value

        self.pose = Pose()
        self.pose.position = Point(x=self.initial_x, y=self.initial_y, z=self.initial_z)
        self.yaw = 0.0

        self.client = None
        self.service_name = ''
        for name in CANDIDATE_SERVICES:
            client = self.create_client(SetEntityState, name)
            self.get_logger().info(f'Trying service {name} ...')
            if client.wait_for_service(timeout_sec=2.0):
                self.client = client
                self.service_name = name
                break
        if self.client is None:
            raise RuntimeError('No SetEntityState service available. Ensure Gazebo is launched with ROS factory plugin.')
        self.get_logger().info(f'Connected to {self.service_name}')

        self.entity_names = [
            self.entity_base_name,
            f'world::{self.entity_base_name}',
            f'/world::{self.entity_base_name}'
        ]

        self._term_settings = termios.tcgetattr(sys.stdin)
        
        # Try to get current pose from Gazebo
        self._get_current_pose_from_gazebo()
        
        # Set initial orientation
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        self.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        
        print(HELP_TEXT)
        self.get_logger().info(f'Box controller initialized at x={self.pose.position.x:.2f}, y={self.pose.position.y:.2f}, z={self.pose.position.z:.2f}')

    def destroy_node(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._term_settings)
        except termios.error:
            pass
        super().destroy_node()
    
    def _get_current_pose_from_gazebo(self):
        """Try to get the current pose of the entity from Gazebo"""
        get_services = [
            '/gazebo/get_entity_state',
            '/world/get_entity_state',
            '/world/world/get_entity_state'
        ]
        
        for service_name in get_services:
            get_client = self.create_client(GetEntityState, service_name)
            if not get_client.wait_for_service(timeout_sec=1.0):
                continue
            
            # Try each entity name
            for entity_name in self.entity_names:
                req = GetEntityState.Request()
                req.name = entity_name
                req.reference_frame = 'world'
                
                future = get_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                
                if future.done() and future.result() and future.result().success:
                    result = future.result()
                    self.pose.position = result.state.pose.position
                    self.pose.orientation = result.state.pose.orientation
                    
                    # Extract yaw from quaternion
                    qz = result.state.pose.orientation.z
                    qw = result.state.pose.orientation.w
                    self.yaw = 2.0 * math.atan2(qz, qw)
                    
                    self.get_logger().info(f'Retrieved current pose from Gazebo: entity={entity_name}')
                    return True
        
        self.get_logger().warn('Could not retrieve current pose from Gazebo, using default initial pose')
        return False

    def _send_pose(self):
        req = SetEntityState.Request()
        req.state.pose = self.pose
        req.state.twist = Twist()
        req.state.reference_frame = 'world'

        for name in self.entity_names:
            req.state.name = name
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result and result.success:
                self.get_logger().debug(f'Pose update succeeded for entity {name}')
                return
        self.get_logger().warn('Failed to update pose for any entity name candidate')

    def _log_pose(self):
        self.get_logger().info(
            f"Pose -> x: {self.pose.position.x:.2f}, y: {self.pose.position.y:.2f}, z: {self.pose.position.z:.2f}, yaw: {self.yaw:.2f}"
        )

    def update_pose(self, dx=0.0, dy=0.0, dyaw=0.0, reset=False):
        if reset:
            self.pose.position.x = self.initial_x
            self.pose.position.y = self.initial_y
            self.pose.position.z = self.initial_z
            self.yaw = 0.0
        else:
            new_x = self.pose.position.x + dx
            new_y = self.pose.position.y + dy
            if self.position_limit > 0:
                new_x = clamp(new_x, self.position_limit)
                new_y = clamp(new_y, self.position_limit)
            self.pose.position.x = new_x
            self.pose.position.y = new_y
            # Keep current z position (don't reset to initial_z)
            # self.pose.position.z = self.initial_z  # REMOVED - maintains current height
            self.yaw += dyaw

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        self.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        self._send_pose()
        self._log_pose()

    def _read_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._term_settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self._read_key()
                if not key:
                    continue

                if key == 'w':
                    self.update_pose(dx=self.step)
                elif key == 's':
                    self.update_pose(dx=-self.step)
                elif key == 'a':
                    self.update_pose(dy=self.step)
                elif key == 'd':
                    self.update_pose(dy=-self.step)
                elif key == 'q':
                    self.update_pose(dyaw=self.yaw_step)
                elif key == 'e':
                    self.update_pose(dyaw=-self.yaw_step)
                elif key == 'k':
                    self.get_logger().info('Stop command received (no motion applied)')
                elif key == ' ':
                    self.update_pose(reset=True)
                elif key == 'h':
                    print(HELP_TEXT)
                elif key == '':
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._term_settings)


def main(args=None):
    rclpy.init(args=args)
    try:
        controller = BoxKeyboardController()
    except Exception:
        rclpy.shutdown()
        raise

    try:
        controller.run()
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
