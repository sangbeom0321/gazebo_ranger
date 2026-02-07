#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class FakeLocalPathPublisher(Node):
    def __init__(self):
        super().__init__('fake_local_path_publisher')
        self.publisher_ = self.create_publisher(Path, '/Planning/local_path', 10)
        self.timer = self.create_timer(0.1, self.publish_fake_path)  # 10Hz

        # Path 메시지 기본 틀
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'  # 필요 시 frame_id 수정

        # Pose 하나 생성 (0.1, 0.1)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 0.1
        pose.pose.position.y = 0.1
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # 방향 없음
        self.path_msg.poses = [pose]

        self.get_logger().info("Publishing fake local path to /Planning/local_path")

    def publish_fake_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses[0].header.stamp = self.path_msg.header.stamp
        self.publisher_.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLocalPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()