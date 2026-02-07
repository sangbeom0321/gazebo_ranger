#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ModelStatesToLocal(Node):
    def __init__(self):
        super().__init__('model_states_to_local')

        # ── Parameters (override with --ros-args -p key:=value)
        self.declare_parameter('model_states_topic', '/world/model_states_world')
        self.declare_parameter('model_name', 'ranger_mini')
        self.declare_parameter('prefix_match', True)          # True: 'ranger_mini_0'도 매칭
        self.declare_parameter('warn_period_sec', 2.0)        # 경고 최소 간격(초)

        self.model_states_topic = self.get_parameter('model_states_topic').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.prefix_match = self.get_parameter('prefix_match').get_parameter_value().bool_value
        self.warn_period_sec = self.get_parameter('warn_period_sec').get_parameter_value().double_value

        # QoS (Gazebo는 보통 RELIABLE/volatile. BEST_EFFORT도 대개 수용됨)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=50
        )

        # Subscribers / Publishers
        self.sub = self.create_subscription(
            ModelStates,
            self.model_states_topic,
            self.model_states_cb,
            qos_profile=qos_profile
        )
        self.xy_pub = self.create_publisher(Float64MultiArray, '/Local/utm', 10)
        self.yaw_pub = self.create_publisher(Float64, '/Local/heading', 10)

        self.get_logger().info(
            f"Listening to [{self.model_states_topic}] for model [{self.model_name}] "
            f"(prefix_match={self.prefix_match}) and publishing /Local/utm (x,y), /Local/heading (yaw, rad)."
        )

        # 내부 상태: 경고 쓰로틀링
        self._last_warn_ns = 0
        self._printed_name_list = False  # 초기에 한 번만 name 목록 찍기

    def _throttled_warn(self, msg: str):
        """Python rclpy에는 warn_throttle이 없으므로, clock 기반으로 직접 구현."""
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_warn_ns) >= int(self.warn_period_sec * 1e9):
            self.get_logger().warn(msg)
            self._last_warn_ns = now_ns

    def _find_model_index(self, names):
        """정확 매칭 → prefix 매칭(옵션) 순으로 인덱스 반환. 못 찾으면 None."""
        try:
            return names.index(self.model_name)
        except ValueError:
            if self.prefix_match:
                for i, n in enumerate(names):
                    if n.startswith(self.model_name):
                        return i
        return None

    def model_states_cb(self, msg: ModelStates):
        # model_name 인덱스 찾기 (못찾아도 노드 계속 실행)
        idx = self._find_model_index(msg.name)
        if idx is None:
            # 첫 실패 시에만 현재 이름 목록을 한 번 보여주면 디버깅에 유용함
            if not self._printed_name_list:
                self.get_logger().info(f"Available models in ModelStates: {list(msg.name)}")
                self._printed_name_list = True
            self._throttled_warn(f"Model [{self.model_name}] not found in ModelStates.")
            return

        # 찾았으면 목록 출력 플래그 리셋(환경이 바뀌었을 수 있으니)
        self._printed_name_list = False

        pose: Pose = msg.pose[idx]
        x = pose.position.x
        y = pose.position.y

        # yaw 추출 (rad)
        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Publish x,y
        xy_msg = Float64MultiArray()
        xy_msg.data = [x, y]
        self.xy_pub.publish(xy_msg)

        # Publish yaw(rad)
        yaw_msg = Float64()
        yaw_msg.data = float(yaw)
        self.yaw_pub.publish(yaw_msg)

        # 필요하면 디버그
        # self.get_logger().debug(f"UTM(x,y)=({x:.3f},{y:.3f}), yaw={yaw:.3f} rad")

def main(args=None):
    rclpy.init(args=args)
    node = ModelStatesToLocal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
