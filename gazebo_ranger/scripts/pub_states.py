#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ranger_msgs.msg import ActuatorStateArray
from ranger_msgs.msg import ActuatorState

class JointToRangerPublisher(Node):
    def __init__(self):
        super().__init__('joint_to_ranger_publisher')
        self.publisher_ = self.create_publisher(ActuatorStateArray, '/ranger_states', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.velocity = 0.0
        self.angle = 0.0
        self.indexes_initialized = True
        self.wheel_indexes = {}
        self.steering_wheel_indexes = {}

    def initialize_indexes(self, msg):
        wheel_names = ["fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel"]
        steering_wheel_names = ["fl_steering_wheel", "fr_steering_wheel", "rl_steering_wheel", "rr_steering_wheel"]
    
        for wheel_name in wheel_names:
            self.wheel_indexes[wheel_name] = next((i for i, name in enumerate(msg.name) if name == wheel_name), None)

        for steering_name in steering_wheel_names:
            self.steering_wheel_indexes[steering_name] = next((i for i, name in enumerate(msg.name) if name == steering_name), None)

        # Check for any None values or duplicate indexes
        all_indexes = list(self.wheel_indexes.values()) + list(self.steering_wheel_indexes.values())
        unique_indexes = set(all_indexes)
        
        # Check if all indexes are found and there are no duplicates (excluding None)
        all_indexes_found = None not in unique_indexes and len(unique_indexes) == len(all_indexes)

        if all_indexes_found:
            self.get_logger().info("All required indexes were successfully found.")
            self.indexes_initialized = False
        else:
            self.get_logger().warning("Not all indexes were found. Will retry on next message.")

    def listener_callback(self, msg):
        
        if self.indexes_initialized and 0 < len(msg.velocity):
            self.initialize_indexes(msg)
        ranger_msg = ActuatorStateArray()

        # msg.name에서 각 휠 및 조향 휠에 해당하는 인덱스를 찾아 해당 값을 할당
        if len(msg.velocity) and len(msg.position) != 0:
            for wheel_name, index in self.wheel_indexes.items():
                actuator_state = ActuatorState()
                actuator_state.id = index

                if index is not None and index < len(msg.velocity):
                    # 휠속도 할당
                    actuator_state.motor.driver_state = msg.velocity[index]
                ranger_msg.states.append(actuator_state)

            for steering_name, index in self.steering_wheel_indexes.items():
                actuator_state = ActuatorState()
                actuator_state.id = index
                if index is not None and index < len(msg.position):
                    # 조향 각도 할당
                    actuator_state.motor.driver_state = msg.position[index]
                ranger_msg.states.append(actuator_state)

        self.publisher_.publish(ranger_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_to_ranger_publisher = JointToRangerPublisher()
    rclpy.spin(joint_to_ranger_publisher)
    # Destroy the node explicitly
    joint_to_ranger_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()