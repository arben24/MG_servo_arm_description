#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

start_array = [70, 85, 45, 75, 30, 30]

class JointStateRelayNode(Node):
    def __init__(self):
        super().__init__('joint_state_relay')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(JointState, 'joint_states_test', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.latest_joint_state = None
        self.latest_joint_state_buffer = None
        self.count = 0
        self.init = 0

    def listener_callback(self, msg):
        self.latest_joint_state = msg

    def timer_callback(self):
        if self.latest_joint_state != self.latest_joint_state_buffer:
            self.init = 1
            for i in range(len(self.latest_joint_state.position)):
                self.latest_joint_state.position[i] = start_array[i] + math.degrees(self.latest_joint_state.position[i])
            
            self.publisher.publish(self.latest_joint_state)
            self.get_logger().info(f'pub Nr. {self.count}')
            self.count += 1
            self.latest_joint_state_buffer = self.latest_joint_state
            
        if self.init:
            self.publisher.publish(self.latest_joint_state_buffer)
            self.get_logger().info(f'no change pub Nr. {self.count}')
            self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

