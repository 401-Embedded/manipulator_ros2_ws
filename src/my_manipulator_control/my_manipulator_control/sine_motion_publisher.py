#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class SineMotionPublisher(Node):
    def __init__(self):
        super().__init__('sine_motion_publisher')
        
        # JointState publisher 생성
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # 타이머 생성 (50Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # 시간 변수
        self.t = 0.0
        
        # Joint 이름들
        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'gripper_left_joint_1'
        ]
        
        self.get_logger().info('Sine Motion Publisher 시작!')

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        # Sin 파형으로 각 joint 움직이기 (0~180도 범위)
        # joint1~3: 90도를 중심으로 ±30도 움직임 (60~120도)
        # joint4, gripper: 180도에서 시작, 줄었다가 180도로 복귀 (150~180도)
        msg.position = [
            1.5708 + math.sin(self.t) * 0.5236,                    # joint1: 90도±30도 (60~120도)
            1.5708 + math.sin(self.t * 0.5) * 0.5236,             # joint2: 90도±30도 (60~120도)
            0.5236 * (1 - math.cos(self.t * 0.7)),                # joint3: 0도에서 60도까지 (0~60도)
            3.1416 - 0.5236 * (1 - math.cos(self.t * 1.2)),       # joint4: 180도에서 120도까지 (120~180도)
            3.1416 - 0.5236 * (1 - math.cos(self.t * 2.0))        # gripper: 180도에서 120도까지 (120~180도)
        ]
        
        self.publisher_.publish(msg)
        self.t += 0.02


def main(args=None):
    rclpy.init(args=args)
    node = SineMotionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
