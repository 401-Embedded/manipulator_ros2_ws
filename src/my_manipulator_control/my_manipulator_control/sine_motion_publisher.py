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
        self.timer = self.create_timer(0.02, self.timer_callback)
        
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
        
        # Sin 파형으로 각 joint 움직이기
        msg.position = [
            math.sin(self.t),                    # joint1: z축 회전
            math.sin(self.t * 0.5) * 1.0,       # joint2: y축 회전
            math.sin(self.t * 0.7) * 0.8,       # joint3: y축 회전
            math.sin(self.t * 1.2) * 1.5,       # joint4: z축 회전
            math.sin(self.t * 2.0) * 0.5        # gripper: 열고 닫기
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
