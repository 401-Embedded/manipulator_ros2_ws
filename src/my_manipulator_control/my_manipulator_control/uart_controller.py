#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math


class UARTController(Node):
    def __init__(self):
        super().__init__('uart_controller')
        
        # 파라미터 선언
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        
        # Serial 포트 초기화
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'UART 연결 성공: {port} @ {baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'UART 연결 실패: {e}')
            self.serial_port = None
        
        # JointState subscriber 생성
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('UART Controller 시작!')

    def joint_state_callback(self, msg):
        """JointState를 받아서 각도(degree)로 변환 후 UART로 전송"""
        if self.serial_port is None or not self.serial_port.is_open:
            return
        
        if len(msg.position) < 5:
            self.get_logger().warn('조인트 개수가 5개 미만입니다.')
            return
        
        # 라디안을 도(degree)로 변환하고 정수로 변환
        angles = []
        for pos in msg.position[:5]:  # 처음 5개 조인트만 사용
            degree = int(math.degrees(pos))
            angles.append(degree)
        
        # UART 메시지 생성: "90 90 0 180 180\n"
        uart_msg = ' '.join(map(str, angles)) + '\n'
        
        try:
            self.serial_port.write(uart_msg.encode('utf-8'))
            self.get_logger().debug(f'전송: {uart_msg.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'UART 전송 실패: {e}')

    def destroy_node(self):
        """노드 종료 시 시리얼 포트 닫기"""
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('UART 포트 닫힘')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UARTController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
