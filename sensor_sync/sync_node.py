#!/usr/bin/env python3
"""
Teensy Sync Controller ROS2 Node

Teensy 동기화 보드를 ROS2에서 제어하는 노드
- 시리얼 통신으로 Teensy와 연결
- 서비스: start, stop, trigger
- 파라미터: fps, port

사용법:
  ros2 run sync_controller sync_node
  ros2 run sync_controller sync_node --ros-args -p fps:=30 -p port:=/dev/ttyACM0

  ros2 service call /sync/start std_srvs/srv/Trigger
  ros2 service call /sync/stop std_srvs/srv/Trigger
  ros2 param set /sync_controller fps 60
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from std_srvs.srv import Trigger
from std_msgs.msg import String, Int32, Float32

import serial
import serial.tools.list_ports
import threading
import time


class SyncControllerNode(Node):
    def __init__(self):
        super().__init__('sync_controller')

        # 파라미터 선언
        self.declare_parameter('port', '/dev/ttyACM0',
            ParameterDescriptor(description='Teensy serial port'))
        self.declare_parameter('baud', 115200,
            ParameterDescriptor(description='Serial baud rate'))
        self.declare_parameter('fps', 1.0,
            ParameterDescriptor(description='Stereo camera FPS (1-120)'))
        self.declare_parameter('auto_connect', True,
            ParameterDescriptor(description='Auto connect on startup'))

        # 파라미터 값 가져오기
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.fps = self.get_parameter('fps').value
        self.auto_connect = self.get_parameter('auto_connect').value

        # 시리얼 연결
        self.serial = None
        self.connected = False
        self.running = False

        # 파라미터 변경 콜백
        self.add_on_set_parameters_callback(self.on_parameter_change)

        # 서비스 생성
        self.srv_start = self.create_service(Trigger, '~/start', self.start_callback)
        self.srv_stop = self.create_service(Trigger, '~/stop', self.stop_callback)
        self.srv_trigger = self.create_service(Trigger, '~/trigger', self.trigger_callback)
        self.srv_connect = self.create_service(Trigger, '~/connect', self.connect_callback)
        self.srv_disconnect = self.create_service(Trigger, '~/disconnect', self.disconnect_callback)

        # 퍼블리셔
        self.pub_status = self.create_publisher(String, '~/status', 10)
        self.pub_fps = self.create_publisher(Float32, '~/current_fps', 10)

        # 시리얼 읽기 스레드
        self.read_thread = None
        self.read_running = False

        # 상태 발행 타이머
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # 자동 연결
        if self.auto_connect:
            self.try_connect()

        self.get_logger().info(f'Sync Controller Node started')
        self.get_logger().info(f'  Port: {self.port}')
        self.get_logger().info(f'  FPS: {self.fps}')
        self.get_logger().info(f'Services: ~/start, ~/stop, ~/trigger, ~/connect, ~/disconnect')

    def find_teensy_port(self):
        """Teensy 포트 자동 검색"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Teensy VID:PID = 16C0:0483
            if port.vid == 0x16C0 and port.pid == 0x0483:
                return port.device
            # 일반적인 Teensy 포트 이름
            if 'ttyACM' in port.device or 'ttyUSB' in port.device:
                return port.device
        return None

    def try_connect(self):
        """시리얼 연결 시도"""
        if self.connected:
            return True

        # 포트 자동 검색
        port = self.port
        if port == 'auto' or not port:
            port = self.find_teensy_port()
            if not port:
                self.get_logger().warn('Teensy not found. Waiting...')
                return False

        try:
            self.serial = serial.Serial(port, self.baud, timeout=0.1)
            time.sleep(2)  # Teensy 리셋 대기
            self.connected = True
            self.port = port

            # 읽기 스레드 시작
            self.read_running = True
            self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.read_thread.start()

            self.get_logger().info(f'Connected to Teensy on {port}')

            # 초기 FPS 설정
            self.send_command(f'FPS {self.fps}')
            time.sleep(0.1)
            self.send_command('START')
            self.running = True

            return True

        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return False

    def disconnect(self):
        """시리얼 연결 해제"""
        self.read_running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)

        if self.serial and self.serial.is_open:
            self.send_command('STOP')
            time.sleep(0.1)
            self.serial.close()

        self.connected = False
        self.running = False
        self.get_logger().info('Disconnected from Teensy')

    def send_command(self, cmd):
        """Teensy에 명령어 전송"""
        if not self.connected or not self.serial:
            self.get_logger().warn('Not connected to Teensy')
            return False

        try:
            self.serial.write(f'{cmd}\n'.encode())
            self.get_logger().debug(f'Sent: {cmd}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Send failed: {e}')
            self.connected = False
            return False

    def serial_read_loop(self):
        """시리얼 읽기 루프 (별도 스레드)"""
        while self.read_running and self.serial:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.get_logger().debug(f'Teensy: {line}')
            except serial.SerialException:
                self.connected = False
                break
            except Exception as e:
                self.get_logger().debug(f'Read error: {e}')
            time.sleep(0.01)

    def on_parameter_change(self, params):
        """파라미터 변경 콜백"""
        for param in params:
            if param.name == 'fps':
                new_fps = param.value
                if 1.0 <= new_fps <= 120.0:
                    self.fps = new_fps
                    self.send_command(f'FPS {new_fps}')
                    self.get_logger().info(f'FPS changed to {new_fps}')
                else:
                    self.get_logger().warn(f'Invalid FPS: {new_fps} (must be 1-120)')
                    return SetParametersResult(successful=False)
            elif param.name == 'port':
                self.port = param.value

        return SetParametersResult(successful=True)

    def start_callback(self, request, response):
        """동기화 시작 서비스"""
        if not self.connected:
            if not self.try_connect():
                response.success = False
                response.message = 'Not connected to Teensy'
                return response

        self.send_command('START')
        self.running = True
        response.success = True
        response.message = f'Sync started at {self.fps} Hz'
        self.get_logger().info(response.message)
        return response

    def stop_callback(self, request, response):
        """동기화 중지 서비스"""
        if not self.connected:
            response.success = False
            response.message = 'Not connected to Teensy'
            return response

        self.send_command('STOP')
        self.running = False
        response.success = True
        response.message = 'Sync stopped'
        self.get_logger().info(response.message)
        return response

    def trigger_callback(self, request, response):
        """단일 트리거 서비스"""
        if not self.connected:
            response.success = False
            response.message = 'Not connected to Teensy'
            return response

        self.send_command('TRIGGER')
        response.success = True
        response.message = 'Single trigger sent'
        return response

    def connect_callback(self, request, response):
        """연결 서비스"""
        if self.connected:
            response.success = True
            response.message = f'Already connected to {self.port}'
            return response

        if self.try_connect():
            response.success = True
            response.message = f'Connected to {self.port}'
        else:
            response.success = False
            response.message = 'Failed to connect'
        return response

    def disconnect_callback(self, request, response):
        """연결 해제 서비스"""
        self.disconnect()
        response.success = True
        response.message = 'Disconnected'
        return response

    def publish_status(self):
        """상태 발행"""
        status = 'connected' if self.connected else 'disconnected'
        if self.connected:
            status += f', {"running" if self.running else "stopped"}'

        msg = String()
        msg.data = status
        self.pub_status.publish(msg)

        fps_msg = Float32()
        fps_msg.data = float(self.fps)
        self.pub_fps.publish(fps_msg)

    def destroy_node(self):
        """노드 종료"""
        self.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SyncControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
