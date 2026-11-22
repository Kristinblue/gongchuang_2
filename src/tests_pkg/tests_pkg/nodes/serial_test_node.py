"""在终端调用 MyCMD 的动作接口，串口收发交给 serial_node。"""

import os
import sys
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from robot_interfaces.msg import Command, SerialFrame

from communication_pkg.my_cmd import MyCMD


HELP = '''00  启动时查询队伍（实车串口自动发送）
01  move_straight 100   前后像素误差，正值向前，负值向后
02  move_turn -50       转向像素误差，正值向右，负值向左
03  grab 0              夹爪，0 开，1 关
04  camera 20           相机俯角，0～80 度
05  move_turn_angle 30  相对转角（度），正值向右，负值向左
06  angle_calib 90      设置当前绝对角度，暂用于模拟测试
07  move_straight_dis 20  按厘米直行，正值向前，负值向后
08  move_stop / stop    停车
    help / quit         查看命令 / 退出'''


class SerialTest(Node):
    def __init__(self):
        super().__init__('serial_test_node')
        self.publisher = self.create_publisher(Command, 'robot/command', 10)
        self.cmd = MyCMD(self.publisher)
        if os.environ.get('GONGCHUANG_SERIAL_FRAMES') == '1':
            self.create_subscription(SerialFrame, 'robot/serial_frames', self.frame_callback, 100)
        self.actions = {
            'move_straight': (self.cmd.move_straight, -32768, 32767),
            'move_straight_dis': (self.cmd.move_straight_dis, -32768, 32767),
            'move_turn': (self.cmd.move_turn, -32768, 32767),
            'move_turn_angle': (self.cmd.move_turn_angle, -32768, 32767),
            'grab': (self.cmd.grab, 0, 1),
            'camera': (self.cmd.camera, 0, 80),
            'angle_calib': (self.cmd.angle_calib, -32768, 32767),
            'set_angle': (self.cmd.angle_calib, -32768, 32767),
        }

    def run(self):
        """等串口节点订阅命令话题，然后逐行读取测试命令。"""
        deadline = time.monotonic() + 3.0
        while not self.publisher.get_subscription_count():
            if time.monotonic() >= deadline:
                self.get_logger().error('[SerialTest] 串口节点未订阅命令话题')
                return 1
            time.sleep(0.1)

        print(HELP)
        while True:
            parts = input('cmd> ').split()
            if parts == ['quit']:
                return 0
            if parts in (['stop'], ['move_stop']):
                self.stop()
                print('[SerialTest] 已发布停车指令')
            elif parts == ['help']:
                print(HELP)
            elif len(parts) == 2 and parts[0] in self.actions:
                self.run_action(*parts)
            elif parts:
                print('[SerialTest] 输入 help 查看命令和参数')

    def frame_callback(self, frame):
        """把串口桥实际记录的帧交给客户端串口页。"""
        direction = 'TX' if frame.tx else 'RX'
        hex_bytes = bytes(frame.data).hex(' ').upper()
        os.write(sys.stdout.fileno(), f'\x1e{direction} {hex_bytes}\n'.encode('ascii'))

    def run_action(self, name, argument):
        """把整数参数交给对应的 MyCMD 方法。"""
        action, low, high = self.actions[name]
        try:
            value = int(argument)
        except ValueError:
            print('[SerialTest] 参数请输入整数')
            return
        if not low <= value <= high:
            print(f'[SerialTest] {name} 参数范围为 {low} 到 {high}')
            return

        if name == 'move_turn_angle' and value == 0:
            print('[SerialTest] 相对转角为 0，无需旋转')
            return
        action(value)
        self.publisher.wait_for_all_acked(Duration(seconds=0.5))
        if name in ('angle_calib', 'set_angle'):
            print(f'[SerialTest] 已发布绝对角度 {value}，06 仅用于伪串口调试')
        else:
            print(f'[SerialTest] 已发布 {name} {value}')

    def stop(self):
        self.cmd.move_stop()
        self.publisher.wait_for_all_acked(Duration(seconds=0.5))


def main():
    # Ctrl+C 先触发 Python 的 finally，让前进测试能发出停车指令。
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SerialTest()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=executor.spin)
    worker.start()
    try:
        return node.run()
    except (EOFError, KeyboardInterrupt):
        print()
        return 0
    finally:
        executor.shutdown()
        worker.join()
        node.destroy_node()
        rclpy.try_shutdown()
