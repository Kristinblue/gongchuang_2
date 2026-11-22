"""对应原工程单项靠近测试，每帧选择指定颜色中最近的目标。"""
import math
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger

from robot_interfaces.msg import Command, Objects, State

from ..utils.runner import run_node


class ApproachTest(Node):
    """单独测目标靠近。每帧选一个目标，计算水平和前后像素误差。"""
    def __init__(self):
        super().__init__('approach_test_node')
        self.color = self.declare_parameter('color', 'orange').value
        self.x = self.declare_parameter('point.x0', 640.).value
        self.y = self.declare_parameter('point.y0', 550.).value
        self.turn_limit = self.declare_parameter('turn_limit_px', 100).value
        self.move_limit = self.declare_parameter('move_limit_px', 180).value
        if not all(map(math.isfinite,(self.x,self.y))) or not all(
                1 <= limit <= 32767 for limit in (self.turn_limit,self.move_limit)):
            raise ValueError('无效参考点或误差限幅')
        self.active = self.serial_open = self.halted = False
        self.last_stamp = 0
        self.last_frame = float('-inf')
        self.errors = self.create_publisher(Command, 'approach_test/command', 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.target = self.create_publisher(Objects, 'approach_test/target', qos)
        # callback：收到感知结果就算误差。默认只发调试话题，drive 打开且 start 成功后才发给串口。
        self.create_subscription(Objects, 'vision/objects', self.on_frame, qos)
        self.publish_motion = self.declare_parameter('publish_motion', False).value
        drive = self.declare_parameter('drive', False).value
        if drive or self.publish_motion:
            self.commands = self.create_publisher(Command, 'robot/command', 10)
        if drive:
            self.create_subscription(State, 'robot/state', self.robot_state,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_service(Trigger, 'approach_test/start', self.start)
            self.create_service(Trigger, 'approach_test/stop', self.stop)
            self.create_timer(.1, self.watchdog)

    def on_frame(self, frame):
        stamp = Time.from_msg(frame.header.stamp).nanoseconds
        if stamp <= self.last_stamp or not 0 <= self.get_clock().now().nanoseconds - stamp <= 500_000_000:
            return
        self.last_stamp, self.last_frame = stamp, time.monotonic()
        # 只选当前帧实际看到的指定颜色。用框的底边中点和参考点 (x, y) 算距离。
        candidates = (obj for obj in frame.items if obj.observed and obj.name == self.color and
                      all(map(math.isfinite, (obj.x1,obj.x2,obj.y2))))
        target = min(candidates, key=lambda obj:math.hypot((obj.x1+obj.x2)*.5-self.x, obj.y2-self.y),
                     default=None)
        if target is None:
            # 目标丢失时补一次停车，免得下位机一直保持上一条误差继续冲；同一次丢失只发一条。
            if not self.halted:
                self.halted = True
                self.send(Command.STOP,0)
            return
        self.halted = False
        self.target.publish(Objects(header=frame.header, items=[target]))
        # 横向误差给转向（02），纵向误差给前后（01），都限制在各自参数的范围内。
        self.send(Command.TURN, max(-self.turn_limit,min(self.turn_limit,(target.x1+target.x2)*.5-self.x)))
        self.send(Command.STRAIGHT, -max(-self.move_limit,min(self.move_limit,target.y2-self.y)))

    def send(self, command, value):
        message = Command(command=command,value=int(value))
        self.errors.publish(message)
        if self.active or self.publish_motion:
            self.commands.publish(message)

    def robot_state(self, state):
        self.serial_open = state.open
        if not self.serial_open and self.active:
            self.stop(None, Trigger.Response())

    def start(self, request, response):
        self.active = self.serial_open and time.monotonic() - self.last_frame <= .5
        response.success = self.active
        if not self.active:
            response.message = '需要打开串口和新检测帧'
        return response

    def stop(self, request, response):
        if self.active:
            self.send(Command.STOP,0)
        self.active = False
        response.success = True
        return response

    def watchdog(self):
        if self.active and time.monotonic() - self.last_frame > .5:
            self.stop(None, Trigger.Response())


def main():
    run_node(ApproachTest)
