"""ROS 订阅、服务和状态发布。任务顺序见 task_controller.py。"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger

from communication_pkg.my_cmd import MyCMD
from robot_interfaces.msg import Command, Feedback, MatchState, Objects, State
from ..task_controller import STOP, Task


class TaskNode(Node):
    """把 ROS 消息转给 Task。出发、搜寻和追踪的顺序由 Task 管。"""
    def __init__(self):
        super().__init__('task_node')
        # 从 robot.yaml 读取参数，再拼回原 Task 使用的 point / distribute / trans。
        point = self.parameters({'x0':640., 'y0':550.}, 'point.')
        distribute = self.parameters({'angle':0, 'move_ms':5000, 'scatter_ms':7000, 'base_forward':-100,
                                      'scatter_forward':-125, 'base_turn':281}, 'distribute.')
        trans = self.parameters({
            'search_camera_1':30, 'search_camera_2':20, 'first':True, 'confirm_frames':3, 'align_px':20,
            'camera_settle_ms':400, 'search_score_min':10, 'lost_move':200, 'lost_move_ms':1000,
            'track_gate_px':150, 'turn_only_px':80, 'turn_limit_px':100, 'move_limit_px':180,
            'distance_reward':.002,
        }, 'trans.')
        trans['score'] = self.parameters({'green':5., 'black':5., 'orange':2., 'count_reward':4.,
                                          'our_safe':6.}, 'trans.score.')
        # Task 调 MyCMD 的动作方法，MyCMD 发话题，SerialNode 才真正写串口。
        self.cmd = MyCMD(self.create_publisher(Command, 'robot/command', 10))
        self.task = Task(self.cmd, {'point':point,'distribute':distribute,'trans':trans})
        self.serial_open = False
        self.start_requested = False
        self.last_frame = float('-inf')
        self.last_stamp = 0
        self.last_status = None
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        frames_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.status = self.create_publisher(String, 'task/state', state_qos)
        self.match_status = self.create_publisher(MatchState, 'match/state', state_qos)
        self.target = self.create_publisher(Objects, 'task/target', frames_qos)
        # 话题 callback：串口状态交给 robot_state，角度回包交给 MyCMD，新检测帧交给 on_frame。
        self.create_subscription(State, 'robot/state', self.robot_state, state_qos)
        self.create_subscription(Feedback, 'robot/feedback', self.feedback_callback, 10)
        self.create_subscription(Objects, 'vision/objects', self.on_frame, frames_qos)
        # 服务 callback：等外部明确请求才开始任务。定时 callback watchdog 每 0.1 秒检查一次。
        self.create_service(Trigger, 'task/start', self.start)
        self.create_service(Trigger, 'task/stop', self.stop)
        self.create_service(Trigger, 'match/reset', self.reset)
        self.create_timer(.1, self.watchdog)
        self.publish_status()

    def parameters(self, defaults, prefix=''):
        return {name:self.declare_parameter(prefix+name,value).value for name,value in defaults.items()}

    def robot_state(self, message):
        self.serial_open = message.open
        self.task.team = message.team or None
        self.task.go_to.team = self.task.team
        if not self.serial_open:
            self.cmd.angle = None
            self.start_requested = False
        if not self.serial_open and self.task.state != STOP:
            self.task.stop()
            self.publish_status()

    def feedback_callback(self, message):
        self.cmd.read_feedback(message)
        if message.command == Feedback.START and message.value == 1 and self.task.state == STOP:
            self.start_requested = True
            self.try_start()

    def on_frame(self, message):
        """收到新感知帧时，推进 Task 一步，并把选中的目标发给预览。"""
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        age = self.get_clock().now().nanoseconds - stamp
        # 跳过过期、时间戳在未来、或比上次还旧的帧。任务推进只用新数据。
        if not 0 <= age <= 500_000_000 or stamp <= self.last_stamp:
            return
        self.last_stamp, self.last_frame = stamp, time.monotonic()
        if self.start_requested:
            self.try_start()
        if self.task.state == STOP:
            return
        # 转回原工程的物块字典，Task 和 GoTo 就能继续用旧接口。
        objs = [
            {'box':(obj.x1,obj.y1,obj.x2,obj.y2), 'name':obj.name, 'id':obj.id,
             'observed':obj.observed, 'in_safe_zone':obj.in_safe_zone, 'score':obj.score}
            for obj in message.items
        ]
        self.task.process_objects(objs)
        selected = None
        goto = self.task.go_to
        target = goto.rank2_target
        # 连续确认后才把追踪目标发给预览，并保留本帧的 header。
        if (self.task.state == 'track' and target and target.get('observed', True) and
                goto.rank2_hits >= goto.config.get('confirm_frames',3)):
            selected = next((obj for obj in message.items if obj.observed and
                             obj.name == target['name'] and obj.id == target['id']), None)
            if selected is not None:
                self.target.publish(Objects(header=message.header, items=[selected]))

        self.publish_status(selected)

    def publish_status(self, selected=None):
        stage = 'init_dis' if self.task.state == 'distribute' else self.task.state
        match = MatchState(stage=stage, frames=self.task.frame_count)
        if selected is not None:
            match.target_known = True
            match.target_id = selected.id
            match.target_name = selected.name
        if self.last_status is None or stage != self.last_status.stage:
            self.status.publish(String(data=json.dumps({'stage':stage}, separators=(',',':'))))
        if match != self.last_status:
            self.match_status.publish(match)
            self.last_status = match

    def try_start(self):
        if (not self.serial_open or self.task.team not in ('red', 'blue') or
                self.task.state != STOP or time.monotonic() - self.last_frame > .5):
            return False
        self.start_requested = False
        self.task.start()
        self.publish_status()
        return True

    def start(self, request, response):
        response.success = self.try_start()
        if not response.success:
            response.message = '需设置红蓝方、打开串口、有新检测帧且任务处于停止状态'
        return response

    def stop(self, request, response):
        self.start_requested = False
        self.task.stop()
        self.publish_status()
        response.success = True
        return response

    def reset(self, request, response):
        """清掉本场记录，要求下一次开始前收到复位之后的画面。"""
        self.start_requested = False
        self.task.reset()
        self.last_stamp = self.get_clock().now().nanoseconds
        self.last_frame = float('-inf')
        self.publish_status()
        response.success = True
        response.message = '上位机已复位，已发布停车指令' if self.serial_open else '上位机已复位，串口未打开'
        return response

    def watchdog(self):
        """半秒没有新检测帧就停车。"""
        if self.task.state == STOP:
            return
        if time.monotonic() - self.last_frame > .5:
            self.task.stop()
            self.publish_status()


def run_node(node_class):
    rclpy.init()
    node = None
    try:
        node = node_class()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    run_node(TaskNode)
