"""ROS 订阅、服务和状态发布。任务顺序见 task_controller.py。"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger

from communication_pkg.my_cmd import MyCMD
from robot_interfaces.msg import Command, Feedback, MatchState, Objects, SafetyDetection, State
from ..task_controller import STOP, Task
from ..utils.my_timer import MyTimer


class TaskNode(Node):
    """把 ROS 消息转给 Task。出发、搜寻、追踪和抓取的顺序由 Task 管。"""
    def __init__(self):
        super().__init__('task_node')
        # 从 robot.yaml 读取参数，再拼回原 Task 使用的 point / distribute / trans。
        point = self.parameters({'x0':640., 'y0':550.}, 'point.')
        distribute = self.parameters({'angle':0, 'move_ms':5000, 'base_forward':-100, 'red_home_deg':0}, 'distribute.')
        trans = self.parameters({
            'search_camera_1':30, 'search_camera_2':20, 'first':True, 'confirm_frames':3, 'align_px':20,
            'camera_settle_ms':400, 'search_near_y':360,
            'lost_move':200, 'lost_move_ms':1000,
            'track_gate_px':150, 'turn_only_px':80, 'turn_limit_px':100, 'move_limit_px':180,
        }, 'trans.')
        trans['score'] = self.parameters({'green':4., 'black':4., 'orange':2.}, 'trans.score.')
        grab = self.parameters({'width':1280, 'height':720, 'camera_deg':55, 'camera_settle_ms':700,
                                'near_camera_deg':67, 'near_y':575,
                                'y_min':670, 'y_max':710,
                                'align_px':20, 'forward_px':150,
                                'confirm_frames':3, 'lost_frames':8, 'settle_ms':500,
                                'timeout_ms':25000, 'verify_deg':72, 'scatter_attempts':3,
                                'scatter_back_cm':8, 'scatter_push_cm':17,
                                'scatter_drive_timeout_ms':3000}, 'grab.')
        # Task 调 MyCMD 的动作方法，MyCMD 发话题，SerialNode 才真正写串口。
        self.cmd = MyCMD(self.create_publisher(Command, 'robot/command', 10))
        deliver = self.declare_parameter('delivery.enabled', False).value
        self.task = Task(self.cmd, {'point':point,'distribute':distribute,'trans':trans,'grab':grab,'deliver':deliver})
        self.safety = None
        self.serial_open = False
        self.robot = State()
        self.vision_timeout_s = self.declare_parameter('vision_timeout_s', .5).value
        if not 0 < self.vision_timeout_s <= 2.0:
            raise ValueError('vision_timeout_s 必须大于 0 且不超过 2 秒')
        self.last_frame = float('-inf')
        self.last_clock_offset = None
        self.last_stamp = 0
        self.last_received_frame = float('-inf')
        self.last_accepted_at = float('-inf')
        self.frame_counts = dict(accepted=0, expired=0, future=0, out_of_order=0)
        self.last_status = None
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        frames_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.use_sim_clock = self.declare_parameter('distribute.use_sim_clock', False).value
        self.sim_ns, self.sim_clock_at = 0, float('-inf')
        if self.use_sim_clock:
            self.task.match_timer = MyTimer(lambda: self.sim_ns // 1_000_000)
            self.task.distributor.timer = MyTimer(lambda: self.sim_ns // 1_000_000)
            self.task.grabber.timer = MyTimer(lambda: self.sim_ns // 1_000_000)
            self.task.grabber.phase_timer = MyTimer(lambda: self.sim_ns // 1_000_000)
            if self.task.delivery:
                self.task.delivery.timer = MyTimer(lambda: self.sim_ns // 1_000_000)
                self.task.delivery.clock = MyTimer(lambda: self.sim_ns // 1_000_000)
            self.create_subscription(UInt64, 'sim/clock', self.sim_clock_callback, 10)
        self.status = self.create_publisher(String, 'task/state', state_qos)
        self.match_status = self.create_publisher(MatchState, 'match/state', state_qos)
        self.target = self.create_publisher(Objects, 'task/target', frames_qos)
        # 话题 callback：串口状态交给 robot_state，角度回包交给 MyCMD，新检测帧交给 on_frame。
        self.create_subscription(State, 'robot/state', self.robot_state, state_qos)
        self.create_subscription(Feedback, 'robot/feedback', self.feedback_callback, 10)
        self.create_subscription(Objects, 'vision/objects', self.on_frame, frames_qos)
        if self.task.delivery:
            self.create_subscription(SafetyDetection, 'vision/safety_detection', self.safety_callback, frames_qos)
        # 服务 callback：等外部明确请求才开始任务。定时 callback watchdog 每 0.1 秒检查一次。
        self.create_service(Trigger, 'task/start', self.start)
        self.create_service(Trigger, 'task/stop', self.stop)
        self.create_service(Trigger, 'match/reset', self.reset)
        self.create_timer(.1, self.watchdog)
        self.publish_status()

    def parameters(self, defaults, prefix=''):
        return {name:self.declare_parameter(prefix+name,value).value for name,value in defaults.items()}

    def robot_state(self, message):
        self.robot = message
        self.serial_open = message.open
        self.task.team = message.team or None
        self.task.go_to.team = self.task.team
        if not self.serial_open:
            self.cmd.angle = None
        if not self.serial_open and self.task.running:
            self.task.stop()
        if self.task.state == STOP:
            self.publish_status()

    def feedback_callback(self, message):
        self.cmd.read_feedback(message)
        if message.command != Feedback.START:
            return
        if message.value == 2:
            self.reset_match()
        elif message.value == 1:
            self.try_start()

    def sim_clock_callback(self, message):
        if message.data != self.sim_ns:
            self.sim_ns, self.sim_clock_at = message.data, time.monotonic()

    def safety_callback(self, message):
        self.safety = message

    def on_frame(self, message):
        """收到新感知帧时，推进 Task 一步，并把选中的目标发给预览。"""
        stamp = Time.from_msg(message.header.stamp).nanoseconds
        now = self.get_clock().now().nanoseconds
        received_at = time.monotonic()
        if self.use_sim_clock:
            # WSL 校时可能向前跳。跨过校时的旧帧无法算帧龄，等下一张新图。
            offset = now / 1e9 - received_at
            if self.last_clock_offset is not None and offset - self.last_clock_offset > .25:
                self.last_stamp = now
            self.last_clock_offset = offset
        age = now - stamp
        self.last_received_frame = received_at
        # 跳过过期、时间戳在未来、或比上次还旧的帧。任务推进只用新数据。
        if not 0 <= age <= self.vision_timeout_s * 1e9:
            self.frame_counts['future' if age < 0 else 'expired'] += 1
            return
        if stamp <= self.last_stamp:
            self.frame_counts['out_of_order'] += 1
            return
        self.frame_counts['accepted'] += 1
        self.last_accepted_at = received_at
        self.last_stamp, self.last_frame = stamp, received_at - age / 1e9
        if not self.task.running:
            self.publish_status()
            return
        # 转回原工程的物块字典，Task 和 GoTo 就能继续用旧接口。
        objs = [
            {'box':(obj.x1,obj.y1,obj.x2,obj.y2), 'name':obj.name, 'id':obj.id,
             'observed':obj.observed, 'in_safe_zone':obj.in_safe_zone, 'score':obj.score}
            for obj in message.items
        ]
        searching = self.task.state in ('search', 'track')
        angle = self.task.go_to.camera_value if searching else self.task.camera_deg
        config = self.task.go_to.config if searching else self.task.grabber.config
        camera_ready = (self.robot.camera_known and self.robot.camera_deg == angle and
                        stamp >= Time.from_msg(self.robot.camera_changed).nanoseconds + config['camera_settle_ms'] * 1_000_000)
        safety = self.safety
        if safety is not None:
            capture = Time.from_msg(safety.header.stamp).nanoseconds
            elapsed = self.get_clock().now().nanoseconds - capture
            if (not self.robot.camera_known or self.robot.camera_deg != 25 or
                    capture < Time.from_msg(self.robot.camera_changed).nanoseconds + 700_000_000 or
                    not 0 <= elapsed <= self.vision_timeout_s * 1e9):
                safety = None
        task_time = self.sim_ns / 1e9 if self.use_sim_clock else received_at
        self.task.process_objects(objs, now=task_time, camera_ready=camera_ready, safety=safety, now_ns=now)
        selected = None
        goto = self.task.go_to
        target = goto.rank2_target
        # 锁定当帧就显示黄框，后续只显示同一目标的当前观测。
        if self.task.state == 'track' and target and not goto.rank2_misses:
            selected = next((obj for obj in message.items if obj.observed and not obj.in_safe_zone and
                             obj.name == target['name'] and obj.id == target['id']), None)
        elif self.task.state == 'grab' and camera_ready:
            selected = next((obj for obj in message.items if obj.observed and not obj.in_safe_zone
                             and (obj.name, obj.id) == self.task.grabber.target_key), None)
        if selected is not None:
            self.target.publish(Objects(header=message.header, items=[selected]))

        self.publish_status(selected)

    def publish_status(self, selected=None):
        stage = 'init_dis' if self.task.state == 'distribute' else self.task.state
        if stage == STOP:
            stage = 'waiting_start' if self.can_start() else 'waiting_ready'
        phase = self.task.grabber.phase if stage == 'grab' else ''
        if stage == 'init_dis':
            phase = self.task.distributor.phase
        if stage in ('deliver', 'align_zone') and self.task.delivery.moves:
            phase = self.task.delivery.moves[0][0]
        match = MatchState(stage=stage, phase=phase, frames=self.task.frame_count)
        if selected is not None:
            match.target_known = True
            match.target_id = selected.id
            match.target_name = selected.name
        if self.last_status is None or stage != self.last_status.stage:
            self.status.publish(String(data=json.dumps({'stage':stage}, separators=(',',':'))))
        if match != self.last_status:
            self.match_status.publish(match)
            self.last_status = match

    def can_start(self):
        return (self.serial_open and self.task.team in ('red', 'blue') and
                self.task.state == STOP and time.monotonic() - self.last_frame <= self.vision_timeout_s and
                (not self.use_sim_clock or time.monotonic() - self.sim_clock_at <= 1.2))

    def try_start(self):
        if not self.can_start():
            return False
        self.task.start()
        self.publish_status()
        return True

    def start(self, request, response):
        response.success = self.try_start()
        if not response.success:
            response.message = '需设置红蓝方、打开串口、有新检测帧且任务处于停止状态'
        return response

    def stop(self, request, response):
        self.task.stop()
        self.publish_status()
        response.success = True
        return response

    def reset(self, request, response):
        self.reset_match()
        response.success = True
        response.message = '已复位，等待启动'
        return response

    def reset_match(self):
        """清掉本场记录，要求下一次开始前收到复位之后的画面。"""
        self.task.reset()
        self.last_stamp = self.get_clock().now().nanoseconds
        self.last_frame = float('-inf')
        self.last_received_frame = float('-inf')
        self.last_accepted_at = float('-inf')
        self.frame_counts = dict.fromkeys(self.frame_counts, 0)
        self.publish_status()

    def watchdog(self):
        """检测帧超时就停车。"""
        if not self.task.running or self.task.check_match_time():
            self.publish_status()
            return
        if time.monotonic() - self.last_frame > self.vision_timeout_s:
            now = time.monotonic()
            since_received = now - self.last_received_frame
            since_accepted = now - self.last_accepted_at
            accepted_age = self.last_accepted_at - self.last_frame
            self.get_logger().warning(
                f'[TaskNode] 检测画面超时，停车并等待启动，'
                f'距最近一次收到检测 {since_received:.2f} s，'
                f'距最近一次接受检测 {since_accepted:.2f} s，'
                f'该帧到达时已过去 {accepted_age:.2f} s，帧统计 {self.frame_counts}'
            )
            self.task.stop()
            self.publish_status()
        elif self.use_sim_clock and time.monotonic() - self.sim_clock_at > 1.2:
            self.get_logger().warning('[TaskNode] 仿真物理时间停滞，停车并等待启动')
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
