"""
这里负责整体的逻辑框架,以及各种类的构建让main看起来清爽，或者包一些别的
"""

import time

from .delivery import Delivery
from .init_dis import InitDisributor
from .grab.grabber import Grabber
from .grab.go_to import GoTo

STOP = 'stop'
GRAB_CLOSED = 'grab_closed'


class Task:
    """逐帧调度出发、搜寻、追踪和抓取。"""

    def __init__(self, cmd, config):
        self.cmd = cmd
        self.team, self.note = None, ''
        point = config.get('point',{})
        self.x0, self.y0 = point.get('x0',640.), point.get('y0',550.)
        # Task 管阶段。InitDisributor 按时间打散，GoTo 按每帧的物块位置搜寻和靠近。
        self.distributor = InitDisributor(cmd,config.get('distribute',{}))
        trans = config.get('trans',{})
        grab = config.get('grab', {})
        self.go_to = GoTo(self.x0, self.y0, grab.get('width', 1280), grab.get('height', 720), first=trans.get('first',True), config=trans, cmd=cmd)
        self.grabber = Grabber(cmd, self.x0, grab)
        self.delivery = Delivery(cmd, grab) if config.get('deliver') else None
        self.state, self.camera_deg = STOP, self.distributor.angle
        self.frame_count = 0

    def start(self):
        """重置任务，等待新帧推进打散。"""
        self.frame_count = 0
        self.camera_deg = self.distributor.angle
        self._change_state('distribute')

    def stop(self):
        """结束当前任务并停车。"""
        self._change_state(STOP)

    @property
    def running(self):
        return self.state not in (STOP, GRAB_CLOSED, 'delivery_timeout')

    def reset(self):
        """停车并清空上位机的本场进度。"""
        self.stop()
        self.distributor.reset()
        self.go_to.reset()
        self.grabber.reset()
        if self.delivery:
            self.delivery.reset()
        self.camera_deg = self.distributor.angle
        self.frame_count = 0
        self.note = ''

    def process_objects(self, objs, now=None, camera_ready=True, safety=None):
        """接收当前帧的感知结果，不在任务内重新做推理。"""
        self._now = time.monotonic() if now is None else now
        if not self.running:
            return
        self.frame_count += 1
        if self.state == 'delivery_done':
            # 一趟的投放动作结束。下一趟可以选绿、黑、橙，直接搜寻。
            self.go_to.first = False
            self._change_state('search')
            return
        # 初次打散只按时间执行，不调用物块检测或计数。
        if self.state=='distribute':
            if self.distributor.run():
                self._change_state('search')
            else:
                self._note(self.distributor.note)
            return
        try:
            if self.state in ('seek', 'align_zone', 'deliver', 'clear_lane'):
                state = self.delivery.run(safety, objs, camera_ready)
                self.camera_deg = self.delivery.camera_deg
                self._note(self.delivery.note)
                if state in ('delivery_done', 'delivery_timeout'):
                    self._change_state(state)
                else:
                    self.state = state
                return
            if self.state == 'grab':
                state = self.grabber.run(objs,camera_ready)
                self.camera_deg = self.grabber.camera_deg
                self._note(self.grabber.note)
                if state == GRAB_CLOSED and self.delivery:
                    self._change_state('seek')
                    self.delivery.start(self.team, self.grabber.target_key[0])
                    self.camera_deg = self.delivery.camera_deg
                elif state != self.state:
                    self._change_state(state)
                return
            # GoTo 的返回值表示有没有到位。还在运行时，用 GoTo 自己的状态更新当前阶段。
            if self.go_to.search_step >= 0 and not camera_ready:
                return
            result = self.go_to.run(objs,now=self._now)
            self.camera_deg = self.go_to.camera_value
            if result:
                self.grabber.start(self.go_to.rank2_target, self.go_to.camera_value)
                self.camera_deg = self.grabber.camera_deg
                self.state = 'grab'
                self._note(self.grabber.note)
            else:
                self.state = self.go_to.state
                self._note(self.go_to.note)
        except Exception:
            self.stop()
            raise

    def _change_state(self, state):
        """总控只切换任务；出发运动不再拆成几个阶段。"""
        self._stop_drive()
        self.state = state
        if state in (STOP, 'distribute'):
            self.go_to.first = self.go_to.config.get('first', True)
        if state=='distribute':
            self.distributor.reset()
        elif state=='search':
            self.go_to.reset()
            self.go_to.team = self.team

    def _stop_drive(self):
        """只停车，不自动打开或闭合夹爪。"""
        self.cmd.move_stop()

    def _note(self, text):
        """只打印变化的说明。"""
        if text != self.note:
            self.note = text
            print('[Task] '+text)

    def close(self):
        """结束任务时停车。"""
        # 这里只发停车，不关串口；串口的生命周期归 serial_node。
        self.cmd.move_stop()
