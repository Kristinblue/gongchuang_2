"""
出发后直接前进打散物块。
初步设定离一堆物块较近的时候边前进边旋转

出发由下位机控制，给定出发区后小车走到大致两个安全区的中轴线上（与长边平行）
"""
#第一个塞到y分量的，负数就是往前
#第二个赛道x分量控制转向。这里我们搭配出一个比较合适的打散方法
BASE_FORWARD = -100

from .utils.my_timer import MyTimer


class InitDisributor:
    """先开到物块堆附近，再按航向和距离反馈逐步打散。"""

    def __init__(self, cmd, config=None):
        self.cmd = cmd
        config = config or {}
        self.base_forward = -abs(int(config.get('base_forward', BASE_FORWARD)))
        self.move_ms = config.get('move_ms', 5000)
        self.angle = config.get('angle', 0)
        self.red_home_deg = config.get('red_home_deg', 0)
        self.timer = MyTimer()
        self.reset()

    def run(self, team):
        """一个动作到位再发下一条，最后停车并返回 True。"""
        if not self.timer.is_running:
            self.cmd.grab(1)
            self.timer.start()
        if self.phase == 'approach':
            if not self.timer.is_timeout(self.move_ms):
                # 配置沿用原来的负值前进，发送时转换为正值前进。
                self.cmd.move_straight(-self.base_forward)
                self.note = '前进到物块堆附近'
                return False
            self.cmd.move_stop()
            self.phase, self.note = 'wait_heading', '停车等待 STM32 航向'
            self.timer.restart()

        if self.phase == 'wait_heading':
            if self.cmd.angle is None:
                if self.timer.is_timeout(3000):
                    self.fail('等待 STM32 航向超时')
                return False
            home = self.red_home_deg + (180 if team == 'blue' else 0)
            opponent = (home + 180) % 360
            # 航向增加是逆时针，05 指令为正则顺时针。正好相反时优先右转。
            turn = (self.cmd.angle - opponent + 180) % 360 - 180
            turn = 180 if turn == -180 else round(turn)
            direction = 1 if turn >= 0 else -1
            self.moves = [('face_opponent', turn), ('push_short', 5), ('sweep_120', direction*120),
                          ('push', 15), ('sweep_180', direction*180)]
            self.next_move()
            return False

        if not self.moves:
            return self.done
        phase, value = self.moves[0]
        driving = phase in ('push_short', 'push')
        ready = self.cmd.straight_done if driving else self.cmd.turn_done
        timeout_ms = 5000 if driving else 3000 + abs(value)*95
        if not ready:
            if self.timer.is_timeout(timeout_ms):
                self.fail(f'开局打散 {phase} 未收到到位反馈')
            return False
        self.moves.pop(0)
        self.next_move()
        return self.done

    def next_move(self):
        self.cmd.move_stop()
        self.timer.restart()
        if not self.moves:
            self.done, self.note = True, '开局打散结束'
            return
        self.phase, value = self.moves[0]
        self.note = f'开局打散 {self.phase} {value}'
        if self.phase in ('push_short', 'push'):
            self.cmd.move_straight_dis(value)
        else:
            self.cmd.move_turn_angle(value)

    def fail(self, note):
        self.cmd.move_stop()
        self.failed, self.note = True, note

    def reset(self):
        """重置开局步骤和计时。"""
        self.phase, self.done, self.failed = 'approach', False, False
        self.note = '准备初次打散'
        self.moves = []
        self.timer.reset()
