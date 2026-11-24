"""
这个是最外层封装好的，直到抓取，对外只要用grab就行，然后才是转运

整个流程是 search--track--check--grab
check之后可能还会需要打散，scatter，直到check成功才会grab，否则二者重复，或者目标丢失。
scatter次数最多三次，否则一直前进打散会走的很偏
整个check失败就回退到search
"""

from vision_pkg.detect.objs import bottom_center
from ..utils.my_timer import MyTimer


class Grabber:
    """锁定追踪到的物块，用新画面判断何时合爪。"""

    def __init__(self, cmd, x0, config):
        self.cmd, self.x0, self.config = cmd, x0, config
        self.timer = MyTimer()
        self.phase_timer = MyTimer()
        self.reset()

    def reset(self):
        self.target_key = None
        self.phase, self.note = 'check', ''
        self.hits, self.misses = 0, 0
        self.timer.reset()
        self.phase_timer.reset()

    def start(self, target):
        self.reset()
        self.target_key = target['name'], target['id']
        self.note = '追踪到位，复核物块后开爪'
        self.timer.start()

    def run(self, objs):
        """返回 grab（继续）、search（重新找）或 grab_closed（合爪后停车）。"""
        if self.phase == 'close':
            if self.phase_timer.is_timeout(self.config['settle_ms']):
                return 'grab_closed'
            return 'grab'

        target = next((obj for obj in objs if (obj['name'], obj['id']) == self.target_key
                       and obj['observed'] and not obj['in_safe_zone']), None)
        if self.timer.is_timeout(self.config['timeout_ms']):
            return self.retry('抓取对准超时，重新搜寻')
        if target is None:
            self.cmd.move_stop()
            self.hits = 0
            self.misses += 1
            if self.misses >= self.config['confirm_frames']:
                return self.retry('抓取目标丢失，重新搜寻')
            return 'grab'
        self.misses = 0

        x1, y1, x2, y2 = target['box']
        width, height = self.config['width'], self.config['height']
        if not (2 <= x1 < x2 <= width - 2 and 2 <= y1 < y2 <= height - 2):
            return self.retry('抓取目标超出画面，重新搜寻')
        x, y = bottom_center(target)
        y_min, y_max = self.config['y_min'], self.config['y_max']
        if y > y_max:
            return self.retry('物块超过合爪范围，重新搜寻')

        if self.phase == 'check':
            self.hits += 1
            if self.hits >= self.config['confirm_frames']:
                self.cmd.grab(0)
                self.phase = 'approach'
                self.phase_timer.start()
                self.hits = 0
                self.note = '已开爪，低速接近物块'
            return 'grab'
        if not self.phase_timer.is_timeout(self.config['settle_ms']):
            return 'grab'

        dx = x - self.x0
        if abs(dx) <= self.config['align_px'] and y >= y_min:
            self.cmd.move_stop()
            self.hits += 1
            if self.hits >= self.config['confirm_frames']:
                self.cmd.grab(1)
                self.phase = 'close'
                self.phase_timer.restart()
                self.note = '已发送合爪，停车'
            return 'grab'

        self.hits = 0
        if abs(dx) > self.config['align_px']:
            # 横向偏差较大时先转正，保持前后停车。
            self.cmd.move_straight(0)
            self.cmd.move_turn(int(max(-60, min(60, dx))))
        else:
            self.cmd.move_turn(0)
            self.cmd.move_straight(self.config['forward_px'])
        return 'grab'

    def retry(self, note):
        self.cmd.move_stop()
        self.note = note
        return 'search'
