"""
这个是最外层封装好的，直到抓取，对外只要用grab就行，然后才是转运

整个流程是 search--track--check--grab
check之后可能还会需要打散，scatter，直到check成功才会grab，否则二者重复，或者目标丢失。
scatter次数最多三次，否则一直前进打散会走的很偏
整个check失败就回退到search
"""

import math

from vision_pkg.detect.objs import bottom_center
from ..utils.my_timer import MyTimer


class Grabber:
    """锁定追踪到的物块，用新画面判断何时合爪。"""

    def __init__(self, cmd, x0, config):
        self.cmd, self.x0, self.config = cmd, x0, config
        self.timer = MyTimer()
        self.phase_timer = MyTimer()
        self.scatter_dir = -1
        self.reset()

    def reset(self):
        self.target_key = None
        self.reacquire_key, self.reacquire_hits = None, 0
        self.anchor = None
        self.anchor_deg = None
        self.last_target = None
        self.verify_point = None
        self.verify_key = None
        self.camera_deg = self.config.get('camera_deg', 55)
        self.phase, self.note = 'camera', ''
        self.hits, self.misses = 0, 0
        self.scatter_count = 0
        self.scatter_step = 0
        self.regripped = 0
        self.timer.reset()
        self.phase_timer.reset()

    def start(self, target, camera_deg=None, near=False):
        self.reset()
        self.anchor = target
        self.anchor_deg = camera_deg
        if near:
            self.camera_deg = self.config['near_camera_deg']
            self.phase = 'near_camera'
        self.cmd.camera(self.camera_deg)
        self.note = '追踪到位，压低相机并等待新画面'
        self.timer.start()
        self.phase_timer.start()

    def run(self, objs, camera_ready=True):
        """返回 grab（继续）、search（重新找）或 grab_closed（合爪后停车）。"""
        if self.timer.is_timeout(self.config['timeout_ms']):
            return self.retry('抓取对准超时，重新搜寻', release=self.phase in ('close','retreat','verify','scatter'))
        if not camera_ready:
            self.cmd.move_stop()
            self.hits = 0
            return 'grab'
        if self.phase == 'close':
            if self.phase_timer.is_timeout(self.config['settle_ms']):
                self.cmd.move_straight_dis(-7)
                self.phase, self.note = 'retreat', '合爪后退开物块堆'
                self.phase_timer.restart()
            return 'grab'
        if self.phase == 'retreat':
            if self.phase_timer.is_timeout(900):
                # 后退末段仍挤在爪口的旁块，低头换角度后可能被绿块遮掉。
                carried = self.in_jaw(objs, .68)
                self.misses = self.misses+1 if self.wrong_load(carried) else 0
                if self.misses >= self.config['confirm_frames']:
                    return self.start_scatter('后退后爪口仍有旁块', carried)
            if not self.cmd.straight_done and self.phase_timer.is_timeout(self.config['scatter_drive_timeout_ms']):
                return self.retry('合爪后退未收到到位回包，开爪重新搜寻', release=True)
            if self.cmd.straight_done:
                self.misses = 0
                self.cmd.move_stop()
                self.camera_deg = self.config['verify_deg']
                self.cmd.camera(self.camera_deg)
                self.phase, self.note = 'verify', '停车低头，只检查爪口范围'
                self.phase_timer.restart()
            return 'grab'
        if self.phase == 'verify':
            return self.verify(objs)
        if self.phase == 'scatter':
            return self.scatter()
        if self.phase == 'scatter_check':
            return self.after_scatter(objs)
        adjusting_camera = self.phase in ('camera', 'near_camera')
        if adjusting_camera and not self.phase_timer.is_timeout(self.config['camera_settle_ms']):
            return 'grab'

        target = self.find_target(objs)
        if target is None:
            self.cmd.move_stop()
            self.hits = 0
            self.misses += 1
            if self.misses >= self.config['lost_frames']:
                if self.phase == 'near' and self.jaw_blocked(objs):
                    return self.start_scatter('近处丢失目标，爪口仍有物块', objs)
                return self.retry(f'抓取目标丢失（{self.phase}），重新搜寻')
            return 'grab'
        self.misses = 0
        self.last_target = target

        x1, y1, x2, y2 = target['box']
        width, height = self.config['width'], self.config['height']
        if not (2 <= x1 < x2 <= width - 2 and 0 <= y1 < y2 <= height):
            return self.retry('抓取目标超出画面，重新搜寻')
        x, y = bottom_center(target)
        y_min, y_max = self.config['y_min'], self.config['y_max']

        if adjusting_camera:
            self.hits += 1
            if self.hits >= self.config['confirm_frames']:
                if self.phase == 'camera':
                    self.cmd.grab(0)
                    self.phase = 'approach'
                    self.note = '已开爪，低速接近物块'
                else:
                    self.phase = 'near'
                    self.note = '近处画面已稳定，复核合爪位置'
                self.phase_timer.restart()
                self.hits = 0
            return 'grab'
        if not self.phase_timer.is_timeout(self.config['settle_ms']):
            return 'grab'

        dx = x - self.x0
        near = self.phase == 'near'
        align = 2*self.config['align_px']
        if near:
            # 走到爪尖附近时，目标能搭进来一部分就值得先试抓。
            align = max(align, min(.08*width, (x2-x1)/4))
            # 近处同类物块横挡在原目标前，直接把它作为下一次对准的目标。
            blockers = [obj for obj in self.visible_blocks(objs)
                        if obj['name'] == target['name'] and not obj['in_safe_zone']
                        and obj['box'][3] >= y+.06*height
                        and obj['box'][0] <= x <= obj['box'][2]]
            if blockers:
                front = max(blockers, key=lambda obj: obj['box'][3])
                self.cmd.move_stop()
                self.anchor = front
                self.target_key = front['name'], front['id']
                self.hits = 0
                self.note = '同类近块挡在原目标前，重新对准近块'
                return 'grab'
        # 旁块一直堵在爪口时也能打散，避免一直退回又靠近。
        if near and self.phase_timer.is_timeout(5500) and self.jaw_blocked(objs, target):
            return self.start_scatter('爪口一直被旁块挡住', objs, target)
        if (y2 > height-2 or (near and y > y_max)) and (not near or abs(dx) > align):
            # 底边贴边或太近时稍微退回，等完整框回到合爪范围再确认。
            self.hits = 0
            self.cmd.move_turn(0)
            self.cmd.move_straight(-40)
            self.note = '物块太近，低速退回合爪范围'
            return 'grab'
        # 爪尖已经搭到目标前部就可尝试合爪，后退后的单块复查决定能否转运。
        goal = y_min - .05*height if near else self.config['near_y']
        if abs(dx) <= align and y >= goal and y1 >= 2:
            self.cmd.move_stop()
            self.hits += 1
            if self.hits >= self.config['confirm_frames']:
                if near:
                    if self.jaw_blocked(objs, target):
                        return self.start_scatter('有旁块挡在爪口正前方', objs, target)
                    self.cmd.grab(1)
                    self.timer.restart()  # 合爪后的后退、复核有自己的时间，避免吃完对准的剩余时间。
                    self.phase = 'close'
                    self.hits = 0
                    self.note = f'先合爪，准备后退。底边 ({x:.0f}, {y:.0f})'
                else:
                    self.anchor, self.target_key = target, None
                    self.reacquire_key, self.reacquire_hits = None, 0
                    self.last_target = None
                    self.anchor_deg = self.camera_deg
                    self.camera_deg = self.config['near_camera_deg']
                    self.cmd.camera(self.camera_deg)
                    self.phase = 'near_camera'
                    self.hits = 0
                    self.note = '已接近，进一步压低相机检查爪口'
                self.phase_timer.restart()
            return 'grab'

        self.hits = 0
        if abs(dx) > align:
            # 离爪口还有距离时允许小幅边走边转，贴近后只转正。
            can_advance = y < goal - 35 and (abs(dx) < 48 or (not near and x1 <= self.x0 <= x2))
            self.cmd.move_straight(45 if can_advance else 0)
            turn = max(30, min(75, abs(dx)))
            self.cmd.move_turn(int(turn if dx > 0 else -turn))
        else:
            self.cmd.move_turn(0)
            speed = min(self.config['forward_px'], 100) if near else self.config['forward_px']
            self.cmd.move_straight(min(speed, max(40, int((goal-y)*2))))
        return 'grab'

    @staticmethod
    def visible_blocks(objs):
        return [obj for obj in objs if obj['name'] in ('green','black','orange','blue')
                and obj['observed']]

    def jaw_blocked(self, objs, target=None):
        """只检查爪口前很近、横向与爪口重叠的旁块。"""
        half = .06*self.config['width']
        height = self.config['height']
        left, right = self.x0-half, self.x0+half
        # 只拦挡在中间并且和目标一样近的旁块。靠侧的块留到合爪后复查。
        near_y = .92*height
        if target is not None:
            near_y = max(.72*height, target['box'][3]-.05*height)
        return any(obj is not target and obj['box'][3] >= near_y
                   and obj['box'][0] <= right and obj['box'][2] >= left
                   for obj in self.visible_blocks(objs))

    def in_jaw(self, objs, min_y):
        """只数爪口内的框。低头后物块在画面里会上移。"""
        half = .26*self.config['width']
        return [obj for obj in self.visible_blocks(objs)
                if obj['box'][3] >= min_y*self.config['height']
                and obj['box'][2] >= self.x0-half and obj['box'][0] <= self.x0+half]

    def wrong_load(self, carried):
        return len(carried) > 1 or any(obj['name'] != self.anchor['name'] for obj in carried)

    def verify(self, objs):
        """换俯角后用当前帧重新数块，旧跟踪 ID 不参与计数。"""
        if not self.phase_timer.is_timeout(self.config['camera_settle_ms']):
            return 'grab'
        width, height = self.config['width'], self.config['height']
        carried = self.in_jaw(objs, .17)
        wrong = self.wrong_load(carried)
        self.misses = self.misses+1 if wrong else 0
        if self.misses >= self.config['confirm_frames']:
            return self.start_scatter('合爪复查发现多块或错块', carried)
        if carried and not wrong:
            obj = carried[0]
            x1, y1, x2, y2 = obj['box']
            x, y = bottom_center(obj)
            complete = (2 <= x1 < x2 <= width-2 and 2 <= y1 < y2 <= height-2 and
                        y >= .23*height and
                        (obj['name'] != 'green' or x2-x1 <= .53*width))
            key = obj['name'], obj['id']
            ready = complete and abs(x-self.x0) <= .10*width
            stable = (self.verify_key == (key, ready) and self.verify_point is not None and
                      abs(x-self.verify_point[0]) <= 35 and abs(y-self.verify_point[1]) <= 35)
            self.hits = self.hits+1 if stable else 1
            self.verify_key = key, ready
            self.verify_point = (x,y)
            if self.hits >= self.config['confirm_frames']:
                if ready:
                    self.target_key = key
                    self.note = '已用新画面确认爪口只有一个目标'
                    return 'grab_closed'
                if self.regripped < 2 and y >= .23*height and abs(x-self.x0) <= .20*width:
                    # 后退后只看到原目标类别的一块，松开，直接从近处重新对准。
                    attempts = self.regripped+1
                    scatters = self.scatter_count
                    self.cmd.grab(0)
                    self.start(obj, self.camera_deg, near=True)
                    self.regripped = attempts
                    self.scatter_count = scatters
                    self.note = '只带到目标一部分，松爪后重新靠近'
                    return 'grab'
        else:
            self.hits = 0
            self.verify_point = None
            self.verify_key = None
        if self.phase_timer.is_timeout(self.config['camera_settle_ms'] + 1700):
            return self.retry('无法确认只抓到一个，开爪重新搜寻', release=True)
        return 'grab'

    def start_scatter(self, note, objs, target=None):
        self.cmd.move_stop()
        if self.scatter_count >= self.config['scatter_attempts']:
            return self.retry('局部打散次数用完，重新搜寻', release=True)
        self.scatter_count += 1
        # 朝贴着目标的旁块斜推。左右都有或看不清时，才轮流换边试。
        blocks = self.visible_blocks(objs)
        if target is None:
            same = [obj for obj in blocks if obj['name'] == self.anchor['name']]
            target = same[0] if len(same) == 1 else None
        self.scatter_dir = 1 if self.scatter_count % 2 else -1
        if target is not None:
            x, y = bottom_center(target)
            offsets = [bottom_center(obj)[0]-x for obj in blocks if obj is not target
                       and abs(bottom_center(obj)[1]-y) < .35*self.config['height']
                       and abs(bottom_center(obj)[0]-x) < .5*self.config['width']]
            if offsets and (min(offsets) > 0 or max(offsets) < 0):
                self.scatter_dir = 1 if offsets[0] > 0 else -1
        self.cmd.grab(0)
        self.phase = 'scatter'
        self.scatter_step = 0
        self.note = f'{note}，开爪，从堆边斜推'
        self.hits = 0
        self.misses = 0
        self.verify_point = None
        self.verify_key = None
        self.phase_timer.restart()
        self.timer.restart()
        return 'grab'

    def scatter(self):
        cfg = self.config
        angle = cfg['scatter_angle_deg'] * self.scatter_dir
        back = -cfg['scatter_back_cm']
        # 每段只发一次。STM32 的转角和距离指令会互相覆盖，所以分段执行。
        steps = (
            (self.cmd.move_straight_dis, back),
            (self.cmd.move_turn_angle, angle),
            (self.cmd.move_straight_dis, cfg['scatter_push_cm']),
            (self.cmd.move_straight_dis, back),
            (self.cmd.move_turn_angle, -angle),
        )
        if self.scatter_step == 0:
            if not self.phase_timer.is_timeout(cfg['settle_ms']):
                return 'grab'
        elif self.scatter_step in (2, 5):
            if not self.cmd.turn_done:
                if self.phase_timer.is_timeout(cfg['scatter_turn_timeout_ms']):
                    return self.retry('打散转向未收到到位角度，停车重新搜寻', release=True)
                return 'grab'
        elif not self.cmd.straight_done:
            if self.phase_timer.is_timeout(cfg['scatter_drive_timeout_ms']):
                return self.retry('打散直行未收到到位回包，停车重新搜寻', release=True)
            return 'grab'

        if self.scatter_step < len(steps):
            send, value = steps[self.scatter_step]
            send(value)
            self.scatter_step += 1
        else:
            self.cmd.move_stop()
            self.camera_deg = cfg['camera_deg']
            self.cmd.camera(self.camera_deg)
            self.phase, self.note = 'scatter_check', '打散后停车，看看近处能不能再夹'
        self.phase_timer.restart()
        return 'grab'

    def after_scatter(self, objs):
        """在当前画面找回可抓的一块，避免刚拨开就转身搜寻。"""
        if not self.phase_timer.is_timeout(self.config['camera_settle_ms']):
            return 'grab'
        width, height = self.config['width'], self.config['height']
        candidates = [obj for obj in self.visible_blocks(objs)
                      if obj['name'] == self.anchor['name'] and not obj['in_safe_zone']
                      and obj['box'][3] >= .30*height]
        # 打散后需要重新选一块。多块同色时按底边远近选，连续确认后再走。
        target = max(candidates, key=lambda obj: (obj['box'][3], -abs(bottom_center(obj)[0]-self.x0)), default=None)
        key = (target['name'],target['id']) if target else None
        self.hits = (self.hits+1 if key == self.verify_key else 1) if key else 0
        self.verify_key = key
        # 相机稳定后，连续看不到同类目标就交回搜寻，无需等满转向时间。
        self.misses = 0 if candidates else self.misses + 1
        if not candidates and not self.phase_timer.is_timeout(self.config['camera_settle_ms'] + 950):
            # 斜推后目标常留在回转的一侧。原地多看一小段，一见到框就交回视觉对准。
            self.cmd.move_straight(0)
            self.cmd.move_turn(-160*self.scatter_dir)
            return 'grab'
        if self.misses >= self.config['lost_frames']:
            return self.retry('打散后近处没有目标，重新搜寻')
        if self.phase_timer.is_timeout(self.config['settle_ms']+5000):
            return self.retry('打散后未能重新对准，重新搜寻')
        if self.hits >= self.config['confirm_frames']:
            # 斜推会把目标留在画面侧边。先原地转到它面前，再低头靠近。
            dx = bottom_center(target)[0] - self.x0
            if abs(dx) > .14*width:
                self.cmd.move_straight(0)
                self.cmd.move_turn(max(-160, min(160, int(dx))))
                self.note = '打散后原地转向目标'
                return 'grab'
            self.cmd.move_stop()
            # 打散后车已经退远，先回到爪口再判断是否还堵，避免隔着一段距离空拨。
            self.anchor, self.anchor_deg = target, self.camera_deg
            self.target_key = key
            self.phase = 'approach'
            self.hits = self.misses = 0
            self.timer.restart()
            self.phase_timer.restart()
            # 同一视角已连续看到该目标，只有真正低头时才重新确认 ID。
            if target['box'][3] >= self.config['near_y']:
                self.target_key = self.last_target = None
                self.reacquire_key, self.reacquire_hits = None, 0
                self.camera_deg = self.config['near_camera_deg']
                self.cmd.camera(self.camera_deg)
                self.phase = 'near_camera'
            self.note = '打散后重新看到目标，直接靠近再夹'
        else:
            self.cmd.move_stop()
        return 'grab'

    def find_target(self, objs):
        # 压低相机会让感知重建 ID。停车时先按同类和横向位置重新对应，
        # 连续看到同一新 ID 后再前进，靠近时始终用新 ID 锁定。
        candidates = [obj for obj in objs if obj['name'] == self.anchor['name']
                      and obj['observed'] and not obj['in_safe_zone']]
        if self.target_key is not None:
            return next((obj for obj in candidates if (obj['name'], obj['id']) == self.target_key), None)
        adjusting_view = self.phase in ('camera', 'near_camera')
        x = bottom_center(self.anchor)[0]
        nearby = [(abs(bottom_center(obj)[0]-x), obj) for obj in candidates
                  if abs(bottom_center(obj)[0]-x) <= 100]
        height = self.config['height']
        old_y = bottom_center(self.anchor)[1]
        if adjusting_view and old_y >= height-3:
            # 旧框贴底时只知道原目标很近。远处同类框不应让近处唯一的候选失效。
            close = [obj for _, obj in nearby if obj['box'][3] >= .55*height]
            if len(close) != 1:
                self.reacquire_key, self.reacquire_hits = None, 0
                return None
            target = close[0]
            key = target['name'], target['id']
            self.reacquire_hits = self.reacquire_hits+1 if key == self.reacquire_key else 1
            self.reacquire_key = key
            if self.reacquire_hits < self.config['confirm_frames']:
                return None
            if self.reacquire_hits < self.config['confirm_frames']:
                return None
            nearby = [(0, target)]
        elif adjusting_view and self.anchor_deg is not None:
            # 俯角变大后，原目标通常上移。横向靠得太近时，用预测行区分前后的物块。
            center_y = height/2
            focus = .71*self.config['width']
            pitch = math.radians(self.camera_deg-self.anchor_deg)
            expect_y = center_y + focus*math.tan(math.atan2(old_y-center_y, focus)-pitch)
            nearby = [(dx+abs(bottom_center(obj)[1]-expect_y), obj) for dx,obj in nearby
                      if abs(bottom_center(obj)[1]-expect_y) <= 150]
        nearby.sort(key=lambda pair: pair[0])
        if not nearby or (len(nearby) > 1 and nearby[1][0]-nearby[0][0] < 25):
            return None
        target = nearby[0][1]
        self.hits = 0
        self.target_key = target['name'], target['id']
        return target

    def retry(self, note, release=False):
        self.cmd.move_stop()
        if release:
            self.cmd.grab(0)
        self.note = note
        return 'search'
