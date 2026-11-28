"""仿真投放：用可见的本方安全区规划停车点，停车后重新解算。"""

import math

import cv2
import numpy as np

from .utils.my_timer import MyTimer


def ground_points(pixels, width, height, degrees, camera):
    """把图像坐标投到车前地面，返回前向和右向距离（米）。"""
    f = width / (2 * math.tan(math.radians(35)))
    u, v = ((pixels - [width/2, height/2]) / f).T
    a = math.radians(degrees)
    down = math.sin(a) + v * math.cos(a)
    scale = camera[1] / np.maximum(down, .02)
    return np.column_stack((camera[0] + scale * (math.cos(a) - v * math.sin(a)), scale*u)), down


def zone_pose(polygon, width, height, degrees, camera):
    """返回朝向安全区的单位向量、前长边距离和前长边中心。"""
    pixels = np.asarray(polygon, dtype=float)
    if len(pixels) < 4 or not np.isfinite(pixels).all():
        return None
    if (pixels.min(axis=0) < 5).any() or (pixels.max(axis=0) > [width-6, height-6]).any():
        return None
    ground, down = ground_points(pixels, width, height, degrees, camera)
    if min(down) <= .02:
        return None
    rect = cv2.minAreaRect(ground.astype(np.float32))
    short, long = sorted(rect[1])
    # 前沿靠近地面，远侧围栏上沿会把投影的短边拉长。
    if not (.22 < short < .61 and .49 < long < .86 and long-short > .08):
        return None
    center = np.asarray(rect[0])
    contour = ground.astype(np.float32)
    corners = cv2.approxPolyDP(contour, .025, True).reshape(-1, 2)
    candidates = []
    for start, end in zip(corners, np.roll(corners, -1, axis=0)):
        edge = end-start
        length = np.linalg.norm(edge)
        if not (.48 < length < .87):
            continue
        middle = (start+end)/2
        direction = np.array([-edge[1], edge[0]]) / length
        if (center-middle) @ direction < 0:
            direction = -direction
        gap = float(middle @ direction)
        if gap > 0:
            candidates.append((direction, gap, middle))
    return min(candidates, key=lambda candidate: candidate[1], default=None)


class Delivery:
    """从合爪转入搬运，移动指令与原任务共用 MyCMD。"""

    def __init__(self, cmd, config):
        self.cmd, self.config = cmd, config
        self.timer = MyTimer()
        self.clock = MyTimer()
        self.camera_deg = 25
        self.reset()

    def reset(self):
        self.phase, self.note = 'seek', ''
        self.moves = []
        self.camera_deg = 25
        self.held = True
        self.samples = []
        self.last_frame = None
        self.turn_goal = None
        self.zone_heading = None
        self.aligned_at = 0
        self.timer.reset()
        self.clock.reset()

    def start(self, team, kind):
        self.reset()
        self.team = team
        # 从场地内正对安全区时，物资在左，橙色伤员在右。
        self.slot = .15 if kind == 'orange' else -.15
        self.cmd.camera(self.camera_deg)
        self.timer.start()
        self.clock.start()
        self.note = '夹爪保持闭合，寻找本方安全区'

    def check_heading(self, heading):
        """开爪前检查车头，偏了就保持闭爪停车。"""
        angle = self.cmd.angle
        if angle is not None and heading is not None and abs((angle-heading+180)%360-180) <= 8:
            return True
        self.cmd.move_stop()
        self.note = '投放前车头偏离方向或角度未知，保持闭爪停车'
        return False

    def run(self, detection, now_ns=0):
        """用当前感知帧更新一次搬运动作，返回任务阶段。"""
        # 开爪后最多再短推和退车，时间到也要留出退车动作。
        if self.held and self.clock.is_timeout(90000):
            self.note = '投放超时，停车'
            return 'delivery_timeout'
        if self.moves:
            return self.run_move(now_ns)
        if self.phase == 'deliver':
            self.note = '投放和退车动作结束'
            return 'delivery_done'
        if not self.timer.is_timeout(700):
            return self.phase

        pose = self.read_pose(detection)
        if pose is None:
            self.samples.clear()
            if self.phase == 'seek':
                self.cmd.move_turn(650)
            elif self.timer.is_timeout(4500):
                self.phase = 'seek'
            return self.phase

        self.cmd.move_stop()
        stamp = detection.header.stamp
        capture = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.last_frame is not None and capture <= self.last_frame:
            return self.phase
        self.last_frame = capture
        direction, gap, middle = pose
        right = np.array([-direction[1], direction[0]])
        target = middle + right * self.slot
        heading = math.degrees(math.atan2(direction[1], direction[0]))
        lateral = float(target @ right)
        self.samples.append((heading, gap, lateral))
        if len(self.samples) < 3:
            return self.phase
        if (np.ptp(self.samples, axis=0) > [6, .08, .08]).any():
            self.samples.pop(0)
            return self.phase

        self.samples.clear()
        aligned = abs(heading) <= 7 and abs(lateral) <= .055 and .25 < gap < 1.15
        if self.phase == 'align_zone' and aligned:
            self.phase = 'deliver'
            angle = self.cmd.angle
            self.zone_heading = None if angle is None else (angle-heading)%360
            # 夹持状态先把物块送过前沿，开爪后再短推，给后退留出余量。
            self.moves = [('drive', round((gap-.07)*100)), ('open', 550),
                          ('drive', 12), ('drive', -25)]
            self.note = '新画面确认安全区已对准，按距离直行投放'
        else:
            self.phase = 'align_zone'
            self.note = '靠近安全区对准点，停车后用新画面复核'
            self.aligned_at = now_ns
            if not aligned:
                # 对准某一半格后，整个安全区仍须在 70° 视野内。
                waypoint = target - .85 * direction
                turn = math.degrees(math.atan2(waypoint[1], waypoint[0]))
                distance = float(np.linalg.norm(waypoint))
                if distance >= .05:
                    self.moves = [('turn', round(turn)), ('drive', round(distance*100)),
                                  ('turn', round((heading-turn+180)%360-180))]
                else:
                    self.moves = [('turn', round(heading))]
        self.next_move()
        return self.phase

    def read_pose(self, detection):
        """只解算本方 OK 的安全区，并跳过对准前的图像。"""
        if detection is None or not detection.ok or detection.flag != self.team[0] + '_safety':
            return None
        stamp = detection.header.stamp
        capture = stamp.sec * 1_000_000_000 + stamp.nanosec
        if capture <= self.aligned_at:
            return None
        polygon = [(p.x, p.y) for p in detection.polygon]
        return zone_pose(polygon, self.config['width'], self.config['height'], self.camera_deg, (.122, .149))

    def run_move(self, now_ns):
        """到位反馈到达后才发送下一条指令。"""
        kind, value = self.moves[0]
        if kind == 'turn':
            angle = self.cmd.angle
            ready = self.cmd.turn_done or (
                angle is not None and self.turn_goal is not None and self.timer.is_timeout(500) and
                abs((angle-self.turn_goal+180)%360-180) <= 3)
            if not ready and self.timer.is_timeout(3000 + abs(value)*95):
                self.note = '投放转向超时，停车'
                return 'delivery_timeout'
        elif kind == 'drive':
            ready = self.cmd.straight_done
            if not ready and self.timer.is_timeout(3000 + abs(value)*150):
                self.note = '投放直行未收到到位回包，停车'
                return 'delivery_timeout'
        else:
            ready = self.timer.is_timeout(value)
        if not ready:
            return self.phase

        self.moves.pop(0)
        if self.moves and self.moves[0][0] == 'open' and not self.check_heading(self.zone_heading):
            return 'delivery_timeout'
        if self.phase == 'align_zone' and not self.moves:
            self.aligned_at = now_ns
            self.note = '对准动作结束，停车等新画面复核'
        self.next_move()
        return self.phase

    def next_move(self):
        self.cmd.move_stop()
        self.timer.restart()
        if self.moves:
            kind, value = self.moves[0]
            if kind == 'turn':
                angle = self.cmd.angle
                self.turn_goal = None if angle is None else (angle-value)%360
                self.cmd.move_turn_angle(value)
            elif kind == 'drive':
                self.cmd.move_straight_dis(value)
            else:
                self.held = False
                self.cmd.grab(0)
