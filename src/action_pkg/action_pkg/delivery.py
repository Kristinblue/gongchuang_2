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
        self.detours = 0
        self.held = True
        self.samples = []
        self.last_frame = None
        self.turn_goal = None
        self.turn_retry = False
        self.zone_heading = None
        self.drive_heading = None
        self.timer.reset()
        self.clock.reset()

    def start(self, team, kind):
        self.reset()
        self.team = team
        self.kind = kind
        # 从场地内正对安全区时，物资在左，橙色伤员在右。
        self.slot = self.side = .15 if kind == 'orange' else -.15
        self.cmd.camera(self.camera_deg)
        self.timer.start()
        self.clock.start()
        self.note = '夹爪保持闭合，寻找本方安全区'

    def obstacle(self, objs, limit=None):
        """只处理挡住前进走廊的新检测框，最近的先处理。"""
        width, height = self.config['width'], self.config['height']
        if limit is None:
            limit = .50 if self.camera_deg == 25 else .36
        camera = (.122, .149) if self.camera_deg == 25 else (.113, .136)
        nearest = None
        payload_seen = False
        for obj in objs:
            if not obj['observed'] or obj['name'] not in ('green', 'blue', 'black', 'orange'):
                continue
            x1, _, x2, y2 = obj['box']
            points, down = ground_points(np.array([[x1, y2], [x2, y2]]), width, height, self.camera_deg, camera)
            (distance, left), (_, right) = points
            # 同类物块只有贴近爪口、横向居中时才按随车目标排除。
            if self.held and not payload_seen and self.kind == obj['name'] and left <= .03 and right >= -.03 and distance < .19:
                payload_seen = True
                continue
            if down[0] > .02 and .12 < distance <= limit and left < .105 and right > -.105:
                side = (left + right) / 2
                if nearest is None or distance < nearest[0]:
                    nearest = (float(distance), float(side))
        return nearest

    def avoid(self, obstacle):
        """停车、退一点，再转向空侧绕开，之后重新找安全区。"""
        if not self.held:
            self.moves = [('drive', -25)]
            self.next_move()
            self.note = '开爪后前方还有物块，取消短推，直接后退'
            return self.phase
        if self.detours >= 3:
            self.cmd.move_stop()
            self.note = '投放路线仍有物块，停车'
            return 'delivery_timeout'
        self.detours += 1
        turn = -65 if obstacle[1] >= 0 else 65
        if abs(obstacle[1]) < .02:
            turn = int(math.copysign(65, self.slot))
        # 同一条路线再次被挡时从格子外侧通过，最后一段才回到格子中间。
        offset = .095 if self.detours == 1 else .175
        self.side = self.slot + math.copysign(offset, turn)
        self.phase = 'clear_lane'
        self.note = f'前方约 {obstacle[0]*100:.0f} cm 有物块，先退再向一侧绕开'
        if self.camera_deg != 25:
            self.camera_deg = 25
            self.cmd.camera(25)
        self.moves = [('drive', -10), ('turn', turn), ('bypass', 24), ('turn', -turn)]
        self.next_move()
        return self.phase

    def check_drive(self, objs, camera_ready):
        """只在新视角可用且前方无遮挡时继续前进。"""
        if not camera_ready:
            self.cmd.move_stop()
            self.note = '投放前视角失效，停车'
            return 'delivery_timeout'
        obstacle = self.obstacle(objs)
        if obstacle:
            return self.avoid(obstacle)
        return None

    def check_heading(self, heading):
        """近距离进场和开爪前都检查车头，偏了就保持闭爪停车。"""
        angle = self.cmd.angle
        if angle is not None and heading is not None and abs((angle-heading+180)%360-180) <= 8:
            return True
        self.cmd.move_stop()
        self.note = '投放前车头偏离方向或角度未知，保持闭爪停车'
        return False

    def run(self, detection, objs=(), camera_ready=True):
        """用当前感知帧更新一次搬运动作，返回任务阶段。"""
        # 开爪后最多再短推和退车，时间到也要留出退车动作。
        if self.held and self.clock.is_timeout(90000):
            self.note = '投放超时，停车'
            return 'delivery_timeout'
        if self.moves:
            kind, value = self.moves[0]
            if kind in ('drive', 'bypass', 'near') and value > 0:
                interrupted = self.check_drive(objs, camera_ready)
                if interrupted:
                    return interrupted
            if kind == 'near' and not self.check_heading(self.drive_heading):
                return 'delivery_timeout'
            if kind == 'check':
                if not camera_ready or not self.timer.is_timeout(700):
                    return self.phase
                obstacle = self.obstacle(objs)
                if obstacle:
                    return self.avoid(obstacle)
                ready = True
            elif kind == 'turn':
                angle = self.cmd.angle
                ready = self.cmd.turn_done or (
                    angle is not None and self.turn_goal is not None and self.timer.is_timeout(500) and
                    abs((angle-self.turn_goal+180)%360-180) <= 3)
                if not ready and self.timer.is_timeout(3000 + abs(value)*95):
                    if self.camera_deg == 25 and not self.turn_retry and angle is not None and self.turn_goal is not None:
                        # 远处转向卡住时，退开一点再补余角，只试一次。
                        self.turn_retry = True
                        remaining = round((angle-self.turn_goal+180)%360-180)
                        self.moves[:1] = [('drive', -5), ('turn', remaining), ('drive', 5)]
                        self.note = '绕行转向卡住，先小退再补剩余角度'
                        self.next_move()
                        return self.phase
                    self.cmd.move_stop()
                    self.note = '投放转向超时，保持闭爪停车'
                    return 'delivery_timeout'
            elif kind in ('drive', 'bypass', 'near'):
                ready = value == 0 or self.cmd.straight_done
                if not ready and self.timer.is_timeout(3000 + abs(value) * 150):
                    self.cmd.move_stop()
                    self.note = '投放直行未收到到位回包，停车'
                    return 'delivery_timeout'
            else:
                ready = self.timer.is_timeout(value)
            if not ready:
                return self.phase
            self.moves.pop(0)
            if self.moves:
                upcoming, distance = self.moves[0]
                if upcoming in ('drive', 'bypass', 'near') and distance > 0:
                    interrupted = self.check_drive(objs, camera_ready)
                    if interrupted:
                        return interrupted
                if upcoming == 'open' and not self.check_heading(self.zone_heading):
                    return 'delivery_timeout'
            self.next_move()
            return self.phase
        if self.phase == 'clear_lane':
            self.phase = 'align_zone'
            self.note = '侧移完成，停车重新对准安全区'
            self.samples.clear()
            return self.phase
        if self.phase == 'deliver':
            self.note = '投放和退车动作结束'
            return 'delivery_done'
        if not self.timer.is_timeout(700):
            return self.phase
        if detection is None or not detection.full or detection.flag != self.team[0] + '_safety':
            self.samples.clear()
            if self.phase == 'seek':
                self.cmd.move_turn(650)
            elif self.timer.is_timeout(4500):
                self.phase = 'seek'
            return self.phase
        polygon = [(p.x, p.y) for p in detection.polygon]
        pose = zone_pose(polygon, self.config['width'], self.config['height'], self.camera_deg, (.122, .149))
        if pose is None:
            self.samples.clear()
            if self.phase == 'seek':
                self.cmd.move_turn(500)
            elif self.timer.is_timeout(4500):
                self.phase = 'seek'
            return self.phase
        self.cmd.move_stop()
        stamp = detection.header.stamp.sec, detection.header.stamp.nanosec
        if stamp == self.last_frame:
            return self.phase
        self.last_frame = stamp
        direction, gap, middle = pose
        right = np.array([-direction[1], direction[0]])
        target = middle + right * self.side
        heading = math.degrees(math.atan2(direction[1], direction[0]))
        lateral = float(target @ right)
        self.samples.append((heading, gap, lateral))
        if len(self.samples) < 3:
            return self.phase
        recent = np.asarray(self.samples[-3:])
        if (np.ptp(recent, axis=0) > [6, .08, .08]).any():
            self.samples = self.samples[-2:]
            return self.phase
        self.samples.clear()
        lateral_limit = .04 if self.detours > 1 else (.025 if self.detours else .055)
        heading_limit = 5 if self.detours > 1 else (3 if self.detours else 7)
        min_gap = .32 if self.detours else .25
        if abs(heading) <= heading_limit and abs(lateral) <= lateral_limit and min_gap < gap < 1.15:
            self.phase = 'deliver'
            angle = self.cmd.angle
            self.zone_heading = None if angle is None else (angle-heading)%360
            # 夹持状态先把物块送过前沿，开爪后再短推，给后退留出余量。
            distance = round((gap-.07)*100)
            obstacle = self.obstacle(objs, min(.50, gap + .05))
            if obstacle:
                return self.avoid(obstacle)
            self.note = '本方安全区已对准，先靠近再低头检查'
            advance = max(0, distance - 26)
            near = distance-advance
            approach = [('near', near)]
            if self.detours:
                # 绕行路线贴近一侧，快进格时回到格子中间，避开两侧边缘。
                turn, leg = (20, 15) if self.detours == 1 else (45, 20)
                if self.detours == 1 and abs(self.side) < abs(self.slot):
                    turn, leg = 26, 17
                straight = near - round(leg * math.cos(math.radians(turn))) - 6
                if self.side > self.slot:
                    turn = -turn
                approach = [('near', straight), ('turn', turn), ('near', leg), ('turn', -turn), ('near', 6)]
            self.moves = ([('drive', advance)] if advance else []) + [('check', 0)] + approach + [
                ('check', 0), ('open', 550), ('drive', 12), ('drive', -25)]

        else:
            self.phase = 'align_zone'
            self.note = '靠近安全区对准点，停车后用新画面复核'
            # 对准某一半格后，整个安全区仍须在 70° 视野内。
            standoff = 1.10 if self.detours > 1 else (.95 if self.detours else .85)
            waypoint = target - standoff * direction
            turn = math.degrees(math.atan2(waypoint[1], waypoint[0]))
            distance = float(np.linalg.norm(waypoint))
            if distance >= (.02 if self.detours else .05):
                self.moves = [('turn', round(turn)), ('drive', min(55, round(distance*100))),
                              ('turn', round((heading-turn+180)%360-180))]
            else:
                self.moves = [('turn', round(heading))]
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
            elif kind in ('drive', 'bypass', 'near'):
                self.drive_heading = self.cmd.angle
                if value:
                    self.cmd.move_straight_dis(value)
            elif kind == 'check':
                if self.camera_deg != 45:
                    self.camera_deg = 45
                    self.cmd.camera(45)
                self.note = '停车低头检查投放路线'
            else:
                self.held = False
                self.cmd.grab(0)
