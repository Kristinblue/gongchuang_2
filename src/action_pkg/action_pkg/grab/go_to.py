"""
自定义的二级评分机制

救援目标是black，green，orange
blue是危险，是物体但是这个阶段会忽略（抓取时要考虑）
安全区是要转运到的地方

这里是搜寻与追踪阶段。第一次评分相当于搜寻，第二次相当于追踪，真正抓取在grabber里面。追踪只选取一个物块。
这里我们先写一次只转运一个物块的，否则有点麻烦.

这里面小写的是变量名，大写的是待定参数，放yaml里这里写的等于都是初次暂定的值

搜寻时angle值为SEARCH_ANGLE_1,暂定70
（相机改用实际俯角后，原位置70/80对应24°/16°，以下旧数值只作原思路记录。）
第二次评分，我们要找出目标点，并且去追踪它，追踪成功的标准是德塔x和德塔y分别小于一个比较小的偏差值。
x0取分辨率宽的一半，y0取这个视角下一个比较合适的离车比较接近的

search的流程是，先转圈（之后再加根据安全区和角度等信息判断转向），暂定右转
1.发送180的move_angle_a,一旦发现S1大于S1_THRES(暂定为10),就发送stop指令，进入track
若期间S1未大于等于S1_THRES,发送360的move_angle_a，直到S1大于零，进入track

2.若依旧未能进入track，发送angle为SEARCH_ANGLE_2=80，再来一次
3.若仍然不能，SEARCH_ANGLE_1，并发送move_straight,值为LOST_MOVE=200。期间若发现S1大于零，可发送stop并且进入第二阶段
循环1、2、3直到进入二级评分

可能会有问题，就是识别的跳变，进入第二阶段后发现没有救援目标，就重复search阶段

track的流程是，为每个救援目标计算S2，
然后发送移动命令（不带angle）去追寻，追寻是一个单片机的PID，这里只管发送直行和转弯的speed，每一帧都更新的


两种track，控制first=True，来决定各项的值

首趟简单点：
S1
1.有效性：
    valid:bool
    绿色。没有S1=0
2.类别增益type_a：
    绿色GREEN_A= 5
3.数量（所有物块数n）num_a：
    num_a=K_NUM/n
    K_NUM=4
4.本方安全区增益safe_a
    SAFE_A=6

正常转运
1.有效性：
    绿色，黑色，橙色
    没有直接S1=0
2.类别增益（每类计算一次）
    绿色GREEN_A= 5
    黑色BLACK_A=5
    橙色ORANGE_A=2
3.4同上

S2，计算每个救援目标的
1.有效性：
    valid:bool
    绿色。没有直接回退track
首趟：

2.类别增益type_a：
    绿色GREEN_A= 5
3.距离增益dis_a:
    dis_a=K_DIS/dis
    dis是救援目标到x0，y0的int(欧式距离)
    K_DIS=0.002

二级只选一个目标

"""
import math
import time

from vision_pkg.detect.objs import bottom_center, pixel_distance


class GoTo:
    """S1评价画面，S2只选一个救援目标，逐帧保持关联。"""

    def __init__(self, x0=640., y0=550., width=1280, height=720, first=True, config=None, cmd=None, team=None):
        """外部传入MyCMD、参考点、首趟标志和配置；构造时不执行动作。"""
        self.x0, self.y0 = x0, y0
        self.width, self.height = width, height
        self.first, self.team, self.cmd = first, team, cmd
        self.config = config or {}
        self.reset()

    def _materials(self, objs):
        """只计当前可见、尚未存入两方安全区的物块。"""
        safe = [obj for obj in objs if obj['name'] in ('rsafe','bsafe') and obj.get('observed',True)]
        result = []
        for obj in objs:
            if obj['name'] not in ('green','black','orange','blue') or not obj.get('observed',True) or obj.get('in_safe_zone',False):
                continue
            x,y = bottom_center(obj)
            if any(zone['box'][0]<=x<=zone['box'][2] and zone['box'][1]<=y<=zone['box'][3] for zone in safe):
                continue
            result.append(obj)
        return result

    def _targets(self, material):
        kinds = ('green',) if self.first else ('green','black','orange')
        return [obj for obj in material if obj['name'] in kinds]

    def first_rank(self, objs, now=None):
        """没有救援目标则S1=0；类别每类一次，数量包括蓝色。"""
        material = self._materials(objs)
        candidates = self._targets(material)
        self.rank1_score = 0.
        self.rank1_target = candidates[0] if candidates else None
        if not candidates:
            return None
        score = self.config.get('score',{})
        values = {'green':score.get('green',5),'black':score.get('black',5),'orange':score.get('orange',2)}
        for name in {obj['name'] for obj in candidates}:
            self.rank1_score += values[name]
        self.rank1_score += score.get('count_reward',4)/len(material)
        own = 'rsafe' if self.team=='red' else 'bsafe' if self.team=='blue' else None
        if any(obj['name']==own and obj.get('observed',True) for obj in objs):
            self.rank1_score += score.get('our_safe',6)
        return self.rank1_target

    def second_rank(self, objs):
        """初选按S2最大；锁定后同类近邻，短暂丢失停车等新帧。"""
        candidates = self._targets(self._materials(objs))
        if self.rank2_target is None:
            target = max(candidates,key=self._second_score,default=None)
        else:
            candidates = [obj for obj in candidates if obj['name']==self.rank2_target['name'] and pixel_distance(obj,self.rank2_target)<=self.config.get('track_gate_px',150)]
            target = min(candidates, key=lambda obj: (
                obj.get('id') != self.rank2_target.get('id'),
                pixel_distance(obj, self.rank2_target),
            ), default=None)
        if target is None:
            self.rank2_hits = 0
            self.rank2_misses += 1
            if self.rank2_misses >= self.config.get('confirm_frames', 3):
                self.rank2_target = self.target_point = None
                self.rank2_score = None
            return None
        self.rank2_misses = 0
        self.rank2_target = target
        self.rank2_score = self._second_score(target)
        self.rank2_hits += 1
        self.target_point = bottom_center(target)
        return target if self.rank2_hits>=self.config.get('confirm_frames',3) else None

    def _second_score(self, obj):
        """沿用原S2公式，数值同时提供给视频叠加层。"""
        score = self.config.get('score',{})
        values = {'green':score.get('green',5),'black':score.get('black',5),'orange':score.get('orange',2)}
        x, y = bottom_center(obj)
        distance = max(1., math.hypot(x - self.x0, (y - self.y0) * self.config.get('track_depth_weight', 1.)))
        return values[obj['name']]+self.config.get('distance_reward',.002)/distance

    def start_track(self, now=None):
        """新一轮追踪不沿用上轮确认次数。"""
        self.state = 'track'
        self._track_at = time.monotonic() if now is None else now
        self.rank2_target, self.target_point = None, None
        self.rank2_score = None
        self.rank2_hits = 0
        self.rank2_misses = 0

    def reset(self):
        """下一次run从Search重新开始；不复位物理场景，不在每帧调用。"""
        self.rank1_score = 0.
        self.rank2_score = None
        self.rank1_target = self.rank2_target = self.target_point = None
        self.rank2_hits = 0
        self.rank2_misses = 0
        self.state, self.note = 'search', '准备搜寻'
        self.search_step, self.search_turn_sent = -1, False
        self.camera_value = self.config.get('search_camera_1',70)
        self._step_at, self._camera_at = 0., None
        self._track_at = float('-inf')
        self.align_hits = 0

    def run(self, objs, now=None, turn_done=None):
        """每张新图调用一次，执行Search→Track，到位返回True，否则返回False。

        Args:
            objs: 当前检测列表，旧记忆项须标observed=False。
            now: 单调秒数，通常不传；电脑测试可传入仿真时钟。
            turn_done: 本次旋转是否完成，不传则读取cmd.turn_done。

        Returns:
            True表示追踪成功且已停车；False表示本次任务仍在继续。
        """
        if self.cmd is None:
            raise ValueError('[GoTo] run需要传入MyCMD动作接口')
        now = time.monotonic() if now is None else now
        if turn_done is None:
            turn_done = self.cmd.turn_done
        if self.state=='done':
            return True
        if self.state=='search':
            if self.search_step<0:
                self._next_search(now)
                return False
            if not self.view_ready(now):
                return False
            target = self.first_rank(objs)
            threshold = self.config.get('search_score_min',10) if self.search_step in (0,2) else 0
            passed = self.rank1_score>=threshold if threshold else self.rank1_score>0
            if target is not None and passed:
                self.cmd.move_stop()
                self.start_track(now)
                self.note = '发现可用画面，停车进入追踪'
            elif self.search_step==4:
                self.cmd.move_straight(-int(self.config.get('lost_move',200)))
                self.cmd.move_turn(0)
                duration = self.config.get('camera_settle_ms',400)+self.config.get('lost_move_ms',1000)
                if (now-self._step_at)*1000>=duration:
                    self._next_search(now)
            elif not self.search_turn_sent:
                self.search_turn_sent = self.cmd.move_turn_angle(180 if self.search_step in (0,2) else 360)
                if self.search_turn_sent:
                    self._step_at = now
                else:
                    self.note = '等待 STM32 上报导航角度'
            elif turn_done:
                self._next_search(now)
            return False
        if not self.view_ready(now) or now - self._track_at < .25:
            self.cmd.move_stop()
            return False
        target = self.second_rank(objs)
        if target is None:
            self.cmd.move_stop()
            self.align_hits = 0
            self.note = '目标未确认，停车等新帧'
            if self.rank2_target is None:
                self.reset()
                self._next_search(now)
                self.note = '目标丢失，重新搜寻'
            return False
        x,y = bottom_center(target)
        dx,dy = x-self.x0,y-self.y0
        tolerance = self.config.get('align_px',20)
        if abs(dx)<=tolerance and abs(dy)<=tolerance:
            self.cmd.move_stop()
            self.align_hits += 1
            if self.align_hits>=self.config.get('confirm_frames',3):
                self.state, self.note = 'done', '追踪到位，停车'
                return True
            return False
        self.align_hits = 0
        turn_limit = self.config.get('turn_limit_px',100)
        move_limit = self.config.get('move_limit_px',180)
        turn = 0 if abs(dx)<=tolerance else int(max(-turn_limit,min(turn_limit,dx)))
        forward = 0
        if abs(dx)<=self.config.get('turn_only_px',80) and abs(dy)>tolerance:
            forward = int(max(-move_limit,min(move_limit,dy)))
        self.cmd.move_turn(turn)
        # 图像纵向误差前负后正，发给 STM32 时取反。
        self.cmd.move_straight(-forward)
        self.note = '追踪目标：前正后负、右正左负'
        return False

    def view_ready(self, now):
        """相机位置改变后只等最短稳定时间，不伪造实际角度反馈。"""
        return self._camera_at is None or (now-self._camera_at)*1000>=self.config.get('camera_settle_ms',400)

    def _next_search(self, now):
        """一条转角命令只发一次，步骤变化留在本类。"""
        self.search_step = (self.search_step+1)%5
        self.cmd.move_stop()
        camera = self.config.get('search_camera_1',70)
        if self.camera_value!=camera or self._camera_at is None:
            self._camera_at = now
            self.camera_value = camera
            self.cmd.camera(int(camera))
        self.search_turn_sent = False
        self._step_at = now
        self.note = '搜寻救援目标'
