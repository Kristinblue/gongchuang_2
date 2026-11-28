"""模拟 STM32 下位机：用伪串口接 serial_node，验证串口和动作链路。

在 WSL 里运行，会打印一个 /dev/pts/N，把它传给 uart:= 参数即可。
只模拟协议和差速运动学，不接真实电机，也不改下位机协议。
"""

import argparse
import math
import os
import pty
import select
import socket
import sys
import time

START = b'\xaa\x55'
CODES = {1: '前进', 2: '转向', 3: '夹爪', 4: '相机', 5: '相对转角', 6: '角度校准（保留）', 7: '距离直行', 8: '停车'}


def checksum(frame):
    return sum(frame[:5]) & 0xFF


def pack(command, value):
    """按 0xAA 0x55 + 命令 + 小端数值 + 校验 打包 6 字节帧。"""
    raw = value & 0xFFFF
    frame = bytearray([0xAA, 0x55, command & 0xFF, raw & 0xFF, (raw >> 8) & 0xFF, 0])
    frame[5] = checksum(frame)
    return bytes(frame)


class Sim:
    """把像素误差按增益换成速度，带死区，避免在目标附近来回蹭。"""

    def __init__(self, speed_gain=0.00015, turn_gain=0.0008,
                 speed_limit=0.1, turn_limit=0.5,
                 speed_deadband=15, turn_deadband=12, min_turn=0., angle_turn_limit=.6, motion_scale=1.):
        self.speed_gain = speed_gain * motion_scale
        self.turn_gain = turn_gain * motion_scale
        self.speed_limit = speed_limit * motion_scale
        self.turn_limit = turn_limit * motion_scale
        self.speed_deadband = speed_deadband
        self.turn_deadband = turn_deadband
        self.min_turn = min_turn * motion_scale
        self.angle_turn_limit = angle_turn_limit * motion_scale
        self.motion_scale = motion_scale
        self.x = self.y = self.yaw = 0.0
        self.speed = self.turn = 0.0
        self.gripper = 0
        self.camera = 0
        self.turn_target = None
        self.distance_left = None
        self.straight_done = False
        self.received = 0
        self.last_report = 0.0
        self.pose_known = False
        self.turn_window = None

    def handle(self, command, value, distributing=False):
        self.received += 1
        if command in (1, 2, 5, 7, 8):
            self.straight_done = False
        if command == 2 and self.distance_left is not None:
            self.distance_left = None
            self.speed = 0.0
        if command == 1:
            self.distance_left = None
            gain = .0016 * self.motion_scale if distributing else self.speed_gain
            limit = .3 * self.motion_scale if distributing else self.speed_limit
            self.speed = 0.0 if abs(value) <= self.speed_deadband else max(
                -limit, min(limit, value * gain))
        elif command == 2:
            self.turn_target = None
            gain = .0032 * self.motion_scale if distributing else self.turn_gain
            limit = 1.2 * self.motion_scale if distributing else self.turn_limit
            minimum = 0. if distributing else self.min_turn
            self.turn = 0.0 if abs(value) <= self.turn_deadband else math.copysign(
                min(limit, max(minimum, abs(value)*gain)), -value)
        elif command == 3:
            self.gripper = 1 if value else 0
        elif command == 4:
            self.camera = max(0, min(80, value))
        elif command == 5:
            self.distance_left = None
            self.speed = 0.0
            self.turn_target = self.yaw - math.radians(value)
            self.turn = math.copysign(self.angle_turn_limit, -value) if value else 0.0
        elif command == 7:
            self.turn = 0.0
            self.turn_target = None
            self.distance_left = value / 100.0
            self.speed = self.distance_speed(self.distance_left)
        elif command == 8:
            self.speed = self.turn = 0.0
            self.turn_window = None
            self.turn_target = None
            self.distance_left = None
        print(f'[Stm32Sim] {CODES.get(command, "未知")}({command:02d}) = {value}', flush=True)

    def distance_speed(self, remaining):
        speed = self.speed_limit
        if self.motion_scale > 1.0:
            # 在目标附近提前减速，给仿真底盘留出刹车距离。
            speed = min(speed, math.sqrt(.7*abs(remaining)))
        return math.copysign(speed, remaining) if remaining else 0.0

    def step(self, dt):
        delta = self.turn * dt
        if self.turn_target is not None and abs(delta) >= abs(self.turn_target - self.yaw):
            self.yaw = self.turn_target
            self.turn = 0.0
            self.turn_target = None
        else:
            self.yaw += delta
        distance = self.speed * dt
        if self.distance_left is not None:
            if abs(distance) >= abs(self.distance_left):
                distance = self.distance_left
                self.distance_left = None
                self.speed = 0.0
                self.straight_done = True
            else:
                self.distance_left -= distance
                self.speed = self.distance_speed(self.distance_left)
        self.x += distance * math.cos(self.yaw)
        self.y += distance * math.sin(self.yaw)

    def report(self):
        return (f'[Stm32Sim] x={self.x:+.3f} y={self.y:+.3f} yaw={math.degrees(self.yaw):+7.1f} 度 '
                f'v={self.speed:+.3f} w={self.turn:+.2f} 相机={self.camera} 夹爪={self.gripper} '
                f'收到={self.received}')

    def update_pose(self, x, y, yaw):
        """接实物理时以实际位姿完成相对转向和距离直行。"""
        if self.pose_known and self.distance_left is not None:
            shift = (x - self.x) * math.cos(self.yaw) + (y - self.y) * math.sin(self.yaw)
            remaining = self.distance_left - shift
            if abs(remaining) < .005 or remaining * self.distance_left <= 0:
                self.distance_left = None
                self.speed = 0.
                self.straight_done = True
            else:
                self.distance_left = remaining
                self.speed = self.distance_speed(remaining)
        if self.pose_known:
            delta = yaw - self.yaw
            self.yaw += math.atan2(math.sin(delta), math.cos(delta))
        else:
            if self.turn_target is not None:
                self.turn_target += yaw - self.yaw
            self.yaw = yaw
        self.x, self.y = x, y
        self.pose_known = True
        if self.turn_target is not None:
            error = self.turn_target - self.yaw
            if abs(error) <= math.radians(1):
                self.turn_target = None
                self.turn = 0.
            else:
                self.turn = math.copysign(min(self.angle_turn_limit, max(self.min_turn, 2*abs(error))), error)

    def turn_output(self, sim_ns, distributing):
        """原地转向没带动车身时，短暂加力一次，用物理时间计时。"""
        if sim_ns is None or not self.min_turn or distributing or self.speed or not self.turn:
            self.turn_window = None
            return self.turn
        sign = math.copysign(1, self.turn)
        start = self.turn_window
        if start is None or sign != start[2] or abs(self.yaw-start[1]) >= math.radians(2):
            self.turn_window = (sim_ns, self.yaw, sign)
            return self.turn
        if 650_000_000 <= sim_ns-start[0] < 900_000_000:
            return sign * max(abs(self.turn), self.turn_limit)
        return self.turn


class Link:
    """把假下位机的速度和镜头角转发给电脑仿真的控制端口。"""

    def __init__(self, host, port):
        self.address = (host, port)
        self.socket = None
        self.buffer = bytearray()

    def send(self, text):
        if self.socket is None and not self.connect():
            return
        try:
            self.socket.sendall((text + '\n').encode())
        except OSError:
            self.close()

    def connect(self):
        try:
            self.socket = socket.create_connection(self.address, timeout=1)
            print(f'[Stm32Sim] 已连上仿真控制 {self.address[0]}:{self.address[1]}', flush=True)
            return True
        except OSError:
            return False

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        self.buffer.clear()

    def read_pose(self):
        if self.socket is None or not select.select([self.socket], [], [], 0)[0]:
            return None
        try:
            data = self.socket.recv(2048)
        except OSError:
            self.close()
            return None
        if not data:
            self.close()
            return None
        self.buffer.extend(data)
        lines = self.buffer.split(b'\n')
        self.buffer = lines.pop()
        if not lines:
            return None
        fields = lines[-1].split()
        if len(fields) in (4, 5) and fields[0] == b'P':
            return (*map(float, fields[1:4]), int(fields[4]) if len(fields) == 5 else None)
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-only', action='store_true', help='标准输出只打印伪串口路径')
    parser.add_argument('--sim-host', default='', help='仿真控制地址，如 WSL 的 172.27.176.1 或派上的 192.168.60.1')
    parser.add_argument('--sim-port', type=int, default=8091, help='仿真控制端口')
    parser.add_argument('--ros-clock', action='store_true', help='发布物理时间并读取打散阶段')
    parser.add_argument('--speed-gain', type=float, default=0.00015, help='前进像素误差→m/s')
    parser.add_argument('--turn-gain', type=float, default=0.0008, help='转向像素误差→rad/s')
    parser.add_argument('--speed-limit', type=float, default=0.1, help='前进速度上限 m/s')
    parser.add_argument('--turn-limit', type=float, default=0.5, help='转向角速度上限 rad/s')
    parser.add_argument('--speed-deadband', type=int, default=15, help='前进死区，像素')
    parser.add_argument('--turn-deadband', type=int, default=12, help='转向死区，像素')
    parser.add_argument('--min-turn', type=float, default=0., help='转向死区外的最小角速度 rad/s')
    parser.add_argument('--angle-turn-limit', type=float, default=.6, help='相对转角的角速度上限 rad/s')
    parser.add_argument('--motion-scale', type=float, default=1., help='仿真移动和转向倍率')
    args = parser.parse_args()
    if not math.isfinite(args.motion_scale) or args.motion_scale <= 0:
        parser.error('--motion-scale 必须为正数')

    ros = None
    distributing = False
    if args.ros_clock:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from robot_interfaces.msg import MatchState
        from std_msgs.msg import UInt64

        def match_state(message):
            nonlocal distributing
            distributing = message.stage == 'init_dis' and message.phase in ('', 'approach')

        rclpy.init(args=[])
        ros = rclpy.create_node('stm32_sim')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ros.create_subscription(MatchState, 'match/state', match_state, qos)
        clock = ros.create_publisher(UInt64, 'sim/clock', 10)

    master, slave = pty.openpty()
    name = os.ttyname(slave)
    # 不长期占着 slave：serial_node 要 TIOCEXCL 独占，上一轮残留的独占标志会挡住重开。
    os.close(slave)
    if args.device_only:
        print(name, flush=True)
        sys.stdout = open(os.devnull, 'w')
    sim = Sim(args.speed_gain, args.turn_gain, args.speed_limit,
              args.turn_limit, args.speed_deadband, args.turn_deadband, args.min_turn, args.angle_turn_limit,
              args.motion_scale)
    link = Link(args.sim_host, args.sim_port) if args.sim_host else None
    print(f'[Stm32Sim] 伪串口 {name}', flush=True)
    if link is None:
        print(f'[Stm32Sim] 启动：bash tools/run.sh uart:={name} ...', flush=True)
    else:
        print(f'[Stm32Sim] 启动：bash tools/run.sh uart:={name} ...，'
              f'动作会转发到仿真 {args.sim_host}:{args.sim_port}', flush=True)
    buffer = bytearray()
    opened = False
    last = time.monotonic()
    last_forward = 0.0
    last_angle = 0.0
    last_pose = float('-inf')
    sim_ns = None
    try:
        while True:
            if ros is not None:
                rclpy.spin_once(ros, timeout_sec=0)
            ready, _, _ = select.select([master], [], [], 0.05)
            now = time.monotonic()
            if link is None:
                sim.step(now - last)
            else:
                pose = link.read_pose()
                if pose is not None:
                    x, y, yaw, sim_ns = pose
                    sim.update_pose(x, y, yaw)
                    if ros is not None and sim_ns is not None:
                        clock.publish(UInt64(data=sim_ns))
                    last_pose = now
            last = now
            if ready:
                try:
                    data = os.read(master, 256)
                except OSError:
                    # 串口节点关掉了，重开后收到一帧就继续上报角度。
                    opened = False
                    buffer.clear()
                    time.sleep(0.05)
                    continue
                buffer.extend(data)
            while len(buffer) >= 6:
                if buffer[:2] != START or buffer[5] != checksum(buffer):
                    del buffer[0]
                    continue
                command = buffer[2]
                raw = buffer[3] | (buffer[4] << 8)
                value = raw - 65536 if raw >= 32768 else raw
                opened = True
                sim.handle(command, value, distributing)
                if link is not None:
                    if command == 4:
                        link.send(f'C {sim.camera}')
                    elif command == 3:
                        link.send(f'G {sim.gripper}')
                del buffer[:6]
            if link is not None and now - last_forward > 0.1:
                if now - last_pose > .5 and (sim.speed or sim.turn):
                    sim.handle(8, 0)
                last_forward = now
                link.send(f'D {sim.speed:.4f} {sim.turn_output(sim_ns, distributing):.4f}')
            if opened and sim.straight_done:
                os.write(master, pack(7, 0))  # 协议只定义命令位，内容不用于判断。
                sim.straight_done = False
            if opened and (link is None or now - last_pose <= .5) and now - last_angle >= 0.05:
                os.write(master, pack(1, round(math.degrees(sim.yaw)) % 360))
                last_angle = now
            if now - sim.last_report > 1.0:
                sim.last_report = now
                print(sim.report(), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if link is not None:
            link.close()
        os.close(master)
        if ros is not None:
            ros.destroy_node()
            rclpy.try_shutdown()
        print('[Stm32Sim] 已退出', flush=True)


if __name__ == '__main__':
    main()
