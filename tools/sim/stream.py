"""把工创仿真画面推成 MJPEG 视频流，给 ROS 的 camera_node 当 http 摄像头。

Windows 上运行，自己启动仿真后端（Godot + WSL 物理）。
WSL 里用 tools/sim/run_sim.sh 启动，脚本会把 Windows 主机地址填进 camera_url。
"""

import argparse
from contextlib import suppress
import json
import os
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from math import atan2, radians
from pathlib import Path

import cv2

DEFAULT_APP = Path(r"D:\my_project\gongchuang\my_project")
# 强杀 stream.py 时仿真宿主可能变成孤儿，启动前按命令行清一遍
CLEAN_STALE = ("ps -eo pid,args | "
               "awk '/[a]lgorithm_world\\.sdf|[g]ripper_algorithm_host/ {print $1}' | "
               "xargs -r kill -9")


def clean_stale():
    """清掉上一次没退干净的 Gazebo/物理宿主，返回清掉的进程数。"""
    try:
        result = subprocess.run(['wsl', '-e', 'bash', '-lc', CLEAN_STALE],
                                capture_output=True, text=True, timeout=10)
    except OSError:
        return 0
    return len([line for line in result.stdout.splitlines() if line.strip()])



def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app-root', type=Path, default=DEFAULT_APP, help='原工程 my_project 目录')
    parser.add_argument('--bind', default='0.0.0.0', help='HTTP 监听地址')
    parser.add_argument('--port', type=int, default=8090, help='MJPEG 端口')
    parser.add_argument('--control-port', type=int, default=8091, help='STM32 指令控制端口')
    parser.add_argument('--fps', type=float, default=15, help='取帧频率')
    parser.add_argument('--pitch', type=float, default=24, help='镜头俯角，0 水平、向下为正')
    parser.add_argument('--quality', type=int, default=85, help='JPEG 质量')
    parser.add_argument('--only-block', default='', help='只显示这个物块，例如 block_green_01')
    parser.add_argument('--client', action='store_true', help='客户端退出时一起关闭仿真')
    parser.add_argument('--ros', action='store_true', help='同时启动 WSL 的 ROS 推理和模拟串口')
    return parser.parse_args()


class Frames:
    """只保留最新一帧，多个 HTTP 连接各自取。"""

    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg = None
        self.count = 0

    def put(self, jpeg):
        with self.condition:
            self.jpeg = jpeg
            self.count += 1
            self.condition.notify_all()

    def latest(self, after):
        """等一帧新的；超时不重复发送旧帧。"""
        with self.condition:
            if not self.condition.wait_for(lambda: self.count != after, timeout=5):
                return None, after
            return self.jpeg, self.count


class Control:
    """接收假 STM32 转发来的指令，转成仿真后端的 D / C / G。"""

    def __init__(self, port, client, mapping):
        self.client = client
        self.mapping = mapping
        self.pose = None
        self.server = socket.socket()
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', port))
        self.server.listen(1)

    def serve(self):
        while True:
            connection, _ = self.server.accept()
            with connection:
                try:
                    with connection.makefile('r', encoding='utf-8') as stream:
                        for line in stream:
                            line = line.strip()
                            self.handle(line)
                            pose = self.pose
                            if line.startswith('D ') and pose is not None and time.monotonic() - pose[0] <= .5:
                                connection.sendall(('P %.6f %.6f %.6f %d\n' % pose[1:]).encode('ascii'))
                except OSError:
                    pass
                finally:
                    with suppress(OSError):
                        self.client.command('D 0 0')

    def handle(self, line):
        parts = line.split()
        if not parts:
            return
        try:
            if parts[0] == 'D' and len(parts) == 3:
                self.client.command(f'D {float(parts[1]):.4f} {float(parts[2]):.4f}')
            elif parts[0] == 'C' and len(parts) == 2:
                degrees = max(0.0, min(self.mapping['camera_max_deg'], float(parts[1])))
                joint = self.mapping['camera_joint_0'] + radians(degrees)
                self.client.command(f'C {joint:.6f}')
            elif parts[0] == 'G' and len(parts) == 2:
                self.client.command(f'G {1 if int(parts[1]) else 0}')
        except (ValueError, OSError):
            pass


class Handler(BaseHTTPRequestHandler):
    frames = None
    client = None
    config = None

    def do_GET(self):
        if self.path == '/':
            self.reply_text('Gongchuang sim MJPEG：/video 是视频流，/status 是帧数。\n')
        elif self.path == '/status':
            self.reply_text(json.dumps({'frames': self.frames.count}) + '\n', 'application/json')
        elif self.path == '/video':
            self.stream()
        elif self.path == '/config' and self.client_address[0] == '127.0.0.1':
            config = dict(self.config['stream'])
            for name in ('scene', 'gripper_manual', 'gripper_camera'):
                path = self.config['root'] / 'config' / f'{name}.json'
                try:
                    config[name] = json.loads(path.read_text(encoding='utf-8'))
                except (OSError, ValueError) as error:
                    config[name] = str(error)
            self.reply_text(json.dumps(config, ensure_ascii=False), 'application/json; charset=utf-8')
        else:
            self.send_error(404)

    def do_POST(self):
        if (self.path != '/overview' or self.client_address[0] != '127.0.0.1' or
                self.headers.get_content_type() != 'application/json'):
            self.send_error(404)
            return
        if self.client is None:
            self.send_error(503)
            return
        try:
            self.client.message('overview', visible=True)
        except OSError:
            self.send_error(503)
            return
        self.reply_text('3D 全景已打开\n')

    def reply_text(self, text, content_type='text/plain; charset=utf-8'):
        body = text.encode()
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        seen = 0
        try:
            while True:
                jpeg, seen = self.frames.latest(seen)
                if jpeg is None:
                    continue
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
        except OSError:
            return

    def log_message(self, *args):
        pass


def host_address():
    """WSL 看 Windows 用的地址，和 tools/sim/run_sim.sh 里一致。"""
    try:
        result = subprocess.run(
            ['wsl', '-e', 'bash', '-lc', "ip route show default | awk '{print $3}'"],
            capture_output=True, text=True, timeout=10)
        return result.stdout.strip()
    except OSError:
        return ''


def main():
    args = parse_args()
    sys.path.insert(0, str(args.app_root))
    from tools.sim.gripper_backend import GripperBackend, SIM_ROOT, camera_mapping

    mapping = camera_mapping()
    pitch = max(0.0, min(args.pitch, mapping['camera_max_deg']))
    joint = mapping['camera_joint_0'] + radians(pitch)
    frames = Frames()
    Handler.frames = frames
    if args.only_block:
        # 给 Godot 侧开关用，只显示一个物块
        os.environ['GC_ONLY_BLOCK'] = args.only_block
    stale = 0 if args.client else clean_stale()
    if stale:
        print(f'[SimStream] 清掉 {stale} 个上次残留的仿真进程', flush=True)
    stop = threading.Event()
    if args.client:
        def wait_for_client():
            os.read(sys.stdin.fileno(), 1)
            stop.set()
        threading.Thread(target=wait_for_client, daemon=True).start()
    variant = 'gripper_1123_01'
    new_world = SIM_ROOT / f'worlds/{variant}.sdf'
    new_model = SIM_ROOT / f'native_render/assets/{variant}.glb'
    use_new_jaws = new_world.is_file() and new_model.is_file()
    if use_new_jaws:
        backend = GripperBackend(stop, variant=variant)
    else:
        backend = GripperBackend(stop)
    if args.ros and not args.only_block:
        world = new_world if use_new_jaws else backend.root / 'worlds/gripper_manual.sdf'
        if world.read_text(encoding='utf-8').count('<model name="block_') < 2:
            raise RuntimeError('[SimStream] 物理场地仍是单物块测试版，请恢复完整场地后再运行任务')
    Handler.config = {'root': backend.root,
                      'stream': {'fps': args.fps, 'pitch': pitch, 'quality': args.quality,
                                 'only_block': args.only_block}}
    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'[SimStream] 视频流 http://{args.bind}:{args.port}/video', flush=True)
    print('[SimStream] 启动仿真，首次约 30 秒', flush=True)
    ros = None
    try:
        client = backend.start()
        Handler.client = client
        # 中位看到的是白地面，先用固定俯角对准场上的物块。
        client.command(f'C {joint:.6f}')
        control = Control(args.control_port, client, mapping)
        threading.Thread(target=control.serve, daemon=True).start()
        if args.ros:
            ros = subprocess.Popen(
                ['wsl', '-d', 'Ubuntu-22.04', '--', 'bash', 'tools/sim/run_sim.sh', '--desktop'],
                cwd=Path(__file__).resolve().parents[2],
                env={**os.environ, 'SIM_PORT': str(args.port), 'SIM_CONTROL_PORT': str(args.control_port),
                     'WSLENV': os.environ.get('WSLENV', '') + ':SIM_PORT:SIM_CONTROL_PORT'},
                stdin=subprocess.PIPE, creationflags=subprocess.CREATE_NO_WINDOW)
        print(f'[SimStream] 控制端口 {args.control_port}，相机俯角 {pitch:g} 度，'
              f'Windows 地址 {host_address() or "未知"}', flush=True)
        period = 1 / args.fps
        count = 0
        while not stop.is_set():
            if ros is not None and ros.poll() is not None:
                raise RuntimeError(f'[SimStream] WSL 推理服务退出，状态 {ros.returncode}')
            start = time.monotonic()
            meta, image = client.frame()
            pose = meta.get('observer', {}).get('poses', {}).get('gc_chassis')
            if meta.get('robot_valid') and pose and len(pose) == 7:
                x, y, _, qx, qy, qz, qw = pose
                yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                control.pose = time.monotonic(), x, y, yaw, meta['sim_ns']
            else:
                control.pose = None
            ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, args.quality])
            if ok:
                frames.put(encoded.tobytes())
                count += 1
                if count % 100 == 0:
                    print(f'[SimStream] 已推送 {count} 帧', flush=True)
            delay = period - (time.monotonic() - start)
            if delay > 0:
                time.sleep(delay)
    except KeyboardInterrupt:
        print('[SimStream] 手动停止', flush=True)
    finally:
        Handler.client = None
        if ros is not None:
            ros.stdin.close()
            try:
                ros.wait(timeout=8)
            except subprocess.TimeoutExpired:
                ros.terminate()
        backend.close()
        server.shutdown()
        server.server_close()
        print('[SimStream] 已关闭仿真', flush=True)


if __name__ == '__main__':
    main()
