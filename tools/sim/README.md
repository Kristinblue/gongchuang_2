# 工创仿真接入

不新增 ROS 包，仿真画面走现成的 `camera_url` 视频流，和真实网络摄像头同一条路；
没有下位机时用伪串口模拟 STM32。这两个脚本只在电脑上用，不进香橙派的功能包。

| 文件 | 在哪跑 | 作用 |
| --- | --- | --- |
| `stream.py` | Windows | 启动仿真（Godot + WSL 物理），把相机画面推成 MJPEG |
| `run_sim.sh` | WSL | 自动解析 Windows 地址，用 `source:=http` 启动 ROS |
| `../stm32_sim.py` | WSL | 伪串口模拟 STM32，上报角度、接收动作命令 |

## 仿真当摄像头

Windows 一个终端：

```powershell
python tools/sim/stream.py
python tools/sim/stream.py --pitch 40 --fps 15
```

看到 `视频流 http://0.0.0.0:8090/video` 后，WSL 里：

```bash
bash tools/sim/run_sim.sh mode:=approach model:="$PWD/assets/models/train5.onnx"
```

`run_sim.sh` 会把 `ip route` 里的网关地址填进 `camera_url`，所以不用手抄 IP。
浏览器打开 `http://localhost:18080` 看预览，`ros2 topic echo /approach_test/command` 看误差。

- 默认 24° 能看到近处物块，看不到就换 `--pitch 40`。
- 默认请求 15 fps，实测约 13 fps；调到 10 fps 时只有约 8 fps，渲染是主要开销。
- 不接假 STM32 时只提供画面，车不会自己动。

## 省 CPU

- `stream.py` 启动时会先清掉上次残留的 `ign gazebo`/物理宿主，避免强杀后越堆越多。
  之前堆了 6 个孤儿，白吃 8 个核。
- 只看画面不测检测时，别带 `model:=`：`run_sim.sh` 不带模型就只有摄像头和预览，
  WSL 里从约 3.4 核降到 1.2 核；带模型时 YOLO 推理会再多占 2 核左右。
- 还想更低就降帧率和质量：`python tools/sim/stream.py --fps 10 --quality 75`。
- 正常情况下只有一个 `ign gazebo`，`ps -eo args | grep "[a]lgorithm_world"` 看到一堆就是有残留。

## 只放一个物块

场地默认 20 个物块，渲染用 `field.glb`、物理用 `gripper_manual.sdf`，两套都要改，用 `field.py` 一次处理：

```bash
python tools/sim/field.py one-green          # 物理删到只剩 block_green_01，并给 Godot 插开关
python tools/sim/stream.py --only-block block_green_01
python tools/sim/field.py restore            # 还原（两个 .bak 都会拷回去）
```

改动都会先留 `.bak`：`worlds/gripper_manual.sdf.bak` 和 `native_render/scripts/gripper_stream.gd.bak`。
`--only-block` 只影响画面；物理那边靠 `field.py` 删模型，两边不一致时车会撞到看不见的块。

## 模拟 STM32

```bash
python3 tools/stm32_sim.py
```

会打印 `伪串口 /dev/pts/N`，然后另开终端：

```bash
bash tools/run.sh uart:=/dev/pts/N team:=red model:="$PWD/assets/models/train5.onnx"
bash tools/run.sh uart:=/dev/pts/N mode:=approach model:="$PWD/assets/models/train5.onnx"
```

伪串口收到启动时的停车帧后，用 `01` 连续上报角度。`01/02/03/04/05/07/08` 按现有协议处理并打印速度和位姿。`01` 正值向前，`07` 按厘米直行，`08` 停车。`/robot/state` 会显示串口状态、队伍和导航角度。

加上 `--sim-host` 后，假板还会把动作转发给仿真，车就真的动起来：

```bash
python3 tools/stm32_sim.py --sim-host 172.27.176.1   # WSL 看 Windows 的网关
python3 tools/stm32_sim.py --sim-host 192.168.60.1   # 派看 PC 的有线地址
```

转发的是 `D 速度 转向`、`C 俯角`、`G 开合`，`stream.py` 的控制端口默认 8091。

## 电脑客户端（有线）

客户端只会连 SSH 别名对应的派，而仿真是本机跑的。用一个反向隧道把派的 18080 让给本机：

```powershell
ssh -N -R 18080:127.0.0.1:18080 pio-wired
```

再打开 `dist/GongchuangClient.exe`，模式选 `pio-wired` 连接，看到的就是本机仿真的实时画面。
派上原来的 ROS 先停掉，不然 18080 被它占着。

## 一起跑

```powershell
# Windows
python tools/sim/stream.py
ssh -N -R 18080:127.0.0.1:18080 pio-wired    # 要用客户端时再加
```

```bash
# WSL
python3 tools/stm32_sim.py --sim-host 172.27.176.1   # 记下 /dev/pts/N
bash tools/sim/run_sim.sh mode:=approach uart:=/dev/pts/N model:="$PWD/assets/models/train5.onnx"
ros2 service call /approach_test/start std_srvs/srv/Trigger '{}'
```

`stream.py` 会自己启动仿真后端，占用 5060，别和已经开着的仿真窗口同时用。
强杀 Windows 脚本时下次启动会自动清残留，也可以用 `ps -eo args | grep "[a]lgorithm_world"` 手动查。
