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
- 本机同时有 `gripper_1123_01.sdf` 和 `gripper_1123_01.glb` 时，客户端仿真会一起加载新版双爪的物理和画面。新版沿用旧场地、底盘及开合限位。旧仿真资源仍可供原手动场景使用。

仿真任务先打散、搜寻和追踪。追踪到位后停车，把相机压到 `grab.camera_deg`，
用新视角重新确认目标，再开爪慢进。到近处后切换到 `grab.near_camera_deg` 看爪口。
旋转中逐帧比较无遮挡的合格物块，用类别权重和框底边位置评分。最佳候选达到门槛就停车锁定，预览黄框标出当前帧的锁定目标。同一视角追踪期间保持类别和 ID，低头换视角后用位置重新对应。爪口对中、目标前部已进入合爪范围且没有旁块时即可尝试合爪。后退 7 cm 后低头复查；只有一个同类目标但偏侧时，允许松开再接近一次。首趟只允许确认一个绿色物块后转运。
参数在 `src/robot_launch/config/robot.yaml`。调整俯角后需要重新核对像素范围。

仿真正式任务通过单块复查后找本方安全区。用完整轮廓算前长边，先到安全区前方对准并停车复查，再直进、开爪、推入和后退。`delivery_done` 表示本趟投放动作结束，下一帧回到搜寻，可以选绿、黑、橙。回放用物块最终位姿检查是否入区。投放超时会停车，整场复位后重新按首趟规则启动。

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

## 电脑客户端

在客户端把输入源切到「仿真」，点「启动仿真」。Godot 的车载原图从 8090 送入 WSL，ROS 2 优先加载 `assets/models/claw_20261125.onnx`，没有这个文件时使用 `assets/models/train5.onnx`，客户端从 18082 显示处理后的预览。收到画面后可点「运行仿真任务」，也可点「复位」。当前客户端的仿真启动默认红方，调试用伪串口和独立 ROS 域 93。

仿真预览为 1280×720。完整场地的初始打散沿用 8 秒直推方案，计时取 Gazebo 物理时间，打散速度沿用原电脑仿真的增益。单物块场地用于单项测试。

「3D 全景」打开同一场仿真的独立 Godot 窗口。「环境配置」可只读查看推流参数及原环境的 `scene.json`、`gripper_manual.json`、`gripper_camera.json`。直接运行 `stream.py` 时客户端也可预览 8090 的原图。

客户端启动的仿真随客户端关闭。客户端优先用原电脑仿真入口使用的 yolov8 环境，其他电脑可用 `GC_SIM_PY` 指定已安装原工程的 Python。首次使用先在 WSL 运行 `bash tools/build.sh`，并确认现有 `tools/.venv` 可导入 ONNX Runtime。

## 一起跑

```powershell
# Windows
python tools/sim/stream.py
```

```bash
# WSL
python3 tools/stm32_sim.py --sim-host 172.27.176.1   # 记下 /dev/pts/N
bash tools/sim/run_sim.sh mode:=approach uart:=/dev/pts/N model:="$PWD/assets/models/train5.onnx"
ros2 service call /approach_test/start std_srvs/srv/Trigger '{}'
```

`stream.py` 会自己启动仿真后端，占用 5060，别和已经开着的仿真窗口同时用。
强杀 Windows 脚本时下次启动会自动清残留，也可以用 `ps -eo args | grep "[a]lgorithm_world"` 手动查。
