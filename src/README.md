---
date created: 2026-11-14  13：33
date modified: 2026-11-14  20：56
---
# ROS 2 功能包

|功能包|构建方式|代码组织|
|---|---|---|
|`vision_pkg`|`ament_cmake`|`src/` 放 C++，`include/vision_pkg/` 放头文件，Python 节点和算法按混合包方式安装|
|[`communication_pkg`](communication_pkg/README.md)|`ament_cmake`|`MyCMD` 提供动作方法，`SerialNode` 接 ROS 话题，`MyUART` 读写字节，`SimpleProtocol` 和 `ProtocolParser` 负责协议|
|`action_pkg`|`ament_python`|`action_pkg/` 放 Python 代码，保留 `setup.py`|
|`tests_pkg`|`ament_python`|`tests_pkg/` 放 Python 测试节点，保留 `setup.py`|
|`desktop_server_pkg`|`ament_cmake`|C++ 服务端|
|`robot_interfaces`|`ament_cmake`|`msg/`、`srv/`、`action/`|
|`robot_launch`|`ament_cmake`|`config/`、`launch/`|

## 环境

Windows 用 WSL2 Ubuntu 22.04，香橙派 5 Plus 用 Ubuntu Jammy ARM64。两端使用 ROS 2 Humble、C++17、Python 3.10 和 OpenCV。ROS 软件源配置好后，在仓库根目录执行：

```bash
sudo apt install ros-humble-ros-base ros-humble-rosidl-default-generators ros-humble-ament-cmake-python \
  ros-humble-serial-driver ros-humble-asio-cmake-module libasio-dev \
  python3-colcon-common-extensions libopencv-dev python3-opencv python3-venv
bash tools/build.sh
```

## 香橙派

用 `ssh pio` 或 `ssh pio-relay` 登录，在 `~/my_ws/gongchuang_ros2` 拉取并编译。该目录对应 GitHub 仓库 [Kristinblue/gongchuang_2](https://github.com/Kristinblue/gongchuang_2)。首次使用派上已有的 RKNNLite wheel 和模型：

```bash
bash tools/setup_pi.sh ~/rknn-toolkit2/rknn-toolkit-lite2/packages/*cp310*.whl \
  ~/my_ws/gongchuang/my_project/assets/models/11_11/train5.rknn
bash tools/run.sh source:=usb camera_device:=/dev/video0 model:="$PWD/assets/models/train5.rknn"
```

编译后，在 Windows 客户端连接香橙派即可看预览，用「测试节点」菜单选择靠近测试或摄像头测试。在香橙派也可以从任意目录运行 `bash ~/my_ws/gongchuang_ros2/tools/preview.sh start preview` 或 `bash ~/my_ws/gongchuang_ros2/tools/preview.sh start approach`。调试时在仓库根目录运行 `bash tools/run.sh`，可直接看前台日志。默认自动找 USB 摄像头，必要时用 `camera_device:=/dev/videoN` 指定。未接相机显示「无视频输入」。预览默认只启动摄像头、视觉处理和 HTTP 节点。Windows 客户端从本机 SSH 配置读取连接别名，连接和队友密钥配置见[客户端说明](../tools/laptop_server/README.md)。

实车联调在启动命令中加 `uart:=/dev/ttyS6 team:=red`（蓝方用 `team:=blue`）。有新画面时，STM32 可发 `00, 1`，也可用 `ros2 service call /task/start std_srvs/srv/Trigger '{}'` 开始。STM32 发 `00, 2` 或调用 `/match/reset` 会停车并复位，随后等待新的启动指令。

模型权重放在仓库的 `assets/models/` 下。`model:=...` 会覆盖 `inference_node` 下的 `yolo.model_path`，两处都留空时只看原图。启动时使用绝对路径（例如 `model:="$PWD/assets/models/train5.rknn"`）。示例用 `$PWD` 从仓库根目录生成绝对路径。

## 电脑仿真

不接车时，Windows 上跑 `python tools/sim/stream.py` 把仿真画面推成 MJPEG，WSL 里用现成的 http 摄像头源读取：

```bash
bash tools/sim/run_sim.sh mode:=approach model:="$PWD/assets/models/train5.onnx"
```

仿真只提供画面，不发运动指令。没有下位机时，`tools/stm32_sim.py` 会用伪串口模拟 STM32。
说明见[仿真接入](../tools/sim/README.md)。

## 配置和单项测试

日常参数在 [robot.yaml](robot_launch/config/robot.yaml)，原版留在 [project_config.yaml](../tools/legacy/project_config.yaml)。参见[单独验证靠近](tests_pkg/README.md)。

视觉 ROS 入口在 `vision_pkg/scripts/`，算法在 `vision_pkg/modules/`。任务顺序在 `action_pkg/action_pkg/task_controller.py`。

安全区按需识别由 C++ 节点 `safety_detector_node` 提供。启动时加 `safety_detector:=true`，然后在已 source 工作区的另一个终端开启识别。

```bash
bash tools/run.sh safety_detector:=true
ros2 service call /vision/safety_detector/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /vision/safety_detection --qos-reliability best_effort
```

开启后持续处理 `camera/image`，图像队列只留一帧。关闭时对同一服务发 `{data: false}`，节点解除图像订阅。结果包含相机时间戳、`flag`、`detected/full/ok` 和原图多边形。有效帧里没有安全区也会发 `detected=False`；相机断流时用结果时间戳判断超时。
