# 单项测试

单项测试节点放在 `tests_pkg/nodes/`，目前有摄像头测试和靠近测试，靠近测试，对应原工程 `tests/single_tests/approach_test.py`。参数同样在 [robot.yaml](../robot_launch/config/robot.yaml)，按节点分组。

## 靠近

`approach_test_node` 对应原工程的单项靠近测试，每帧选指定颜色中底边中心离参考点最近的物块，输出左右和前后像素误差。看不见指定颜色的物块时补发一次停车，同一次丢失只发一条，重新看见目标后自动恢复逐帧输出。

在香橙派仓库根目录编译后运行。默认只输出误差，不接串口：

```bash
bash tools/build.sh
bash tools/run.sh approach color:=orange model:="$PWD/assets/models/train5.rknn"
```

另开一个派上的终端查看输出：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 topic echo /approach_test/command
```

`command: 2` 是转向误差，`command: 1` 是前后误差，`command: 7` 是按厘米直行，`command: 8` 表示停车。命令位常量定义在 [Command.msg](../robot_interfaces/msg/Command.msg)，节点里用 `Command.TURN`、`Command.STRAIGHT`、`Command.STOP`，不写字面量。候选框可查看 `/approach_test/target`。已有视觉节点在跑时，也可单独运行 `ros2 run tests_pkg approach_test_node --ros-args --params-file src/robot_launch/config/robot.yaml`。

需要实车动起来时，先关闭其他动作节点，再给上述启动命令加 `uart:=/dev/ttyS6`。这个模式只启动靠近测试，不启动完整任务。用 `ros2 service call /approach_test/start std_srvs/srv/Trigger '{}'` 开始，`/approach_test/stop` 停止。测试沿用原来逐帧靠近的做法，加了误差限幅、丢目标及断流停车。

## 客户端靠近测试的模拟串口

点「测试节点」里的靠近测试会启动模拟串口。命令行在上面的靠近测试命令末尾加 `simulate_uart:=true` 即可。`tools/run.sh` 会启动 STM32 模拟器，把生成的 `/dev/pts/N` 交给 `serial_node`，仍走同一套串口收发。在香橙派加载工作区后，可以查看运动话题和打包出的帧。

```bash
ROS_DOMAIN_ID=27 ros2 topic echo /robot/command
ROS_DOMAIN_ID=27 ros2 topic echo /robot/serial_frames
```

开启模拟串口的靠近测试使用独立 ROS 域 27，避免测试消息进入正式任务的串口。

## 摄像头

`camera_test_node` 订阅 `/camera/image`，每秒在日志记录分辨率和实际收到的帧率。有图时客户端也会显示画面。使用客户端「测试节点」菜单启动，或在香橙派的仓库根目录运行：

```bash
bash tools/preview.sh start camera_test
journalctl _SYSTEMD_USER_UNIT=gongchuang-preview.service -f
```

## 串口动作测试

在客户端点「测试节点 → 运行串口测试」，选模拟串口或实车串口。右侧终端出现 `cmd>` 后直接输入命令。

也可以从香橙派的仓库根目录启动：

```bash
bash tools/run.sh serial
```

脚本会同时启动串口桥接和交互测试。出现 `cmd>` 后直接输入命令，可以连续测试：

```text
cmd> move_straight 0
cmd> move_straight 30
cmd> move_straight_dis 20
cmd> move_turn 10
cmd> move_turn_angle 30
cmd> grab 0
cmd> camera 20
cmd> stop
cmd> help
cmd> quit
```

测试命令和 `MyCMD` 方法同名。每条输入发布一次对应命令。`move_turn_angle` 直接发送相对转角，正值向右，负值向左。模拟串口也能查看它的发送帧。

`06` 预留给绝对角度校准，参数待定，串口节点暂不发送该指令。

默认由 `stm32_sim.py` 提供伪串口，`serial_node` 按普通串口读写。接好 STM32 并腾出 UART 后，用 `bash tools/run.sh serial /dev/ttyS6` 启动实车测试。
