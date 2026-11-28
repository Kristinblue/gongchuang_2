# 动作

参数集中在 [robot.yaml](../robot_launch/config/robot.yaml)，`task_node.ros__parameters` 下沿用 `point`、`distribute` 和 `trans`。感知节点也用 `trans.camera_settle_ms`。原 Python 配置在 `gongchuang/my_project/project_config.yaml`，可对照 `point`、`distribute`、`trans`。运行时修改 ROS 2 的 YAML，或用 `config:=/绝对路径/robot.yaml` 加载另一份。

通信包里的 [MyCMD](../communication_pkg/communication_pkg/my_cmd.py) 用 `move_straight()`、`move_straight_dis()`、`move_turn()` 和 `move_turn_angle()` 将命令发到 `/robot/command`。角度回包从 `/robot/feedback` 交给 `MyCMD.read_feedback()`。

`task_node` 开场打散一次。GoTo 选定并靠近目标，Grabber 低头对准、合爪并复查。启用投放时，Delivery 把单块送往本方对应格，退回后再搜寻。首趟只选绿，后续可选绿、黑、橙；整场复位后恢复 YAML 的首趟设置。单项靠近测试见 [tests_pkg](../tests_pkg/README.md)。

抓取时可以先合爪、后退。爪口被挡住，或后退后看到多块，就开爪从堆边斜推。退开后先转回目标，再靠近试夹。开爪后按顺序后退、转向、前推、后退、回转。距离和角度见 YAML 的 `grab.scatter_back_cm`、`grab.scatter_push_cm` 和 `grab.scatter_angle_deg`。转向靠航向回包确认完成。直行段等 STM32 的 07 到位回包。超过 `grab.scatter_drive_timeout_ms` 就停车并重新搜寻。

## GoTo 和 STM32

`Task` 每收到一帧物块就调用 `GoTo.run()`。GoTo 用目标框底边中心和 `point.x0/y0` 算横向和前后像素误差，再调用 `MyCMD`。STM32 负责根据误差做 PID 控制。`MyCMD` 直接发送 `05` 相对转角，用已有的 `01` 角度样本或者发令后的第一帧角度作为累计起点，达到目标角度（允许 1° 误差）后将 `turn_done` 置为真。

命令编号和帧格式见[串口说明](../communication_pkg/README.md)。当前 `/task/start` 和 `/task/stop` 是服务。将来拆 GoTo 节点时，可以用 Action 管一次搜寻和靠近的目标、进度、结果和取消，逐帧像素误差仍走现有话题。

## 本场状态与复位

`TaskNode` 用 `/match/state` 发布当前阶段、已处理帧数和已确认目标。桌面预览通过这个话题获取阶段。运行正式任务时，在已加载工作区的终端查看状态或复位。

```bash
ros2 topic echo /match/state
ros2 service call /match/reset std_srvs/srv/Trigger '{}'
```

复位发布停车命令，清掉本场目标、计时和进度，然后回到等待开始的 `stop` 阶段。下一次 `/task/start` 需要先收到复位后的视觉帧。服务回复确认的是上位机复位，STM32 继续使用原有停车命令。
