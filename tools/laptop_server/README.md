# Windows 客户端原型

打开 `dist/GongchuangClient.exe`，左上角选择香橙派或文件夹。默认左侧显示单张画面，右侧可切换终端和串口。客户端恢复上次的输入源、文件夹、SSH 别名和本地 HSV 阈值。

- 「香橙派」会自动用保存的 SSH 别名连接，首次默认 `pio`。画面服务按需启动，上方显示当前阶段或测试节点。首次使用需在香橙派编译工作区。
- 「文件夹」会重新打开上次的目录。首次点「打开图片文件夹」，可选 `tools/experiments/detect_safety/assets/first`。按 A 看上一张，按 D 看下一张。

| 别名 | 用途 |
| --- | --- |
| `pio` | 无线局域网直连 |
| `pio-relay` | 云服务器中转 |
| `pio-wired` | 网线直连 |

客户端通过系统 OpenSSH 读取当前用户的 `%USERPROFILE%/.ssh/config`，包括地址、SSH 端口、密钥和跳板设置。客户端自动转发派上的预览端口 18080。右上方「连接」可手动重连香橙派。

队友各自生成密钥，将公钥交给管理员添加到香橙派。用 `ssh -G pio-wired` 检查实际地址，用 `ssh pio-wired hostname` 验证登录。首次连接在终端核对主机指纹。无线和中转同理，切换为对应别名。

正式任务选择红方或蓝方后点「应用队伍」，香橙派会启动正式任务节点并进入「等待启动」。有新画面后，可由 STM32 发 `00, 1`，也可点「启动任务」。STM32 发 `00, 2` 或点「复位任务」会停车并清空本场，回到「等待启动」。复位后沿用已选队伍。切换到正式任务前先退出实车串口测试。

「测试节点」菜单列出 ROS 节点原名。选 `approach_test_node` 会启动 YOLO 和靠近测试，使用 `assets/models/train5.rknn`。误差发到 `/approach_test/command`。选 `camera_test_node` 会记录分辨率和实际帧率。客户端的靠近测试会发布运动命令，再由模拟串口打包。点「结束测试」回到原图。摄像头以 YUYV、10 fps 采图。

「运行串口测试 · `serial_test_node`」可选「模拟串口」或「实车串口 · `/dev/ttyS6`」。右侧终端出现 `cmd>` 后可输入 `move_straight 30`、`move_straight_dis 20`、`move_turn -50`、`move_turn_angle 30` 或 `stop`。`quit` 或「结束测试」会回到 SSH 提示符。实车测试前先腾出串口。

右侧「串口」页显示串口是否打开和十六进制字节。`TX` 是发出，`RX` 是收到的有效帧。选择模拟串口时，由 `stm32_sim.py` 提供伪串口，串口页沿用相同的显示方式。

运行 `serial_test_node` 时，这一页显示当前测试的完整协议帧。蓝色 `TX` 是发送帧，红色 `RX` 是校验通过的接收帧。界面支持按收发方向和命令位筛选。

SSH 终端会把输入命令和收到的标准输出、错误输出写到本机。下一次连接会覆盖记录，可以随时删除。

在 `$` 或 `cmd>` 提示符后直接输入。左右键移动光标，上下键翻历史。输出可以拖选、双击选词和按 `Ctrl+C` 复制，命令行支持 `Ctrl+V` 粘贴。

```powershell
Get-Content "$env:LOCALAPPDATA/Gongchuang/GongchuangClient/ssh-terminal.log" -Encoding utf8 -Wait -Tail 30
```

「已连接 · 最近 2 秒未收到新画面」表示派上的预览服务在线，摄像头暂时没有新帧。一键启动通过 `tools/preview_fastdds.xml` 给本机 ROS 图像传输预留共享内存。派上的启动和取帧日志可用 `journalctl _SYSTEMD_USER_UNIT=gongchuang-preview.service -n 30 --no-pager` 查看。

## 安全区 HSV

勾选画面下方的「安全区调试」才显示四图和 HSV 调色。香橙派预览配置了模型时，YOLO 持续推理，传来一张原尺寸无损图和同帧检测坐标。客户端在原图视图显示 YOLO 框，HSV 和安全区分析使用未画框的原始像素。旧版派端缺少无损画面接口时，自动改用原视频流。

点串口旁的「HSV 调色」，在左侧原图上拖框可看选区直方图，右键恢复整图统计。修改紫、红、蓝的上下限会更新本地画面，点击「应用到香橙派」后三组阈值一起更新到正在运行的 ROS 节点。节点重启时读取 `robot.yaml`。

更新香橙派工作区后，需要重新构建并重启预览服务，让预览脚本启动安全区节点。

## 构建

原型用 Qt 6.12.0（MinGW 64 位）编译。关闭客户端后，在 PowerShell 构建：

```powershell
./tools/laptop_server/build.ps1 -Qt D:/MySoftware/QT/6.12.0/mingw_64 -MinGW D:/MySoftware/QT/Tools/mingw1310_64
```

首次构建会校验并编译 OpenCV 4.5.4 的 core 和 imgproc，之后复用缓存。
