---
date created: 2026-11-14  12：55
date modified: 2026-11-14  21：10
---
# 项目说明

2027 年工创赛智能救援赛道。本仓只有上位机。负责机械、视觉和导航；板卡是香橙派 5 Plus。

- Windows 本地开发，WSL2 Ubuntu 22.04 / ROS 2 Humble 编译。派上 Ubuntu Jammy / ARM64 / Python 3.10。
- 采图、串口和 HTTP 用 C++17，推理、感知和任务复用 Python。HTTP 预览默认 640×360、10 fps。默认预览香橙派摄像头，未接时显示「无视频输入」。默认不启动任务和串口节点。
- 本地编辑前先拉取，冲突及时反馈。使用 `ssh pi` / `ssh pi-relay`，或以 orangepi 身份用 `ssh pio` / `ssh pio-relay`。
- 上位机 GitHub 仓库为 [Kristinblue/gongchuang_2](https://github.com/Kristinblue/gongchuang_2)。Windows 工作目录为 `D:\my_project\gongchuang_ros2`，派上工程在 `/home/orangepi/my_ws/gongchuang_ros2`。原仓和派的 `my_configs` 独立。
- 不改下位机协议。Python docstring 用 Google 风格，调试信息用 [类名] 前缀。
- 原始不完整的工程在D:\my_project\gongchuang\my_project


## 我的说明
- 我是ROS2和c++初学者
- c++我基本只会些基础类，智能指针，lambda，RAII思想，vector，和异常处理，但是有c语言基础。我特别喜欢类。

## 我的要求
续。
- md开头有`hyf-docs`的是专门我自己写的，agent可看但勿动
- Git 提交说明使用中文。
- `tools/logs/` 记时间、改动和关键命令。模型权重放在 `assets/models/`。模型、账号私钥、本地配置和编译输出不入 Git。里面专门存放做一些事情的记录，我通常会让你把日志写进去，要记录好时间，每一天的放一个文件夹里。如果是md就稍微整理一下，这种主要是给我看的，记录做过的事情，简单说明，以及那些命令，因为你只是替我做事了但我仍然需要学习这些命令；如果输入输出太多就放txt文件里
- 不要改我的注释，那都是我的心血
- 保留已有变量名、类和方法名，以及 YAML 参数名和层级。需要改名时先说明原因。
- 电脑客户端还没完善，不要弄压缩包，就是exe就行，因为我一直要改

## 进度
- 已经从原来就残缺的python工程移植过来了
- 当前动作只迁移打散 → GoTo →
- 停车，抓取和投放后续补。
