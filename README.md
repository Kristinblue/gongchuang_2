---
date created: 2026-11-14  11：56
date modified: 2026-11-15  14：42
---
# 项目说明
这是 2027 工创赛的上位机代码仓库 [Kristinblue/gongchuang_2](https://github.com/Kristinblue/gongchuang_2)，从原工程转过来的，现在仍有很多没完善。
服务运行在香橙派上，默认预览车上的摄像头。

## 一、环境说明
- 香橙派系统：Ubuntu Jammy ARM64 / ROS 2 Humble
- 电脑：Windows + WSL2 Ubuntu 22.04 / ROS 2 Humble
- 主要语言：C++ 和 Python

## 二、整体文件夹说明
这是部分树形结构
```text
.
├─ src/
│  ├─ action_pkg/          打散、GoTo、抓取和任务流程
│  ├─ communication_pkg/   STM32 串口、协议和反馈
│  ├─ desktop_server_pkg/  HTTP 画面、状态和设置
│  ├─ robot_interfaces/    ROS 2 自定义消息
│  ├─ robot_launch/        启动文件和 robot.yaml
│  ├─ vision_pkg/          C++ 采图，Python 推理和感知
│  ├─ tests_pkg/           单项测试，目前是靠近
│  └─ README.md            功能包和启动说明
├─ tools/
│  ├─ experiments/        算法实验
│  ├─ laptop_server/      Windows Qt 客户端原型，待审批
│  ├─ legacy/             迁移前的 Python 源码
│  ├─ logs/               开发记录和本地运行日志
│  ├─ build.sh            编译 ROS 2 工作区
│  ├─ run.sh              启动 ROS 2 节点
│  ├─ setup_pi.sh         准备香橙派的 RKNN 环境和模型
│  └─ verify_preview.py   验证摄像头和 HTTP 预览
├─ assets/                 项目资源
│  └─ models/              RKNN / ONNX 权重，本地存放
├─ build/                  本地编译中间文件
├─ install/                本地安装结果
├─ README.md
└─ AGENTS.md
```
### 源码
七个功能包，具体见[源码里的 README](src/README.md)。

### 一些工具
用来辅助调试用的，
#### 电脑客户端
qt框架做的，在这里面可以看到画面，更改一些值，显示代码的执行阶段，其他功能后续再加

Windows 客户端位于 [tools/laptop_server](tools/laptop_server/README.md)。
当前是验证用原型。正式功能和分发等我审批后再继续。

保底的下位机加单独香橙派就能完成整个任务，电脑能控制是最好的，

#### 日志
在 `tools/logs/` 按照日期记录改动等。
#### 实验
对比一些方法，得到更高效率
- 找安全区的方法（未完善）

## 三、文档说明
- 开头标注`hyf-docs`表明这是完全由hyf即主要作者写的，说明文档、思考、记录。
- 没标的一般是我和agent共同写的。
- 整个文件夹其实也是一个obsidian仓库（一个笔记本），md里面如果有**图片**，会被放到同目录下的assets

## 四、使用说明

在香橙派的仓库根目录执行 `bash tools/build.sh` 和 `bash tools/run.sh`，在 Windows 通过 SSH 转发打开 [预览](http://localhost:18080)。依赖安装、USB 摄像头设置和香橙派启动见[源码说明](src/README.md)。
