# 接入 serial_driver

- 用 Humble 的 `serial_driver` 库打开、配置和读写串口。`MyUART` 保留原来的调用方法。Linux 设备句柄用来独占串口、检查读写就绪和清空接收缓存。
- WSL 安装依赖 `sudo apt install ros-humble-serial-driver ros-humble-asio-cmake-module libasio-dev`。编译时用 `colcon build --base-paths src --packages-up-to tests_pkg --symlink-install`。
- 伪串口验证了字节收发、独占打开、坏校验原样显示、分段帧解析、断流停车和串口断开。再用 `bash tools/run.sh serial` 验证 STM32 模拟器的正常启动路径。
- 香橙派的 SSH 有线、无线和中转连接均未成功，本次修改还未部署到板子。
