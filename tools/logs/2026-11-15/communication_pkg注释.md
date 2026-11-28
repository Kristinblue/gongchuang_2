# 2026-11-15 · communication_pkg 补注释

时间：14:20–14:45

目标：给 `communication_pkg` 加注释，方便第一次读 ROS 2 + 串口代码时对照。
只加注释，没有改任何代码逻辑、变量名、话题名和参数名。

## 分层

```text
任务 / 靠近测试 / 客户端
        │  /robot/command   (Command: code + value)
        ▼
SerialNode（serial_node.cpp）        ← ROS 这一侧
        │  检查参数、看门狗、转成 State/Feedback
        ▼
SimpleProtocol（my_ct_protocol.cpp） ← 只负责 6 字节协议
        │  AA 55 code lo hi checksum
        ▼
MyUART（my_uart.cpp）                ← 只负责 Linux 串口
        │  open / read / write / tcflush
        ▼
STM32
```

## 每个文件

- `my_uart.hpp/.cpp`：串口类。`fd_` 是 Linux 的文件描述符，`-1` 表示没打开；
  析构自动 `close()`（RAII）。`open()` 用 `termios` 配成原始字节模式，
  `TIOCEXCL` 防止别的进程抢同一个串口；`write()` 处理一次写不完、
  `EAGAIN`/`EINTR` 重试；`read()` 非阻塞，没数据返回 0。
- `my_ct_protocol.hpp/.cpp`：协议类。`pack()` 打包并算校验；
  `unpack()` 检查长度、帧头、校验，把两个字节还原成 int16（小端、补码）。
  `ProtocolParser` 缓存串口字节流，`parse()` 每次取一帧；对不上就丢一个字节
  重新找帧头，所以中间混入噪声也能重新对齐。
- `serial_node.cpp`：ROS 节点。构造函数声明参数、建发布/订阅；
  `command_callback()` 检查 code/value 后 `send()`；
  `read()` 每 10 ms 跑一次，先看看门狗（750 ms 没命令就补一帧 06），
  再读串口拆帧，发 `feedback`、更新 `state`；
  `trace()` 把每帧原始字节发到 `robot/serial_frames`，客户端串口页就是看这个。
- `my_cmd.py`：任务侧的动作接口。只把动作翻译成命令位，通过
  `/robot/command` 发出去；真正的打包和串口在 C++ 节点里。
  `heartbeat()` 发 code=0，不产生串口帧，只刷新看门狗时间。
- `CMakeLists.txt`：加了几行注释说明每个 `find_package` / `install` 的作用。

## 读代码时容易卡住的点

- `transient_local`：QoS 的一种，最后一条消息会保留，晚启动的节点也能收到
  `/robot/state`。
- `std::optional`：表示“可能没有值”，`unpack`/`parse` 用它表达解析失败，
  比返回特殊值清楚。
- 校验和只加前 5 个字节、只留低 8 位，所以 `checksum()` 返回 `uint8_t`。
- int16 负数在协议里是补码：-120 存成 `0xFF88`，字节序是低字节在前。
- 串口是独占资源：两个节点同时打开同一个 `/dev/ttyS6` 会失败，
  所以只有 `serial_node` 碰串口，其它节点发话题。

## 命令

```bash
# WSL 编译
bash tools/build.sh

# 派上只编这一个包（--log-base 要放在 build 前面）
colcon --log-base tools/logs/colcon build --base-paths src \
  --packages-select communication_pkg --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 不接单片机，只看打包出来的帧
ros2 run communication_pkg serial_node --ros-args -p simulate:=true
ros2 topic pub --once /robot/command robot_interfaces/msg/Command '{code: 1, value: -120}'
ros2 topic echo /robot/serial_frames
```

## 结果

- WSL 7 个包编译通过；派上 `communication_pkg` 单独重编通过；
- 只改注释，行为不变，派上正在跑的服务没有重启。
