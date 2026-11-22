/*
对外的串口节点，外部不用在意协议帧格式
*/
#include "communication_pkg/my_ct_protocol.hpp"
#include "communication_pkg/my_uart.hpp"

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/command.hpp>
#include <robot_interfaces/msg/feedback.hpp>
#include <robot_interfaces/msg/state.hpp>
#include <robot_interfaces/msg/serial_frame.hpp>

using namespace std::chrono_literals;
// 类型别名
using Command = robot_interfaces::msg::Command;
using Feedback = robot_interfaces::msg::Feedback;
using State = robot_interfaces::msg::State;
using SerialFrame = robot_interfaces::msg::SerialFrame;

// ROS 和下位机之间的桥。发动作走 robot/command，回包走 robot/feedback。
//
// 数据流：
//   任务/测试 --/robot/command--> SerialNode --my_uart--> STM32
//   STM32 --my_uart--> SerialNode --/robot/feedback--> 任务节点
//                                  --/robot/serial_frames--> 客户端串口页
//
// 这个类自己不做运动决策，只做三件事：
//   1. 把 Command 按协议打包发出去（参数检查、限位）；
//   2. 定时读串口、拆帧，把回包转成 Feedback 和 State；
//   3. 连续的前后或转向误差断流超过 750 ms 时发停车。
class SerialNode : public rclcpp::Node {
public:
  SerialNode();
  ~SerialNode() override;

private:
  void command_callback(const Command &msg, int64_t max_pitch, bool allow_calib);
  bool send(uint8_t command, int16_t value);
  void trace(const SimpleProtocol::Frame &frame, bool tx);
  void close_serial();
  void read();

  // 成员按“先资源、后话题”的顺序声明；构造时按这个顺序初始化，析构时反过来释放。
  MyUART my_uart_;              // 串口本身，析构自动 close
  ProtocolParser parser_;       // 串口字节流 -> 完整帧
  bool simulated_{false};       // true 时不打开真实串口
  bool moving_{false};          // 正在接收连续的前后或转向误差
  std::chrono::steady_clock::time_point last_command_{};  // 看门狗计时起点
  State state_;                 // 最后一条 robot/state
  rclcpp::Publisher<State>::SharedPtr state_pub_;
  rclcpp::Publisher<Feedback>::SharedPtr feedback_;
  rclcpp::Publisher<SerialFrame>::SharedPtr frames_;
  rclcpp::Subscription<Command>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

SerialNode::SerialNode() : Node("serial_node") {
  // simulate=true 时不打开真实串口，只把打包结果发到 serial_frames，
  // 用于没接单片机时验证话题和字节帧。
  simulated_ = declare_parameter("simulate", false);
  // uart.device / uart.baudrate 和 robot.yaml 里的层级一致。
  const auto dev = declare_parameter("uart.device", "/dev/ttyS6");
  const bool allow_calib = declare_parameter("uart.allow_reserved_calib", simulated_);
  const auto baud = declare_parameter("uart.baudrate", 115200);
  // 相机俯角的软件限位，0 度水平，向下为正。
  const auto max_pitch = declare_parameter("max_pitch_deg", 80);
  if (max_pitch < 0 || max_pitch > 80) throw std::invalid_argument("max_pitch_deg 要在 0～80 之间");
  // 保留最后一条状态，感知和任务节点晚一点启动也可以收到当前队伍、串口状态等。
  // transient_local 相当于“持久化最后一条”，后来的订阅者也能收到。
  state_pub_ = create_publisher<State>("robot/state", rclcpp::QoS(1).transient_local());
  feedback_ = create_publisher<Feedback>("robot/feedback", 10);
  // 完整协议帧给客户端串口页看：tx=true 是发出去的，false 是收到的。
  frames_ = create_publisher<SerialFrame>("robot/serial_frames", 100);
  // callback：收到 MyCMD 的命令，交给下面的 command_callback()。
  sub_ = create_subscription<Command>("robot/command", 10, [this, max_pitch, allow_calib](const Command &msg) {
    command_callback(msg, max_pitch, allow_calib);
  });
  if (simulated_) {
    RCLCPP_INFO(get_logger(), "[SerialNode] 模拟模式，记录发送帧");
    return;
  }
  // callback：每 10 ms 调 read() 检查回包。非阻塞打开串口，没数据时立即返回。
  timer_ = create_wall_timer(10ms, [this] { read(); });
  // 先发 08,0 停车，再发 00,0 查询队伍。
  if (!my_uart_.open(dev, baud) || !send(Command::STOP, 0) || !send(Command::GET_FLAG, 0)) {
    close_serial();
    throw std::runtime_error("无法初始化串口 " + dev);
  }
  state_.open = true;
  state_pub_->publish(state_);
  RCLCPP_INFO(get_logger(), "[SerialNode] %s %lld", dev.c_str(), static_cast<long long>(baud));
}

// 析构时如果串口还开着，尽量发一帧停车再关。
SerialNode::~SerialNode() {
  if (my_uart_.is_open()) send(Command::STOP, 0);
}

// 收到 MyCMD 发布的动作后，检查参数，再按协议写串口。
void SerialNode::command_callback(const Command &msg, int64_t max_pitch, bool allow_calib) {
  // 串口没打开、又不是模拟模式时，任何命令都不发。
  if (!state_.open && !simulated_) return;

  const auto cmd = msg.command;
  const auto value = msg.value;
  // 06 校准参数待定，先保留编号。队伍查询参数固定为 0，夹爪只能 0/1。
  // 不合法的命令直接丢弃，不去赌下位机能不能处理。
  if (cmd > Command::STOP || (!allow_calib && cmd == Command::ANGLE_CALIB) ||
      (cmd == Command::GET_FLAG && value != 0) ||
      (cmd == Command::GRAB && (value < 0 || value > 1)) ||
      (cmd == Command::CAMERA && (value < 0 || value > max_pitch))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[SerialNode] 忽略非法命令 command=0x%02X value=%d",
                         static_cast<unsigned>(cmd), static_cast<int>(value));
    return;
  }
  // 开始一次新的相对转向前，把积压的角度回包清掉。
  if (!simulated_ && cmd == Command::TURN_ANGLE) {
    if (!my_uart_.clear_input()) {
      close_serial();
      return;
    }
    parser_.clear();
  }
  if (!send(cmd, value)) {
    close_serial();
    return;
  }
  // 模拟模式：不写串口，直接打包并发布到 serial_frames，给客户端看。
  if (simulated_) return;
  // 这里记录的是发给舵机的目标角度和时间。感知节点根据时间等相机画面稳定。
  if (cmd == Command::CAMERA && (!state_.camera_known || state_.camera_deg != value)) {
    state_.camera_known = true;
    state_.camera_deg = value;
    state_.camera_changed = now();
    state_pub_->publish(state_);
  }
  // 只有需要持续发送的运动误差使用短超时。一条相对转向可由 STM32 独立执行。
  if (cmd == Command::STRAIGHT || cmd == Command::TURN) {
    moving_ = true;
    last_command_ = std::chrono::steady_clock::now();
  } else if (cmd == Command::STOP || cmd == Command::TURN_ANGLE ||
             cmd == Command::STRAIGHT_DIS) {
    moving_ = false;
  }
}

// SimpleProtocol 打包，MyUART 只负责把字节写出去。
bool SerialNode::send(uint8_t command, int16_t value) {
  const auto frame = SimpleProtocol::pack(command, value);
  // 模拟模式不碰硬件；真实模式写失败说明串口断了。
  if (!simulated_ && !my_uart_.write(frame.data(), frame.size())) return false;
  // 不管有没有真的写串口，都记录一帧 TX 给客户端。
  trace(frame, true);
  return true;
}

// 把完整协议帧发布到 robot/serial_frames。
// tx=true 是发出去的，false 是收到的；simulated 标记是不是模拟模式。
void SerialNode::trace(const SimpleProtocol::Frame &frame, bool tx) {
  SerialFrame msg;
  msg.tx = tx;
  msg.simulated = simulated_;
  msg.data = frame;
  frames_->publish(msg);
}

// 关闭串口：先尽力发一帧停车，再发布 open=false 给任务节点和客户端。
void SerialNode::close_serial() {
  if (my_uart_.is_open() && state_.open) send(Command::STOP, 0);
  my_uart_.close();
  state_.open = false;
  state_pub_->publish(state_);
}

// 每 10 ms 调一次：先看看门狗，再把串口里能读到的字节全部读出来拆帧。
void SerialNode::read() {
  if (!my_uart_.is_open()) return;
  // 连续运动误差超过 750 ms 没更新时，由串口节点发停车。
  if (moving_ && std::chrono::steady_clock::now() - last_command_ > 750ms) {
    if (!send(Command::STOP, 0)) { close_serial(); return; }
    moving_ = false;
  }
  // 一次最多读 256 字节，够 42 帧；剩下的下一轮再读。
  uint8_t data[256];
  const auto count = my_uart_.read(data, sizeof(data));
  if (count < 0) { close_serial(); return; }
  if (count == 0) return;
  parser_.feed(data, count);
  // while 循环把缓存里所有完整帧都取出来，一帧也不落下。
  while (auto packet = parser_.parse()) {
    // 有效帧固定为六字节，按解析值重组可得到逐字节相同的回包。
    trace(SimpleProtocol::pack(packet->command, packet->value), false);
    Feedback msg;
    msg.command = packet->command;
    msg.value = packet->value;
    feedback_->publish(msg);
    // 回包 command=0 是队伍确认：0 红方，1 蓝方，记到 State 里。
    if (msg.command == Feedback::FLAG && (msg.value == 0 || msg.value == 1)) {
      state_.team = msg.value == 0 ? "red" : "blue";
      state_pub_->publish(state_);
    } else if (msg.command == Feedback::ANGLE && msg.value >= 0 && msg.value <= 360 &&
               (!state_.angle_known || state_.angle_deg != msg.value)) {
      // 回包 command=1 是绝对角度 0~360；状态只在角度变化时更新。
      state_.angle_deg = msg.value;
      state_.angle_known = true;
      state_pub_->publish(state_);
    }
  }
}

int main(int argc, char **argv) {
  // rclcpp::init 解析 ROS 参数（--ros-args ...），之后才能创建节点。
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    // 构造失败会抛异常（比如串口打不开），在这里统一打印并返回 1。
    rclcpp::spin(std::make_shared<SerialNode>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("serial_node"), "[SerialNode] %s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
