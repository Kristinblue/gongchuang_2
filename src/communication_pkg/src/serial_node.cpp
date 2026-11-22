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
  void command_callback(const Command &msg, int64_t max_pitch);
  bool send(uint8_t command, int16_t value);
  void trace(const SimpleProtocol::Frame &frame, bool tx);
  void close_serial();
  void read();

  // 成员按“先资源、后话题”的顺序声明；构造时按这个顺序初始化，析构时反过来释放。
  MyUART my_uart_;              // 串口本身，析构自动 close
  ProtocolParser parser_;       // 串口字节流 -> 完整帧
  bool moving_{false};          // 正在接收连续的前后或转向误差
  std::chrono::steady_clock::time_point last_command_{};  // 看门狗计时起点
  State state_;                 // 最后一条 robot/state
  rclcpp::Publisher<State>::SharedPtr state_pub_;
  rclcpp::Publisher<Feedback>::SharedPtr feedback_pub_;
  rclcpp::Publisher<SerialFrame>::SharedPtr frames_pub_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

SerialNode::SerialNode() : Node("serial_node") {
  // uart.device / uart.baudrate 和 robot.yaml 里的层级一致。
  const auto dev = declare_parameter("uart.device", "/dev/ttyS6");
  const auto baud = declare_parameter("uart.baudrate", 115200);
  // 相机俯角的软件限位，0 度水平，向下为正。
  const auto max_pitch = declare_parameter("max_pitch_deg", 80);
  if (max_pitch < 0 || max_pitch > 80) throw std::invalid_argument("max_pitch_deg 要在 0～80 之间");
  // 保留最后一条状态，感知和任务节点晚一点启动也可以收到当前队伍、串口状态等。
  state_pub_ = create_publisher<State>("robot/state", rclcpp::QoS(1).transient_local());
  feedback_pub_ = create_publisher<Feedback>("robot/feedback", 10);

  // 完整协议帧给客户端串口页看：tx=true 是发出去的，false 是收到的。
  frames_pub_ = create_publisher<SerialFrame>("robot/serial_frames", 100);

  command_sub_ = create_subscription<Command>("robot/command", 10, [this, max_pitch](const Command &msg) {
    command_callback(msg, max_pitch);
  });
  // callback：每 10 ms 调 read() 检查回包。非阻塞打开串口，没数据时立即返回。
  timer_ = create_wall_timer(10ms, [this] { read(); });
  // 先发停车，再发 00,0 查询队伍。
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
void SerialNode::command_callback(const Command &msg, int64_t max_pitch) {
  if (!state_.open) return;

  const auto cmd = msg.command;
  const auto value = msg.value;
  // 06 校准参数待定，先保留编号。队伍查询参数固定为 0，夹爪只能 0/1。
  // 不合法的命令直接丢弃，不去赌下位机能不能处理。
  bool valid = true;
  switch (cmd) {
    case Command::GET_FLAG:
      valid = value == 0;
      break;
    case Command::GRAB:
      valid = value == 0 || value == 1;
      break;
    case Command::CAMERA:
      valid = value >= 0 && value <= max_pitch;
      break;
    case Command::STRAIGHT:
    case Command::TURN:
    case Command::TURN_ANGLE:
    case Command::STRAIGHT_DIS:
    case Command::STOP:
      break;
    case Command::ANGLE_CALIB:
    default:
      valid = false;
      break;
  }
  if (!valid) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[SerialNode] 忽略非法命令 command=0x%02X value=%d",
                         static_cast<unsigned>(cmd), static_cast<int>(value));
    return;
  }
  if (!send(cmd, value)) {
    close_serial();
    return;
  }
  switch (cmd) {
    case Command::CAMERA:
      // 这里记录的是发给舵机的目标角度和时间。感知节点根据时间等相机画面稳定。
      if (state_.camera_known && state_.camera_deg == value) break;
      state_.camera_known = true;
      state_.camera_deg = value;
      state_.camera_changed = now();
      state_pub_->publish(state_);
      break;
    // 只有需要持续发送的运动误差使用短超时。一条相对转向可由 STM32 独立执行。
    case Command::STRAIGHT:
    case Command::TURN:
      moving_ = true;
      last_command_ = std::chrono::steady_clock::now();
      break;
    case Command::STOP:
    case Command::TURN_ANGLE:
    case Command::STRAIGHT_DIS:
      moving_ = false;
      break;
    default:
      break;
  }
}

// SimpleProtocol 打包，MyUART 只负责把字节写出去。
bool SerialNode::send(uint8_t command, int16_t value) {
  const auto frame = SimpleProtocol::pack(command, value);
  if (!my_uart_.write(frame.data(), frame.size())) return false;
  trace(frame, true);
  return true;
}

// 把完整协议帧发布到 robot/serial_frames，调试的客户端用的
// tx=true 是发出去的，false 是收到的。
void SerialNode::trace(const SimpleProtocol::Frame &frame, bool tx) {
  SerialFrame msg;
  msg.tx = tx;
  msg.data = frame;
  frames_pub_->publish(msg);
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
  const auto count = my_uart_.read(data, sizeof(data));//小于零表示失败
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
    feedback_pub_->publish(msg);
    switch (msg.command) {
      case Feedback::FLAG:
        // 回包 command=0 是队伍确认：0 红方，1 蓝方，记到 State 里。
        if (msg.value != 0 && msg.value != 1) break;
        state_.team = msg.value == 0 ? "red" : "blue";
        state_pub_->publish(state_);
        break;
      case Feedback::ANGLE:
        // 回包 command=1 是绝对角度 0~360；状态只在角度变化时更新。
        if (msg.value < 0 || msg.value > 360) break;
        if (state_.angle_known && state_.angle_deg == msg.value) break;
        state_.angle_deg = msg.value;
        state_.angle_known = true;
        state_pub_->publish(state_);
        break;
      default:
        break;
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
