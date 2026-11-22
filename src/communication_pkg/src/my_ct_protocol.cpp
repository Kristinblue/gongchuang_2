/*
hyf
这里是**协议应用层**的说明
直接移植之前的

*/

#include "communication_pkg/my_ct_protocol.hpp"

// 前 5 个字节相加，返回 uint8_t，只保留和的低 8 位。
static uint8_t checksum(const uint8_t *data) {
  return data[0] + data[1] + data[2] + data[3] + data[4];
}

SimpleProtocol::Frame SimpleProtocol::pack(uint8_t command, int16_t value) {
  // 先转成无符号，右移时才是逻辑移位，不会把符号位带进来。
  const uint16_t raw = static_cast<uint16_t>(value);
  // 0xAA 0x55 是固定的两个帧头；command 是命令位；value 低字节在前。
  Frame frame{0xaa, 0x55, command, uint8_t(raw), uint8_t(raw >> 8), 0};
  frame.back() = checksum(frame.data());
  return frame;
}

std::optional<SimpleProtocol::Message> SimpleProtocol::unpack(const uint8_t *data, std::size_t size) {
  // 三个检查缺一不可：长度必须是 6、两个帧头必须对、校验必须对。
  if (size != Frame{}.size() || data[0] != 0xaa || data[1] != 0x55 || data[5] != checksum(data)) {
    return std::nullopt;
  }
  // 高字节左移 8 位再和低字节合并，还原协议里的有符号数。
  const int raw = data[3] | (data[4] << 8);
  // raw 现在在 0~65535，大于等于 32768 的其实是负数补码，减 65536 还原。
  return Message{data[2], static_cast<int16_t>(raw < 32768 ? raw : raw - 65536)};
}

void ProtocolParser::feed(const uint8_t *data, std::size_t size) {
  // 每次 read() 的字节数与协议帧无关。先拼到缓存，凑够 6 字节才尝试解析。
  input_.insert(input_.end(), data, data + size);
}

std::optional<SimpleProtocol::Message> ProtocolParser::parse() {
  constexpr auto frame_size = SimpleProtocol::Frame{}.size();
  // 只有缓存里够 6 个字节才可能有一整帧。
  while (input_.size() >= frame_size) {
    if (auto message = SimpleProtocol::unpack(input_.data(), frame_size)) {
      // 解析成功：从缓存里删掉这 6 个字节，把消息返回给调用方。
      input_.erase(input_.begin(), input_.begin() + frame_size);
      return message;
    }
    // 帧头或校验对不上，就往后挪一个字节继续找。
    // 不能一次丢 6 个字节，因为真正的帧头可能就在这 6 个字节中间。
    input_.erase(input_.begin());
  }
  return std::nullopt;
}
