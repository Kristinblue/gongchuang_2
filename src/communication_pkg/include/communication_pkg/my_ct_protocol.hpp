#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

// SimpleProtocol：下位机（STM32）串口协议的打包和拆包。
//
// 一帧固定 6 个字节：
//   [0] 0xAA      帧头 1
//   [1] 0x55      帧头 2
//   [2] command      命令位，1~6，含义见 my_cmd.py
//   [3] value 低  数值低 8 位
//   [4] value 高  数值高 8 位
//   [5] checksum  前 5 个字节相加，只留低 8 位
//
// value 是 int16_t，按小端（低字节在前）放；负数的补码表示由强转自动完成。
class SimpleProtocol {
public:
  // 下位机协议一帧固定 6 字节：AA 55 | 命令 | 数值低位 高位 | 校验。
  using Frame = std::array<uint8_t, 6>;

  // 拆出来的一条消息：命令位 + 有符号数值。
  struct Message {
    uint8_t command;
    int16_t value;
  };

  // 把一条命令打包成 6 字节，会自动算校验。
  static Frame pack(uint8_t command, int16_t value);
  // 把 6 字节还原成一条消息。
  // 长度不对、帧头不对、校验不对都返回 std::nullopt，调用方用 if 判断即可。
  static std::optional<Message> unpack(const uint8_t *data, std::size_t size);
};

// ProtocolParser：把串口读到的“字节流”重新拼成一条条完整帧。
//
// 串口 read() 一次读到的长度和帧边界没有关系，可能半帧、也可能两帧半，
// 所以先缓存，再循环尝试解析。帧头/校验对不上时会丢掉一个字节继续找，
// 这样即使中间混入了噪声也能重新对齐。
class ProtocolParser {
public:
  // 把刚从串口读到的一段字节追加到缓存。
  void feed(const uint8_t *data, std::size_t size);
  // 从缓存里取出一条完整帧；缓存里没有完整帧时返回 std::nullopt。
  // 通常在 while (auto message = parser.parse()) 里循环调用，一次取完所有帧。
  std::optional<SimpleProtocol::Message> parse();

private:
  std::vector<uint8_t> input_;
};
