/*
hyf移植以前的
这里是控制串口收发的，以及资源管理


*/
#include "communication_pkg/my_uart.hpp"

#include <algorithm>
#include <cerrno>
#include <exception>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

MyUART::~MyUART() {
  close();
}

bool MyUART::open(const std::string &device, int64_t baudrate) {
  close();
  if (baudrate <= 0 || baudrate > std::numeric_limits<uint32_t>::max()) return false;

  // O_NOCTTY 不把串口当成进程的控制终端。先拿锁，避免两个节点同时初始化。
  control_fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (control_fd_ < 0 || ::flock(control_fd_, LOCK_EX | LOCK_NB) != 0) {
    close();
    return false;
  }
  try {
    using namespace drivers::serial_driver;
    const SerialPortConfig config(static_cast<uint32_t>(baudrate), FlowControl::NONE, Parity::NONE, StopBits::ONE);
    port_ = std::make_unique<SerialPort>(io_, device, config);
    port_->open();
  } catch (const std::exception &) {
    close();
    return false;
  }
  // TIOCEXCL 让别的进程不能再打开同一个串口，防止两个节点抢串口。
  if (::ioctl(control_fd_, TIOCEXCL) != 0) {
    close();
    return false;
  }
  return true;
}

bool MyUART::is_open() const {
  return port_ && port_->is_open();
}

bool MyUART::write(const uint8_t *data, std::size_t size) {
  if (!is_open()) return false;
  try {
    // 一次发送可能只写出一部分。等待可写后，把剩余字节交给库。
    for (std::size_t sent = 0; sent < size;) {
      pollfd output{control_fd_, POLLOUT, 0};
      if (::poll(&output, 1, 20) <= 0 || !(output.revents & POLLOUT) ||
          (output.revents & (POLLERR | POLLHUP | POLLNVAL))) return false;
      const auto count = port_->send(std::vector<uint8_t>(data + sent, data + size));
      if (count == 0) return false;
      sent += count;
    }
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

ssize_t MyUART::read(uint8_t *data, std::size_t size) {
  if (!is_open()) return -1;
  if (size == 0) return 0;
  // 库的 receive() 会等待数据，ROS 定时器只在确认有字节可读时调用它。
  pollfd input{control_fd_, POLLIN, 0};
  const int ready = ::poll(&input, 1, 0);
  if (ready < 0) return errno == EINTR ? 0 : -1;
  if (input.revents & (POLLERR | POLLHUP | POLLNVAL)) return -1;
  if (!(input.revents & POLLIN)) return 0;

  int available = 0;
  if (::ioctl(control_fd_, FIONREAD, &available) != 0) return -1;
  if (available <= 0) return 0;
  try {
    std::vector<uint8_t> bytes(std::min(size, static_cast<std::size_t>(available)));
    const auto count = port_->receive(bytes);
    std::copy_n(bytes.begin(), count, data);
    return static_cast<ssize_t>(count);
  } catch (const std::exception &) {
    return -1;
  }
}

void MyUART::close() {
  port_.reset();
  if (control_fd_ >= 0) ::close(control_fd_);
  control_fd_ = -1;
}
