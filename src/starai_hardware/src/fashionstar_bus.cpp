#include "starai_hardware/fashionstar_bus.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace starai_hardware
{
namespace
{
bool baud_constant(int baud, speed_t & out)
{
  switch (baud) {
    case 9600: out = B9600; return true;
    case 19200: out = B19200; return true;
    case 38400: out = B38400; return true;
    case 57600: out = B57600; return true;
    case 115200: out = B115200; return true;
    case 230400: out = B230400; return true;
    case 460800: out = B460800; return true;
    case 500000: out = B500000; return true;
    case 576000: out = B576000; return true;
    case 921600: out = B921600; return true;
    case 1000000: out = B1000000; return true;
    case 1500000: out = B1500000; return true;
    case 2000000: out = B2000000; return true;
    default: return false;
  }
}
}  // namespace

FashionStarBus::~FashionStarBus() { close(); }

bool FashionStarBus::open(const std::string & port, int baudrate, std::string & error)
{
  close();
  open_warnings_.clear();
  speed_t speed;
  if (!baud_constant(baudrate, speed)) {
    error = "unsupported baud rate " + std::to_string(baudrate);
    return false;
  }
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    error = "cannot open " + port + ": " + std::strerror(errno);
    return false;
  }
  // Refuse to share the bus with another process (e.g. the legacy Python driver).
  if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
    error = port + " is in use by another process (flock: " + std::strerror(errno) + ")";
    close();
    return false;
  }
  ::ioctl(fd_, TIOCEXCL);

  termios tio{};
  if (::tcgetattr(fd_, &tio) != 0) {
    error = "tcgetattr failed on " + port + ": " + std::strerror(errno);
    close();
    return false;
  }
  ::cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  ::cfsetispeed(&tio, speed);
  ::cfsetospeed(&tio, speed);
  if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
    error = "tcsetattr failed on " + port + ": " + std::strerror(errno);
    close();
    return false;
  }

  // USB serial adapters buffer up to 16 ms by default; ask for low latency.
  serial_struct ser{};
  if (::ioctl(fd_, TIOCGSERIAL, &ser) == 0) {
    ser.flags |= ASYNC_LOW_LATENCY;
    if (::ioctl(fd_, TIOCSSERIAL, &ser) != 0) {
      open_warnings_.push_back(
        "could not enable ASYNC_LOW_LATENCY (" + std::string(std::strerror(errno)) +
        "); for FTDI adapters set /sys/bus/usb-serial/devices/<dev>/latency_timer to 1");
    }
  } else {
    open_warnings_.push_back("TIOCGSERIAL unsupported; low-latency mode not set");
  }

  ::tcflush(fd_, TCIOFLUSH);
  port_ = port;
  tx_.reserve(512);
  rx_.resize(512);
  parser_.reset();
  return true;
}

void FashionStarBus::close()
{
  if (fd_ >= 0) {
    ::tcdrain(fd_);
    ::flock(fd_, LOCK_UN);
    ::close(fd_);
    fd_ = -1;
  }
}

bool FashionStarBus::write_frame()
{
  if (fd_ < 0) {
    return false;
  }
  size_t off = 0;
  const auto deadline = Clock::now() + std::chrono::milliseconds(20);
  while (off < tx_.size()) {
    const ssize_t n = ::write(fd_, tx_.data() + off, tx_.size() - off);
    if (n > 0) {
      off += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EINTR) {
      return false;
    }
    if (Clock::now() > deadline) {
      return false;
    }
    pollfd pfd{fd_, POLLOUT, 0};
    ::poll(&pfd, 1, 2);
  }
  return true;
}

void FashionStarBus::drain_input()
{
  // Drop stale bytes (late replies from a timed-out request, echoes) before a new query.
  while (::read(fd_, rx_.data(), rx_.size()) > 0) {
  }
  parser_.reset();
}

bool FashionStarBus::wait_readable(Clock::time_point deadline)
{
  const auto now = Clock::now();
  if (now >= deadline) {
    return false;
  }
  const auto remaining =
    std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();
  // poll() has millisecond resolution; round up so we never spin.
  const int timeout_ms = static_cast<int>((remaining + 999) / 1000);
  pollfd pfd{fd_, POLLIN, 0};
  const int r = ::poll(&pfd, 1, timeout_ms);
  if (r < 0 && errno == EINTR) {
    return true;
  }
  return r > 0 && (pfd.revents & POLLIN);
}

bool FashionStarBus::ping(uint8_t id, std::chrono::microseconds timeout)
{
  if (fd_ < 0) {
    return false;
  }
  drain_input();
  tx_.clear();
  fsus::encode_ping(id, tx_);
  if (!write_frame()) {
    return false;
  }
  const auto deadline = Clock::now() + timeout;
  while (wait_readable(deadline)) {
    const ssize_t n = ::read(fd_, rx_.data(), rx_.size());
    for (ssize_t i = 0; i < n; ++i) {
      if (parser_.push(rx_[i])) {
        const auto & p = parser_.packet();
        if (p.cmd == fsus::kPing && p.len >= 1 && p.payload[0] == id) {
          return true;
        }
      }
    }
  }
  return false;
}

size_t FashionStarBus::read_monitor(
  const std::vector<uint8_t> & ids, std::vector<fsus::MonitorData> & out,
  std::vector<uint8_t> & valid, std::chrono::microseconds timeout)
{
  std::fill(valid.begin(), valid.end(), 0);
  if (fd_ < 0 || ids.empty()) {
    return 0;
  }
  drain_input();
  tx_.clear();
  fsus::encode_sync_monitor(ids.data(), ids.size(), tx_);
  if (!write_frame()) {
    return 0;
  }

  size_t received = 0;
  fsus::MonitorData m;
  const auto deadline = Clock::now() + timeout;
  while (received < ids.size() && wait_readable(deadline)) {
    const ssize_t n = ::read(fd_, rx_.data(), rx_.size());
    if (n < 0 && errno != EAGAIN && errno != EINTR) {
      break;  // device unplugged etc.
    }
    for (ssize_t i = 0; i < n; ++i) {
      if (!parser_.push(rx_[i]) || !fsus::decode_monitor(parser_.packet(), m)) {
        continue;
      }
      for (size_t k = 0; k < ids.size(); ++k) {
        if (ids[k] == m.id && !valid[k]) {
          out[k] = m;
          valid[k] = 1;
          ++received;
          break;
        }
      }
    }
  }
  return received;
}

bool FashionStarBus::write_sync_velocity(const std::vector<fsus::VelocityTarget> & targets)
{
  if (targets.empty()) {
    return true;
  }
  tx_.clear();
  return fsus::encode_sync_velocity(targets.data(), targets.size(), tx_) && write_frame();
}

bool FashionStarBus::stop(uint8_t id, fsus::StopMode mode, uint16_t power_mw)
{
  tx_.clear();
  return fsus::encode_stop(id, mode, power_mw, tx_) && write_frame();
}

bool FashionStarBus::reset_multi_turn(uint8_t id)
{
  tx_.clear();
  return fsus::encode_reset_multi_turn(id, tx_) && write_frame();
}

}  // namespace starai_hardware
