#include "dm_imu_cpp/dm_serial.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <optional>
#include <stdexcept>
#include <thread>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace dm_imu_cpp {
namespace {
constexpr uint8_t kHdr0 = 0x55;
constexpr uint8_t kHdr1 = 0xAA;
constexpr uint8_t kTail = 0x0A;
constexpr size_t kFrameLen = 19;
constexpr bool kSkipHdrInCrc = false;

speed_t baud_to_speed(int baudrate) {
  switch (baudrate) {
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
#ifdef B921600
    case 921600: return B921600;
#endif
#ifdef B1000000
    case 1000000: return B1000000;
#endif
    default: return 0;
  }
}

std::string errno_string(const char* prefix) {
  std::string msg(prefix);
  msg.append(": ");
  msg.append(std::strerror(errno));
  return msg;
}

bool valid_rid(uint8_t rid) {
  return rid == 0x01 || rid == 0x02 || rid == 0x03;
}

float le_bytes_to_float(const uint8_t* data) {
  float out = 0.0f;
  std::memcpy(&out, data, sizeof(float));
  return out;
}

}  // namespace

DMSerial::DMSerial(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate) {
  if (!open_port()) {
    throw std::runtime_error(last_error());
  }
}

DMSerial::~DMSerial() {
  stop_reader();
  close_port();
}

bool DMSerial::start_reader(double read_sleep) {
  if (reader_.joinable()) {
    read_sleep_ = read_sleep;
    return true;
  }
  if (fd_ < 0 && !open_port()) {
    return false;
  }
  stop_ = false;
  read_sleep_ = read_sleep;
  reader_ = std::thread(&DMSerial::reader_loop, this);
  return true;
}

void DMSerial::stop_reader() {
  stop_ = true;
  if (reader_.joinable()) {
    reader_.join();
  }
}

std::tuple<std::optional<Packet>, double, uint64_t> DMSerial::get_latest() {
  std::lock_guard<std::mutex> lock(latest_mutex_);
  return std::make_tuple(latest_pkt_, latest_ts_, latest_count_);
}

DebugInfo DMSerial::get_debug() {
  DebugInfo info;
  info.buf_len = buf_.size();
  info.last_read_len = last_read_.size();
  info.last_read_hex = to_hex(last_read_);
  info.last_read_ts = last_read_ts_;
  if (fd_ >= 0) {
    int n = 0;
    if (ioctl(fd_, FIONREAD, &n) == 0) {
      info.in_waiting = n;
    }
  }
  info.cnt_ok = cnt_ok_;
  info.cnt_crc = cnt_crc_;
  info.cnt_short = cnt_short_;
  info.cnt_nohdr = cnt_nohdr_;
  info.last_error = last_error();
  return info;
}

std::string DMSerial::last_error() const {
  std::lock_guard<std::mutex> lock(err_mutex_);
  return last_error_;
}

bool DMSerial::open_port() {
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::lock_guard<std::mutex> lock(err_mutex_);
    last_error_ = errno_string("open failed");
    return false;
  }

  termios tty;
  if (tcgetattr(fd_, &tty) != 0) {
    std::lock_guard<std::mutex> lock(err_mutex_);
    last_error_ = errno_string("tcgetattr failed");
    close_port();
    return false;
  }

  cfmakeraw(&tty);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  speed_t speed = baud_to_speed(baudrate_);
  if (speed == 0) {
    std::lock_guard<std::mutex> lock(err_mutex_);
    last_error_ = "Unsupported baudrate";
    close_port();
    return false;
  }

  if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
    std::lock_guard<std::mutex> lock(err_mutex_);
    last_error_ = errno_string("cfset speed failed");
    close_port();
    return false;
  }

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::lock_guard<std::mutex> lock(err_mutex_);
    last_error_ = errno_string("tcsetattr failed");
    close_port();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

void DMSerial::close_port() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void DMSerial::reader_loop() {
  while (!stop_) {
    auto pkt = read_into_buf(std::nullopt);
    if (pkt > 0) {
      auto frames = parse_all();
      if (!frames.empty()) {
        std::lock_guard<std::mutex> lock(latest_mutex_);
        latest_pkt_ = frames.back();
        latest_ts_ = std::chrono::duration<double>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
        latest_count_ += 1;
      }
    }
    if (read_sleep_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(read_sleep_));
    }
  }
}

size_t DMSerial::read_into_buf(std::optional<size_t> max_bytes) {
  if (fd_ < 0) {
    return 0;
  }
  int n = 0;
  if (ioctl(fd_, FIONREAD, &n) != 0) {
    std::lock_guard<std::mutex> lock(err_mutex_);
    last_error_ = errno_string("ioctl FIONREAD failed");
    return 0;
  }
  if (max_bytes && n > static_cast<int>(*max_bytes)) {
    n = static_cast<int>(*max_bytes);
  }
  if (n <= 0) {
    return 0;
  }

  std::vector<uint8_t> tmp(static_cast<size_t>(n));
  ssize_t r = ::read(fd_, tmp.data(), tmp.size());
  if (r <= 0) {
    return 0;
  }

  tmp.resize(static_cast<size_t>(r));
  buf_.insert(buf_.end(), tmp.begin(), tmp.end());
  last_read_ = tmp;
  last_read_ts_ = std::chrono::duration<double>(
                     std::chrono::system_clock::now().time_since_epoch())
                     .count();
  return static_cast<size_t>(r);
}

std::vector<Packet> DMSerial::parse_all() {
  std::vector<Packet> results;
  size_t start = 0;

  while (true) {
    ssize_t j = -1;
    for (size_t i = start; i + 1 < buf_.size(); ++i) {
      if (buf_[i] == kHdr0 && buf_[i + 1] == kHdr1) {
        j = static_cast<ssize_t>(i);
        break;
      }
    }

    if (j < 0) {
      if (!buf_.empty()) {
        cnt_nohdr_++;
        uint8_t keep = buf_.back();
        buf_.assign(1, keep);
      }
      break;
    }

    if (buf_.size() - static_cast<size_t>(j) < kFrameLen) {
      cnt_short_++;
      buf_.assign(buf_.begin() + j, buf_.end());
      break;
    }

    std::array<uint8_t, kFrameLen> frame{};
    std::copy_n(buf_.begin() + j, kFrameLen, frame.begin());
    start = static_cast<size_t>(j) + 1;

    if (frame[kFrameLen - 1] != kTail) {
      continue;
    }

    uint8_t rid = frame[3];
    if (!valid_rid(rid)) {
      continue;
    }

    uint16_t crc_calc = 0;
    if (kSkipHdrInCrc) {
      crc_calc = dm_crc16(frame.data() + 2, 14);
    } else {
      crc_calc = dm_crc16(frame.data(), 16);
    }
    uint16_t crc_wire = static_cast<uint16_t>(frame[16] | (frame[17] << 8));
    if (crc_calc != crc_wire) {
      uint16_t alt = kSkipHdrInCrc ? dm_crc16(frame.data(), 16)
                                   : dm_crc16(frame.data() + 2, 14);
      if (alt != crc_wire) {
        cnt_crc_++;
        continue;
      }
    }

    Packet pkt;
    pkt.rid = rid;
    pkt.v1 = le_bytes_to_float(frame.data() + 4);
    pkt.v2 = le_bytes_to_float(frame.data() + 8);
    pkt.v3 = le_bytes_to_float(frame.data() + 12);
    results.push_back(pkt);

    buf_.erase(buf_.begin(), buf_.begin() + j + kFrameLen);
    start = 0;
  }

  cnt_ok_ += results.size();
  return results;
}

uint16_t DMSerial::dm_crc16(const uint8_t* data, size_t len) {
  static const uint16_t table[256] = {
      0x0000,0x1021,0x2042,0x3063,0x4084,0x50A5,0x60C6,0x70E7,0x8108,0x9129,0xA14A,0xB16B,0xC18C,0xD1AD,0xE1CE,0xF1EF,
      0x1231,0x0210,0x3273,0x2252,0x52B5,0x4294,0x72F7,0x62D6,0x9339,0x8318,0xB37B,0xA35A,0xD3BD,0xC39C,0xF3FF,0xE3DE,
      0x2462,0x3443,0x0420,0x1401,0x64E6,0x74C7,0x44A4,0x5485,0xA56A,0xB54B,0x8528,0x9509,0xE5EE,0xF5CF,0xC5AC,0xD58D,
      0x3653,0x2672,0x1611,0x0630,0x76D7,0x66F6,0x5695,0x46B4,0xB75B,0xA77A,0x9719,0x8738,0xF7DF,0xE7FE,0xD79D,0xC7BC,
      0x48C4,0x58E5,0x6886,0x78A7,0x0840,0x1861,0x2802,0x3823,0xC9CC,0xD9ED,0xE98E,0xF9AF,0x8948,0x9969,0xA90A,0xB92B,
      0x5AF5,0x4AD4,0x7AB7,0x6A96,0x1A71,0x0A50,0x3A33,0x2A12,0xDBFD,0xCBDC,0xFBBF,0xEB9E,0x9B79,0x8B58,0xBB3B,0xAB1A,
      0x6CA6,0x7C87,0x4CE4,0x5CC5,0x2C22,0x3C03,0x0C60,0x1C41,0xEDAE,0xFD8F,0xCDEC,0xDDCD,0xAD2A,0xBD0B,0x8D68,0x9D49,
      0x7E97,0x6EB6,0x5ED5,0x4EF4,0x3E13,0x2E32,0x1E51,0x0E70,0xFF9F,0xEFBE,0xDFDD,0xCFFC,0xBF1B,0xAF3A,0x9F59,0x8F78,
      0x9188,0x81A9,0xB1CA,0xA1EB,0xD10C,0xC12D,0xF14E,0xE16F,0x1080,0x00A1,0x30C2,0x20E3,0x5004,0x4025,0x7046,0x6067,
      0x83B9,0x9398,0xA3FB,0xB3DA,0xC33D,0xD31C,0xE37F,0xF35E,0x02B1,0x1290,0x22F3,0x32D2,0x4235,0x5214,0x6277,0x7256,
      0xB5EA,0xA5CB,0x95A8,0x8589,0xF56E,0xE54F,0xD52C,0xC50D,0x34E2,0x24C3,0x14A0,0x0481,0x7466,0x6447,0x5424,0x4405,
      0xA7DB,0xB7FA,0x8799,0x97B8,0xE75F,0xF77E,0xC71D,0xD73C,0x26D3,0x36F2,0x0691,0x16B0,0x6657,0x7676,0x4615,0x5634,
      0xD94C,0xC96D,0xF90E,0xE92F,0x99C8,0x89E9,0xB98A,0xA9AB,0x5844,0x4865,0x7806,0x6827,0x18C0,0x08E1,0x3882,0x28A3,
      0xCB7D,0xDB5C,0xEB3F,0xFB1E,0x8BF9,0x9BD8,0xABBB,0xBB9A,0x4A75,0x5A54,0x6A37,0x7A16,0x0AF1,0x1AD0,0x2AB3,0x3A92,
      0xFD2E,0xED0F,0xDD6C,0xCD4D,0xBDAA,0xAD8B,0x9DE8,0x8DC9,0x7C26,0x6C07,0x5C64,0x4C45,0x3CA2,0x2C83,0x1CE0,0x0CC1,
      0xEF1F,0xFF3E,0xCF5D,0xDF7C,0xAF9B,0xBFBA,0x8FD9,0x9FF8,0x6E17,0x7E36,0x4E55,0x5E74,0x2E93,0x3EB2,0x0ED1,0x1EF0
  };

  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    uint8_t index = static_cast<uint8_t>((crc >> 8) ^ data[i]);
    crc = static_cast<uint16_t>(((crc << 1) ^ table[index]) & 0xFFFF);
  }
  return crc;
}

std::string DMSerial::to_hex(const std::vector<uint8_t>& data) {
  static const char kHex[] = "0123456789abcdef";
  std::string out;
  out.reserve(data.size() * 2);
  for (uint8_t b : data) {
    out.push_back(kHex[(b >> 4) & 0x0F]);
    out.push_back(kHex[b & 0x0F]);
  }
  return out;
}

}  // namespace dm_imu_cpp
