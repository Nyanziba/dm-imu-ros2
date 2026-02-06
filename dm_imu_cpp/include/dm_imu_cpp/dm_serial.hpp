#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace dm_imu_cpp {

struct Packet {
  uint8_t rid = 0;
  float v1 = 0.0f;
  float v2 = 0.0f;
  float v3 = 0.0f;
};

struct DebugInfo {
  size_t buf_len = 0;
  size_t last_read_len = 0;
  std::string last_read_hex;
  double last_read_ts = 0.0;
  int in_waiting = 0;
  uint64_t cnt_ok = 0;
  uint64_t cnt_crc = 0;
  uint64_t cnt_short = 0;
  uint64_t cnt_nohdr = 0;
  std::string last_error;
};

class DMSerial {
 public:
  DMSerial(const std::string& port, int baudrate);
  ~DMSerial();

  bool start_reader(double read_sleep = 0.001);
  void stop_reader();

  std::tuple<std::optional<Packet>, double, uint64_t> get_latest();
  DebugInfo get_debug();
  std::string last_error() const;

 private:
  bool open_port();
  void close_port();
  void reader_loop();
  size_t read_into_buf(std::optional<size_t> max_bytes);
  std::vector<Packet> parse_all();

  static uint16_t dm_crc16(const uint8_t* data, size_t len);
  static std::string to_hex(const std::vector<uint8_t>& data);

  std::string port_;
  int baudrate_ = 0;
  int fd_ = -1;

  std::vector<uint8_t> buf_;
  std::vector<uint8_t> last_read_;
  double last_read_ts_ = 0.0;

  uint64_t cnt_ok_ = 0;
  uint64_t cnt_crc_ = 0;
  uint64_t cnt_short_ = 0;
  uint64_t cnt_nohdr_ = 0;

  std::thread reader_;
  bool stop_ = false;
  double read_sleep_ = 0.001;

  mutable std::mutex latest_mutex_;
  std::optional<Packet> latest_pkt_;
  double latest_ts_ = 0.0;
  uint64_t latest_count_ = 0;

  mutable std::mutex err_mutex_;
  std::string last_error_;
};

}  // namespace dm_imu_cpp
