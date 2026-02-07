#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "dm_imu_cpp/dm_serial.hpp"

namespace dm_imu_cpp {
namespace {
constexpr double kPi = 3.14159265358979323846;

std::array<double, 4> euler_rpy_to_quat(double roll, double pitch, double yaw) {
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  const double qw = cr * cp * cy + sr * sp * sy;
  const double qx = sr * cp * cy - cr * sp * sy;
  const double qy = cr * sp * cy + sr * cp * sy;
  const double qz = cr * cp * sy - sr * sp * cy;
  return {qx, qy, qz, qw};
}

bool finite_all(double a, double b, double c, double d) {
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) && std::isfinite(d);
}

}  // namespace

class DmImuNode : public rclcpp::Node {
 public:
  DmImuNode() : rclcpp::Node("dm_imu") {
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 921600);
    declare_parameter<std::string>("frame_id", "imu_link");
    declare_parameter<bool>("publish_rpy_in_degree", true);
    declare_parameter<bool>("verbose", true);
    declare_parameter<bool>("qos_reliable", true);
    declare_parameter<bool>("debug_raw", false);
    declare_parameter<int>("debug_raw_max_bytes", 64);
    declare_parameter<bool>("publish_imu_data", true);
    declare_parameter<bool>("publish_rpy", true);
    declare_parameter<bool>("publish_pose", false);
    declare_parameter<double>("publish_rate_hz", 1000.0);

    // Data type IDs (PDF default: accel=1, gyro=2, rpy=3, quat=4)
    declare_parameter<int>("data_type_accel", 0x01);
    declare_parameter<int>("data_type_gyro", 0x02);
    declare_parameter<int>("data_type_rpy", 0x03);
    declare_parameter<int>("data_type_quat", 0x04);
    declare_parameter<bool>("warn_unknown_data_type", true);

    declare_parameter<bool>("gyro_in_degree", true);
    declare_parameter<bool>("accel_in_g", true);
    declare_parameter<double>("orientation_covariance", 0.02);
    declare_parameter<double>("angular_velocity_covariance", 0.02);
    declare_parameter<double>("linear_acceleration_covariance", 0.02);
    declare_parameter<bool>("usb_configure", true);
    declare_parameter<double>("usb_output_rate_hz", 1000.0);
    declare_parameter<bool>("usb_enable_accel", true);
    declare_parameter<bool>("usb_enable_gyro", true);
    declare_parameter<bool>("usb_enable_rpy", true);
    declare_parameter<bool>("usb_force_output_interface", true);
    declare_parameter<int>("usb_output_interface", 0);
    declare_parameter<bool>("usb_factory_reset_on_start", true);
    declare_parameter<int>("usb_factory_reset_wait_ms", 1200);
    declare_parameter<bool>("usb_save_params", true);
    declare_parameter<double>("usb_config_sleep_ms", 50.0);

    port_ = get_parameter("port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    publish_rpy_ = get_parameter("publish_rpy").as_bool();
    publish_rpy_in_degree_ = get_parameter("publish_rpy_in_degree").as_bool();
    publish_imu_data_ = get_parameter("publish_imu_data").as_bool();
    publish_pose_ = get_parameter("publish_pose").as_bool();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    data_type_accel_ = get_parameter("data_type_accel").as_int();
    data_type_gyro_ = get_parameter("data_type_gyro").as_int();
    data_type_rpy_ = get_parameter("data_type_rpy").as_int();
    data_type_quat_ = get_parameter("data_type_quat").as_int();
    warn_unknown_data_type_ = get_parameter("warn_unknown_data_type").as_bool();
    verbose_ = get_parameter("verbose").as_bool();
    debug_raw_ = get_parameter("debug_raw").as_bool();
    debug_raw_max_bytes_ = get_parameter("debug_raw_max_bytes").as_int();
    gyro_in_degree_ = get_parameter("gyro_in_degree").as_bool();
    accel_in_g_ = get_parameter("accel_in_g").as_bool();
    orientation_cov_ = get_parameter("orientation_covariance").as_double();
    angular_velocity_cov_ = get_parameter("angular_velocity_covariance").as_double();
    linear_acceleration_cov_ = get_parameter("linear_acceleration_covariance").as_double();
    usb_configure_ = get_parameter("usb_configure").as_bool();
    usb_output_rate_hz_ = get_parameter("usb_output_rate_hz").as_double();
    usb_enable_accel_ = get_parameter("usb_enable_accel").as_bool();
    usb_enable_gyro_ = get_parameter("usb_enable_gyro").as_bool();
    usb_enable_rpy_ = get_parameter("usb_enable_rpy").as_bool();
    usb_force_output_interface_ = get_parameter("usb_force_output_interface").as_bool();
    usb_output_interface_ = get_parameter("usb_output_interface").as_int();
    usb_factory_reset_on_start_ = get_parameter("usb_factory_reset_on_start").as_bool();
    usb_factory_reset_wait_ms_ = get_parameter("usb_factory_reset_wait_ms").as_int();
    usb_save_params_ = get_parameter("usb_save_params").as_bool();
    usb_config_sleep_ms_ = get_parameter("usb_config_sleep_ms").as_double();

    const bool qos_reliable = get_parameter("qos_reliable").as_bool();
    rclcpp::QoS qos(50);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    if (qos_reliable) {
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    } else {
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    }

    if (publish_imu_data_) {
      pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
    }
    if (publish_rpy_) {
      pub_rpy_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy", qos);
    }
    if (publish_pose_) {
      pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("imu/pose", 10);
    }

    try {
      serial_ = std::make_unique<DMSerial>(port_, baudrate_);
      if (usb_configure_) {
        configure_device();
      }
      serial_->start_reader();
      RCLCPP_INFO(get_logger(), "Opened serial %s @ %d", port_.c_str(), baudrate_);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Init serial failed: %s", e.what());
      throw;
    }

    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid publish_rate_hz=%.3f; fallback to 1000.0", publish_rate_hz_);
      publish_rate_hz_ = 1000.0;
    }
    warn_every_ticks_ = static_cast<uint64_t>(std::round(publish_rate_hz_));
    timer_pub_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz_),
                                   std::bind(&DmImuNode::on_timer_publish, this));
    timer_stat_ = create_wall_timer(std::chrono::seconds(2),
                                    std::bind(&DmImuNode::on_timer_stats, this));
  }

  ~DmImuNode() override {
    if (serial_) {
      serial_->stop_reader();
    }
  }

 private:
  void configure_device() {
    if (!serial_) {
      return;
    }

    auto to_hex = [](const std::vector<uint8_t>& cmd) -> std::string {
      std::ostringstream oss;
      oss << std::hex << std::setfill('0');
      for (uint8_t b : cmd) {
        oss << std::setw(2) << static_cast<int>(b);
      }
      return oss.str();
    };

    const double sleep_s = std::max(0.0, usb_config_sleep_ms_ / 1000.0);
    auto send_cmd = [&](const std::vector<uint8_t>& cmd) {
      const bool ok = serial_->write_bytes(cmd);
      if (verbose_) {
        RCLCPP_INFO(get_logger(), "USB cmd sent (%zuB): %s", cmd.size(), to_hex(cmd).c_str());
      }
      if (!ok) {
        RCLCPP_WARN(get_logger(), "USB cmd write failed: %s", serial_->last_error().c_str());
      }
      if (sleep_s > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
      }
    };

    if (usb_factory_reset_on_start_) {
      send_cmd({0xAA, 0x06, 0x01, 0x0D});  // enter setting mode
      send_cmd({0xAA, 0x0B, 0x01, 0x0D});  // factory reset
      send_cmd({0xAA, 0x03, 0x01, 0x0D});  // save
      send_cmd({0xAA, 0x00, 0x00, 0x0D});  // soft reboot

      const auto wait_ms = std::max(0, usb_factory_reset_wait_ms_);
      if (wait_ms > 0) {
        if (verbose_) {
          RCLCPP_INFO(get_logger(),
                      "Waiting %dms for sensor reboot after factory reset",
                      wait_ms);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
      }
      if (!serial_->reopen()) {
        RCLCPP_WARN(get_logger(),
                    "Serial reopen after factory reset failed: %s",
                    serial_->last_error().c_str());
      }
    }

    int interval_ms = -1;
    if (usb_output_rate_hz_ > 0.0) {
      if (usb_output_rate_hz_ < 100.0 || usb_output_rate_hz_ > 1000.0) {
        RCLCPP_WARN(get_logger(),
                    "usb_output_rate_hz=%.3f out of 100-1000Hz range; sending anyway.",
                    usb_output_rate_hz_);
      }
      interval_ms = static_cast<int>(std::lround(1000.0 / usb_output_rate_hz_));
      if (interval_ms < 1) {
        interval_ms = 1;
      } else if (interval_ms > 0xFFFF) {
        interval_ms = 0xFFFF;
      }
    }

    std::vector<std::vector<uint8_t>> cmds;
    cmds.push_back({0xAA, 0x06, 0x01, 0x0D});  // enter setting mode
    if (usb_enable_accel_) {
      cmds.push_back({0xAA, 0x01, 0x14, 0x0D});
    }
    if (usb_enable_gyro_) {
      cmds.push_back({0xAA, 0x01, 0x15, 0x0D});
    }
    if (usb_enable_rpy_) {
      cmds.push_back({0xAA, 0x01, 0x16, 0x0D});
    }
    if (usb_force_output_interface_) {
      int iface = usb_output_interface_;
      if (iface < 0) {
        iface = 0;
      } else if (iface > 3) {
        iface = 3;
      }
      cmds.push_back({0xAA, 0x0A, static_cast<uint8_t>(iface), 0x0D});
    }
    if (interval_ms > 0) {
      cmds.push_back({
          0xAA,
          0x02,
          static_cast<uint8_t>(interval_ms & 0xFF),
          static_cast<uint8_t>((interval_ms >> 8) & 0xFF),
          0x0D,
      });
    }
    if (usb_save_params_) {
      cmds.push_back({0xAA, 0x03, 0x01, 0x0D});
    }
    cmds.push_back({0xAA, 0x06, 0x00, 0x0D});  // exit setting mode

    for (const auto& cmd : cmds) {
      send_cmd(cmd);
    }
  }

  void on_timer_publish() {
    if (!serial_) {
      return;
    }
    auto [packets_by_type, stamp_ts, count] = serial_->get_latest_by_type();
    (void)stamp_ts;
    if (packets_by_type.empty()) {
      no_frame_ticks_ += 1;
      if (warn_every_ticks_ > 0 && no_frame_ticks_ % warn_every_ticks_ == 0 && verbose_) {
        RCLCPP_WARN(get_logger(), "No frames yet from serial (≈1s). Check IMU streaming/baud/crc.");
      }
      return;
    }

    if (count == last_count_) {
      return;
    }
    last_count_ = count;

    for (const auto& kv : packets_by_type) {
      update_from_packet(kv.second);
    }

    if (!has_rpy_) {
      no_frame_ticks_ = 0;
      return;
    }

    const double r_rad = r_deg_ * kPi / 180.0;
    const double p_rad = p_deg_ * kPi / 180.0;
    const double y_rad = y_deg_ * kPi / 180.0;

    auto stamp = get_clock()->now();
    auto quat = euler_rpy_to_quat(r_rad, p_rad, y_rad);

    if (pub_rpy_) {
      geometry_msgs::msg::Vector3Stamped rpy_msg;
      rpy_msg.header.stamp = stamp;
      rpy_msg.header.frame_id = frame_id_;
      if (publish_rpy_in_degree_) {
        rpy_msg.vector.x = r_deg_;
        rpy_msg.vector.y = p_deg_;
        rpy_msg.vector.z = y_deg_;
      } else {
        rpy_msg.vector.x = r_rad;
        rpy_msg.vector.y = p_rad;
        rpy_msg.vector.z = y_rad;
      }
      pub_rpy_->publish(rpy_msg);
    }

    if (!publish_imu_data_ && !publish_pose_) {
      return;
    }

    double qx = quat[0];
    double qy = quat[1];
    double qz = quat[2];
    double qw = quat[3];

    if (!finite_all(qx, qy, qz, qw)) {
      if (verbose_) {
        RCLCPP_WARN(get_logger(), "Quaternion has NaN/Inf, publishing identity (0,0,0,1)");
      }
      qx = 0.0;
      qy = 0.0;
      qz = 0.0;
      qw = 1.0;
    } else {
      const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
      if (n < 1e-6) {
        if (verbose_) {
          RCLCPP_WARN(get_logger(), "Quaternion norm ~0, publishing identity (0,0,0,1)");
        }
        qx = 0.0;
        qy = 0.0;
        qz = 0.0;
        qw = 1.0;
      } else {
        qx /= n;
        qy /= n;
        qz /= n;
        qw /= n;
      }
    }

    if (pub_imu_) {
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = stamp;
      imu.header.frame_id = frame_id_;
      imu.orientation.x = qx;
      imu.orientation.y = qy;
      imu.orientation.z = qz;
      imu.orientation.w = qw;

      double gx = gyro_x_;
      double gy = gyro_y_;
      double gz = gyro_z_;
      if (gyro_in_degree_) {
        const double scale = kPi / 180.0;
        gx *= scale;
        gy *= scale;
        gz *= scale;
      }
      imu.angular_velocity.x = gx;
      imu.angular_velocity.y = gy;
      imu.angular_velocity.z = gz;

      double ax = accel_x_;
      double ay = accel_y_;
      double az = accel_z_;
      if (accel_in_g_) {
        const double g = 9.80665;
        ax *= g;
        ay *= g;
        az *= g;
      }
      imu.linear_acceleration.x = ax;
      imu.linear_acceleration.y = ay;
      imu.linear_acceleration.z = az;

      for (int i = 0; i < 9; ++i) {
        imu.orientation_covariance[i] = 0.0;
        imu.angular_velocity_covariance[i] = 0.0;
        imu.linear_acceleration_covariance[i] = 0.0;
      }
      imu.orientation_covariance[0] = orientation_cov_;
      imu.orientation_covariance[4] = orientation_cov_;
      imu.orientation_covariance[8] = orientation_cov_;

      imu.angular_velocity_covariance[0] = angular_velocity_cov_;
      imu.angular_velocity_covariance[4] = angular_velocity_cov_;
      imu.angular_velocity_covariance[8] = angular_velocity_cov_;

      imu.linear_acceleration_covariance[0] = linear_acceleration_cov_;
      imu.linear_acceleration_covariance[4] = linear_acceleration_cov_;
      imu.linear_acceleration_covariance[8] = linear_acceleration_cov_;

      pub_imu_->publish(imu);
    }

    if (pub_pose_) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.frame_id = frame_id_;
      pose.pose.position.x = 0.0;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = qx;
      pose.pose.orientation.y = qy;
      pose.pose.orientation.z = qz;
      pose.pose.orientation.w = qw;
      pub_pose_->publish(pose);
    }

    pub_count_ += 1;
    no_frame_ticks_ = 0;
    if (verbose_) {
      RCLCPP_INFO(get_logger(),
                  "#%lu RPY(deg)=(%.2f, %.2f, %.2f) RPY(rad)=(%.3f, %.3f, %.3f)",
                  pub_count_, r_deg_, p_deg_, y_deg_, r_rad, p_rad, y_rad);
    }
  }

  void on_timer_stats() {
    if (!serial_) {
      return;
    }
    auto dbg = serial_->get_debug();
    if (verbose_) {
      RCLCPP_INFO(get_logger(),
                  "[stats] ok=%lu crc=%lu short=%lu nohdr=%lu",
                  dbg.cnt_ok, dbg.cnt_crc, dbg.cnt_short, dbg.cnt_nohdr);
    }
    if (debug_raw_) {
      std::string raw_hex = dbg.last_read_hex;
      if (debug_raw_max_bytes_ > 0 && raw_hex.size() > static_cast<size_t>(debug_raw_max_bytes_ * 2)) {
        raw_hex.resize(static_cast<size_t>(debug_raw_max_bytes_ * 2));
      }
      if (no_frame_ticks_ > 0) {
        RCLCPP_INFO(get_logger(),
                    "[raw] in_waiting=%d buf_len=%zu last_read_len=%zu cnt_ok=%lu cnt_crc=%lu "
                    "cnt_short=%lu cnt_nohdr=%lu last_error=%s last_read_hex=%s",
                    dbg.in_waiting, dbg.buf_len, dbg.last_read_len,
                    dbg.cnt_ok, dbg.cnt_crc, dbg.cnt_short, dbg.cnt_nohdr,
                    dbg.last_error.c_str(), raw_hex.c_str());
      }
    }
  }

  void update_from_packet(const Packet& pkt) {
    const uint8_t dtype = pkt.data_type;

    if (dtype == static_cast<uint8_t>(data_type_rpy_)) {
      r_deg_ = pkt.v1;
      p_deg_ = pkt.v2;
      y_deg_ = pkt.v3;
      has_rpy_ = true;
    } else if (dtype == static_cast<uint8_t>(data_type_gyro_)) {
      gyro_x_ = pkt.v1;
      gyro_y_ = pkt.v2;
      gyro_z_ = pkt.v3;
      has_gyro_ = true;
    } else if (dtype == static_cast<uint8_t>(data_type_accel_)) {
      accel_x_ = pkt.v1;
      accel_y_ = pkt.v2;
      accel_z_ = pkt.v3;
      has_accel_ = true;
    } else if (dtype == static_cast<uint8_t>(data_type_quat_)) {
      // Quaternion output is available but unused in this node.
    } else if (warn_unknown_data_type_ && !unknown_data_type_logged_[dtype]) {
      unknown_data_type_logged_[dtype] = true;
      if (verbose_) {
        if (pkt.dev_id) {
          RCLCPP_WARN(get_logger(),
                      "Unknown data_type=0x%02X dev_id=0x%02X. Update data_type_* params if needed.",
                      dtype, pkt.dev_id);
        } else {
          RCLCPP_WARN(get_logger(),
                      "Unknown data_type=0x%02X. Update data_type_* params if needed.",
                      dtype);
        }
      }
    }

    if (!has_gyro_ && !logged_missing_gyro_) {
      logged_missing_gyro_ = true;
      if (verbose_) {
        RCLCPP_WARN(get_logger(), "Gyro frame not received yet; angular_velocity will be zero until it arrives.");
      }
    }
    if (!has_accel_ && !logged_missing_accel_) {
      logged_missing_accel_ = true;
      if (verbose_) {
        RCLCPP_WARN(get_logger(), "Accel frame not received yet; linear_acceleration will be zero until it arrives.");
      }
    }
  }

  std::unique_ptr<DMSerial> serial_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_rpy_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;

  rclcpp::TimerBase::SharedPtr timer_pub_;
  rclcpp::TimerBase::SharedPtr timer_stat_;

  std::string port_;
  int baudrate_ = 921600;
  std::string frame_id_ = "imu_link";
  bool publish_rpy_ = true;
  bool publish_rpy_in_degree_ = true;
  bool publish_imu_data_ = true;
  bool publish_pose_ = false;
  double publish_rate_hz_ = 1000.0;
  int data_type_accel_ = 0x01;
  int data_type_gyro_ = 0x02;
  int data_type_rpy_ = 0x03;
  int data_type_quat_ = 0x04;
  bool warn_unknown_data_type_ = true;
  bool verbose_ = true;
  bool debug_raw_ = false;
  int debug_raw_max_bytes_ = 64;
  bool gyro_in_degree_ = true;
  bool accel_in_g_ = true;
  double orientation_cov_ = 0.02;
  double angular_velocity_cov_ = 0.02;
  double linear_acceleration_cov_ = 0.02;
  bool usb_configure_ = true;
  double usb_output_rate_hz_ = 1000.0;
  bool usb_enable_accel_ = true;
  bool usb_enable_gyro_ = true;
  bool usb_enable_rpy_ = true;
  bool usb_force_output_interface_ = true;
  int usb_output_interface_ = 0;
  bool usb_factory_reset_on_start_ = true;
  int usb_factory_reset_wait_ms_ = 1200;
  bool usb_save_params_ = true;
  double usb_config_sleep_ms_ = 50.0;

  uint64_t last_count_ = 0;
  uint64_t no_frame_ticks_ = 0;
  uint64_t warn_every_ticks_ = 200;
  uint64_t pub_count_ = 0;

  bool has_rpy_ = false;
  bool has_gyro_ = false;
  bool has_accel_ = false;
  bool logged_missing_gyro_ = false;
  bool logged_missing_accel_ = false;
  std::array<bool, 256> unknown_data_type_logged_{};

  double r_deg_ = 0.0;
  double p_deg_ = 0.0;
  double y_deg_ = 0.0;
  double gyro_x_ = 0.0;
  double gyro_y_ = 0.0;
  double gyro_z_ = 0.0;
  double accel_x_ = 0.0;
  double accel_y_ = 0.0;
  double accel_z_ = 0.0;
};

}  // namespace dm_imu_cpp

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dm_imu_cpp::DmImuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
