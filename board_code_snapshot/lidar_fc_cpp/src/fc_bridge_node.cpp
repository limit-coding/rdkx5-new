#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <fcntl.h>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

using Clock = std::chrono::steady_clock;

class FcBridgeNode : public rclcpp::Node
{
public:
  FcBridgeNode() : Node("fc_bridge_node")
  {
    port_ = declare_parameter<std::string>("serial_port", "/dev/ttyFC");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    send_freq_ = declare_parameter<double>("send_freq", 20.0);
    status_send_freq_ = declare_parameter<double>("status_send_freq", 10.0);
    max_xy_meters_ = declare_parameter<double>("max_xy_meters", 10.0);
    clamp_xy_instead_of_zero_ = declare_parameter<bool>("clamp_xy_instead_of_zero", true);
    valid_timeout_sec_ = declare_parameter<double>("valid_timeout_sec", 0.5);

    tryOpenSerial();

    error_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/position_error", 10,
      std::bind(&FcBridgeNode::errorCallback, this, std::placeholders::_1));

    yaw_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/relative_pose", 10,
      std::bind(&FcBridgeNode::yawPoseCallback, this, std::placeholders::_1));

    valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/localization_valid", 10,
      std::bind(&FcBridgeNode::validCallback, this, std::placeholders::_1));

    mission_status_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/mission_status", 10,
      std::bind(&FcBridgeNode::missionStatusCallback, this, std::placeholders::_1));

    fc_frame_debug_pub_ = create_publisher<std_msgs::msg::String>("/fc_tx_frame_debug", 50);

    const double safe_send_freq = std::max(send_freq_, 1.0);
    const auto timer_period = std::chrono::duration<double>(1.0 / safe_send_freq);
    send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&FcBridgeNode::sendCallback, this));

    const double safe_status_send_freq = std::max(status_send_freq_, 1.0);
    const auto status_timer_period = std::chrono::duration<double>(1.0 / safe_status_send_freq);
    status_send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(status_timer_period),
      std::bind(&FcBridgeNode::sendMissionStatusCallback, this));

    RCLCPP_INFO(get_logger(), "飞控桥接C++节点已启动，等待 /position_error 与 /relative_pose 数据...");
  }

  ~FcBridgeNode() override
  {
    closeSerial();
  }

private:
  static double elapsedSeconds(Clock::time_point start)
  {
    return std::chrono::duration<double>(Clock::now() - start).count();
  }

  static speed_t baudToTermios(int baudrate)
  {
    switch (baudrate) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      case 460800:
        return B460800;
      case 921600:
        return B921600;
      default:
        return B115200;
    }
  }

  static double radiansToDegrees(double radians)
  {
    return radians * 180.0 / 3.14159265358979323846;
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static int16_t clampToS16(int value)
  {
    return static_cast<int16_t>(std::clamp(value, -32768, 32767));
  }

  static void putS16Be(std::array<uint8_t, 11> & frame, size_t index, int16_t value)
  {
    const auto raw = static_cast<uint16_t>(value);
    frame[index] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    frame[index + 1] = static_cast<uint8_t>(raw & 0xFF);
  }

  template<size_t N>
  static std::string frameToHex(const std::array<uint8_t, N> & frame)
  {
    std::ostringstream oss;
    oss << std::uppercase << std::hex << std::setfill('0');
    for (size_t i = 0; i < frame.size(); ++i) {
      if (i > 0) {
        oss << ' ';
      }
      oss << std::setw(2) << static_cast<int>(frame[i]);
    }
    return oss.str();
  }

  bool tryOpenSerial()
  {
    if (fd_ >= 0) {
      return true;
    }

    const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_DEBUG(
        get_logger(), "串口打开失败 %s: %s", port_.c_str(), std::strerror(errno));
      return false;
    }

    termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
      RCLCPP_WARN(
        get_logger(), "读取串口配置失败 %s: %s", port_.c_str(), std::strerror(errno));
      ::close(fd);
      return false;
    }

    cfmakeraw(&tty);
    const speed_t speed = baudToTermios(baudrate_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_WARN(
        get_logger(), "写入串口配置失败 %s: %s", port_.c_str(), std::strerror(errno));
      ::close(fd);
      return false;
    }

    int modem_bits = 0;
    if (ioctl(fd, TIOCMGET, &modem_bits) == 0) {
      modem_bits |= TIOCM_DTR;
      modem_bits |= TIOCM_RTS;
      (void)ioctl(fd, TIOCMSET, &modem_bits);
    }

    tcflush(fd, TCIOFLUSH);
    fd_ = fd;
    RCLCPP_INFO(get_logger(), "串口已打开: %s @ %dbps", port_.c_str(), baudrate_);
    return true;
  }

  void closeSerial()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void startReconnectTimer()
  {
    if (reconnect_timer_) {
      return;
    }

    RCLCPP_WARN(get_logger(), "串口断开，每 2 秒尝试重连 %s...", port_.c_str());
    reconnect_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&FcBridgeNode::reconnectCallback, this));
  }

  void stopReconnectTimer()
  {
    if (reconnect_timer_) {
      reconnect_timer_->cancel();
      reconnect_timer_.reset();
    }
  }

  void reconnectCallback()
  {
    if (tryOpenSerial()) {
      RCLCPP_INFO(get_logger(), "串口重连成功！");
      stopReconnectTimer();
    }
  }

  void errorCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    current_error_ = msg;
  }

  void yawPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_yaw_pose_ = msg;
  }

  void validCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    localization_valid_ = msg->data;
    if (localization_valid_) {
      last_valid_time_ = Clock::now();
    }
  }

  void missionStatusCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(get_logger(), "ignored invalid /mission_status payload");
      return;
    }

    mission_task_state_ = static_cast<uint8_t>(msg->data[0] & 0xFF);
    mission_landing_state_ = static_cast<uint8_t>(msg->data[1] & 0xFF);
  }

  void sendCallback()
  {
    if (fd_ < 0) {
      startReconnectTimer();
      return;
    }

    double x = 0.0;
    double y = 0.0;
    double yaw_deg = 0.0;
    std::string zero_reason;

    const bool valid_recent =
      localization_valid_ &&
      last_valid_time_ &&
      elapsedSeconds(*last_valid_time_) <= valid_timeout_sec_;

    if (!current_error_ || !current_yaw_pose_) {
      zero_reason = "尚未收到 /position_error 或 /relative_pose，发送 0cm 心跳帧";
    } else if (!valid_recent) {
      zero_reason = "定位状态无效，发送 0cm 心跳帧";
    } else {
      x = current_error_->x;
      y = current_error_->y;
      // 飞控接收的是航向误差；目标 yaw 默认为 0，因此应发送 -当前 yaw。
      yaw_deg = -radiansToDegrees(yawFromQuaternion(current_yaw_pose_->pose.orientation));
    }

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw_deg)) {
      zero_reason = "收到非法坐标 (NaN/Inf)，发送 0cm 心跳帧";
      x = 0.0;
      y = 0.0;
      yaw_deg = 0.0;
    }

    if (std::fabs(x) > max_xy_meters_ || std::fabs(y) > max_xy_meters_) {
      if (clamp_xy_instead_of_zero_ && valid_recent) {
        zero_reason = "坐标超出发送范围，已限幅发送";
        x = std::clamp(x, -max_xy_meters_, max_xy_meters_);
        y = std::clamp(y, -max_xy_meters_, max_xy_meters_);
      } else {
        zero_reason = "坐标超出安全范围，发送 0cm 心跳帧";
        x = 0.0;
        y = 0.0;
        yaw_deg = 0.0;
      }
    }

    const int16_t x_cm = clampToS16(static_cast<int>(x * 100.0));
    const int16_t y_cm = clampToS16(static_cast<int>(y * 100.0));
    const int16_t yaw = clampToS16(static_cast<int>(std::lround(yaw_deg)));

    std::array<uint8_t, 11> frame {};
    frame[0] = 0xAA;
    frame[1] = 0xFF;
    frame[2] = 0x01;
    frame[3] = 0x06;
    putS16Be(frame, 4, x_cm);
    putS16Be(frame, 6, y_cm);
    putS16Be(frame, 8, yaw);

    uint8_t checksum = 0;
    for (size_t i = 0; i < frame.size() - 1; ++i) {
      checksum = static_cast<uint8_t>(checksum + frame[i]);
    }
    frame[10] = checksum;

    const ssize_t written = ::write(fd_, frame.data(), frame.size());
    if (written != static_cast<ssize_t>(frame.size())) {
      RCLCPP_WARN(
        get_logger(), "串口写入失败: %s", written < 0 ? std::strerror(errno) : "short write");
      closeSerial();
      startReconnectTimer();
      return;
    }

    publishFrameDebug(
      "xy", x_cm, y_cm, yaw, frameToHex(frame), zero_reason.empty() ? "normal" : zero_reason);

    const auto now = Clock::now();
    if (!zero_reason.empty() &&
      (zero_reason != last_zero_reason_ || !last_invalid_log_time_ ||
      std::chrono::duration<double>(now - *last_invalid_log_time_).count() >= 1.0))
    {
      last_invalid_log_time_ = now;
      last_zero_reason_ = zero_reason;
      RCLCPP_WARN(get_logger(), "%s", zero_reason.c_str());
    }

    if (!last_log_time_ ||
      std::chrono::duration<double>(now - *last_log_time_).count() >= 1.0)
    {
      last_log_time_ = now;
      RCLCPP_INFO(
        get_logger(),
        "串口发送 -> X=%dcm, Y=%dcm, YAW=%ddeg | hex: "
        "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        static_cast<int>(x_cm), static_cast<int>(y_cm), static_cast<int>(yaw),
        frame[0], frame[1], frame[2], frame[3], frame[4],
        frame[5], frame[6], frame[7], frame[8], frame[9], frame[10]);
    }
  }

  void sendMissionStatusCallback()
  {
    if (fd_ < 0) {
      startReconnectTimer();
      return;
    }

    std::array<uint8_t, 7> frame {};
    frame[0] = 0xAA;
    frame[1] = 0xFF;
    frame[2] = 0x02;
    frame[3] = 0x02;
    frame[4] = mission_task_state_;
    frame[5] = mission_landing_state_;

    uint8_t checksum = 0;
    for (size_t i = 0; i < frame.size() - 1; ++i) {
      checksum = static_cast<uint8_t>(checksum + frame[i]);
    }
    frame[6] = checksum;

    const ssize_t written = ::write(fd_, frame.data(), frame.size());
    if (written != static_cast<ssize_t>(frame.size())) {
      RCLCPP_WARN(
        get_logger(), "mission status write failed: %s",
        written < 0 ? std::strerror(errno) : "short write");
      closeSerial();
      startReconnectTimer();
      return;
    }

    publishFrameDebug(
      "mission", mission_task_state_, mission_landing_state_, 0, frameToHex(frame), "normal");

    const auto now = Clock::now();
    if (!last_status_log_time_ ||
      std::chrono::duration<double>(now - *last_status_log_time_).count() >= 1.0)
    {
      last_status_log_time_ = now;
      RCLCPP_INFO(
        get_logger(),
        "mission status -> task=0x%02X, landing=0x%02X | hex: "
        "%02X %02X %02X %02X %02X %02X %02X",
        frame[4], frame[5],
        frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6]);
    }
  }

  std::string port_{"/dev/ttyFC"};
  int baudrate_{115200};
  double send_freq_{20.0};
  double status_send_freq_{10.0};
  double max_xy_meters_{10.0};
  bool clamp_xy_instead_of_zero_{true};
  double valid_timeout_sec_{0.5};
  int fd_{-1};

  bool localization_valid_{false};
  uint8_t mission_task_state_{0x01};
  uint8_t mission_landing_state_{0x01};
  geometry_msgs::msg::Point::SharedPtr current_error_;
  geometry_msgs::msg::PoseStamped::SharedPtr current_yaw_pose_;
  std::optional<Clock::time_point> last_valid_time_;
  std::optional<Clock::time_point> last_invalid_log_time_;
  std::optional<Clock::time_point> last_log_time_;
  std::optional<Clock::time_point> last_status_log_time_;
  std::string last_zero_reason_;

  void publishFrameDebug(
    const std::string & type, int x_or_task, int y_or_landing, int yaw,
    const std::string & hex, const std::string & reason)
  {
    if (!fc_frame_debug_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    std::ostringstream oss;
    oss << "type=" << type
        << ",x_or_task=" << x_or_task
        << ",y_or_landing=" << y_or_landing
        << ",yaw=" << yaw
        << ",hex=" << hex
        << ",reason=" << reason;
    msg.data = oss.str();
    fc_frame_debug_pub_->publish(msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr error_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr yaw_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mission_status_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr status_send_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fc_frame_debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FcBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
