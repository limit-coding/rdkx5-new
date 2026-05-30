#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <fcntl.h>
#include <memory>
#include <optional>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using Clock = std::chrono::steady_clock;

// 这个节点负责把 ROS2 里的相对定位结果发给匿名凌霄飞控：
//   输入：/relative_pose，来自 relative_pose_node，单位是米和弧度。
//        /localization_valid，定位是否可信。
//   输出：串口二进制帧，发到 /dev/ttyFC，单位转成厘米和度。
//
// 串口帧格式：
//   AA FF 01 06 X_H X_L Y_H Y_L YAW_H YAW_L CHECKSUM
//   X/Y   : int16，大端，高字节在前，单位 cm
//   YAW   : int16，大端，高字节在前，单位 degree
//   CHECKSUM: 前 10 个字节累加后取低 8 位
//
// 安全策略：
//   定位无效、没有收到坐标、坐标非法时，不发真实坐标，而是发 0cm 心跳帧。
//   这样飞控端还能知道串口链路活着，但不会被坏坐标带飞。
class FcBridgeNode : public rclcpp::Node
{
public:
  FcBridgeNode() : Node("fc_bridge_node")
  {
    // 串口设备名。部署时通常用 udev 规则固定成 /dev/ttyFC。
    port_ = declare_parameter<std::string>("serial_port", "/dev/ttyFC");
    baudrate_ = declare_parameter<int>("baudrate", 115200);

    // 串口发送频率。20Hz 对飞控位置环通常够用，也不会给串口太大压力。
    send_freq_ = declare_parameter<double>("send_freq", 20.0);

    // 发送给飞控前的水平坐标保护范围，单位米。
    max_xy_meters_ = declare_parameter<double>("max_xy_meters", 10.0);

    // true：超范围但定位仍有效时，把坐标压到边界；
    // false：超范围直接发 0cm 心跳帧。
    clamp_xy_instead_of_zero_ = declare_parameter<bool>("clamp_xy_instead_of_zero", true);

    // /localization_valid 必须在这个时间内刚刚为 true，才认为定位仍然新鲜。
    valid_timeout_sec_ = declare_parameter<double>("valid_timeout_sec", 0.5);

    tryOpenSerial();

    // 相对坐标输入。单位：x/y 是米，yaw 在四元数里。
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/relative_pose", 10,
      std::bind(&FcBridgeNode::poseCallback, this, std::placeholders::_1));

    // 定位健康标志。false 时本节点只发 0cm 心跳帧。
    valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/localization_valid", 10,
      std::bind(&FcBridgeNode::validCallback, this, std::placeholders::_1));

    const double safe_send_freq = std::max(send_freq_, 1.0);
    const auto timer_period = std::chrono::duration<double>(1.0 / safe_send_freq);
    send_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&FcBridgeNode::sendCallback, this));

    RCLCPP_INFO(get_logger(), "飞控桥接C++节点已启动，等待 /relative_pose 数据...");
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
    // termios 不能直接使用整数 115200，需要转换成 B115200 这种常量。
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
    // 只提取绕 Z 轴的 yaw，飞控定位帧不使用 roll/pitch。
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static int16_t clampToS16(int value)
  {
    // 串口协议字段是 int16，必须限制到 -32768 到 32767。
    return static_cast<int16_t>(std::clamp(value, -32768, 32767));
  }

  static void putS16Be(std::array<uint8_t, 11> & frame, size_t index, int16_t value)
  {
    // 飞控解析端按“高字节在前”读取，所以这里手动拆成 big-endian。
    // 负数先转成 uint16_t 后再拆字节，可以保留二进制补码表示。
    const auto raw = static_cast<uint16_t>(value);
    frame[index] = static_cast<uint8_t>((raw >> 8) & 0xFF);
    frame[index + 1] = static_cast<uint8_t>(raw & 0xFF);
  }

  bool tryOpenSerial()
  {
    if (fd_ >= 0) {
      return true;
    }

    // O_NONBLOCK 避免串口暂时异常时卡住整个 ROS2 节点。
    const int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_DEBUG(
        get_logger(), "串口打开失败 %s: %s", port_.c_str(), std::strerror(errno));
      return false;
    }

    // 配置成原始串口模式：不做换行转换、不做奇偶校验、不做软件流控。
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

    // VMIN=0, VTIME=5 表示读操作最多等 0.5s。
    // 当前节点主要写串口，这里只是让串口配置完整。
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_WARN(
        get_logger(), "写入串口配置失败 %s: %s", port_.c_str(), std::strerror(errno));
      ::close(fd);
      return false;
    }

    // 有些 USB-TTL/飞控串口需要 DTR/RTS 拉起来才稳定。
    int modem_bits = 0;
    if (ioctl(fd, TIOCMGET, &modem_bits) == 0) {
      modem_bits |= TIOCM_DTR;
      modem_bits |= TIOCM_RTS;
      (void)ioctl(fd, TIOCMSET, &modem_bits);
    }

    // 清掉历史残留数据，避免刚连接时飞控收到半帧旧数据。
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

    // 串口掉线后不退出程序，而是后台每 2 秒重试一次。
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

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // 只保存最新坐标。真正发送由 sendCallback 按固定频率进行。
    current_pose_ = msg;
  }

  void validCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    localization_valid_ = msg->data;
    if (localization_valid_) {
      // 记录最近一次“定位有效”的时间，用于判断这个 true 是否已经过期。
      last_valid_time_ = Clock::now();
    }
  }

  void sendCallback()
  {
    // 发送前刷新可热调参数。
    refreshRuntimeParameters();

    if (fd_ < 0) {
      startReconnectTimer();
      return;
    }

    double x = 0.0;
    double y = 0.0;
    double yaw_deg = 0.0;
    std::string zero_reason;

    // 定位必须同时满足：
    //   1. 最近一次 /localization_valid 是 true
    //   2. 这个 true 没有超过 valid_timeout_sec_
    // 第二条是为了防止 relative_pose_node 卡住后，旧的 true 一直被沿用。
    const bool valid_recent =
      localization_valid_ &&
      last_valid_time_ &&
      elapsedSeconds(*last_valid_time_) <= valid_timeout_sec_;

    if (!current_pose_) {
      zero_reason = "尚未收到 /relative_pose，发送 0cm 心跳帧";
    } else if (!valid_recent) {
      zero_reason = "定位状态无效，发送 0cm 心跳帧";
    } else {
      // ROS 坐标单位是米；yaw 是四元数，需要转成角度。
      x = current_pose_->pose.position.x;
      y = current_pose_->pose.position.y;
      yaw_deg = radiansToDegrees(yawFromQuaternion(current_pose_->pose.orientation));
    }

    // 第一道发送保护：坐标必须是正常数字。
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw_deg)) {
      zero_reason = "收到非法坐标 (NaN/Inf)，发送 0cm 心跳帧";
      x = 0.0;
      y = 0.0;
      yaw_deg = 0.0;
    }

    // 第二道发送保护：限制最大 XY 坐标。
    // 如果 clamp_xy_instead_of_zero_ 为 true 且定位仍有效，则限幅；
    // 否则发 0cm 心跳帧，避免飞控追一个离谱目标。
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

    // 单位转换：
    //   米 -> 厘米，double -> int16
    //   yaw 角度四舍五入到整数度
    const int16_t x_cm = clampToS16(static_cast<int>(x * 100.0));
    const int16_t y_cm = clampToS16(static_cast<int>(y * 100.0));
    const int16_t yaw = clampToS16(static_cast<int>(std::lround(yaw_deg)));

    // 按匿名飞控定位帧协议组包。
    std::array<uint8_t, 11> frame {};
    frame[0] = 0xAA;
    frame[1] = 0xFF;
    frame[2] = 0x01;
    frame[3] = 0x06;
    putS16Be(frame, 4, x_cm);
    putS16Be(frame, 6, y_cm);
    putS16Be(frame, 8, yaw);

    // 校验和：前 10 个字节相加，溢出部分自然丢掉，只保留低 8 位。
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

    // 零帧/限幅原因最多每秒打印一次，防止定位无效时刷屏。
    const auto now = Clock::now();
    if (!zero_reason.empty() &&
      (zero_reason != last_zero_reason_ || !last_invalid_log_time_ ||
      std::chrono::duration<double>(now - *last_invalid_log_time_).count() >= 1.0))
    {
      last_invalid_log_time_ = now;
      last_zero_reason_ = zero_reason;
      RCLCPP_WARN(get_logger(), "%s", zero_reason.c_str());
    }

    // 正常发送日志每秒打印一次，既能看到数值，也能看到原始十六进制帧。
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

  std::string port_{"/dev/ttyFC"};
  int baudrate_{115200};
  double send_freq_{20.0};
  double max_xy_meters_{10.0};
  bool clamp_xy_instead_of_zero_{true};
  double valid_timeout_sec_{0.5};
  int fd_{-1};

  bool localization_valid_{false};
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_;
  std::optional<Clock::time_point> last_valid_time_;
  std::optional<Clock::time_point> last_invalid_log_time_;
  std::optional<Clock::time_point> last_log_time_;
  std::string last_zero_reason_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  void refreshRuntimeParameters()
  {
    // 这些参数允许飞行前/调试时热修改，不需要重启节点。
    // 串口号、波特率、发送频率没有放这里，因为运行中修改会涉及重开串口/重建定时器。
    max_xy_meters_ = std::max(0.0, get_parameter("max_xy_meters").as_double());
    clamp_xy_instead_of_zero_ = get_parameter("clamp_xy_instead_of_zero").as_bool();
    valid_timeout_sec_ = std::max(0.1, get_parameter("valid_timeout_sec").as_double());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FcBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
