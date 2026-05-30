#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"

using Clock = std::chrono::steady_clock;

class LioDebugNode : public rclcpp::Node
{
public:
  LioDebugNode() : Node("lio_debug_node")
  {
    print_freq_ = declare_parameter<double>("print_freq", 1.0);
    min_lidar_points_ = declare_parameter<int>("min_lidar_points", 15000);
    max_lidar_frame_dt_sec_ = declare_parameter<double>("max_lidar_frame_dt_sec", 0.20);
    min_lidar_scan_ms_ = declare_parameter<double>("min_lidar_scan_ms", 70.0);
    max_lidar_scan_ms_ = declare_parameter<double>("max_lidar_scan_ms", 130.0);
    max_imu_gap_sec_ = declare_parameter<double>("max_imu_gap_sec", 0.03);
    max_odom_jump_m_ = declare_parameter<double>("max_odom_jump_m", 0.25);
    max_odom_speed_mps_ = declare_parameter<double>("max_odom_speed_mps", 1.0);
    max_yaw_rate_dps_ = declare_parameter<double>("max_yaw_rate_dps", 60.0);
    stationary_mode_ = declare_parameter<bool>("stationary_mode", true);
    max_stationary_drift_mps_ = declare_parameter<double>("max_stationary_drift_mps", 0.05);

    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", 20,
      std::bind(&LioDebugNode::lidarCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", 100,
      std::bind(&LioDebugNode::imuCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 20,
      std::bind(&LioDebugNode::odomCallback, this, std::placeholders::_1));
    rel_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/relative_pose", 20,
      std::bind(&LioDebugNode::relPoseCallback, this, std::placeholders::_1));
    valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/localization_valid", 20,
      std::bind(&LioDebugNode::validCallback, this, std::placeholders::_1));

    resetReportWindow();
    const double safe_print_freq = std::max(print_freq_, 0.1);
    const auto timer_period = std::chrono::duration<double>(1.0 / safe_print_freq);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&LioDebugNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "LIO debug node started. Watching /livox/lidar /livox/imu /Odometry /relative_pose.");
  }

private:
  static double stampToSec(const builtin_interfaces::msg::Time & stamp)
  {
    return rclcpp::Time(stamp).seconds();
  }

  static double elapsedSeconds(Clock::time_point start)
  {
    return std::chrono::duration<double>(Clock::now() - start).count();
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double radiansToDegrees(double radians)
  {
    return radians * 180.0 / 3.14159265358979323846;
  }

  static const char * yesNo(bool value)
  {
    return value ? "yes" : "no";
  }

  void resetReportWindow()
  {
    window_start_ = Clock::now();
    lidar_frames_window_ = 0;
    imu_msgs_window_ = 0;
    odom_msgs_window_ = 0;
    lidar_bad_offset_window_ = 0;
    imu_max_gap_window_ = 0.0;
    odom_max_step_window_ = 0.0;
    odom_max_speed_window_ = 0.0;
    odom_max_yaw_rate_window_ = 0.0;
  }

  void lidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    last_lidar_wall_time_ = Clock::now();
    ++lidar_frames_window_;

    const double stamp = stampToSec(msg->header.stamp);
    if (last_lidar_stamp_) {
      lidar_frame_dt_sec_ = stamp - *last_lidar_stamp_;
    }
    last_lidar_stamp_ = stamp;

    lidar_point_count_ = msg->points.size();
    lidar_scan_ms_ = 0.0;
    uint32_t min_offset = std::numeric_limits<uint32_t>::max();
    uint32_t max_offset = 0;
    bool offset_monotonic = true;
    uint32_t prev_offset = 0;
    bool first = true;

    for (const auto & point : msg->points) {
      min_offset = std::min(min_offset, point.offset_time);
      max_offset = std::max(max_offset, point.offset_time);
      if (!first && point.offset_time < prev_offset) {
        offset_monotonic = false;
      }
      first = false;
      prev_offset = point.offset_time;
    }

    if (!msg->points.empty()) {
      lidar_scan_ms_ = static_cast<double>(max_offset - min_offset) / 1.0e6;
    }
    lidar_offsets_monotonic_ = offset_monotonic;
    if (lidar_scan_ms_ < min_lidar_scan_ms_ ||
      lidar_scan_ms_ > max_lidar_scan_ms_)
    {
      ++lidar_bad_offset_window_;
    }
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_wall_time_ = Clock::now();
    ++imu_msgs_window_;

    const double stamp = stampToSec(msg->header.stamp);
    if (last_imu_stamp_) {
      imu_dt_sec_ = stamp - *last_imu_stamp_;
      if (imu_dt_sec_ > 0.0) {
        imu_max_gap_window_ = std::max(imu_max_gap_window_, *imu_dt_sec_);
      }
    }
    last_imu_stamp_ = stamp;

    const auto & a = msg->linear_acceleration;
    const auto & g = msg->angular_velocity;
    imu_acc_norm_ = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    imu_gyro_norm_dps_ = radiansToDegrees(std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z));
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_wall_time_ = Clock::now();
    ++odom_msgs_window_;

    const double stamp = stampToSec(msg->header.stamp);
    const auto & pos = msg->pose.pose.position;
    const double yaw = yawFromQuaternion(msg->pose.pose.orientation);

    if (last_odom_stamp_) {
      const double dt = stamp - *last_odom_stamp_;
      if (dt > 1.0e-6) {
        const double step_xy = std::hypot(pos.x - last_odom_x_, pos.y - last_odom_y_);
        const double speed = step_xy / dt;
        const double yaw_rate = std::fabs(radiansToDegrees(normalizeAngle(yaw - last_odom_yaw_)) / dt);
        odom_step_xy_ = step_xy;
        odom_speed_mps_ = speed;
        odom_yaw_rate_dps_ = yaw_rate;
        odom_max_step_window_ = std::max(odom_max_step_window_, step_xy);
        odom_max_speed_window_ = std::max(odom_max_speed_window_, speed);
        odom_max_yaw_rate_window_ = std::max(odom_max_yaw_rate_window_, yaw_rate);
      }
    }

    last_odom_stamp_ = stamp;
    last_odom_x_ = pos.x;
    last_odom_y_ = pos.y;
    last_odom_z_ = pos.z;
    last_odom_yaw_ = yaw;
  }

  void relPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_rel_wall_time_ = Clock::now();
    rel_x_ = msg->pose.position.x;
    rel_y_ = msg->pose.position.y;
    rel_yaw_deg_ = radiansToDegrees(yawFromQuaternion(msg->pose.orientation));
    rel_dist_ = std::hypot(rel_x_, rel_y_);
  }

  void validCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    localization_valid_ = msg->data;
    last_valid_wall_time_ = Clock::now();
  }

  std::string missingSources() const
  {
    std::string missing;
    if (!last_lidar_wall_time_) {
      missing += " lidar";
    }
    if (!last_imu_wall_time_) {
      missing += " imu";
    }
    if (!last_odom_wall_time_) {
      missing += " odom";
    }
    if (!last_rel_wall_time_) {
      missing += " relative_pose";
    }
    return missing.empty() ? "none" : missing;
  }

  std::string likelyCause() const
  {
    const bool lidar_missing = !last_lidar_wall_time_ || elapsedSeconds(*last_lidar_wall_time_) > 1.0;
    const bool imu_missing = !last_imu_wall_time_ || elapsedSeconds(*last_imu_wall_time_) > 1.0;
    const bool lidar_time_bad =
      lidar_bad_offset_window_ > 0 ||
      lidar_scan_ms_ < min_lidar_scan_ms_ ||
      lidar_scan_ms_ > max_lidar_scan_ms_;
    const bool imu_gap_bad = imu_max_gap_window_ > max_imu_gap_sec_;
    const bool odom_jump_bad =
      odom_max_step_window_ > max_odom_jump_m_ ||
      odom_max_speed_window_ > max_odom_speed_mps_;
    const bool yaw_bad = odom_max_yaw_rate_window_ > max_yaw_rate_dps_;
    const bool stationary_bad =
      stationary_mode_ && odom_max_speed_window_ > max_stationary_drift_mps_;

    if (lidar_missing) {
      return "lidar data missing or network/driver stalled";
    }
    if (imu_missing) {
      return "imu data missing; FAST-LIO cannot deskew reliably";
    }
    if (lidar_time_bad) {
      return "lidar point offset_time abnormal; check xfer_format/custom msg/timestamp";
    }
    if (imu_gap_bad) {
      return "imu timestamp gap; check livox imu stream/cpu load/network";
    }
    if (odom_jump_bad && !lidar_time_bad && !imu_gap_bad) {
      return "FAST-LIO odometry jump; likely vibration, weak geometry, or bad extrinsic";
    }
    if (yaw_bad) {
      return "yaw rate spike; check vibration/IMU mounting";
    }
    if (stationary_bad) {
      return "stationary drift too high; check rigidity, vibration, extrinsic, environment";
    }
    return "no obvious fault in this window";
  }

  void timerCallback()
  {
    const double window_sec = std::max(0.001, elapsedSeconds(window_start_));
    const double lidar_hz = static_cast<double>(lidar_frames_window_) / window_sec;
    const double imu_hz = static_cast<double>(imu_msgs_window_) / window_sec;
    const double odom_hz = static_cast<double>(odom_msgs_window_) / window_sec;

    const bool lidar_ok =
      last_lidar_wall_time_ &&
      elapsedSeconds(*last_lidar_wall_time_) <= 1.0 &&
      static_cast<int>(lidar_point_count_) >= min_lidar_points_ &&
      (!lidar_frame_dt_sec_ || *lidar_frame_dt_sec_ <= max_lidar_frame_dt_sec_) &&
      lidar_bad_offset_window_ == 0;
    const bool imu_ok =
      last_imu_wall_time_ &&
      elapsedSeconds(*last_imu_wall_time_) <= 1.0 &&
      imu_max_gap_window_ <= max_imu_gap_sec_;
    const bool odom_ok =
      last_odom_wall_time_ &&
      elapsedSeconds(*last_odom_wall_time_) <= 1.0 &&
      odom_max_step_window_ <= max_odom_jump_m_ &&
      odom_max_speed_window_ <= max_odom_speed_mps_ &&
      (!stationary_mode_ || odom_max_speed_window_ <= max_stationary_drift_mps_);

    RCLCPP_INFO(
      get_logger(),
      "DIAG lidar=%s hz=%.1f points=%zu frame_dt=%.3fs scan=%.1fms ordered=%s bad_scan=%d",
      lidar_ok ? "OK" : "WARN", lidar_hz, lidar_point_count_,
      lidar_frame_dt_sec_.value_or(-1.0), lidar_scan_ms_, yesNo(lidar_offsets_monotonic_),
      lidar_bad_offset_window_);
    RCLCPP_INFO(
      get_logger(),
      "DIAG imu=%s hz=%.1f dt=%.4fs max_gap=%.4fs acc=%.2fmps2 gyro=%.1fdps",
      imu_ok ? "OK" : "WARN", imu_hz, imu_dt_sec_.value_or(-1.0), imu_max_gap_window_,
      imu_acc_norm_, imu_gyro_norm_dps_);
    RCLCPP_INFO(
      get_logger(),
      "DIAG odom=%s hz=%.1f step=%.3fm max_step=%.3fm speed=%.3fmps yaw_rate=%.1fdps",
      odom_ok ? "OK" : "WARN", odom_hz, odom_step_xy_, odom_max_step_window_,
      odom_max_speed_window_, odom_max_yaw_rate_window_);
    RCLCPP_INFO(
      get_logger(),
      "DIAG relative valid=%s x=%.3fm y=%.3fm dist=%.3fm yaw=%.1fdeg missing=%s cause=%s",
      yesNo(localization_valid_), rel_x_, rel_y_, rel_dist_, rel_yaw_deg_,
      missingSources().c_str(), likelyCause().c_str());

    resetReportWindow();
  }

  double print_freq_{1.0};
  int min_lidar_points_{15000};
  double max_lidar_frame_dt_sec_{0.20};
  double min_lidar_scan_ms_{70.0};
  double max_lidar_scan_ms_{130.0};
  double max_imu_gap_sec_{0.03};
  double max_odom_jump_m_{0.25};
  double max_odom_speed_mps_{1.0};
  double max_yaw_rate_dps_{60.0};
  bool stationary_mode_{true};
  double max_stationary_drift_mps_{0.05};

  std::optional<Clock::time_point> last_lidar_wall_time_;
  std::optional<Clock::time_point> last_imu_wall_time_;
  std::optional<Clock::time_point> last_odom_wall_time_;
  std::optional<Clock::time_point> last_rel_wall_time_;
  std::optional<Clock::time_point> last_valid_wall_time_;
  Clock::time_point window_start_;

  std::optional<double> last_lidar_stamp_;
  std::optional<double> lidar_frame_dt_sec_;
  size_t lidar_point_count_{0};
  double lidar_scan_ms_{0.0};
  bool lidar_offsets_monotonic_{true};
  int lidar_frames_window_{0};
  int lidar_bad_offset_window_{0};

  std::optional<double> last_imu_stamp_;
  std::optional<double> imu_dt_sec_;
  int imu_msgs_window_{0};
  double imu_max_gap_window_{0.0};
  double imu_acc_norm_{0.0};
  double imu_gyro_norm_dps_{0.0};

  std::optional<double> last_odom_stamp_;
  int odom_msgs_window_{0};
  double last_odom_x_{0.0};
  double last_odom_y_{0.0};
  double last_odom_z_{0.0};
  double last_odom_yaw_{0.0};
  double odom_step_xy_{0.0};
  double odom_speed_mps_{0.0};
  double odom_yaw_rate_dps_{0.0};
  double odom_max_step_window_{0.0};
  double odom_max_speed_window_{0.0};
  double odom_max_yaw_rate_window_{0.0};

  bool localization_valid_{false};
  double rel_x_{0.0};
  double rel_y_{0.0};
  double rel_yaw_deg_{0.0};
  double rel_dist_{0.0};

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rel_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioDebugNode>());
  rclcpp::shutdown();
  return 0;
}
