#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unistd.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using Clock = std::chrono::steady_clock;
namespace fs = std::filesystem;

class DebugRecorderNode : public rclcpp::Node
{
public:
  DebugRecorderNode() : Node("debug_recorder_node")
  {
    base_dir_ = declare_parameter<std::string>(
      "base_dir", "/home/sunrise/project/debug_logs/lidar_fc");
    lidar_sample_stride_ = std::max(
      1, static_cast<int>(declare_parameter<int>("lidar_sample_stride", 100)));
    lidar_max_samples_per_frame_ = std::max(
      0, static_cast<int>(declare_parameter<int>("lidar_max_samples_per_frame", 200)));
    summary_freq_ = std::max(
      0.2, declare_parameter<double>("summary_freq", 1.0));

    openSessionFiles();

    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", 20,
      std::bind(&DebugRecorderNode::lidarCallback, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", 100,
      std::bind(&DebugRecorderNode::imuCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 50,
      std::bind(&DebugRecorderNode::odomCallback, this, std::placeholders::_1));
    rel_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/relative_pose", 20,
      std::bind(&DebugRecorderNode::relPoseCallback, this, std::placeholders::_1));
    valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/localization_valid", 20,
      std::bind(&DebugRecorderNode::validCallback, this, std::placeholders::_1));
    fc_frame_sub_ = create_subscription<std_msgs::msg::String>(
      "/fc_tx_frame_debug", 50,
      std::bind(&DebugRecorderNode::fcFrameCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / summary_freq_);
    summary_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DebugRecorderNode::summaryTimerCallback, this));

    RCLCPP_INFO(
      get_logger(), "debug recorder started, session dir: %s", session_dir_.c_str());
  }

  ~DebugRecorderNode() override
  {
    flushAll();
  }

private:
  static double stampToSec(const builtin_interfaces::msg::Time & stamp)
  {
    return rclcpp::Time(stamp).seconds();
  }

  static int64_t wallMs()
  {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
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

  static std::string nowText()
  {
    const auto now = std::chrono::system_clock::now();
    const std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm {};
    localtime_r(&tt, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
  }

  static std::string csvEscape(const std::string & value)
  {
    bool need_quote = false;
    std::string out;
    out.reserve(value.size());
    for (const char c : value) {
      if (c == '"' || c == ',' || c == '\n' || c == '\r') {
        need_quote = true;
      }
      if (c == '"') {
        out += "\"\"";
      } else {
        out += c;
      }
    }
    if (!need_quote) {
      return out;
    }
    return "\"" + out + "\"";
  }

  void openSessionFiles()
  {
    fs::create_directories(base_dir_);
    session_dir_ = base_dir_ + "/session_" + nowText() + "_pid" + std::to_string(getpid());
    fs::create_directories(session_dir_);

    const fs::path latest = fs::path(base_dir_) / "latest";
    std::error_code ec;
    if (fs::is_symlink(latest, ec)) {
      fs::remove(latest, ec);
    }
    fs::create_directory_symlink(session_dir_, latest, ec);

    lidar_summary_.open(session_dir_ + "/lidar_summary.csv", std::ios::out | std::ios::app);
    lidar_xy_.open(session_dir_ + "/lidar_xy.csv", std::ios::out | std::ios::app);
    fastlio_.open(session_dir_ + "/fastlio.csv", std::ios::out | std::ios::app);
    fc_frames_.open(session_dir_ + "/fc_frames.csv", std::ios::out | std::ios::app);
    readme_.open(session_dir_ + "/README.txt", std::ios::out | std::ios::trunc);

    lidar_summary_
      << "wall_ms,stamp_sec,seq,point_num,frame_dt_sec,scan_ms,valid_points,zero_points,"
      << "mean_x,mean_y,mean_z,min_range,min_x,min_y,min_z,front_min_range,front_min_x,"
      << "front_min_y,front_min_z\n";
    lidar_xy_
      << "wall_ms,stamp_sec,seq,point_index,offset_time,line,tag,x,y,z,range\n";
    fastlio_
      << "wall_ms,stamp_sec,event,lidar_hz,imu_hz,odom_hz,lidar_points,lidar_scan_ms,"
      << "imu_dt_sec,imu_max_gap_sec,imu_acc_norm,imu_gyro_norm_dps,odom_x,odom_y,odom_z,"
      << "odom_yaw_deg,odom_dt_sec,odom_step_xy,odom_speed_mps,odom_yaw_rate_dps,"
      << "relative_x,relative_y,relative_yaw_deg,relative_dist,localization_valid,cause\n";
    fc_frames_
      << "wall_ms,raw_message\n";

    readme_
      << "This directory is one lidar/FAST-LIO/FC debug session.\n\n"
      << "Files:\n"
      << "- lidar_summary.csv: one row per /livox/lidar frame, raw radar statistics.\n"
      << "- lidar_xy.csv: sampled raw radar point coordinates from /livox/lidar.\n"
      << "- fastlio.csv: /Odometry, /livox/imu, /relative_pose and localization health.\n"
      << "- fc_frames.csv: actual frames published by fc_bridge after serial write.\n\n"
      << "Copy latest session back to Mac:\n"
      << "scp -r sunrise@172.20.10.2:/home/sunrise/project/debug_logs/lidar_fc/latest ./lidar_debug_latest\n\n"
      << "Important columns:\n"
      << "- wall_ms: board system time in milliseconds since epoch.\n"
      << "- stamp_sec: ROS message timestamp in seconds.\n"
      << "- lidar_xy.csv x/y/z: raw point coordinates in Livox lidar frame.\n"
      << "- fc_frames.csv raw_message: frame type, cm values, yaw, hex bytes, and reason.\n"
      << "- fastlio.csv cause: quick diagnosis computed from current window.\n";
    readme_.close();
  }

  void lidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    const int64_t now = wallMs();
    const double stamp = stampToSec(msg->header.stamp);
    ++lidar_seq_;
    ++lidar_frames_window_;
    last_lidar_wall_time_ = Clock::now();

    if (last_lidar_stamp_) {
      lidar_frame_dt_sec_ = stamp - *last_lidar_stamp_;
    }
    last_lidar_stamp_ = stamp;

    size_t zero_points = 0;
    size_t valid_points = 0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    double min_range = std::numeric_limits<double>::infinity();
    double min_x = 0.0;
    double min_y = 0.0;
    double min_z = 0.0;
    double front_min_range = std::numeric_limits<double>::infinity();
    double front_min_x = 0.0;
    double front_min_y = 0.0;
    double front_min_z = 0.0;
    uint32_t min_offset = std::numeric_limits<uint32_t>::max();
    uint32_t max_offset = 0;
    int samples_written = 0;

    for (size_t i = 0; i < msg->points.size(); ++i) {
      const auto & p = msg->points[i];
      min_offset = std::min(min_offset, p.offset_time);
      max_offset = std::max(max_offset, p.offset_time);

      const double range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      if (range < 1.0e-6) {
        ++zero_points;
        continue;
      }
      ++valid_points;
      sum_x += p.x;
      sum_y += p.y;
      sum_z += p.z;
      if (range < min_range) {
        min_range = range;
        min_x = p.x;
        min_y = p.y;
        min_z = p.z;
      }

      if (p.x > 0.0 && std::fabs(p.y) < 1.0 && std::fabs(p.z) < 1.0 && range < front_min_range) {
        front_min_range = range;
        front_min_x = p.x;
        front_min_y = p.y;
        front_min_z = p.z;
      }

      if (lidar_max_samples_per_frame_ > 0 &&
        i % static_cast<size_t>(lidar_sample_stride_) == 0 &&
        samples_written < lidar_max_samples_per_frame_)
      {
        lidar_xy_ << now << ',' << std::fixed << std::setprecision(9) << stamp << ','
                  << lidar_seq_ << ',' << i << ',' << p.offset_time << ','
                  << static_cast<int>(p.line) << ',' << static_cast<int>(p.tag) << ','
                  << std::setprecision(6) << p.x << ',' << p.y << ',' << p.z << ','
                  << range << '\n';
        ++samples_written;
      }
    }

    lidar_point_count_ = msg->points.size();
    lidar_scan_ms_ = msg->points.empty() ? 0.0 :
      static_cast<double>(max_offset - min_offset) / 1.0e6;

    const double denom = std::max<size_t>(valid_points, 1);
    lidar_summary_ << now << ',' << std::fixed << std::setprecision(9) << stamp << ','
                   << lidar_seq_ << ',' << msg->points.size() << ','
                   << lidar_frame_dt_sec_.value_or(0.0) << ','
                   << std::setprecision(3) << lidar_scan_ms_ << ','
                   << valid_points << ',' << zero_points << ','
                   << std::setprecision(6) << sum_x / denom << ',' << sum_y / denom << ','
                   << sum_z / denom << ','
                   << finiteOrZero(min_range) << ',' << min_x << ',' << min_y << ',' << min_z << ','
                   << finiteOrZero(front_min_range) << ',' << front_min_x << ','
                   << front_min_y << ',' << front_min_z << '\n';
  }

  static double finiteOrZero(double value)
  {
    return std::isfinite(value) ? value : 0.0;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    ++imu_msgs_window_;
    last_imu_wall_time_ = Clock::now();
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
    ++odom_msgs_window_;
    last_odom_wall_time_ = Clock::now();
    const int64_t now = wallMs();
    const double stamp = stampToSec(msg->header.stamp);
    const auto & pos = msg->pose.pose.position;
    const double yaw_deg = radiansToDegrees(yawFromQuaternion(msg->pose.pose.orientation));

    double dt = 0.0;
    double step_xy = 0.0;
    double speed = 0.0;
    double yaw_rate = 0.0;
    if (last_odom_stamp_) {
      dt = stamp - *last_odom_stamp_;
      if (dt > 1.0e-6) {
        step_xy = std::hypot(pos.x - odom_x_, pos.y - odom_y_);
        speed = step_xy / dt;
        yaw_rate = std::fabs((yaw_deg - odom_yaw_deg_) / dt);
        odom_max_step_window_ = std::max(odom_max_step_window_, step_xy);
        odom_max_speed_window_ = std::max(odom_max_speed_window_, speed);
        odom_max_yaw_rate_window_ = std::max(odom_max_yaw_rate_window_, yaw_rate);
      }
    }

    last_odom_stamp_ = stamp;
    odom_x_ = pos.x;
    odom_y_ = pos.y;
    odom_z_ = pos.z;
    odom_yaw_deg_ = yaw_deg;

    fastlio_ << now << ',' << std::fixed << std::setprecision(9) << stamp
             << ",odom,,,,,,,,,,"
             << std::setprecision(6) << odom_x_ << ',' << odom_y_ << ',' << odom_z_ << ','
             << odom_yaw_deg_ << ',' << dt << ',' << step_xy << ',' << speed << ','
             << yaw_rate << ',' << rel_x_ << ',' << rel_y_ << ',' << rel_yaw_deg_ << ','
             << rel_dist_ << ',' << (localization_valid_ ? 1 : 0) << ','
             << csvEscape(currentCause()) << '\n';
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

  void fcFrameCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    fc_frames_ << wallMs() << ',' << csvEscape(msg->data) << '\n';
  }

  void summaryTimerCallback()
  {
    const int64_t now = wallMs();
    const double window_sec = std::max(0.001, elapsedSeconds(window_start_));
    const double lidar_hz = static_cast<double>(lidar_frames_window_) / window_sec;
    const double imu_hz = static_cast<double>(imu_msgs_window_) / window_sec;
    const double odom_hz = static_cast<double>(odom_msgs_window_) / window_sec;

    fastlio_ << now << ",0.000000000,summary,"
             << std::fixed << std::setprecision(3)
             << lidar_hz << ',' << imu_hz << ',' << odom_hz << ','
             << lidar_point_count_ << ',' << lidar_scan_ms_ << ','
             << imu_dt_sec_.value_or(0.0) << ',' << imu_max_gap_window_ << ','
             << imu_acc_norm_ << ',' << imu_gyro_norm_dps_ << ','
             << std::setprecision(6) << odom_x_ << ',' << odom_y_ << ',' << odom_z_ << ','
             << odom_yaw_deg_ << ",0,"
             << odom_max_step_window_ << ',' << odom_max_speed_window_ << ','
             << odom_max_yaw_rate_window_ << ',' << rel_x_ << ',' << rel_y_ << ','
             << rel_yaw_deg_ << ',' << rel_dist_ << ','
             << (localization_valid_ ? 1 : 0) << ',' << csvEscape(currentCause()) << '\n';

    flushAll();
    resetWindow();
  }

  std::string currentCause() const
  {
    const bool lidar_missing = !last_lidar_wall_time_ || elapsedSeconds(*last_lidar_wall_time_) > 1.0;
    const bool imu_missing = !last_imu_wall_time_ || elapsedSeconds(*last_imu_wall_time_) > 1.0;
    const bool odom_missing = !last_odom_wall_time_ || elapsedSeconds(*last_odom_wall_time_) > 1.0;
    const bool rel_missing = !last_rel_wall_time_ || elapsedSeconds(*last_rel_wall_time_) > 1.0;

    if (lidar_missing) {
      return "lidar missing";
    }
    if (imu_missing) {
      return "imu missing";
    }
    if (odom_missing) {
      return "FAST-LIO odom missing";
    }
    if (rel_missing || !localization_valid_) {
      return "relative pose invalid or blocked";
    }
    if (lidar_point_count_ < 15000) {
      return "too few lidar points";
    }
    if (lidar_scan_ms_ < 70.0 || lidar_scan_ms_ > 130.0) {
      return "lidar scan time abnormal";
    }
    if (imu_max_gap_window_ > 0.03) {
      return "imu gap too large";
    }
    if (odom_max_step_window_ > 0.25 || odom_max_speed_window_ > 1.0) {
      return "FAST-LIO odom jump or speed abnormal";
    }
    if (odom_max_yaw_rate_window_ > 60.0) {
      return "FAST-LIO yaw rate abnormal";
    }
    return "OK";
  }

  void resetWindow()
  {
    window_start_ = Clock::now();
    lidar_frames_window_ = 0;
    imu_msgs_window_ = 0;
    odom_msgs_window_ = 0;
    imu_max_gap_window_ = 0.0;
    odom_max_step_window_ = 0.0;
    odom_max_speed_window_ = 0.0;
    odom_max_yaw_rate_window_ = 0.0;
  }

  void flushAll()
  {
    lidar_summary_.flush();
    lidar_xy_.flush();
    fastlio_.flush();
    fc_frames_.flush();
  }

  std::string base_dir_;
  std::string session_dir_;
  int lidar_sample_stride_{100};
  int lidar_max_samples_per_frame_{200};
  double summary_freq_{1.0};

  std::ofstream lidar_summary_;
  std::ofstream lidar_xy_;
  std::ofstream fastlio_;
  std::ofstream fc_frames_;
  std::ofstream readme_;

  uint64_t lidar_seq_{0};
  size_t lidar_point_count_{0};
  double lidar_scan_ms_{0.0};
  std::optional<double> last_lidar_stamp_;
  std::optional<double> lidar_frame_dt_sec_;
  std::optional<Clock::time_point> last_lidar_wall_time_;
  std::optional<Clock::time_point> last_imu_wall_time_;
  std::optional<Clock::time_point> last_odom_wall_time_;
  std::optional<Clock::time_point> last_rel_wall_time_;
  std::optional<Clock::time_point> last_valid_wall_time_;

  Clock::time_point window_start_{Clock::now()};
  int lidar_frames_window_{0};
  int imu_msgs_window_{0};
  int odom_msgs_window_{0};

  std::optional<double> last_imu_stamp_;
  std::optional<double> imu_dt_sec_;
  double imu_max_gap_window_{0.0};
  double imu_acc_norm_{0.0};
  double imu_gyro_norm_dps_{0.0};

  std::optional<double> last_odom_stamp_;
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_z_{0.0};
  double odom_yaw_deg_{0.0};
  double odom_max_step_window_{0.0};
  double odom_max_speed_window_{0.0};
  double odom_max_yaw_rate_window_{0.0};

  double rel_x_{0.0};
  double rel_y_{0.0};
  double rel_yaw_deg_{0.0};
  double rel_dist_{0.0};
  bool localization_valid_{false};

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rel_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fc_frame_sub_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DebugRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
