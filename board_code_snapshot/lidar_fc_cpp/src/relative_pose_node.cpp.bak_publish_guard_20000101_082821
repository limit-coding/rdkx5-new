#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using Clock = std::chrono::steady_clock;

class RelativePoseNode : public rclcpp::Node
{
public:
  RelativePoseNode() : Node("relative_pose_node")
  {
    target_x_ = declare_parameter<double>("target_x", 0.0);
    target_y_ = declare_parameter<double>("target_y", 0.0);
    target_z_ = declare_parameter<double>("target_z", 0.0);
    print_freq_ = declare_parameter<double>("print_freq", 1.0);
    max_position_jump_ = declare_parameter<double>("max_position_jump", 5.0);
    max_consecutive_errors_ = declare_parameter<int>("max_consecutive_errors", 5);
    init_max_meters_ = declare_parameter<double>("init_max_meters", 50.0);
    max_relative_meters_ = declare_parameter<double>("max_relative_meters", 10.0);
    valid_after_sec_ = declare_parameter<double>("valid_after_sec", 2.0);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 1.0);
    use_initial_heading_frame_ = declare_parameter<bool>("use_initial_heading_frame", true);
    yaw_offset_ = degreesToRadians(declare_parameter<double>("yaw_offset_deg", 0.0));
    swap_xy_ = declare_parameter<bool>("swap_xy", false);
    invert_x_ = declare_parameter<bool>("invert_x", false);
    invert_y_ = declare_parameter<bool>("invert_y", false);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&RelativePoseNode::odomCallback, this, std::placeholders::_1));

    rel_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/relative_pose", 10);
    error_pub_ = create_publisher<geometry_msgs::msg::Point>("/position_error", 10);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("/localization_valid", 10);

    const double safe_print_freq = std::max(print_freq_, 0.1);
    const auto timer_period = std::chrono::duration<double>(1.0 / safe_print_freq);
    log_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&RelativePoseNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "相对定位C++节点已启动");
    RCLCPP_INFO(get_logger(), "等待 FAST_LIO /Odometry 数据以设定原点...");
    RCLCPP_INFO(
      get_logger(), "当前目标点: target=(%.3f, %.3f, %.3f)",
      target_x_, target_y_, target_z_);
    RCLCPP_INFO(
      get_logger(),
      "坐标模式: %s; swap_xy=%s, invert_x=%s, invert_y=%s, yaw_offset=%.1fdeg",
      use_initial_heading_frame_ ? "开机位置为原点，开机机头方向为 +X" :
      "开机位置为原点，FAST_LIO map 坐标原样输出",
      boolText(swap_xy_), boolText(invert_x_), boolText(invert_y_),
      radiansToDegrees(yaw_offset_));
  }

private:
  static const char * boolText(bool value)
  {
    return value ? "true" : "false";
  }

  static double degreesToRadians(double degrees)
  {
    return degrees * pi() / 180.0;
  }

  static double radiansToDegrees(double radians)
  {
    return radians * 180.0 / pi();
  }

  static constexpr double pi()
  {
    return 3.14159265358979323846;
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

  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void applyAxisOptions(double & x, double & y) const
  {
    if (swap_xy_) {
      std::swap(x, y);
    }
    if (invert_x_) {
      x = -x;
    }
    if (invert_y_) {
      y = -y;
    }
  }

  void publishValid(bool valid)
  {
    if (localization_valid_ != valid) {
      if (valid) {
        RCLCPP_INFO(get_logger(), "定位状态恢复有效，允许飞控桥接发送坐标");
      } else {
        RCLCPP_WARN(get_logger(), "定位状态无效，飞控桥接应停止发送真实坐标");
      }
    }

    localization_valid_ = valid;
    std_msgs::msg::Bool msg;
    msg.data = valid;
    valid_pub_->publish(msg);
  }

  void logWarnThrottle(const std::string & message)
  {
    const auto now = Clock::now();
    if (!last_error_log_time_ ||
      std::chrono::duration<double>(now - *last_error_log_time_).count() >= 1.0)
    {
      last_error_log_time_ = now;
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    refreshRuntimeParameters();
    last_odom_time_ = Clock::now();
    const auto & pos = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const double current_yaw = yawFromQuaternion(q);

    if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) {
      logWarnThrottle(
        "Odometry 包含非法值，跳过: x=" + std::to_string(pos.x) +
        ", y=" + std::to_string(pos.y) +
        ", z=" + std::to_string(pos.z));
      handleError();
      publishValid(false);
      return;
    }

    if (!origin_set_) {
      if (std::fabs(pos.x) > init_max_meters_ || std::fabs(pos.y) > init_max_meters_) {
        logWarnThrottle(
          "首次 Odometry 值过大，拒绝设定原点: x=" + std::to_string(pos.x) +
          ", y=" + std::to_string(pos.y));
        publishValid(false);
        return;
      }

      origin_x_ = pos.x;
      origin_y_ = pos.y;
      origin_z_ = pos.z;
      origin_yaw_ = current_yaw + yaw_offset_;
      origin_set_ = true;
      last_x_ = pos.x;
      last_y_ = pos.y;
      last_z_ = pos.z;
      last_pos_set_ = true;
      error_count_ = 0;
      origin_set_time_ = Clock::now();
      publishValid(false);
      RCLCPP_INFO(
        get_logger(), "原点已设定: (%.3f, %.3f), 开机yaw=%.1fdeg",
        origin_x_, origin_y_, radiansToDegrees(origin_yaw_));
    } else {
      if (!last_pos_set_) {
        last_x_ = pos.x;
        last_y_ = pos.y;
        last_z_ = pos.z;
        last_pos_set_ = true;
      }

      const double jump = std::hypot(pos.x - last_x_, pos.y - last_y_);
      if (jump > max_position_jump_) {
        ++error_count_;
        logWarnThrottle(
          "Odometry 跳变过大: " + std::to_string(jump) + "m (阈值 " +
          std::to_string(max_position_jump_) + "m), 连续异常 " +
          std::to_string(error_count_) + "/" + std::to_string(max_consecutive_errors_));

        if (error_count_ >= max_consecutive_errors_) {
          RCLCPP_ERROR(get_logger(), "连续异常次数过多，判定为定位失效，重置原点等待恢复...");
          resetOrigin();
          publishValid(false);
        }
        return;
      }

      if (error_count_ > 0) {
        RCLCPP_INFO(get_logger(), "Odometry 恢复正常");
      }
      error_count_ = 0;
      last_x_ = pos.x;
      last_y_ = pos.y;
      last_z_ = pos.z;
    }

    map_rel_x_ = pos.x - origin_x_;
    map_rel_y_ = pos.y - origin_y_;

    double rel_x = map_rel_x_;
    double rel_y = map_rel_y_;
    if (use_initial_heading_frame_) {
      const double cos_yaw = std::cos(origin_yaw_);
      const double sin_yaw = std::sin(origin_yaw_);
      rel_x = map_rel_x_ * cos_yaw + map_rel_y_ * sin_yaw;
      rel_y = -map_rel_x_ * sin_yaw + map_rel_y_ * cos_yaw;
    }

    applyAxisOptions(rel_x, rel_y);
    rel_x_ = rel_x;
    rel_y_ = rel_y;
    rel_z_ = pos.z - origin_z_;

    const double rel_dist = std::hypot(rel_x_, rel_y_);
    if (rel_dist > max_relative_meters_) {
      logWarnThrottle(
        "相对位移超出安全范围，跳过发布: " + std::to_string(rel_dist) +
        "m (max=" + std::to_string(max_relative_meters_) + "m)");
      handleError();
      publishValid(false);
      return;
    }

    yaw_ = normalizeAngle(current_yaw - origin_yaw_);

    const double stable_sec = origin_set_time_ ?
      elapsedSeconds(*origin_set_time_) : 0.0;
    publishValid(stable_sec >= valid_after_sec_);

    geometry_msgs::msg::PoseStamped rel_pose;
    rel_pose.header = msg->header;
    rel_pose.header.frame_id = "relative_origin";
    rel_pose.pose.position.x = rel_x_;
    rel_pose.pose.position.y = rel_y_;
    rel_pose.pose.position.z = 0.0;
    rel_pose.pose.orientation = quaternionFromYaw(yaw_);
    rel_pose_pub_->publish(rel_pose);

    geometry_msgs::msg::Point error;
    error.x = target_x_ - rel_x_;
    error.y = target_y_ - rel_y_;
    error.z = 0.0;
    error_pub_->publish(error);
  }

  void handleError()
  {
    ++error_count_;
    if (error_count_ >= max_consecutive_errors_ && origin_set_) {
      RCLCPP_ERROR(get_logger(), "连续收到非法 Odometry，重置原点等待恢复...");
      resetOrigin();
      publishValid(false);
    }
  }

  void resetOrigin()
  {
    origin_set_ = false;
    origin_x_ = 0.0;
    origin_y_ = 0.0;
    origin_z_ = 0.0;
    origin_yaw_ = 0.0;
    last_pos_set_ = false;
    origin_set_time_.reset();
    error_count_ = 0;
  }

  void timerCallback()
  {
    refreshRuntimeParameters();

    if (!origin_set_) {
      publishValid(false);
      RCLCPP_WARN(get_logger(), "尚未收到 /Odometry，请确认 FAST_LIO 已启动");
      return;
    }

    if (!last_odom_time_ || elapsedSeconds(*last_odom_time_) > odom_timeout_sec_) {
      publishValid(false);
      const double age = last_odom_time_ ? elapsedSeconds(*last_odom_time_) : -1.0;
      RCLCPP_WARN(
        get_logger(), "/Odometry 超时 %.1fs，定位状态无效，请检查 FAST_LIO 是否仍在发布",
        age);
      return;
    }

    const double dist = std::hypot(target_x_ - rel_x_, target_y_ - rel_y_);
    RCLCPP_INFO(
      get_logger(),
      "相对位移: dx=%+.3fm, dy=%+.3fm | map=(%+.3f,%+.3f)m | yaw=%.1fdeg | 到目标点距离=%.3fm",
      rel_x_, rel_y_, map_rel_x_, map_rel_y_, radiansToDegrees(yaw_), dist);

    if (dist < 0.05) {
      RCLCPP_INFO(get_logger(), ">>> 已到达目标点！<<<");
    }
  }

  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double print_freq_{1.0};
  double max_position_jump_{5.0};
  int max_consecutive_errors_{5};
  double init_max_meters_{50.0};
  double max_relative_meters_{10.0};
  double valid_after_sec_{2.0};
  double odom_timeout_sec_{1.0};
  bool use_initial_heading_frame_{true};
  double yaw_offset_{0.0};
  bool swap_xy_{false};
  bool invert_x_{false};
  bool invert_y_{false};

  bool origin_set_{false};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_z_{0.0};
  double origin_yaw_{0.0};
  std::optional<Clock::time_point> origin_set_time_;

  double rel_x_{0.0};
  double rel_y_{0.0};
  double rel_z_{0.0};
  double map_rel_x_{0.0};
  double map_rel_y_{0.0};
  double yaw_{0.0};

  bool last_pos_set_{false};
  double last_x_{0.0};
  double last_y_{0.0};
  double last_z_{0.0};
  int error_count_{0};
  bool localization_valid_{false};
  std::optional<Clock::time_point> last_error_log_time_;
  std::optional<Clock::time_point> last_odom_time_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rel_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
  rclcpp::TimerBase::SharedPtr log_timer_;

  void refreshRuntimeParameters()
  {
    target_x_ = get_parameter("target_x").as_double();
    target_y_ = get_parameter("target_y").as_double();
    target_z_ = get_parameter("target_z").as_double();
    max_position_jump_ = std::max(0.1, get_parameter("max_position_jump").as_double());
    max_consecutive_errors_ = std::max(
      1, static_cast<int>(get_parameter("max_consecutive_errors").as_int()));
    init_max_meters_ = std::max(1.0, get_parameter("init_max_meters").as_double());
    max_relative_meters_ = std::max(0.1, get_parameter("max_relative_meters").as_double());
    valid_after_sec_ = std::max(0.0, get_parameter("valid_after_sec").as_double());
    odom_timeout_sec_ = std::max(0.1, get_parameter("odom_timeout_sec").as_double());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelativePoseNode>());
  rclcpp::shutdown();
  return 0;
}
