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

// 这个节点是雷达定位链路的核心：
//   输入：FAST-LIO 发布的 /Odometry，表示雷达在 FAST-LIO map 坐标系里的位姿。
//   输出：/relative_pose，表示无人机从“开机位置”开始算的相对位姿。
//        /position_error，表示当前相对位置到目标点的误差。
//        /localization_valid，表示当前定位是否可信，给飞控桥接节点做保护。
//
// 最重要的设计：
//   1. 第一次正常 /Odometry 被当作原点，避免直接使用 FAST-LIO 的全局 map 数值。
//   2. 默认把“开机时机头方向”当作 +X，这样比赛里更符合人的直觉。
//   3. 对 NaN、坐标跳变、超范围、数据超时做保护，定位不可信时不让飞控吃真实坐标。
class RelativePoseNode : public rclcpp::Node
{
public:
  RelativePoseNode() : Node("relative_pose_node")
  {
    // 目标点，单位是米。现在主要用于发布 /position_error 和日志显示。
    // 可以运行时 ros2 param set 修改。
    target_x_ = declare_parameter<double>("target_x", 0.0);
    target_y_ = declare_parameter<double>("target_y", 0.0);
    target_z_ = declare_parameter<double>("target_z", 0.0);

    // 日志打印频率，不影响 /relative_pose 的发布频率。
    print_freq_ = declare_parameter<double>("print_freq", 1.0);

    // 连续两帧 /Odometry 的 XY 位移如果超过这个值，认为 FAST-LIO 跳变。
    // 室内小飞机正常运动不可能一帧跳几米，所以默认 5m 是偏宽松的保护。
    max_position_jump_ = declare_parameter<double>("max_position_jump", 5.0);

    // 连续异常达到这个次数，才重置原点。这样可以容忍偶发一帧坏数据。
    max_consecutive_errors_ = declare_parameter<int>("max_consecutive_errors", 5);

    // 第一次拿到 /Odometry 时，如果坐标绝对值过大，不设为原点。
    // 这是为了挡住 FAST-LIO 刚启动时偶发的离谱初值。
    init_max_meters_ = declare_parameter<double>("init_max_meters", 50.0);

    // 相对原点的水平距离超过这个范围，就认为不可信，不继续发布给飞控。
    max_relative_meters_ = declare_parameter<double>("max_relative_meters", 10.0);

    // 原点刚设定后先等一小段时间再标记有效，避免刚启动的定位抖动直接进飞控。
    valid_after_sec_ = declare_parameter<double>("valid_after_sec", 2.0);

    // 超过这个时间没有新的 /Odometry，就认为定位断流。
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 1.0);

    // true：把开机机头方向旋转成相对坐标系 +X。
    // false：只减掉开机位置，不旋转坐标轴，保留 FAST-LIO map 方向。
    use_initial_heading_frame_ = declare_parameter<bool>("use_initial_heading_frame", true);

    // yaw_offset_deg 用于现场微调安装角度。例如雷达坐标和机头差 90 度，就填 90。
    yaw_offset_ = degreesToRadians(declare_parameter<double>("yaw_offset_deg", 0.0));

    // 下面三个开关是坐标方向修正。现场发现 X/Y 反了、左右反了，不用改代码，
    // 直接改参数即可。
    swap_xy_ = declare_parameter<bool>("swap_xy", false);
    invert_x_ = declare_parameter<bool>("invert_x", false);
    invert_y_ = declare_parameter<bool>("invert_y", false);

    // FAST-LIO 输出的里程计。它是本节点唯一的定位输入。
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", 10,
      std::bind(&RelativePoseNode::odomCallback, this, std::placeholders::_1));

    // 给飞控桥接节点使用的相对坐标。
    rel_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/relative_pose", 10);

    // 给上层任务或调试使用：目标点 - 当前位置。
    error_pub_ = create_publisher<geometry_msgs::msg::Point>("/position_error", 10);

    // 安全标志：false 时 fc_bridge_node 会发 0cm 心跳帧，不发送真实位置。
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
    // 四元数转平面 yaw。无人机水平定位只需要绕 Z 轴角度。
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
    // 把角度压回 [-pi, pi]，避免出现 370 度这类绕圈后的值。
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  void applyAxisOptions(double & x, double & y) const
  {
    // 现场最常见的问题是坐标方向和机体方向不一致。
    // 这里集中处理换轴、反向，避免后面控制代码里到处写负号。
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
    // valid 状态发生变化时才重点打日志，避免屏幕被刷满。
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
    // 每次都会发布当前状态，fc_bridge_node 用它判断是否能发送真实坐标。
    valid_pub_->publish(msg);
  }

  void logWarnThrottle(const std::string & message)
  {
    // 异常时最多每秒打印一次，防止坏数据高频出现时日志太多看不清。
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
    // 允许比赛现场用 ros2 param set 热调部分保护参数，不用重启节点。
    refreshRuntimeParameters();
    last_odom_time_ = Clock::now();
    const auto & pos = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    const double current_yaw = yawFromQuaternion(q);

    // 第一层保护：坐标必须是正常数字。
    // NaN/Inf 一旦进入飞控，PID 会直接炸，所以这里坚决丢弃。
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
      // 还没有原点时，第一帧正常里程计会成为原点。
      // 但如果第一帧本身非常离谱，就先拒绝，继续等下一帧。
      if (std::fabs(pos.x) > init_max_meters_ || std::fabs(pos.y) > init_max_meters_) {
        logWarnThrottle(
          "首次 Odometry 值过大，拒绝设定原点: x=" + std::to_string(pos.x) +
          ", y=" + std::to_string(pos.y));
        publishValid(false);
        return;
      }

      // 记录开机位置和开机 yaw。
      // 后面所有相对坐标都用“当前值 - 原点值”算出来。
      origin_x_ = pos.x;
      origin_y_ = pos.y;
      origin_z_ = pos.z;

      // yaw_offset_ 用于补偿雷达安装方向和机头方向的固定偏差。
      origin_yaw_ = current_yaw + yaw_offset_;
      origin_set_ = true;

      // 记录上一帧位置，后面用来判断是否跳变。
      last_x_ = pos.x;
      last_y_ = pos.y;
      last_z_ = pos.z;
      last_pos_set_ = true;
      error_count_ = 0;
      origin_set_time_ = Clock::now();

      // 刚设定原点时先发布 false，等 valid_after_sec_ 秒后再恢复 true。
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

      // 第二层保护：相邻两帧的水平位移不能突然太大。
      // 这主要是防 FAST-LIO 偶发跳点。异常帧直接 return，不更新 last_x_/last_y_，
      // 这样下一帧会继续跟最后一帧正常数据比较。
      const double jump = std::hypot(pos.x - last_x_, pos.y - last_y_);
      if (jump > max_position_jump_) {
        ++error_count_;
        logWarnThrottle(
          "Odometry 跳变过大: " + std::to_string(jump) + "m (阈值 " +
          std::to_string(max_position_jump_) + "m), 连续异常 " +
          std::to_string(error_count_) + "/" + std::to_string(max_consecutive_errors_));

        if (error_count_ >= max_consecutive_errors_) {
          // 连续异常说明定位链路可能已经漂了，重置原点，等待重新稳定。
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

    // 先在 FAST-LIO map 坐标系里减掉开机原点，得到“相对开机位置”的平移。
    map_rel_x_ = pos.x - origin_x_;
    map_rel_y_ = pos.y - origin_y_;

    double rel_x = map_rel_x_;
    double rel_y = map_rel_y_;
    if (use_initial_heading_frame_) {
      // 把 FAST-LIO map 坐标旋转到“开机机头为 +X”的坐标系。
      // 公式等价于用 -origin_yaw_ 旋转相对位移：
      //   rel_x =  map_x*cos + map_y*sin
      //   rel_y = -map_x*sin + map_y*cos
      const double cos_yaw = std::cos(origin_yaw_);
      const double sin_yaw = std::sin(origin_yaw_);
      rel_x = map_rel_x_ * cos_yaw + map_rel_y_ * sin_yaw;
      rel_y = -map_rel_x_ * sin_yaw + map_rel_y_ * cos_yaw;
    }

    applyAxisOptions(rel_x, rel_y);
    rel_x_ = rel_x;
    rel_y_ = rel_y;
    rel_z_ = pos.z - origin_z_;

    // 第三层保护：相对原点跑得太远也不可信。
    // 对室内比赛来说，超过 max_relative_meters_ 基本就是定位漂移或坐标系错了。
    const double rel_dist = std::hypot(rel_x_, rel_y_);
    if (rel_dist > max_relative_meters_) {
      logWarnThrottle(
        "相对位移超出安全范围，跳过发布: " + std::to_string(rel_dist) +
        "m (max=" + std::to_string(max_relative_meters_) + "m)");
      handleError();
      publishValid(false);
      return;
    }

    // 当前 yaw 也转成“相对开机 yaw”。
    yaw_ = normalizeAngle(current_yaw - origin_yaw_);

    // 原点设定后一段时间才允许有效，避免启动瞬间抖动。
    const double stable_sec = origin_set_time_ ?
      elapsedSeconds(*origin_set_time_) : 0.0;
    publishValid(stable_sec >= valid_after_sec_);

    // 发布给 fc_bridge_node。这里 z 暂时发 0，因为当前飞控串口帧只使用 X/Y/YAW。
    geometry_msgs::msg::PoseStamped rel_pose;
    rel_pose.header = msg->header;
    rel_pose.header.frame_id = "relative_origin";
    rel_pose.pose.position.x = rel_x_;
    rel_pose.pose.position.y = rel_y_;
    rel_pose.pose.position.z = 0.0;
    rel_pose.pose.orientation = quaternionFromYaw(yaw_);
    rel_pose_pub_->publish(rel_pose);

    // 发布当前位置到目标点的误差，方便上层控制/调试。
    // 注意：fc_bridge_node 不使用 /position_error，它使用 /relative_pose。
    geometry_msgs::msg::Point error;
    error.x = target_x_ - rel_x_;
    error.y = target_y_ - rel_y_;
    error.z = 0.0;
    error_pub_->publish(error);
  }

  void handleError()
  {
    // 非法值、超范围等异常共用这一段计数逻辑。
    ++error_count_;
    if (error_count_ >= max_consecutive_errors_ && origin_set_) {
      RCLCPP_ERROR(get_logger(), "连续收到非法 Odometry，重置原点等待恢复...");
      resetOrigin();
      publishValid(false);
    }
  }

  void resetOrigin()
  {
    // 重置后节点会等待下一帧正常 /Odometry 重新设定原点。
    // 这比继续沿用已经漂掉的原点更安全。
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

    // 定时器除了打印日志，也负责检查“长时间没有 /Odometry”的情况。
    if (!origin_set_) {
      publishValid(false);
      RCLCPP_WARN(get_logger(), "尚未收到 /Odometry，请确认 FAST_LIO 已启动");
      return;
    }

    // 第四层保护：FAST-LIO 断流时，定位标记为无效。
    if (!last_odom_time_ || elapsedSeconds(*last_odom_time_) > odom_timeout_sec_) {
      publishValid(false);
      const double age = last_odom_time_ ? elapsedSeconds(*last_odom_time_) : -1.0;
      RCLCPP_WARN(
        get_logger(), "/Odometry 超时 %.1fs，定位状态无效，请检查 FAST_LIO 是否仍在发布",
        age);
      return;
    }

    // 这段只是人看的日志：当前相对位移、原始 map 位移、yaw、离目标多远。
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
    // 这些参数允许运行时修改。和坐标系方向相关的参数没有放这里，
    // 是为了避免飞行中突然换轴导致控制方向瞬间改变。
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
