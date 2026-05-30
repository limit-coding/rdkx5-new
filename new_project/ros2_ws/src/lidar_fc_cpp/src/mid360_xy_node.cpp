#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

// 这个节点只负责一件事：
//   把 Livox MID360 驱动发布的自定义点云 /livox/lidar
//   转成 ROS2 标准 PointCloud2 /mid360/xy_points。
//
// 为什么要单独转一次？
//   FAST-LIO 通常吃标准点云格式更方便；Livox 自定义消息里虽然也有 x/y/z，
//   但字段布局不是通用 PointCloud2，所以这里做一层“翻译”。
//
// 注意：这里不做避障、不做定位、不改坐标系，只保留每个点的 x/y/z。
class Mid360XYNode : public rclcpp::Node
{
public:
  Mid360XYNode() : Node("mid360_xy_node")
  {
    // MID360 原始点云。这个话题由 livox_ros_driver2 发布。
    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", 10,
      std::bind(&Mid360XYNode::lidarCallback, this, std::placeholders::_1));

    // 转换后的标准点云。后面的 FAST-LIO 或调试工具订阅这个话题。
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/mid360/xy_points", 10);

    // 每秒打印一次雷达状态，方便现场判断：有没有数据、点数是否正常、距离是否离谱。
    log_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Mid360XYNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "MID360 XY C++节点已启动，订阅 /livox/lidar");
  }

private:
  void lidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    // 保存最近一帧，timerCallback 里用它打印点数和平均距离。
    latest_msg_ = msg;

    // 如果当前没有任何订阅者，就不做格式转换，省一点 CPU。
    // 注意：如果 FAST-LIO 没启动，这里会直接 return，但 latest_msg_ 仍会更新，
    // 所以日志依然能显示“雷达是否有数据”。
    if (pointcloud_pub_->get_subscription_count() == 0) {
      return;
    }

    // 真正发布给 FAST-LIO 的标准点云。
    pointcloud_pub_->publish(customToPointCloud2(*msg));
  }

  sensor_msgs::msg::PointCloud2 customToPointCloud2(
    const livox_ros_driver2::msg::CustomMsg & msg) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    // 沿用原始雷达消息的时间戳和 frame_id，让下游能知道这一帧是什么时候采的。
    cloud.header = msg.header;

    // PointCloud2 支持二维排列的点云。这里用 height=1 表示“无组织点云”，
    // width 就是这一帧里点的数量。
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(msg.points.size());

    // 定义每个点里有哪些字段。这里每个点只有三个 float32：
    //   x offset=0, y offset=4, z offset=8
    // 一个 float32 是 4 字节，所以一个点总共 12 字节。
    sensor_msgs::msg::PointField field;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;

    field.name = "x";
    field.offset = 0;
    cloud.fields.push_back(field);

    field.name = "y";
    field.offset = 4;
    cloud.fields.push_back(field);

    field.name = "z";
    field.offset = 8;
    cloud.fields.push_back(field);

    cloud.is_bigendian = false;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;

    // 这里直接标记为 dense，意思是默认没有 NaN 点。
    // 如果以后发现雷达点里可能有 NaN/Inf，再在这里过滤或改成 false。
    cloud.is_dense = true;
    cloud.data.resize(cloud.row_step);

    size_t offset = 0;
    for (const auto & point : msg.points) {
      // Livox 自定义点里还有 reflectivity、tag、line 等字段。
      // FAST-LIO 当前链路只需要 xyz，所以这里只拷贝三个 float。
      const float xyz[3] = {point.x, point.y, point.z};
      std::memcpy(cloud.data.data() + offset, xyz, sizeof(xyz));
      offset += sizeof(xyz);
    }

    return cloud;
  }

  void timerCallback()
  {
    if (!latest_msg_) {
      RCLCPP_WARN(get_logger(), "尚未收到雷达数据，请检查雷达连接和网络配置...");
      return;
    }

    // 计算所有点到雷达原点的平均距离，只用于日志粗略判断。
    // 它不是“前方障碍距离”，也不会参与控制。
    double total_dist = 0.0;
    for (const auto & point : latest_msg_->points) {
      total_dist += std::sqrt(
        static_cast<double>(point.x) * point.x +
        static_cast<double>(point.y) * point.y +
        static_cast<double>(point.z) * point.z);
    }

    const auto point_count = latest_msg_->points.size();
    const double avg_dist = point_count > 0 ?
      total_dist / static_cast<double>(point_count) : 0.0;

    RCLCPP_INFO(
      get_logger(), "雷达帧: 点数=%zu, 中心平均距离=%.2fm",
      point_count, avg_dist);
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::TimerBase::SharedPtr log_timer_;
  livox_ros_driver2::msg::CustomMsg::SharedPtr latest_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mid360XYNode>());
  rclcpp::shutdown();
  return 0;
}
