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

class Mid360XYNode : public rclcpp::Node
{
public:
  Mid360XYNode() : Node("mid360_xy_node")
  {
    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", 10,
      std::bind(&Mid360XYNode::lidarCallback, this, std::placeholders::_1));

    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/mid360/xy_points", 10);

    log_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&Mid360XYNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "MID360 XY C++节点已启动，订阅 /livox/lidar");
  }

private:
  void lidarCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    latest_msg_ = msg;

    if (pointcloud_pub_->get_subscription_count() == 0) {
      return;
    }

    pointcloud_pub_->publish(customToPointCloud2(*msg));
  }

  sensor_msgs::msg::PointCloud2 customToPointCloud2(
    const livox_ros_driver2::msg::CustomMsg & msg) const
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg.header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(msg.points.size());

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
    cloud.is_dense = true;
    cloud.data.resize(cloud.row_step);

    size_t offset = 0;
    for (const auto & point : msg.points) {
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
