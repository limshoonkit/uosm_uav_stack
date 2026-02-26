#include "scan_to_pointcloud_component.hpp"
#include <cstring>

namespace uosm
{
namespace perception
{

ScanToPointCloudComponent::ScanToPointCloudComponent(const rclcpp::NodeOptions & options)
: Node("scan_to_pointcloud_node", options)
{
  declare_parameter("output_frame", std::string(""));
  std::string output_frame;
  get_parameter("output_frame", output_frame);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) { scan_callback(msg); });

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("scan_cloud", 10);

  RCLCPP_INFO(get_logger(), "ScanToPointCloud: /scan -> scan_cloud (for KISS-ICP)");
}

void ScanToPointCloudComponent::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = msg->header;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  cloud.fields.resize(3);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;

  cloud.point_step = 12;
  std::vector<float> data;

  float angle = msg->angle_min;
  for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
    float r = msg->ranges[i];
    if (std::isnan(r) || std::isinf(r) || r < msg->range_min || r > msg->range_max) {
      continue;
    }
    float x = r * std::cos(angle);
    float y = r * std::sin(angle);
    float z = 0.0f;
    data.push_back(x);
    data.push_back(y);
    data.push_back(z);
  }

  cloud.width = static_cast<uint32_t>(data.size() / 3);
  if (cloud.width == 0) {
    return;
  }
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  std::memcpy(cloud.data.data(), data.data(), data.size() * sizeof(float));

  cloud_pub_->publish(cloud);
}

}  // namespace perception
}  // namespace uosm

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(uosm::perception::ScanToPointCloudComponent)
