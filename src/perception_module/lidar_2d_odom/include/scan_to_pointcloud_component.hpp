#ifndef SCAN_TO_POINTCLOUD_COMPONENT_HPP_
#define SCAN_TO_POINTCLOUD_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace uosm
{
namespace perception
{

class ScanToPointCloudComponent : public rclcpp::Node
{
public:
  explicit ScanToPointCloudComponent(const rclcpp::NodeOptions & options);

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

}  // namespace perception
}  // namespace uosm

#endif  // SCAN_TO_POINTCLOUD_COMPONENT_HPP_
