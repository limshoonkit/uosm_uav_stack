#ifndef TILT_GATED_ODOM_COMPONENT_HPP_
#define TILT_GATED_ODOM_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace uosm
{
namespace perception
{

class TiltGatedOdomComponent : public rclcpp::Node
{
public:
  explicit TiltGatedOdomComponent(const rclcpp::NodeOptions & options);

private:
  void odom_in_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void orientation_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  double max_tilt_rad_{ 0.26 };  // ~15 deg
  std::string odom_frame_{ "odom" };
  std::string base_frame_{ "base_link" };

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_in_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr orientation_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_out_pub_;

  double last_roll_{ 0.0 };
  double last_pitch_{ 0.0 };
  bool has_orientation_{ false };
};

}  // namespace perception
}  // namespace uosm

#endif  // TILT_GATED_ODOM_COMPONENT_HPP_
