#include "tilt_gated_odom_component.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <cmath>

namespace uosm
{
namespace perception
{

static void quat_to_rpy(
  double qw, double qx, double qy, double qz,
  double & r, double & p, double & y)
{
  r = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
  p = std::asin(std::max(-1.0, std::min(1.0, 2.0 * (qw * qy - qz * qx))));
  y = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

TiltGatedOdomComponent::TiltGatedOdomComponent(const rclcpp::NodeOptions & options)
: Node("tilt_gated_odom_node", options)
{
  declare_parameter("max_tilt_deg", 15.0);
  declare_parameter("odom_frame", std::string("odom"));
  declare_parameter("base_frame", std::string("base_link"));

  double max_tilt_deg;
  get_parameter("max_tilt_deg", max_tilt_deg);
  get_parameter("odom_frame", odom_frame_);
  get_parameter("base_frame", base_frame_);
  max_tilt_rad_ = max_tilt_deg * M_PI / 180.0;

  odom_in_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom_in", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_in_callback(msg); });

  orientation_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "orientation", 10,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { orientation_callback(msg); });

  odom_out_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom/lidar_2d", 10);

  RCLCPP_INFO(get_logger(), "TiltGatedOdom: max_tilt=%.1f deg, odom_in -> odom/lidar_2d", max_tilt_deg);
}

void TiltGatedOdomComponent::orientation_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & q = msg->pose.pose.orientation;
  double yaw;
  quat_to_rpy(q.w, q.x, q.y, q.z, last_roll_, last_pitch_, yaw);
  has_orientation_ = true;
}

void TiltGatedOdomComponent::odom_in_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!has_orientation_) {
    return;
  }
  if (std::fabs(last_roll_) > max_tilt_rad_ || std::fabs(last_pitch_) > max_tilt_rad_) {
    return;
  }

  nav_msgs::msg::Odometry out;
  out.header = msg->header;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = base_frame_;
  out.pose.pose.position.x = msg->pose.pose.position.x;
  out.pose.pose.position.y = msg->pose.pose.position.y;
  out.pose.pose.position.z = 0.0;
  out.pose.pose.orientation = msg->pose.pose.orientation;

  out.twist = msg->twist;

  for (int i = 0; i < 36; ++i) {
    out.pose.covariance[i] = msg->pose.covariance[i];
    out.twist.covariance[i] = msg->twist.covariance[i];
  }
  out.pose.covariance[14] = 1e6;
  out.pose.covariance[21] = 1e6;
  out.pose.covariance[28] = 1e6;

  odom_out_pub_->publish(out);
}

}  // namespace perception
}  // namespace uosm

RCLCPP_COMPONENTS_REGISTER_NODE(uosm::perception::TiltGatedOdomComponent)
