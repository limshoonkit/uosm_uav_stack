#ifndef ODOM_REPUBLISHER_COMPONENT_HPP_
#define ODOM_REPUBLISHER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace uosm
{
namespace perception
{

class OdomRepublisher : public rclcpp::Node
{
public:
  explicit OdomRepublisher(const rclcpp::NodeOptions & options);

private:
  void cache_static_transform();
  void handle_vio_callback(const nav_msgs::msg::Odometry::SharedPtr & msg);
  void handle_lidar2d_callback(const nav_msgs::msg::Odometry::SharedPtr & msg);
  void broadcast_odom_tf(const nav_msgs::msg::Odometry & odom_msg);
  void publish_fused_or_passthrough(const nav_msgs::msg::Odometry & vio_base);

  void update_state_from_vio(const nav_msgs::msg::Odometry & msg);
  void update_state_from_lidar2d(const nav_msgs::msg::Odometry & msg);
  void state_to_odom_msg(nav_msgs::msg::Odometry & out, const rclcpp::Time & stamp);

  Eigen::Isometry3d transform_msg_to_eigen(const geometry_msgs::msg::TransformStamped & tf_msg);
  geometry_msgs::msg::Pose transform_pose_eigen(
    const geometry_msgs::msg::Pose & pose_in,
    const Eigen::Isometry3d & transform);
  geometry_msgs::msg::Twist transform_twist_eigen(
    const geometry_msgs::msg::Twist & twist_in,
    const Eigen::Isometry3d & transform);

  void quat_to_rpy(const Eigen::Quaterniond & q, double & r, double & p, double & y);

  bool static_transform_cached_{ false };
  bool broadcast_tf_{ true };
  bool fuse_lidar_2d_{ false };
  bool has_received_lidar2d_{ false };

  std::string odom_frame_;
  std::string base_frame_;
  std::string camera_frame_;

  Eigen::Isometry3d cached_transform_eigen_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar2d_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr cache_timer_;

  // Fusion state (only used when fuse_lidar_2d_ and we have received lidar)
  static constexpr int kStateDim = 9;  // p(3), v(3), yaw(1) + roll,pitch fixed from VIO
  Eigen::Vector3d p_;
  Eigen::Vector3d v_;
  Eigen::Quaterniond q_;
  Eigen::Matrix<double, kStateDim, kStateDim> P_;
  bool fusion_initialized_{ false };
  double lidar2d_xy_noise_{ 0.02 };
  double lidar2d_yaw_noise_{ 0.03 };
};

}  // namespace perception
}  // namespace uosm

#endif  // ODOM_REPUBLISHER_COMPONENT_HPP_
