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
      explicit OdomRepublisher(const rclcpp::NodeOptions &options);

    private:
      void cache_static_transform();
      void handle_vio_callback(const nav_msgs::msg::Odometry::SharedPtr& msg);
      void broadcast_odom_tf(const nav_msgs::msg::Odometry& odom_msg);

      Eigen::Isometry3d transform_msg_to_eigen(const geometry_msgs::msg::TransformStamped& tf_msg);
      geometry_msgs::msg::Pose transform_pose_eigen(
        const geometry_msgs::msg::Pose& pose_in,
        const Eigen::Isometry3d& transform);
      geometry_msgs::msg::Twist transform_twist_eigen(
        const geometry_msgs::msg::Twist& twist_in,
        const Eigen::Isometry3d& transform);

      bool static_transform_cached_;
      bool broadcast_tf_;

      std::string odom_frame_;
      std::string base_frame_;
      std::string camera_frame_;

      Eigen::Isometry3d cached_transform_eigen_;

      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vio_sub_;
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
      rclcpp::TimerBase::SharedPtr cache_timer_;
    };

  } // namespace perception
} // namespace uosm

#endif // ODOM_REPUBLISHER_COMPONENT_HPP_
