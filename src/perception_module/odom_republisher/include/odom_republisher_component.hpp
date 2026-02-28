#ifndef ODOM_REPUBLISHER_COMPONENT_HPP_
#define ODOM_REPUBLISHER_COMPONENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/inference/Symbol.h>

#include "uosm_uav_interface/msg/trunk_observation_array.hpp"
#include "node_params.hpp"

#include <array>
#include <mutex>
#include <set>
#include <vector>

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
      void handle_vio_callback(const nav_msgs::msg::Odometry::SharedPtr &msg);
      void broadcast_odom_tf(const nav_msgs::msg::Odometry &odom_msg);

      Eigen::Isometry3d transform_msg_to_eigen(const geometry_msgs::msg::TransformStamped &tf_msg);
      geometry_msgs::msg::Pose transform_pose_eigen(
          const geometry_msgs::msg::Pose &pose_in,
          const Eigen::Isometry3d &transform);
      geometry_msgs::msg::Twist transform_twist_eigen(
          const geometry_msgs::msg::Twist &twist_in,
          const Eigen::Isometry3d &transform);
      void quat_to_rpy(const Eigen::Quaterniond &q, double &r, double &p, double &y);

      void handle_trunk_callback(
          const uosm_uav_interface::msg::TrunkObservationArray::SharedPtr &msg);
      void handle_alignment_done(const std_msgs::msg::Bool::SharedPtr &msg);
      void handle_alignment_transform(
          const geometry_msgs::msg::TransformStamped::SharedPtr &msg);
      void try_arm_isam2();
      void process_keyframe(
          const gtsam::Pose2 &vio_pose2d,
          const nav_msgs::msg::Odometry &vio_base);
      nav_msgs::msg::Odometry build_corrected_odom(
          const gtsam::Pose2 &corrected_pose2d,
          const nav_msgs::msg::Odometry &vio_base);

      bool static_transform_cached_{false};
      bool broadcast_tf_{true};
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

      bool use_landmark_fusion_{false};

      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr alignment_done_sub_;
      rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr
          alignment_transform_sub_;
      bool alignment_transform_received_{false};
      bool alignment_done_flag_{false};
      bool alignment_done_received_{false};

      rclcpp::Subscription<uosm_uav_interface::msg::TrunkObservationArray>::SharedPtr
          trunk_sub_;
      std::vector<uosm_uav_interface::msg::TrunkObservation> pending_observations_;
      std::mutex obs_mutex_;

      std::unique_ptr<gtsam::ISAM2> isam_;
      gtsam::NonlinearFactorGraph new_factors_;
      gtsam::Values new_values_;
      gtsam::Values current_estimate_;

      bool isam_initialized_{false};
      int keyframe_idx_{0};
      gtsam::Pose2 last_keyframe_vio_pose2d_;
      gtsam::Pose2 last_corrected_pose2d_;
      std::array<double, 36> last_vio_cov_{};

      std::set<uint64_t> known_landmarks_;

      double keyframe_dist_threshold_{0.3};
      double keyframe_yaw_threshold_{0.087};
      double prior_xy_noise_{0.1};
      double prior_yaw_noise_{0.05};
      double isam2_relinearize_threshold_{0.1};
      int isam2_relinearize_skip_{10};
    };

  } // namespace perception
} // namespace uosm

#endif // ODOM_REPUBLISHER_COMPONENT_HPP_
