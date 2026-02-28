/*
 * SPDX-FileCopyrightText: University of Southampton Malaysia
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "odom_republisher_component.hpp"

namespace uosm
{
  namespace perception
  {

    OdomRepublisher::OdomRepublisher(const rclcpp::NodeOptions &options)
        : Node("odom_republisher_node", options)
    {
      RCLCPP_INFO(get_logger(), "Odom Republisher (VIO -> base_link; iSAM2 landmark fusion)");

      odom_frame_ = util::getParam<std::string>(this, "odom_frame", "odom", "  odom_frame: ");
      base_frame_ = util::getParam<std::string>(this, "base_frame", "base_link", "  base_frame: ");
      camera_frame_ = util::getParam<std::string>(this, "camera_frame", "zed_camera_link", "  camera_frame: ");
      broadcast_tf_ = util::getParam<bool>(this, "broadcast_tf", true, "  broadcast_tf: ");

      use_landmark_fusion_ = util::getParam<bool>(this, "use_landmark_fusion", false, "  use_landmark_fusion: ");
      keyframe_dist_threshold_ = util::getParam<double>(this, "keyframe_dist_threshold", 0.15, "  keyframe_dist_threshold: ");
      keyframe_yaw_threshold_ = util::getParam<double>(this, "keyframe_yaw_threshold", 0.05, "  keyframe_yaw_threshold: ");
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      cache_timer_ = create_wall_timer(
          std::chrono::milliseconds(100),
          std::bind(&OdomRepublisher::cache_static_transform, this));

      auto qos = rclcpp::QoS(1)
                     .reliability(rclcpp::ReliabilityPolicy::Reliable)
                     .durability(rclcpp::DurabilityPolicy::Volatile)
                     .history(rclcpp::HistoryPolicy::KeepLast);

      vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          "vio/odom", qos,
          [this](const nav_msgs::msg::Odometry::SharedPtr msg)
          { handle_vio_callback(msg); });

      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom/out", qos);

      if (use_landmark_fusion_)
      {
        RCLCPP_INFO(get_logger(), "iSAM2 landmark fusion enabled — waiting for alignment");

        trunk_sub_ = create_subscription<uosm_uav_interface::msg::TrunkObservationArray>(
            "trunk_observations", 10,
            [this](const uosm_uav_interface::msg::TrunkObservationArray::SharedPtr msg)
            {
              handle_trunk_callback(msg);
            });

        alignment_done_sub_ = create_subscription<std_msgs::msg::Bool>(
            "alignment_done", rclcpp::QoS(1),
            [this](const std_msgs::msg::Bool::SharedPtr msg)
            { handle_alignment_done(msg); });

        alignment_transform_sub_ = create_subscription<geometry_msgs::msg::TransformStamped>(
            "alignment_transform", rclcpp::QoS(1),
            [this](const geometry_msgs::msg::TransformStamped::SharedPtr msg)
            {
              handle_alignment_transform(msg);
            });

        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.1;
        params.relinearizeSkip = 10;
        isam_ = std::make_unique<gtsam::ISAM2>(params);
      }
    }

    void OdomRepublisher::cache_static_transform()
    {
      try
      {
        auto tf_msg = tf_buffer_->lookupTransform(
            base_frame_, camera_frame_, tf2::TimePointZero);
        cached_transform_eigen_ = transform_msg_to_eigen(tf_msg).inverse();
        static_transform_cached_ = true;
        cache_timer_->cancel();
        RCLCPP_INFO(get_logger(), "Cached static transform %s -> %s",
                    camera_frame_.c_str(), base_frame_.c_str());
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN(get_logger(), "Failed to lookup static transform: %s", ex.what());
      }
    }

    Eigen::Isometry3d OdomRepublisher::transform_msg_to_eigen(
        const geometry_msgs::msg::TransformStamped &tf_msg)
    {
      Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
      T.translation() = Eigen::Vector3d(
          tf_msg.transform.translation.x,
          tf_msg.transform.translation.y,
          tf_msg.transform.translation.z);
      Eigen::Quaterniond q(
          tf_msg.transform.rotation.w,
          tf_msg.transform.rotation.x,
          tf_msg.transform.rotation.y,
          tf_msg.transform.rotation.z);
      T.linear() = q.toRotationMatrix();
      return T;
    }

    geometry_msgs::msg::Pose OdomRepublisher::transform_pose_eigen(
        const geometry_msgs::msg::Pose &pose_in,
        const Eigen::Isometry3d &transform)
    {
      Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
      camera_pose.translation() = Eigen::Vector3d(
          pose_in.position.x, pose_in.position.y, pose_in.position.z);
      camera_pose.linear() = Eigen::Quaterniond(
                                 pose_in.orientation.w, pose_in.orientation.x,
                                 pose_in.orientation.y, pose_in.orientation.z)
                                 .toRotationMatrix();
      Eigen::Isometry3d baselink_pose = camera_pose * transform;

      geometry_msgs::msg::Pose pose_out;
      pose_out.position.x = baselink_pose.translation().x();
      pose_out.position.y = baselink_pose.translation().y();
      pose_out.position.z = baselink_pose.translation().z();
      Eigen::Quaterniond q_out(baselink_pose.linear());
      pose_out.orientation.w = q_out.w();
      pose_out.orientation.x = q_out.x();
      pose_out.orientation.y = q_out.y();
      pose_out.orientation.z = q_out.z();
      return pose_out;
    }

    geometry_msgs::msg::Twist OdomRepublisher::transform_twist_eigen(
        const geometry_msgs::msg::Twist &twist_in,
        const Eigen::Isometry3d &transform)
    {
      Eigen::Matrix3d R = transform.linear();
      geometry_msgs::msg::Twist twist_out;
      Eigen::Vector3d linear_out = R * Eigen::Vector3d(
                                           twist_in.linear.x, twist_in.linear.y, twist_in.linear.z);
      Eigen::Vector3d angular_out = R * Eigen::Vector3d(
                                            twist_in.angular.x, twist_in.angular.y, twist_in.angular.z);
      twist_out.linear.x = linear_out.x();
      twist_out.linear.y = linear_out.y();
      twist_out.linear.z = linear_out.z();
      twist_out.angular.x = angular_out.x();
      twist_out.angular.y = angular_out.y();
      twist_out.angular.z = angular_out.z();
      return twist_out;
    }

    void OdomRepublisher::quat_to_rpy(
        const Eigen::Quaterniond &q, double &r, double &p, double &y)
    {
      Eigen::Matrix3d R = q.toRotationMatrix();
      r = std::atan2(R(2, 1), R(2, 2));
      p = std::asin(-std::max(-1.0, std::min(1.0, R(2, 0))));
      y = std::atan2(R(1, 0), R(0, 0));
    }

    void OdomRepublisher::broadcast_odom_tf(const nav_msgs::msg::Odometry &odom_msg)
    {
      if (!broadcast_tf_)
        return;
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = odom_msg.header.stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = odom_msg.pose.pose.position.x;
      tf.transform.translation.y = odom_msg.pose.pose.position.y;
      tf.transform.translation.z = odom_msg.pose.pose.position.z;
      tf.transform.rotation = odom_msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }

    void OdomRepublisher::try_arm_isam2()
    {
      if (alignment_done_received_)
        return;
      if (!alignment_done_flag_ || !alignment_transform_received_)
        return;
      alignment_done_received_ = true;
      RCLCPP_INFO(get_logger(), "Alignment complete — iSAM2 landmark fusion armed");
    }

    void OdomRepublisher::handle_alignment_transform(
        const geometry_msgs::msg::TransformStamped::SharedPtr &msg)
    {
      if (alignment_transform_received_)
        return;
      alignment_transform_received_ = true;
      RCLCPP_INFO(get_logger(), "Received alignment transform (map->odom)");
      (void)msg;
      try_arm_isam2();
    }

    void OdomRepublisher::handle_alignment_done(const std_msgs::msg::Bool::SharedPtr &msg)
    {
      if (!msg->data || alignment_done_received_)
        return;
      alignment_done_flag_ = true;
      try_arm_isam2();
    }

    void OdomRepublisher::handle_trunk_callback(
        const uosm_uav_interface::msg::TrunkObservationArray::SharedPtr &msg)
    {
      if (!alignment_done_received_)
        return;
      std::lock_guard<std::mutex> lock(obs_mutex_);
      for (const auto &obs : msg->observations)
      {
        pending_observations_.push_back(obs);
      }
    }

    void OdomRepublisher::handle_vio_callback(
        const nav_msgs::msg::Odometry::SharedPtr &msg)
    {
      if (!static_transform_cached_)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Static transform not cached, skipping VIO");
        return;
      }

      nav_msgs::msg::Odometry vio_base;
      vio_base.header = msg->header;
      vio_base.header.frame_id = odom_frame_;
      vio_base.child_frame_id = base_frame_;
      vio_base.pose.pose = transform_pose_eigen(msg->pose.pose, cached_transform_eigen_);
      vio_base.pose.covariance = msg->pose.covariance;
      vio_base.twist.twist = transform_twist_eigen(msg->twist.twist, cached_transform_eigen_);
      vio_base.twist.covariance = msg->twist.covariance;

      if (!use_landmark_fusion_ || !alignment_done_received_)
      {
        broadcast_odom_tf(vio_base);
        odom_pub_->publish(vio_base);
        return;
      }

      Eigen::Quaterniond vio_q(
          vio_base.pose.pose.orientation.w, vio_base.pose.pose.orientation.x,
          vio_base.pose.pose.orientation.y, vio_base.pose.pose.orientation.z);
      double roll, pitch, yaw;
      quat_to_rpy(vio_q, roll, pitch, yaw);
      gtsam::Pose2 current_vio_pose2d(
          vio_base.pose.pose.position.x, vio_base.pose.pose.position.y, yaw);

      bool is_keyframe = false;
      if (!isam_initialized_)
      {
        is_keyframe = true;
      }
      else
      {
        gtsam::Pose2 delta = last_keyframe_vio_pose2d_.between(current_vio_pose2d);
        double dist = std::hypot(delta.x(), delta.y());
        double dyaw = std::abs(delta.theta());
        is_keyframe = (dist > keyframe_dist_threshold_ || dyaw > keyframe_yaw_threshold_);
      }

      if (is_keyframe)
      {
        process_keyframe(current_vio_pose2d, vio_base);
      }
      else
      {
        gtsam::Pose2 correction =
            last_corrected_pose2d_.compose(last_keyframe_vio_pose2d_.inverse());
        gtsam::Pose2 corrected_current = correction.compose(current_vio_pose2d);
        nav_msgs::msg::Odometry corrected_odom =
            build_corrected_odom(corrected_current, vio_base);
        broadcast_odom_tf(corrected_odom);
        odom_pub_->publish(corrected_odom);
      }
    }

    void OdomRepublisher::process_keyframe(
        const gtsam::Pose2 &vio_pose2d,
        const nav_msgs::msg::Odometry &vio_base)
    {
      using gtsam::Symbol;

      const auto &cov = vio_base.pose.covariance;

      if (!isam_initialized_)
      {
        constexpr double kMinPriorXY = 0.1;
        constexpr double kMinPriorYaw = 0.05;
        double sx = std::max(std::sqrt(cov[0]), kMinPriorXY);
        double sy = std::max(std::sqrt(cov[7]), kMinPriorXY);
        double syaw = std::max(std::sqrt(cov[35]), kMinPriorYaw);
        auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << sx, sy, syaw).finished());
        new_factors_.addPrior(Symbol('x', 0), vio_pose2d, prior_noise);
        new_values_.insert(Symbol('x', 0), vio_pose2d);
        last_vio_cov_ = cov;
        isam_initialized_ = true;
        RCLCPP_INFO(get_logger(), "iSAM2 initialized at (%.2f, %.2f, %.2f)",
                    vio_pose2d.x(), vio_pose2d.y(), vio_pose2d.theta());
      }
      else
      {
        gtsam::Pose2 delta = last_keyframe_vio_pose2d_.between(vio_pose2d);
        constexpr double kMinBetweenXY = 0.01;
        constexpr double kMinBetweenYaw = 0.005;
        double dx_sigma = std::max(std::sqrt(std::abs(cov[0] - last_vio_cov_[0])), kMinBetweenXY);
        double dy_sigma = std::max(std::sqrt(std::abs(cov[7] - last_vio_cov_[7])), kMinBetweenXY);
        double dyaw_sigma = std::max(std::sqrt(std::abs(cov[35] - last_vio_cov_[35])), kMinBetweenYaw);
        auto vio_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << dx_sigma, dy_sigma, dyaw_sigma).finished());
        new_factors_.add(gtsam::BetweenFactor<gtsam::Pose2>(
            Symbol('x', keyframe_idx_ - 1), Symbol('x', keyframe_idx_), delta, vio_noise));
        new_values_.insert(Symbol('x', keyframe_idx_), vio_pose2d);
        last_vio_cov_ = cov;
      }

      {
        std::lock_guard<std::mutex> lock(obs_mutex_);
        for (const auto &obs : pending_observations_)
        {
          double dx_odom = obs.position.x - vio_pose2d.x();
          double dy_odom = obs.position.y - vio_pose2d.y();
          double cos_yaw = std::cos(vio_pose2d.theta());
          double sin_yaw = std::sin(vio_pose2d.theta());
          double body_dx = cos_yaw * dx_odom + sin_yaw * dy_odom;
          double body_dy = -sin_yaw * dx_odom + cos_yaw * dy_odom;
          double range = std::hypot(body_dx, body_dy);
          double bearing = std::atan2(body_dy, body_dx);

          if (range < 0.5 || range > 20.0)
            continue;

          Eigen::Matrix2d C_odom;
          C_odom << obs.covariance[0], obs.covariance[1],
              obs.covariance[2], obs.covariance[3];

          Eigen::Matrix2d R_body;
          R_body << cos_yaw, sin_yaw,
              -sin_yaw, cos_yaw;
          Eigen::Matrix2d C_body = R_body * C_odom * R_body.transpose();

          double r2 = range * range;
          Eigen::Matrix2d J_br;
          J_br << -body_dy / r2, body_dx / r2,
              body_dx / range, body_dy / range;
          Eigen::Matrix2d C_br = J_br * C_body * J_br.transpose();

          constexpr double kMinBearingSigma = 0.01;
          constexpr double kMinRangeSigma = 0.05;
          double bearing_sigma = std::max(std::sqrt(std::abs(C_br(0, 0))), kMinBearingSigma);
          double range_sigma = std::max(std::sqrt(std::abs(C_br(1, 1))), kMinRangeSigma);

          auto obs_noise = gtsam::noiseModel::Diagonal::Sigmas(
              (gtsam::Vector(2) << bearing_sigma, range_sigma).finished());

          Symbol pose_key('x', keyframe_idx_);
          Symbol landmark_key('l', obs.landmark_id);

          new_factors_.add(
              gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
                  pose_key, landmark_key,
                  gtsam::Rot2::fromAngle(bearing), range,
                  obs_noise));

          if (known_landmarks_.find(obs.landmark_id) == known_landmarks_.end())
          {
            gtsam::Point2 lm_odom(obs.position.x, obs.position.y);
            new_values_.insert(landmark_key, lm_odom);
            known_landmarks_.insert(obs.landmark_id);
          }
        }
        pending_observations_.clear();
      }

      isam_->update(new_factors_, new_values_);
      new_factors_.resize(0);
      new_values_.clear();

      current_estimate_ = isam_->calculateEstimate();
      gtsam::Pose2 corrected_pose2d =
          current_estimate_.at<gtsam::Pose2>(Symbol('x', keyframe_idx_));

      last_keyframe_vio_pose2d_ = vio_pose2d;
      last_corrected_pose2d_ = corrected_pose2d;
      keyframe_idx_++;

      nav_msgs::msg::Odometry corrected_odom =
          build_corrected_odom(corrected_pose2d, vio_base);
      broadcast_odom_tf(corrected_odom);
      odom_pub_->publish(corrected_odom);

      RCLCPP_DEBUG(get_logger(), "KF %d: vio(%.2f,%.2f) -> corr(%.2f,%.2f)  landmarks=%zu",
                   keyframe_idx_ - 1, vio_pose2d.x(), vio_pose2d.y(),
                   corrected_pose2d.x(), corrected_pose2d.y(), known_landmarks_.size());
    }

    nav_msgs::msg::Odometry OdomRepublisher::build_corrected_odom(
        const gtsam::Pose2 &corrected_pose2d,
        const nav_msgs::msg::Odometry &vio_base)
    {
      nav_msgs::msg::Odometry out = vio_base;
      out.pose.pose.position.x = corrected_pose2d.x();
      out.pose.pose.position.y = corrected_pose2d.y();

      Eigen::Quaterniond vio_q(
          vio_base.pose.pose.orientation.w, vio_base.pose.pose.orientation.x,
          vio_base.pose.pose.orientation.y, vio_base.pose.pose.orientation.z);
      double r, p, y;
      quat_to_rpy(vio_q, r, p, y);

      Eigen::Quaterniond corrected_q =
          Eigen::AngleAxisd(corrected_pose2d.theta(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX());
      corrected_q.normalize();

      out.pose.pose.orientation.w = corrected_q.w();
      out.pose.pose.orientation.x = corrected_q.x();
      out.pose.pose.orientation.y = corrected_q.y();
      out.pose.pose.orientation.z = corrected_q.z();
      return out;
    }

  } // namespace perception
} // namespace uosm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uosm::perception::OdomRepublisher)
