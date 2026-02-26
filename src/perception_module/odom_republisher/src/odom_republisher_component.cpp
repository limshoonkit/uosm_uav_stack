#include "odom_republisher_component.hpp"

namespace uosm
{
namespace perception
{

OdomRepublisher::OdomRepublisher(const rclcpp::NodeOptions & options)
: Node("odom_republisher_node", options)
{
  RCLCPP_INFO(get_logger(), "Odom Republisher (VIO -> base_link; optional 2D lidar fusion)");

  declare_parameter("odom_frame", std::string("odom"));
  declare_parameter("base_frame", std::string("base_link"));
  declare_parameter("camera_frame", std::string("zed_camera_link"));
  declare_parameter("broadcast_tf", true);
  declare_parameter("fuse_lidar_2d", false);
  declare_parameter("lidar2d_xy_noise", 0.02);
  declare_parameter("lidar2d_yaw_noise", 0.03);

  get_parameter("odom_frame", odom_frame_);
  get_parameter("base_frame", base_frame_);
  get_parameter("camera_frame", camera_frame_);
  get_parameter("broadcast_tf", broadcast_tf_);
  get_parameter("fuse_lidar_2d", fuse_lidar_2d_);
  get_parameter("lidar2d_xy_noise", lidar2d_xy_noise_);
  get_parameter("lidar2d_yaw_noise", lidar2d_yaw_noise_);

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
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { handle_vio_callback(msg); });

  if (fuse_lidar_2d_) {
    lidar2d_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom/lidar_2d", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { handle_lidar2d_callback(msg); });
    RCLCPP_INFO(get_logger(), "2D lidar fusion enabled (odom/lidar_2d)");
  }

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom/out", qos);
}

void OdomRepublisher::cache_static_transform()
{
  try {
    auto tf_msg = tf_buffer_->lookupTransform(
      base_frame_, camera_frame_, tf2::TimePointZero);
    cached_transform_eigen_ = transform_msg_to_eigen(tf_msg).inverse();
    static_transform_cached_ = true;
    cache_timer_->cancel();
    RCLCPP_INFO(get_logger(), "Cached static transform %s -> %s",
      camera_frame_.c_str(), base_frame_.c_str());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "Failed to lookup static transform: %s", ex.what());
  }
}

Eigen::Isometry3d OdomRepublisher::transform_msg_to_eigen(
  const geometry_msgs::msg::TransformStamped & tf_msg)
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
  const geometry_msgs::msg::Pose & pose_in,
  const Eigen::Isometry3d & transform)
{
  Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
  camera_pose.translation() = Eigen::Vector3d(
    pose_in.position.x, pose_in.position.y, pose_in.position.z);
  camera_pose.linear() = Eigen::Quaterniond(
    pose_in.orientation.w, pose_in.orientation.x,
    pose_in.orientation.y, pose_in.orientation.z).toRotationMatrix();
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
  const geometry_msgs::msg::Twist & twist_in,
  const Eigen::Isometry3d & transform)
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

void OdomRepublisher::quat_to_rpy(const Eigen::Quaterniond & q, double & r, double & p, double & y)
{
  Eigen::Matrix3d R = q.toRotationMatrix();
  r = std::atan2(R(2, 1), R(2, 2));
  p = std::asin(-std::max(-1.0, std::min(1.0, R(2, 0))));
  y = std::atan2(R(1, 0), R(0, 0));
}

void OdomRepublisher::handle_vio_callback(const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  if (!static_transform_cached_) {
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

  if (!fuse_lidar_2d_ || !has_received_lidar2d_) {
    broadcast_odom_tf(vio_base);
    odom_pub_->publish(vio_base);
    return;
  }

  update_state_from_vio(vio_base);
  nav_msgs::msg::Odometry out;
  state_to_odom_msg(out, msg->header.stamp);
  broadcast_odom_tf(out);
  odom_pub_->publish(out);
}

void OdomRepublisher::update_state_from_vio(const nav_msgs::msg::Odometry & msg)
{
  p_ << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
  v_ << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;
  q_.w() = msg.pose.pose.orientation.w;
  q_.x() = msg.pose.pose.orientation.x;
  q_.y() = msg.pose.pose.orientation.y;
  q_.z() = msg.pose.pose.orientation.z;

  if (!fusion_initialized_) {
    P_.setIdentity();
    P_.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    fusion_initialized_ = true;
  }
}

void OdomRepublisher::handle_lidar2d_callback(const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  has_received_lidar2d_ = true;
  if (!fusion_initialized_) return;

  update_state_from_lidar2d(*msg);

  nav_msgs::msg::Odometry out;
  state_to_odom_msg(out, msg->header.stamp);
  broadcast_odom_tf(out);
  odom_pub_->publish(out);
}

void OdomRepublisher::update_state_from_lidar2d(const nav_msgs::msg::Odometry & msg)
{
  double z_x = msg.pose.pose.position.x;
  double z_y = msg.pose.pose.position.y;
  double z_yaw = std::atan2(
    2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
           msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
    1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y +
                 msg.pose.pose.orientation.z * msg.pose.pose.orientation.z));

  double r, p, y;
  quat_to_rpy(q_, r, p, y);

  Eigen::Matrix<double, 3, 1> residual;
  residual << z_x - p_.x(), z_y - p_.y(), z_yaw - y;

  Eigen::Matrix<double, 3, kStateDim> H = Eigen::Matrix<double, 3, kStateDim>::Zero();
  H(0, 0) = 1;
  H(1, 1) = 1;
  H(2, 8) = 1;

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R(0, 0) = lidar2d_xy_noise_ * lidar2d_xy_noise_;
  R(1, 1) = lidar2d_xy_noise_ * lidar2d_xy_noise_;
  R(2, 2) = lidar2d_yaw_noise_ * lidar2d_yaw_noise_;

  Eigen::Matrix<double, kStateDim, 3> K =
    P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
  Eigen::Matrix<double, kStateDim, 1> dx = K * residual;

  p_.x() += dx(0);
  p_.y() += dx(1);
  p_.z() += dx(2);
  v_.x() += dx(3);
  v_.y() += dx(4);
  v_.z() += dx(5);
  double droll = dx(6), dpitch = dx(7), dyaw = dx(8);
  Eigen::AngleAxisd aa_yaw(dyaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd aa_pitch(dpitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd aa_roll(droll, Eigen::Vector3d::UnitX());
  q_ = q_ * Eigen::Quaterniond(aa_yaw * aa_pitch * aa_roll);
  q_.normalize();

  P_ = (Eigen::Matrix<double, kStateDim, kStateDim>::Identity() - K * H) * P_;
}

void OdomRepublisher::state_to_odom_msg(nav_msgs::msg::Odometry & out, const rclcpp::Time & stamp)
{
  out.header.stamp = stamp;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = base_frame_;
  out.pose.pose.position.x = p_.x();
  out.pose.pose.position.y = p_.y();
  out.pose.pose.position.z = p_.z();
  out.pose.pose.orientation.w = q_.w();
  out.pose.pose.orientation.x = q_.x();
  out.pose.pose.orientation.y = q_.y();
  out.pose.pose.orientation.z = q_.z();
  out.twist.twist.linear.x = v_.x();
  out.twist.twist.linear.y = v_.y();
  out.twist.twist.linear.z = v_.z();
  out.twist.twist.angular.x = 0.0;
  out.twist.twist.angular.y = 0.0;
  out.twist.twist.angular.z = 0.0;
  for (int i = 0; i < 36; ++i) out.pose.covariance[i] = 0.0;
  for (int i = 0; i < 6; ++i) out.pose.covariance[i * 7] = (i < 3 ? P_(i, i) : 0.01);
  for (int i = 0; i < 36; ++i) out.twist.covariance[i] = 0.0;
  for (int i = 0; i < 6; ++i) out.twist.covariance[i * 7] = (i < 3 ? P_(i + 3, i + 3) : 0.01);
}

void OdomRepublisher::broadcast_odom_tf(const nav_msgs::msg::Odometry & odom_msg)
{
  if (!broadcast_tf_) return;
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

}  // namespace perception
}  // namespace uosm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uosm::perception::OdomRepublisher)
