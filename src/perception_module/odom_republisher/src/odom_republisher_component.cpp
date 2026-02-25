#include "odom_republisher_component.hpp"

namespace uosm
{
    namespace perception
    {
        OdomRepublisher::OdomRepublisher(const rclcpp::NodeOptions &options)
            : Node("odom_republisher_node", options),
              static_transform_cached_(false),
              odom_frame_("odom"),
              base_frame_("base_link"),
              camera_frame_("zed_camera_link")
        {
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), "   Odom Republisher Component   ");
            RCLCPP_INFO(get_logger(), "********************************");
            RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
            RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
            RCLCPP_INFO(get_logger(), "********************************");

            declare_parameter("odom_frame", "odom");
            declare_parameter("base_frame", "base_link");
            declare_parameter("camera_frame", "zed_camera_link");
            declare_parameter("broadcast_tf", true);

            get_parameter("odom_frame", odom_frame_);
            get_parameter("base_frame", base_frame_);
            get_parameter("camera_frame", camera_frame_);
            get_parameter("broadcast_tf", broadcast_tf_);

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // Cache static transform after a short delay to let tf tree populate
            cache_timer_ = create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&OdomRepublisher::cache_static_transform, this));

            auto qos = rclcpp::QoS(1)
                           .reliability(rclcpp::ReliabilityPolicy::Reliable)
                           .durability(rclcpp::DurabilityPolicy::Volatile)
                           .history(rclcpp::HistoryPolicy::KeepLast);

            vio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "/vio/odom", qos,
                [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    handle_vio_callback(msg);
                });

            odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom/out", qos);
        }

        void OdomRepublisher::cache_static_transform()
        {
            try
            {
                auto tf_msg = tf_buffer_->lookupTransform(
                    base_frame_,
                    camera_frame_,
                    tf2::TimePointZero);

                cached_transform_eigen_ = transform_msg_to_eigen(tf_msg).inverse();
                static_transform_cached_ = true;
                cache_timer_->cancel();

                RCLCPP_INFO(get_logger(), "Cached static transform from %s to %s",
                            camera_frame_.c_str(), base_frame_.c_str());
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_WARN(get_logger(), "Failed to lookup static transform: %s", ex.what());
            }
        }

        Eigen::Isometry3d OdomRepublisher::transform_msg_to_eigen(
            const geometry_msgs::msg::TransformStamped& tf_msg)
        {
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

            transform.translation() = Eigen::Vector3d(
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z);

            Eigen::Quaterniond q(
                tf_msg.transform.rotation.w,
                tf_msg.transform.rotation.x,
                tf_msg.transform.rotation.y,
                tf_msg.transform.rotation.z);
            transform.linear() = q.toRotationMatrix();

            return transform;
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
                pose_in.orientation.y, pose_in.orientation.z).toRotationMatrix();

            // T_odom_baselink = T_odom_camera * T_camera_baselink
            Eigen::Isometry3d baselink_pose = camera_pose * transform;

            geometry_msgs::msg::Pose pose_out;
            pose_out.position.x = baselink_pose.translation().x();
            pose_out.position.y = baselink_pose.translation().y();
            pose_out.position.z = baselink_pose.translation().z();

            Eigen::Quaterniond result_quat(baselink_pose.linear());
            pose_out.orientation.w = result_quat.w();
            pose_out.orientation.x = result_quat.x();
            pose_out.orientation.y = result_quat.y();
            pose_out.orientation.z = result_quat.z();

            return pose_out;
        }

        geometry_msgs::msg::Twist OdomRepublisher::transform_twist_eigen(
            const geometry_msgs::msg::Twist& twist_in,
            const Eigen::Isometry3d& transform)
        {
            Eigen::Matrix3d rotation = transform.linear();

            geometry_msgs::msg::Twist twist_out;
            Eigen::Vector3d linear_out = rotation * Eigen::Vector3d(
                twist_in.linear.x, twist_in.linear.y, twist_in.linear.z);
            Eigen::Vector3d angular_out = rotation * Eigen::Vector3d(
                twist_in.angular.x, twist_in.angular.y, twist_in.angular.z);

            twist_out.linear.x = linear_out.x();
            twist_out.linear.y = linear_out.y();
            twist_out.linear.z = linear_out.z();
            twist_out.angular.x = angular_out.x();
            twist_out.angular.y = angular_out.y();
            twist_out.angular.z = angular_out.z();

            return twist_out;
        }

        void OdomRepublisher::broadcast_odom_tf(const nav_msgs::msg::Odometry& odom_msg)
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

        void OdomRepublisher::handle_vio_callback(const nav_msgs::msg::Odometry::SharedPtr &msg)
        {
            if (!static_transform_cached_)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "Static transform not cached yet, cannot process VIO message");
                return;
            }

            nav_msgs::msg::Odometry out;
            out.header.stamp = msg->header.stamp;
            out.header.frame_id = odom_frame_;
            out.child_frame_id = base_frame_;
            out.pose.pose = transform_pose_eigen(msg->pose.pose, cached_transform_eigen_);
            out.pose.covariance = msg->pose.covariance;
            out.twist.twist = transform_twist_eigen(msg->twist.twist, cached_transform_eigen_);
            out.twist.covariance = msg->twist.covariance;

            broadcast_odom_tf(out);
            odom_pub_->publish(out);
        }

    } // namespace perception
} // namespace uosm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uosm::perception::OdomRepublisher)
