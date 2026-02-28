/*
 * SPDX-FileCopyrightText: University of Southampton Malaysia
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MAP_ALIGNMENT_HPP
#define MAP_ALIGNMENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>

#include "uosm_uav_interface/msg/trunk_observation_array.hpp"
#include "node_params.hpp"

#include <atomic>
#include <map>
#include <vector>

namespace uosm::perception
{
    struct MapAlignParameters
    {
        bool solve_for_scale;
        double max_initial_guess_xy_deviation;
        double max_scale_deviation;
        double association_radius;
        int min_correspondences;
        int max_iterations;
        double convergence_threshold;
        double voxel_leaf_size;
        double initial_yaw_rad;
        double starting_point_x;
        double starting_point_y;
        int max_alignment_attempts;
    };

    class MapAlignmentComponent : public rclcpp::Node
    {
    public:
        explicit MapAlignmentComponent(const rclcpp::NodeOptions &options);

    private:
        bool loadMap();

        void observationCallback(const uosm_uav_interface::msg::TrunkObservationArray::SharedPtr msg);
        void attemptAlignment();
        void prepareObservationData(
            std::vector<Eigen::Vector3d> &odom_points,
            std::vector<Eigen::Matrix2d> &covariances_inv);
        Eigen::Matrix4f performIterativeAlignment(
            const std::vector<Eigen::Vector3d> &odom_points,
            const std::vector<Eigen::Matrix2d> &covariances_inv,
            bool &succeeded);
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> findCorrespondences(
            const std::vector<Eigen::Vector3d> &odom_points,
            const Eigen::Matrix4f &transform);
        Eigen::Matrix4f solveWeightedAlignment(
            const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &correspondences,
            const std::vector<Eigen::Matrix2d> &covariances_inv);

        bool validateAlignment(const Eigen::Matrix4f &transform);
        void finalizeAlignment(
            const Eigen::Matrix4f &transform,
            const std::vector<Eigen::Vector3d> &odom_points);
        void publishTransform();
        void publishAlignmentVisualization(const std::vector<Eigen::Vector3d> &odom_points);
        void publishAlignmentStatus();
        void handleAlignmentStatusRequest(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        rclcpp::Subscription<uosm_uav_interface::msg::TrunkObservationArray>::SharedPtr landmark_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr alignment_marker_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr alignment_status_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr alignment_transform_pub_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr alignment_status_srv_;
        rclcpp::TimerBase::SharedPtr alignment_timer_;
        rclcpp::TimerBase::SharedPtr status_pub_timer_;

        std::string map_frame_;
        std::string odom_frame_;
        std::string pcd_file_path_;
        MapAlignParameters params_;

        std::atomic<bool> is_alignment_done_;
        int alignment_attempt_count_ = 0;
        std::map<uint16_t, uosm_uav_interface::msg::TrunkObservation> observed_landmarks_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr map_landmarks_pcl_;
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr map_kdtree_;
        Eigen::Matrix4f T_map_odom_;
    };

} // namespace uosm::perception

#endif // MAP_ALIGNMENT_HPP
