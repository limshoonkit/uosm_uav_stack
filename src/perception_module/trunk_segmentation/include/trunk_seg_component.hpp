/*
 * SPDX-FileCopyrightText: University of Southampton Malaysia
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef TRUNK_SEG_COMPONENT_HPP_
#define TRUNK_SEG_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "uosm_uav_interface/msg/trunk_observation.hpp"
#include "uosm_uav_interface/msg/trunk_observation_array.hpp"
#include "node_params.hpp"

#include "circle_fitting.hpp"
#include "clustering_algorithms.hpp"
#include "trunk_tracking.hpp"
#include "trunk_seg_viz.hpp"

#include "timer_profile.hpp"

namespace uosm
{
    namespace perception
    {
        // Type aliases
        using Point = std::pair<double, double>;
        using Cluster = std::vector<Point>;
        using VizMarkerArr = visualization_msgs::msg::MarkerArray;

        struct TrunkSegmentationParams
        {
            double map_filter_z_min = 1.5;
            double map_filter_z_max = 4.0;
            double max_tilt_deg = 10.0;
            double max_tilt_rad = max_tilt_deg * M_PI / 180.0;

            int clustering_method = 1;
            int circle_fit_method = 1;
        };

        class TrunkSegmentationComponent : public rclcpp::Node
        {
        public:
            explicit TrunkSegmentationComponent(const rclcpp::NodeOptions &options);
            ~TrunkSegmentationComponent() = default;

        private:
            void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
            void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

            void publishClusterMarkers(const std::vector<Cluster> &clusters, const geometry_msgs::msg::Pose &robot_pose);
            void publishCircleMarkers(const std::vector<Circle> &circles, const geometry_msgs::msg::Pose &robot_pose);
            VizMarkerArr getTrackingMarkers(const std::unordered_map<uint16_t, TrackedTrunk> &tracks, const std::string ns);

            // Class
            std::shared_ptr<ClusteringAlgorithms> clustering_;
            std::shared_ptr<CircleFitting> circle_fitting_;
            std::shared_ptr<TrunkTracking> trunk_tracking_;

            // ROS2 subscribers and publishers
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

            rclcpp::Publisher<VizMarkerArr>::SharedPtr cluster_marker_pub_;
            rclcpp::Publisher<VizMarkerArr>::SharedPtr circle_marker_pub_;
            rclcpp::Publisher<VizMarkerArr>::SharedPtr active_track_marker_pub_;
            rclcpp::Publisher<VizMarkerArr>::SharedPtr lost_track_marker_pub_;
            rclcpp::Publisher<uosm_uav_interface::msg::TrunkObservationArray>::SharedPtr trunk_observations_pub_;

            // Diagnostics
            std::map<std::string, std::chrono::nanoseconds> section_durations_;
            diagnostic_updater::Updater updater_;
            bool enable_diagnostics_ = false;

            // Current robot pose from odometry
            geometry_msgs::msg::Pose current_pose_;
            bool pose_received_ = false;

            // Parameters
            TrunkSegmentationParams seg_params_;
            ClusteringParams clustering_params_;
            CircleFittingParams circle_fitting_params_;
            TrackingParams tracking_params_;

            std::string map_frame_;
            std::string odom_frame_;
            std::string base_frame_;
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

            // Core Functions
            void fullPCLPipeline(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                                 const Eigen::Isometry3d &transform,
                                 std::vector<Circle> &fitted_circles);
            void publishTrunkObservations(const std::unordered_map<uint16_t, TrackedTrunk> &tracks);

            // Helper Functions
            bool shouldIgnoreScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        };
    } // namespace perception
} // namespace uosm
#endif // TRUNK_SEG_COMPONENT_HPP_