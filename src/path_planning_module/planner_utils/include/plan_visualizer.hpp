/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Portions adapted from EGO-Planner-v2: https://github.com/ZJU-FAST-Lab/EGO-Planner-v2
 */

#ifndef _PLAN_VISUALIZER_HPP_
#define _PLAN_VISUALIZER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <iostream>

namespace ego_planner
{
    constexpr double ALPHA_THRESHOLD = 1e-5;
    // Type aliases
    using MarkerPtr = rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr;
    using MarkerArrayPtr = rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr;

    class PlanVisualizer
    {
    private:
        rclcpp::Node::SharedPtr node_;
        MarkerPtr goal_point_pub;
        MarkerArrayPtr global_list_pub;
        MarkerArrayPtr init_list_pub;
        MarkerArrayPtr optimal_list_pub;
        MarkerArrayPtr failed_list_pub;

        std::string frame_id_;

    public:
        PlanVisualizer(const rclcpp::Node::SharedPtr &node);
        ~PlanVisualizer() = default;

        struct DisplayData
        {
            std::vector<Eigen::Vector3d> points;
            double scale;
            Eigen::Vector4d color;
            int id;
        };

        void displayMarkerList(MarkerArrayPtr pub, const DisplayData &data, bool show_sphere = true);
        void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array, const DisplayData &data);
        void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array, const DisplayData &data);
        void displayGoalPoint(const Eigen::Vector3d goal_point, const DisplayData &data);
        void displayGlobalPathList(DisplayData &data);
        void displayInitPathList(DisplayData &data);
        void displayMultiOptimalPathList(const std::vector<std::vector<Eigen::Vector3d>> &optimal_trajs, double scale);
        void displayOptimalList(const Eigen::MatrixXd &optimal_pts, int id);
        void displayFailedList(const Eigen::MatrixXd &failed_pts, int id);
        void displayArrowList(MarkerArrayPtr pub, const DisplayData &data);

    }; // class PlanVisualizer
} // namespace ego_planner
#endif // _PLAN_VISUALIZER_HPP_