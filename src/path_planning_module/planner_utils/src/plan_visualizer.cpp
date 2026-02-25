#include "plan_visualizer.hpp"
#include <node_params.hpp>

namespace ego_planner
{
    PlanVisualizer::PlanVisualizer(const rclcpp::Node::SharedPtr &node)
    {
        node_ = node;
        frame_id_ = uosm::util::getParam<std::string>(node, "visualizer.frame_id", "map");

        goal_point_pub = node_->create_publisher<visualization_msgs::msg::Marker>("goal_point", 10);
        global_list_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("global_list", 10);
        init_list_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("init_list", 10);
        optimal_list_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("optimal_list", 10);
        failed_list_pub = node_->create_publisher<visualization_msgs::msg::MarkerArray>("failed_list", 10);
    }

    void PlanVisualizer::displayMarkerList(MarkerArrayPtr pub, const DisplayData &data, bool show_sphere)
    {
        visualization_msgs::msg::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = frame_id_;
        sphere.header.stamp = line_strip.header.stamp = node_->now();
        sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
        sphere.id = data.id;
        line_strip.id = data.id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = data.color(0);
        sphere.color.g = line_strip.color.g = data.color(1);
        sphere.color.b = line_strip.color.b = data.color(2);
        sphere.color.a = line_strip.color.a = data.color(3) > ALPHA_THRESHOLD ? data.color(3) : 1.0;
        sphere.scale.x = data.scale;
        sphere.scale.y = data.scale;
        sphere.scale.z = data.scale;
        line_strip.scale.x = data.scale / 2.0;

        geometry_msgs::msg::Point pt;
        for (size_t i = 0; i < data.points.size(); i++)
        {
            pt.x = data.points[i](0);
            pt.y = data.points[i](1);
            pt.z = data.points[i](2);
            if (show_sphere)
                sphere.points.push_back(pt);
            line_strip.points.push_back(pt);
        }

        visualization_msgs::msg::MarkerArray marker_array;
        if (show_sphere)
            marker_array.markers.push_back(sphere);
        marker_array.markers.push_back(line_strip);

        pub->publish(marker_array);
    }

    void PlanVisualizer::generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array, const DisplayData &data)
    {
        visualization_msgs::msg::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = frame_id_;
        sphere.header.stamp = line_strip.header.stamp = node_->now();
        sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
        sphere.id = data.id;
        line_strip.id = data.id + 1;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = data.color(0);
        sphere.color.g = line_strip.color.g = data.color(1);
        sphere.color.b = line_strip.color.b = data.color(2);
        sphere.color.a = line_strip.color.a = data.color(3) > ALPHA_THRESHOLD ? data.color(3) : 1.0;
        sphere.scale.x = data.scale;
        sphere.scale.y = data.scale;
        sphere.scale.z = data.scale;
        line_strip.scale.x = data.scale / 3.0;

        geometry_msgs::msg::Point pt;
        for (size_t i = 0; i < data.points.size(); i++)
        {
            pt.x = data.points[i](0);
            pt.y = data.points[i](1);
            pt.z = data.points[i](2);
            sphere.points.push_back(pt);
            line_strip.points.push_back(pt);
        }
        array.markers.push_back(sphere);
        array.markers.push_back(line_strip);
    }

    void PlanVisualizer::generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array, const DisplayData &data)
    {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame_id_;
        arrow.header.stamp = node_->now();
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.action = visualization_msgs::msg::Marker::ADD;

        arrow.color.r = data.color(0);
        arrow.color.g = data.color(1);
        arrow.color.b = data.color(2);
        arrow.color.a = data.color(3) > ALPHA_THRESHOLD ? data.color(3) : 1.0;
        arrow.scale.x = data.scale;
        arrow.scale.y = 2 * data.scale;
        arrow.scale.z = 2 * data.scale;

        geometry_msgs::msg::Point start, end;
        for (int i = 0; i < static_cast<int>(data.points.size() / 2); i++)
        {
            start.x = data.points[2 * i](0);
            start.y = data.points[2 * i](1);
            start.z = data.points[2 * i](2);
            end.x = data.points[2 * i + 1](0);
            end.y = data.points[2 * i + 1](1);
            end.z = data.points[2 * i + 1](2);
            arrow.points.clear();
            arrow.points.push_back(start);
            arrow.points.push_back(end);
            arrow.id = i + data.id * 1000;

            array.markers.push_back(arrow);
        }
    }

    void PlanVisualizer::displayGoalPoint(const Eigen::Vector3d goal_point, const DisplayData &data)
    {
        if (goal_point_pub->get_subscription_count() == 0)
        {
            return;
        }
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = frame_id_;
        sphere.header.stamp = node_->now();
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.id = data.id;

        sphere.pose.orientation.w = 1.0;
        sphere.color.r = data.color(0);
        sphere.color.g = data.color(1);
        sphere.color.b = data.color(2);
        sphere.color.a = data.color(3);
        sphere.scale.x = data.scale;
        sphere.scale.y = data.scale;
        sphere.scale.z = data.scale;
        sphere.pose.position.x = goal_point(0);
        sphere.pose.position.y = goal_point(1);
        sphere.pose.position.z = goal_point(2);

        goal_point_pub->publish(sphere);
    }

    void PlanVisualizer::displayGlobalPathList(DisplayData &data)
    {

        if (global_list_pub->get_subscription_count() == 0)
        {
            return;
        }

        data.color(0) = 0.0;
        data.color(1) = 0.5;
        data.color(2) = 0.5;
        data.color(3) = 1.0;
        displayMarkerList(global_list_pub, data);
    }

    void PlanVisualizer::displayInitPathList(DisplayData &data)
    {
        if (init_list_pub->get_subscription_count() == 0)
        {
            return;
        }

        data.color(0) = 0.0;
        data.color(1) = 0.0;
        data.color(2) = 1.0;
        data.color(3) = 1.0;
        displayMarkerList(init_list_pub, data);
    }

    void PlanVisualizer::displayMultiOptimalPathList(const std::vector<std::vector<Eigen::Vector3d>> &optimal_trajs, double scale)
    {
        if (optimal_list_pub->get_subscription_count() == 0)
        {
            return;
        }

        static int last_nums = 0;

        // Clear previous markers by publishing empty markers with 0 alpha
        for (int id = 0; id < last_nums; id++)
        {
            DisplayData blank_data;
            blank_data.points = {};
            blank_data.scale = scale;
            blank_data.color = Eigen::Vector4d(0, 0, 0, 0);
            blank_data.id = id + 10;
            displayMarkerList(optimal_list_pub, blank_data, false);
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }
        last_nums = 0;

        // Publish new optimal trajectories
        for (int id = 0; id < static_cast<int>(optimal_trajs.size()); id++)
        {
            DisplayData data;
            data.points = optimal_trajs[id];
            data.scale = scale;
            data.color = Eigen::Vector4d(1, 0, 0, 0.7);
            data.id = id + 10;
            displayMarkerList(optimal_list_pub, data, false);
            rclcpp::sleep_for(std::chrono::milliseconds(1));
            last_nums++;
        }
    }

    void PlanVisualizer::displayOptimalList(const Eigen::MatrixXd &optimal_pts, int id)
    {

        if (optimal_list_pub->get_subscription_count() == 0)
        {
            return;
        }

        std::vector<Eigen::Vector3d> list;
        for (int i = 0; i < optimal_pts.cols(); i++)
        {
            Eigen::Vector3d pt = optimal_pts.col(i).transpose();
            list.push_back(pt);
        }
        DisplayData data;
        data.points = list;
        data.scale = 0.15;
        data.id = id;
        data.color = Eigen::Vector4d(0.2, 0.2, 0.8, 1);
        displayMarkerList(optimal_list_pub, data);
    }

    void PlanVisualizer::displayFailedList(const Eigen::MatrixXd &failed_pts, int id)
    {

        if (failed_list_pub->get_subscription_count() == 0)
        {
            return;
        }

        std::vector<Eigen::Vector3d> list;
        for (int i = 0; i < failed_pts.cols(); i++)
        {
            Eigen::Vector3d pt = failed_pts.col(i).transpose();
            list.push_back(pt);
        }
        DisplayData data;
        data.points = list;
        data.scale = 0.15;
        data.id = id;
        data.color = Eigen::Vector4d(0.3, 0, 0.3, 0.8);
        displayMarkerList(failed_list_pub, data);
    }

    void PlanVisualizer::displayArrowList(MarkerArrayPtr pub, const DisplayData &data)
    {
        visualization_msgs::msg::MarkerArray array;
        // clear
        pub->publish(array);
        generateArrowDisplayArray(array, data);
        pub->publish(array);
    }

} // namespace ego_planner