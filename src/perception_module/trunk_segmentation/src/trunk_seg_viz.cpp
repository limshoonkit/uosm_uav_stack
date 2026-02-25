#include "../include/trunk_seg_viz.hpp"

std_msgs::msg::ColorRGBA generateRandomColor(size_t id)
{
    std_msgs::msg::ColorRGBA color;
    color.a = 0.7; // Alpha channel

    // Generate colors using HSV
    float hue = fmod(id * 137.5, 360.0);
    hsvToRgb(hue, 0.8, 0.9, color.r, color.g, color.b);

    return color;
}

void hsvToRgb(float h, float s, float v, float &r, float &g, float &b)
{
    h = fmod(h, 360.0);
    float c = v * s;
    float x = c * (1.0 - fabs(fmod(h / 60.0, 2.0) - 1.0));
    float m = v - c;

    if (h >= 0 && h < 60)
    {
        r = c;
        g = x;
        b = 0;
    }
    else if (h >= 60 && h < 120)
    {
        r = x;
        g = c;
        b = 0;
    }
    else if (h >= 120 && h < 180)
    {
        r = 0;
        g = c;
        b = x;
    }
    else if (h >= 180 && h < 240)
    {
        r = 0;
        g = x;
        b = c;
    }
    else if (h >= 240 && h < 300)
    {
        r = x;
        g = 0;
        b = c;
    }
    else
    {
        r = c;
        g = 0;
        b = x;
    }

    r += m;
    g += m;
    b += m;
}

void clearMarkers(visualization_msgs::msg::MarkerArray &marker_array,
                  const std::string &ns,
                  const std::string &map_frame,
                  const rclcpp::Clock::SharedPtr &clock)
{
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = map_frame;
    clear_marker.header.stamp = clock->now();
    clear_marker.ns = ns;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
}

visualization_msgs::msg::Marker createLineMarker(
    int id, double x, double y, double z,
    std_msgs::msg::ColorRGBA color,
    double scale, const std::string &ns,
    const std::string &map_frame, const rclcpp::Clock::SharedPtr &clock)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame;
    marker.header.stamp = clock->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    return marker;
}

visualization_msgs::msg::Marker createTextMarker(
    int id, double x, double y, double z,
    double r, double g, double b, double a,
    const std::string &text, double scale, double height,
    const std::string &ns, const std::string &map_frame, const rclcpp::Clock::SharedPtr &clock)
{
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = map_frame;
    text_marker.header.stamp = clock->now();
    text_marker.ns = ns;
    text_marker.id = id;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    text_marker.pose.position.x = x;
    text_marker.pose.position.y = y;
    text_marker.pose.position.z = z;
    text_marker.pose.orientation.w = 1.0;

    text_marker.color.r = r;
    text_marker.color.g = g;
    text_marker.color.b = b;
    text_marker.color.a = a;

    text_marker.scale.x = scale;
    text_marker.scale.y = scale;
    text_marker.scale.z = height;
    text_marker.text = text;

    text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    return text_marker;
}

visualization_msgs::msg::Marker createCylinderMarker(
    int id, double x, double y, double z, double radius, double height,
    double r, double g, double b, double a,
    const std::string &ns, const std::string &map_frame, const rclcpp::Clock::SharedPtr &clock)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame;
    marker.header.stamp = clock->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = radius * 2.0; // Diameter
    marker.scale.y = radius * 2.0; // Diameter
    marker.scale.z = height;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    return marker;
}

visualization_msgs::msg::Marker createSphereMarker(
    int id, double x, double y, double z,
    double scale_x, double scale_y, double scale_z,
    geometry_msgs::msg::Quaternion orientation,
    double r, double g, double b, double a,
    const std::string &ns, const std::string &map_frame, const rclcpp::Clock::SharedPtr &clock)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame;
    marker.header.stamp = clock->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation = orientation;

    marker.scale.x = scale_x;
    marker.scale.y = scale_y;
    marker.scale.z = scale_z;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    return marker;
}