#ifndef TRUNK_SEG_VIZ_HPP
#define TRUNK_SEG_VIZ_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <math.h>

std_msgs::msg::ColorRGBA generateRandomColor(size_t id);
void hsvToRgb(float h, float s, float v, float &r, float &g, float &b);

void clearMarkers(visualization_msgs::msg::MarkerArray &marker_array,
                  const std::string &ns,
                  const std::string &map_frame,
                  const rclcpp::Clock::SharedPtr &clock);

visualization_msgs::msg::Marker createLineMarker(int id, double x, double y, double z,
                                                 std_msgs::msg::ColorRGBA color,
                                                 double scale, const std::string &ns,
                                                 const std::string &map_frame,
                                                 const rclcpp::Clock::SharedPtr &clock);

visualization_msgs::msg::Marker createTextMarker(int id, double x, double y, double z,
                                                 double r, double g, double b, double a,
                                                 const std::string &text, double scale, double height,
                                                 const std::string &ns,
                                                 const std::string &map_frame,
                                                 const rclcpp::Clock::SharedPtr &clock);

visualization_msgs::msg::Marker createCylinderMarker(int id, double x, double y, double z,
                                                     double radius, double height,
                                                     double r, double g, double b, double a,
                                                     const std::string &ns,
                                                     const std::string &map_frame,
                                                     const rclcpp::Clock::SharedPtr &clock);

visualization_msgs::msg::Marker createSphereMarker(int id, double x, double y, double z,
                                                   double scale_x, double scale_y, double scale_z,
                                                   geometry_msgs::msg::Quaternion orientation,
                                                   double r, double g, double b, double a,
                                                   const std::string &ns,
                                                   const std::string &map_frame,
                                                   const rclcpp::Clock::SharedPtr &clock);

#endif // TRUNK_SEG_VIZ_HPP