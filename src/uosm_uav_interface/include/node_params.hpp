#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <type_traits>

namespace uosm
{
namespace util
{

/**
 * Declare a ROS 2 parameter, retrieve its value (YAML override or default),
 * and optionally log it in one call.
 *
 * @param node       Pointer to the owning node (pass `this` from a Node subclass).
 * @param name       Fully qualified parameter name (e.g. "fc.takeoff_height").
 * @param def        Default value used when no YAML / launch override exists.
 * @param label      If non-empty, an INFO line is printed:  "<label><value>".
 * @return           The resolved parameter value.
 */
template <typename T>
T getParam(rclcpp::Node *node, const std::string &name, const T &def,
           const std::string &label = "")
{
    auto val = node->declare_parameter<T>(name, def);

    if (!label.empty())
    {
        if constexpr (std::is_same_v<T, bool>)
        {
            RCLCPP_INFO(node->get_logger(), "%s%s", label.c_str(),
                        val ? "true" : "false");
        }
        else if constexpr (std::is_same_v<T, std::string>)
        {
            RCLCPP_INFO(node->get_logger(), "%s%s", label.c_str(), val.c_str());
        }
        else if constexpr (std::is_floating_point_v<T>)
        {
            RCLCPP_INFO(node->get_logger(), "%s%.2f", label.c_str(),
                        static_cast<double>(val));
        }
        else if constexpr (std::is_integral_v<T>)
        {
            RCLCPP_INFO(node->get_logger(), "%s%d", label.c_str(),
                        static_cast<int>(val));
        }
    }

    return val;
}

template <typename T>
T getParam(const rclcpp::Node::SharedPtr &node, const std::string &name,
           const T &def, const std::string &label = "")
{
    return getParam<T>(node.get(), name, def, label);
}

}  // namespace util
}  // namespace uosm
