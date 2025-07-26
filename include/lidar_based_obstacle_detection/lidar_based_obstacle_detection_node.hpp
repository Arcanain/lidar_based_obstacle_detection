#pragma once

#include <rclcpp/rclcpp.hpp>

namespace lidar_based_obstacle_detection {
    std::shared_ptr<rclcpp::Node> create_node();
}