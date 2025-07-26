#include "lidar_based_obstacle_detection/lidar_based_obstacle_detection_component.hpp"
#include "lidar_based_obstacle_detection/lidar_based_obstacle_detection_node.hpp"

namespace lidar_based_obstacle_detection {

std::shared_ptr<rclcpp::Node> create_node()
{
    return std::make_shared<LidarBasedObstacleDetection>();
}

}