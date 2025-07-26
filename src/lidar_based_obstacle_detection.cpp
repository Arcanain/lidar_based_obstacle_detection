#include "rclcpp/rclcpp.hpp"
#include "lidar_based_obstacle_detection/lidar_based_obstacle_detection_component.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lidar_based_obstacle_detection::LidarBasedObstacleDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
