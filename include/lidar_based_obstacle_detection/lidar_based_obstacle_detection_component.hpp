#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>

namespace lidar_based_obstacle_detection {

class LidarBasedObstacleDetection : public rclcpp::Node
{
public:
    explicit LidarBasedObstacleDetection(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void filter_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void createFOVMarker(visualization_msgs::msg::Marker &marker,
        float forward_min_x, float forward_max_x,
        float lateral_half_width,
        const std::string &frame_id,
        const std_msgs::msg::ColorRGBA &color);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr closest_point_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_marker_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_pub;

    float threshold_;
    float min_threshold_;
    float forward_angle_min_;
    float forward_angle_max_;
    double forward_min_x_;
    double forward_max_x_;
    double lateral_half_width_;
    int num_points_;
    int downsample_rate_;
    std::string frame_id_;

};

} // namespace lidar_based_obstacle_detection