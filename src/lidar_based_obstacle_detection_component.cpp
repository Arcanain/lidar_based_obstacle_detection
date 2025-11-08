#include "lidar_based_obstacle_detection/lidar_based_obstacle_detection_component.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace lidar_based_obstacle_detection {

    LidarBasedObstacleDetection::LidarBasedObstacleDetection(const rclcpp::NodeOptions & options)
    : Node("lidar_based_obstacle_detection", options)
    {
        //デフォルト値がないと動かない
        this->declare_parameter<double>("threshold", 1.0);
        this->declare_parameter<double>("min_threshold", 0.3);
        this->declare_parameter<double>("forward_angle_min", -0.785398); // -π/4
        forward_angle_max_ = this->declare_parameter<double>("forward_angle_max", 0.785398);  // π/4
        num_points_ = static_cast<size_t>(this->declare_parameter<int>("num_points", 30));
        this->declare_parameter("frame_id", "laser");

        this->declare_parameter<int>("downsample_rate", 5);

        this->get_parameter("threshold", threshold_);
        this->get_parameter("min_threshold", min_threshold_);
        this->get_parameter("forward_angle_min", forward_angle_min_);
        this->get_parameter("forward_angle_max", forward_angle_max_);
        this->get_parameter("num_points", num_points_);
        this->get_parameter("frame_id", frame_id_);

        this->get_parameter("downsample_rate", downsample_rate_);

        

        RCLCPP_INFO(this->get_logger(), "Using frame_id: %s", frame_id_.c_str());

        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarBasedObstacleDetection::filter_callback, this, _1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/closest_point_marker", 10);
        closest_point_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/closest_point", 10);
        fov_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/fov_marker", 10);
        obstacle_detected_pub = this->create_publisher<std_msgs::msg::Bool>("obstacle_detected", 10);
    }

    void LidarBasedObstacleDetection::filter_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // --- 追加パラメータ ---
        this->get_parameter("downsample_rate", downsample_rate_);
        int downsample_rate = downsample_rate_;
        auto filtered_msg = *msg;
    
        // rangesを間引く
        std::vector<float> downsampled_ranges;
        downsampled_ranges.reserve(msg->ranges.size() / downsample_rate);
    
        float new_angle_min = msg->angle_min;
        float new_angle_increment = msg->angle_increment * downsample_rate;
    
        for (size_t i = 0; i < msg->ranges.size(); i += downsample_rate)
        {
            downsampled_ranges.push_back(msg->ranges[i]);
        }
    
        // downsampled_rangesをfiltered_msgに反映
        filtered_msg.ranges = downsampled_ranges;
        filtered_msg.angle_min = new_angle_min;
        filtered_msg.angle_increment = new_angle_increment;
        filtered_msg.angle_max = new_angle_min + (downsampled_ranges.size() - 1) * new_angle_increment;
    
        // ---- 以下、元の処理をそのまま利用 ----
        float closest_distance = std::numeric_limits<float>::infinity();
        float closest_x = 0.0;
        float closest_y = 0.0;
        bool obstacle_detected = false;
    
        for (size_t i = 0; i < filtered_msg.ranges.size(); ++i)
        {
            float angle = filtered_msg.angle_min + i * filtered_msg.angle_increment;
            float range = filtered_msg.ranges[i];
    
            if (std::isnan(range) || range <= 0.0) {
                filtered_msg.ranges[i] = std::numeric_limits<float>::infinity();
                continue;
            }
    
            float x = range * std::cos(angle);
            float y = range * std::sin(angle);
            float distance = std::hypot(x, y);
    
            if (distance < min_threshold_ || distance > threshold_ ||
                angle < forward_angle_min_ || angle > forward_angle_max_)
            {
                filtered_msg.ranges[i] = std::numeric_limits<float>::infinity();
            }
            else
            {
                if (distance < closest_distance)
                {
                    closest_distance = distance;
                    closest_x = x;
                    closest_y = y;
                }
                obstacle_detected = true;
            }
        }

        // 最も近い点の可視化マーカー
        visualization_msgs::msg::MarkerArray marker_array;
        if (closest_distance != std::numeric_limits<float>::infinity())
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "closest_point";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = closest_x;
            marker.pose.position.y = closest_y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            marker_array.markers.push_back(marker);

            geometry_msgs::msg::Point point;
            point.x = closest_x;
            point.y = closest_y;
            point.z = 0.0;
            closest_point_publisher_->publish(point);
        }
        else
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "closest_point";
            marker.id = 0;
            marker.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(marker);
        }

        marker_publisher_->publish(marker_array);

        std_msgs::msg::Bool flag;
        flag.data = obstacle_detected;
        obstacle_detected_pub->publish(flag);

        // 前方扇形のマーカー
        std_msgs::msg::ColorRGBA green;
        green.a = 0.5; green.r = 0.0; green.g = 1.0; green.b = 1.0;

        visualization_msgs::msg::Marker fov_marker;
        createFOVMarker(fov_marker, forward_angle_min_, forward_angle_max_,
                        min_threshold_, threshold_, num_points_, frame_id_, green);
        fov_marker.ns = "fov"; fov_marker.id = 1;
        fov_marker_publisher_->publish(fov_marker);

        publisher_->publish(filtered_msg);
    }

    void LidarBasedObstacleDetection::createFOVMarker(visualization_msgs::msg::Marker &marker, float angle_min, float angle_max,
        float min_radius, float max_radius, size_t num_points,
        const std::string &frame_id, const std_msgs::msg::ColorRGBA &color)
    {
        marker.header.frame_id = frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
        marker.color = color;

        std::vector<geometry_msgs::msg::Point> inner, outer;
        float step = (angle_max - angle_min) / num_points;

        for (size_t i = 0; i <= num_points; ++i)
        {
            float a = angle_min + i * step;

            geometry_msgs::msg::Point p1, p2;
            p1.x = min_radius * std::cos(a); p1.y = min_radius * std::sin(a);
            p2.x = max_radius * std::cos(a); p2.y = max_radius * std::sin(a);
            p1.z = p2.z = 0.0;
            inner.push_back(p1);
            outer.push_back(p2);
        }

        for (size_t i = 0; i < num_points; ++i)
        {
            marker.points.push_back(inner[i]);
            marker.points.push_back(outer[i]);
            marker.points.push_back(inner[i+1]);

            marker.points.push_back(inner[i+1]);
            marker.points.push_back(outer[i]);
            marker.points.push_back(outer[i+1]);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr closest_point_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_marker_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_detected_pub;

    double threshold_;
    double min_threshold_;
    double forward_angle_min_;
    double forward_angle_max_;
    int num_points_;
    int downsample_rate_;
    std::string frame_id_;

}  // namespace lidar_based_obstacle_detection
