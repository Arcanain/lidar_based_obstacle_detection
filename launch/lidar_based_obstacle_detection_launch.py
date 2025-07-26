from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # SLLIDAR launch
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_a1_launch.py'
            )
        )
    )

    # Obstacle Detection Node
    obstacle_node = Node(
        package='lidar_based_obstacle_detection',
        executable='lidar_based_obstacle_detection_node',
        name='lidar_based_obstacle_detection',
        parameters=[os.path.join(
            get_package_share_directory('lidar_based_obstacle_detection'),
            'config',
            'params.yaml'
        )],
        output='screen'
    )

    # RViz Node
    rviz_config_path = os.path.join(
        get_package_share_directory('lidar_based_obstacle_detection'),
        'rviz',
        'lidar_based_obstacle_detection.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        sllidar_launch,
        obstacle_node,
        #rviz_node
    ])
