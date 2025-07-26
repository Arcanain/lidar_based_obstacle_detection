cmake_minimum_required(VERSION 3.10)
project(lidar_based_obstacle_detection)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# Include directories
include_directories(include)

# Component library
add_library(${PROJECT_NAME} SHARED
  src/lidar_based_obstacle_detection_component.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  rclcpp_components
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "lidar_based_obstacle_detection::LidarBasedObstacleDetection"
)

# Node wrapper (main executable)
add_executable(${PROJECT_NAME}_node
  src/lidar_based_obstacle_detection.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rclcpp_components
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(rclcpp_components)

ament_package()
