cmake_minimum_required(VERSION 3.8)
project(obstacle_avoidance)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "Suppress third-party CMake developer warnings")

# --------------------------------------------------
# Find dependencies
# --------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)
unset(FLANN_ROOT)
find_package(PCL REQUIRED)
set(CMAKE_WARN_DEPRECATED FALSE)
find_package(Open3D REQUIRED)
set(CMAKE_WARN_DEPRECATED TRUE)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --------------------------------------------------
# Include directories
# --------------------------------------------------
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# --------------------------------------------------
# Library
# --------------------------------------------------
add_library(${PROJECT_NAME}_lib
  include/${PROJECT_NAME}/obstacleAvoider.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  image_transport
  vision_msgs
  pcl_conversions
  tf2
  tf2_geometry_msgs
  PCL
  Open3D
)
target_link_libraries(${PROJECT_NAME}_lib
  Open3D::Open3D
  ${PCL_LIBRARIES}
)

# --------------------------------------------------
# Executable
# --------------------------------------------------
add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
ament_target_dependencies(obstacle_avoidance_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  image_transport
  vision_msgs
  pcl_conversions
  tf2
  tf2_geometry_msgs
  PCL
  Open3D
)
target_link_libraries(obstacle_avoidance_node
  ${PROJECT_NAME}_lib
)

# --------------------------------------------------
# Install
# --------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}_lib
  obstacle_avoidance_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# --------------------------------------------------
# Export
# --------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_package()