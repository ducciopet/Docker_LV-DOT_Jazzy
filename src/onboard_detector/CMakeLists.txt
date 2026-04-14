cmake_minimum_required(VERSION 3.8)
project(onboard_detector)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "Suppress third-party CMake developer warnings")

# --------------------------------------------------
# Compile options
# --------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

# Set RPATH so binary can find libraries
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

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
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Open3D from /usr/local may export hardcoded LLVM libc++ paths that are not
# present on this image (e.g. /usr/lib/llvm-18/lib/libc++.so.1). Rewrite them
# to system locations when available.
if(TARGET Open3D::Open3D)
  get_target_property(_o3d_link_libs Open3D::Open3D INTERFACE_LINK_LIBRARIES)
  if(_o3d_link_libs)
    set(_o3d_link_libs_fixed "${_o3d_link_libs}")

    if(NOT EXISTS "/usr/lib/llvm-18/lib/libc++.so.1" AND EXISTS "/usr/lib/x86_64-linux-gnu/libc++.so.1")
      string(REPLACE "/usr/lib/llvm-18/lib/libc++.so.1" "/usr/lib/x86_64-linux-gnu/libc++.so.1" _o3d_link_libs_fixed "${_o3d_link_libs_fixed}")
    endif()

    if(NOT EXISTS "/usr/lib/llvm-18/lib/libc++abi.so" AND EXISTS "/usr/lib/x86_64-linux-gnu/libc++abi.so.1")
      string(REPLACE "/usr/lib/llvm-18/lib/libc++abi.so" "/usr/lib/x86_64-linux-gnu/libc++abi.so.1" _o3d_link_libs_fixed "${_o3d_link_libs_fixed}")
    endif()

    set_target_properties(Open3D::Open3D PROPERTIES INTERFACE_LINK_LIBRARIES "${_o3d_link_libs_fixed}")
  endif()
endif()

# For services
find_package(rosidl_default_generators REQUIRED)

# --------------------------------------------------
# Service definition
# --------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDynamicObstacles.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

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
  include/${PROJECT_NAME}/dynamicDetector.cpp
  include/${PROJECT_NAME}/uvDetector.cpp
  include/${PROJECT_NAME}/dbscan.cpp
  include/${PROJECT_NAME}/kalmanFilter.cpp
  include/${PROJECT_NAME}/lidarDetector.cpp
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
)

target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${PROJECT_NAME}_lib
  "${cpp_typesupport_target}")

# --------------------------------------------------
# Executable
# --------------------------------------------------
add_executable(detector_node src/detector_node.cpp)

target_link_libraries(detector_node ${PROJECT_NAME}_lib)
ament_target_dependencies(detector_node
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
)

# Calibration node executable
add_executable(calibration_node src/calibration_node.cpp)

ament_target_dependencies(calibration_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  pcl_conversions
  tf2
  tf2_geometry_msgs
)

target_link_libraries(calibration_node
  ${PCL_LIBRARIES}
)

# Calibration ICP node executable
add_executable(calibration_icp_node src/calibration_icp_node.cpp)

ament_target_dependencies(calibration_icp_node
  rclcpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_filters
  pcl_conversions
  tf2
  tf2_geometry_msgs
)

target_link_libraries(calibration_icp_node
  ${PCL_LIBRARIES}
  Open3D::Open3D
)

# Wall detector node executable (standalone, under scripts/wall_detector/)
add_executable(wall_detector_node
  scripts/wall_detector/wall_detector_node.cpp
  scripts/wall_detector/wallDetector.cpp
)
target_include_directories(wall_detector_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/wall_detector
)
target_link_libraries(wall_detector_node ${PCL_LIBRARIES})
ament_target_dependencies(wall_detector_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_conversions
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# --------------------------------------------------
# Install
# --------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}_lib
  detector_node
  calibration_node
  calibration_icp_node
  wall_detector_node
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

# Install Python node entrypoints as executables
install(PROGRAMS
  scripts/yolo_detector/yolo_detector_node.py
  scripts/yolo_detector/yolov11_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install all scripts (for weights, modules, etc.)
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

# --------------------------------------------------
# Export
# --------------------------------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_package()