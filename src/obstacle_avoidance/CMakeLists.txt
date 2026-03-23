cmake_minimum_required(VERSION 3.8)
project(obstacle_avoidance)

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

ament_export_include_directories(include)
ament_package()