cmake_minimum_required(VERSION 3.5)
project(radar_cable_follower)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(radar_cable_follower_msgs REQUIRED)

find_package(PCL 1.2 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(drone_frame_broadcaster
  src/drone_frame_broadcaster_node.cpp
  src/geometry.cpp
)
  ament_target_dependencies(drone_frame_broadcaster
  rclcpp
  Eigen3
  geometry_msgs
  px4_msgs
  tf2
  tf2_ros
)
install(TARGETS drone_frame_broadcaster DESTINATION lib/${PROJECT_NAME})


add_executable(offboard_control
  src/offboard_control.cpp
  src/geometry.cpp
)
  ament_target_dependencies(offboard_control
  rclcpp
  Eigen3
  geometry_msgs
  px4_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_msgs
  radar_cable_follower_msgs
)
install(TARGETS offboard_control DESTINATION lib/${PROJECT_NAME})


add_executable(radar_pointcloud_filter
src/radar_pointcloud_filter.cpp
src/geometry.cpp
)
target_link_libraries (radar_pointcloud_filter 
${PCL_LIBRARIES}
)
  ament_target_dependencies(radar_pointcloud_filter
  rclcpp
  sensor_msgs
  px4_msgs
  tf2
  tf2_ros
  tf2_msgs
  OpenCV
  cv_bridge
  radar_cable_follower_msgs
  PCL
)
install(TARGETS radar_pointcloud_filter DESTINATION lib/${PROJECT_NAME})



add_executable(lidar_to_mmwave_node
src/lidar_to_mmwave.cpp
)
  ament_target_dependencies(lidar_to_mmwave_node
  rclcpp sensor_msgs
  geometry_msgs
)
install(TARGETS lidar_to_mmwave_node DESTINATION lib/${PROJECT_NAME})



############
# Install ##
############
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/image_drawer.py
  scripts/image_decompress.py
  scripts/draw_PL_IDs.py
  scripts/print_PL_IDs.py
  scripts/image_drawer_HW_compressed.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export information to downstream packages
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs OpenCV)
else()
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime Eigen3 px4_msgs geometry_msgs sensor_msgs)
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  )


ament_package()

