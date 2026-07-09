cmake_minimum_required(VERSION 3.8)
project(bring_up)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Executable
add_executable(odom_monitor src/odom_monitor.cpp)
ament_target_dependencies(odom_monitor rclcpp nav_msgs)

add_executable(map_monitor src/map_monitor.cpp)
ament_target_dependencies(map_monitor rclcpp tf2_ros geometry_msgs)

add_executable(lidar_global_position_publisher src/lidar_global_position_publisher.cpp)
ament_target_dependencies(lidar_global_position_publisher rclcpp tf2_ros geometry_msgs)

# Install
install(TARGETS
  odom_monitor
  map_monitor
  lidar_global_position_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/odin_map_save.py
  scripts/print_global_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
