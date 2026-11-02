cmake_minimum_required(VERSION 3.8)
project(robot_raw_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_lidar REQUIRED)
find_package(robot_base REQUIRED)

add_executable(lidar_raw_bridge src/lidar_raw_bridge.cpp)
target_link_libraries(lidar_raw_bridge robot_lidar::robot_lidar_protocol)
ament_target_dependencies(lidar_raw_bridge
  rclcpp std_msgs sensor_msgs
)

add_executable(chassis_raw_bridge src/chassis_raw_bridge.cpp)
target_link_libraries(chassis_raw_bridge robot_base::robot_base_protocol)
ament_target_dependencies(chassis_raw_bridge
  rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros robot_interfaces
)

add_executable(cmd_raw_publisher src/cmd_raw_publisher.cpp)
target_link_libraries(cmd_raw_publisher robot_base::robot_base_protocol)
ament_target_dependencies(cmd_raw_publisher rclcpp std_msgs)

install(TARGETS lidar_raw_bridge chassis_raw_bridge cmd_raw_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
