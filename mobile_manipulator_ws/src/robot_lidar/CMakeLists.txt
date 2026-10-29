cmake_minimum_required(VERSION 3.8)
project(robot_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# X2 codec: pure C++, no ROS and no serial dependency.
add_library(robot_lidar_protocol src/x2_protocol.cpp)
target_include_directories(robot_lidar_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# UART wrapper with DTR control.
add_library(robot_lidar_serial src/x2_serial.cpp)
target_link_libraries(robot_lidar_serial PUBLIC robot_lidar_protocol)

add_executable(x2_lidar src/x2_node.cpp)
target_link_libraries(x2_lidar robot_lidar_serial)
ament_target_dependencies(x2_lidar rclcpp sensor_msgs)

install(TARGETS x2_lidar
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS robot_lidar_protocol robot_lidar_serial
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_x2_protocol test/test_x2_protocol.cpp)
  target_link_libraries(test_x2_protocol robot_lidar_protocol)
endif()

ament_package()
