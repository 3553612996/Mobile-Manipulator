cmake_minimum_required(VERSION 3.8)
project(robot_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Threads REQUIRED)

# Frame codec. No rclcpp dependency on purpose: it is tested on a PC without
# ROS and can later be reused by a standalone transport node.
add_library(robot_base_protocol src/protocol.cpp)
target_include_directories(robot_base_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# UART ownership plus the small helpers that do not need a node.
add_library(robot_base_link
  src/stm32_link.cpp
  src/termios_serial_io.cpp
  src/cmd_vel_monitor.cpp
)
target_link_libraries(robot_base_link PUBLIC robot_base_protocol Threads::Threads)

add_executable(base_bridge src/base_bridge.cpp)
target_link_libraries(base_bridge robot_base_link)
ament_target_dependencies(base_bridge
  rclcpp
  geometry_msgs
  nav_msgs
  std_srvs
  robot_interfaces
  tf2
  tf2_ros
  diagnostic_msgs
)

install(TARGETS base_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS robot_base_protocol robot_base_link
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  std_srvs
  robot_interfaces
  tf2
  tf2_ros
  diagnostic_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol robot_base_protocol)

  ament_add_gtest(test_heading_unwrapper test/test_heading_unwrapper.cpp)
  target_link_libraries(test_heading_unwrapper robot_base_protocol)

  ament_add_gtest(test_cmd_vel_monitor test/test_cmd_vel_monitor.cpp)
  target_link_libraries(test_cmd_vel_monitor robot_base_link)

  ament_add_gtest(test_stm32_link test/test_stm32_link.cpp)
  target_link_libraries(test_stm32_link robot_base_link)
endif()

ament_package()
