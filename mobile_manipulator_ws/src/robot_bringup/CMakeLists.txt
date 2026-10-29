cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".gitkeep" EXCLUDE
)

ament_package()
