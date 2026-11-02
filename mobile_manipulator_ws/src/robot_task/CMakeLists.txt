cmake_minimum_required(VERSION 3.8)
project(robot_task)

find_package(ament_cmake REQUIRED)

# M1 只建立包骨架。状态机在 M8 前保持纯逻辑，可在 x86 上单测。
ament_package()
