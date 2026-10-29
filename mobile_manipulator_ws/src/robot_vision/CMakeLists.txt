cmake_minimum_required(VERSION 3.8)
project(robot_vision)

find_package(ament_cmake REQUIRED)

# M1 只建立包骨架。推理节点与 RKNN 依赖在 M7 加入后，再补编译目标。
ament_package()
