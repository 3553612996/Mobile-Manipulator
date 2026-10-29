cmake_minimum_required(VERSION 3.8)
project(robot_base)

find_package(ament_cmake REQUIRED)

# M1 只建立包骨架。节点与库在 M3 加入后，再补 find_package 和 install 规则。
ament_package()
