cmake_minimum_required(VERSION 3.8)
project(robot_arm)

find_package(ament_cmake REQUIRED)

# M1 只建立包骨架。关节控制与抓取动作在 M8 加入后，再补编译目标。
ament_package()
