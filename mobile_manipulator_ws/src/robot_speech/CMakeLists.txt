cmake_minimum_required(VERSION 3.8)
project(robot_speech)

find_package(ament_cmake REQUIRED)

# M1 只建立包骨架。语音节点与 Vosk 依赖在 M9 加入后，再补编译目标。
ament_package()
