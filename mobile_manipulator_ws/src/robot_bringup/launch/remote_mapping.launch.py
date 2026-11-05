"""PC 端：接收板卡发来的 /scan、/odom 和 TF，运行 SLAM Toolbox + RViz。

板卡端先运行 board.launch.py；PC 端不要启动 robot_state_publisher/base_bridge/
x2_lidar，否则会出现重复的 TF 发布者。

用法：
  ros2 launch robot_bringup remote_mapping.launch.py
  ros2 launch robot_bringup remote_mapping.launch.py rviz:=false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("robot_bringup"))
    slam_params = share / "config" / "slam_toolbox.yaml"
    rviz_config = share / "config" / "mapping.rviz"

    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2 with config/mapping.rviz",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[str(slam_params), {"use_sim_time": False}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
                condition=IfCondition(rviz),
            ),
        ]
    )
