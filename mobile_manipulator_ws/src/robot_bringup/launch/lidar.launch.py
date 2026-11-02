"""YDLIDAR X2 驱动 + 可选 RViz2。

启动后：
  /scan          sensor_msgs/LaserScan   一圈扫描
  /lidar/points  sensor_msgs/PointCloud2 同一圈的点云

用法：
  ros2 launch robot_bringup lidar.launch.py
  ros2 launch robot_bringup lidar.launch.py port:=/dev/ttyUSB0 rviz:=true

RViz2 的固定坐标系是 laser_link；等 URDF 提供 base_link -> laser_link
后再切到 base_link。
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
    config = share / "config" / "lidar.yaml"
    rviz_config = share / "config" / "lidar.rviz"

    port = LaunchConfiguration("port")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/dev/robot_lidar",
                description="X2 serial device; use /dev/ttyUSB0 on the PC bench",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz2 with config/lidar.rviz",
            ),
            Node(
                package="robot_lidar",
                executable="x2_lidar",
                name="x2_lidar",
                output="screen",
                parameters=[str(config), {"port": port}],
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
