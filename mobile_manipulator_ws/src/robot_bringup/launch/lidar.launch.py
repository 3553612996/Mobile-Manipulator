"""YDLIDAR X2 驱动。参数来自 config/lidar.yaml。

启动后：
  /scan          sensor_msgs/LaserScan   一圈扫描
  /lidar/points  sensor_msgs/PointCloud2 同一圈的点云（RViz 直接看）

RViz 的 Fixed Frame 先设为 laser_link；等 URDF 提供 base_link -> laser_link
后再切到 base_link。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("robot_bringup")) / "config" / "lidar.yaml"

    return LaunchDescription(
        [
            Node(
                package="robot_lidar",
                executable="x2_lidar",
                name="x2_lidar",
                output="screen",
                parameters=[str(config)],
            ),
        ]
    )
