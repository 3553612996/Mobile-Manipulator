"""轮式里程计 + X2 雷达建图。

启动链：
  robot_state_publisher   base_link → laser_link / wheel_*（URDF）
  base_bridge             /cmd_vel、/odom、odom → base_link
  x2_lidar                /scan（frame=laser_link）、/lidar/points
  slam_toolbox            /scan + odom→base_link → /map + map→odom
  rviz2                   config/mapping.rviz

用法（PC 上两个串口按实际设备改）：
  ros2 launch robot_bringup mapping.launch.py \
      base_port:=/dev/robot_stm32 lidar_port:=/dev/ttyUSB0 rviz:=true

保存地图：
  ros2 run nav2_map_server map_saver_cli -f maps/<名字>

注意：本 launch 不使用 IMU/EKF，`odom→base_link` 只由 base_bridge 发布。
以后加 EKF 时，把 base_bridge 的 publish_tf 设为 false，由 EKF 发。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("robot_bringup"))
    base_launch = share / "launch" / "base.launch.py"
    lidar_launch = share / "launch" / "lidar.launch.py"
    slam_params = share / "config" / "slam_toolbox.yaml"
    rviz_config = share / "config" / "mapping.rviz"

    base_port = LaunchConfiguration("base_port")
    lidar_port = LaunchConfiguration("lidar_port")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "base_port",
                default_value="/dev/robot_stm32",
                description="STM32 chassis serial device",
            ),
            DeclareLaunchArgument(
                "lidar_port",
                default_value="/dev/robot_lidar",
                description="X2 lidar serial device; /dev/ttyUSB0 on the PC bench",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2 with config/mapping.rviz",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(base_launch)),
                launch_arguments={"port": base_port, "rviz": "false"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(lidar_launch)),
                launch_arguments={"port": lidar_port, "rviz": "false"}.items(),
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
