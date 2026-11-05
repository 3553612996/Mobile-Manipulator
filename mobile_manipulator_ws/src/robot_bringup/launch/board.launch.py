"""RK3566 板端：只负责底盘和雷达，发 /odom、/scan 和 TF。

板端不做建图/导航（2+8G 存储留给后续 4+32G 版本），PC 端通过 DDS 接收：

  /odom, TF odom -> base_link       base_bridge
  /scan, /lidar/points              x2_lidar
  /robot_description, TF base_link -> laser_link/wheels   robot_state_publisher
  /robot_status, /system_event      base_bridge
  /cmd_vel 订阅                     base_bridge（PC 的 Nav2 下发）

用法：
  ros2 launch robot_bringup board.launch.py
  ros2 launch robot_bringup board.launch.py \
      base_port:=/dev/robot_stm32 lidar_port:=/dev/robot_lidar
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("robot_bringup"))
    base_config = share / "config" / "base.yaml"
    lidar_config = share / "config" / "lidar.yaml"
    urdf = share / "urdf" / "robot.urdf.xacro"

    base_port = LaunchConfiguration("base_port")
    lidar_port = LaunchConfiguration("lidar_port")

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
                description="X2 lidar serial device",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(["xacro ", str(urdf)]), value_type=str
                        )
                    }
                ],
            ),
            Node(
                package="robot_base",
                executable="base_bridge",
                name="base_bridge",
                output="screen",
                parameters=[str(base_config), {"port": base_port}],
            ),
            Node(
                package="robot_lidar",
                executable="x2_lidar",
                name="x2_lidar",
                output="screen",
                parameters=[
                    str(lidar_config),
                    {"port": lidar_port, "motor_dtr": True},
                ],
            ),
        ]
    )
