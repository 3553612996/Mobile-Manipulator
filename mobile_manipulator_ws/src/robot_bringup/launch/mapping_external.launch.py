"""使用主机转发来的原始字节建图（本机不开串口）。

输入（std_msgs/msg/UInt8MultiArray）：
  /lidar/raw    X2 原始包流（AA 55 ...）
  /stm32/raw    STM32 协议原始帧流（AA 55 ...）

输出/建图链：
  lidar_raw_bridge   -> /scan、/lidar/points
  chassis_raw_bridge -> /odom、TF odom->base_link、/robot_status
  slam_toolbox       -> /map、TF map->odom
  rviz2              -> config/mapping.rviz

用法：
  ros2 launch robot_bringup mapping_external.launch.py
  ros2 launch robot_bringup mapping_external.launch.py \
      lidar_raw_topic:=/lidar/raw chassis_raw_topic:=/stm32/raw rviz:=true
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("robot_bringup"))
    urdf = share / "urdf" / "robot.urdf.xacro"
    slam_params = share / "config" / "slam_toolbox.yaml"
    rviz_config = share / "config" / "mapping.rviz"

    lidar_raw_topic = LaunchConfiguration("lidar_raw_topic")
    chassis_raw_topic = LaunchConfiguration("chassis_raw_topic")
    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lidar_raw_topic",
                default_value="/lidar/raw",
                description="Host topic carrying raw X2 bytes",
            ),
            DeclareLaunchArgument(
                "chassis_raw_topic",
                default_value="/stm32/raw",
                description="Host topic carrying raw STM32 frames",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2 with config/mapping.rviz",
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
                package="robot_raw_bridge",
                executable="lidar_raw_bridge",
                name="lidar_raw_bridge",
                output="screen",
                parameters=[
                    {"input_topic": lidar_raw_topic},
                    {"frame_id": "laser_link"},
                    {"scan_bins": 720},
                    {"range_min_m": 0.12},
                    {"range_max_m": 8.0},
                    {"invert_angle": True},
                ],
            ),
            Node(
                package="robot_raw_bridge",
                executable="chassis_raw_bridge",
                name="chassis_raw_bridge",
                output="screen",
                parameters=[
                    {"input_topic": chassis_raw_topic},
                    {"odom_frame_id": "odom"},
                    {"base_frame_id": "base_link"},
                    {"wheel_separation_m": 0.165},
                    {"publish_tf": True},
                ],
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
