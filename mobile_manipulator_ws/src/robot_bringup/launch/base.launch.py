"""底盘串口桥接 + 机器人模型 + 可选 RViz2。参数全部来自 config/base.yaml。

启动内容：
  robot_state_publisher  从 urdf/robot.urdf.xacro 发 /tf_static（base_link →
                         wheel_* / laser_link）与 /robot_description
  base_bridge            独占 /dev/robot_stm32，发 odom → base_link 与 /odom

只启动 base_bridge：它独占串口。跑 tools/bench/proto_loop.py 等台架脚本前
必须先停掉它，两个进程抢同一个口的现象是 CRC 全崩。

用法：
  ros2 launch robot_bringup base.launch.py
  ros2 launch robot_bringup base.launch.py rviz:=true
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
    config = share / "config" / "base.yaml"
    urdf = share / "urdf" / "robot.urdf.xacro"
    rviz_config = share / "config" / "robot.rviz"

    rviz = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz2 with config/robot.rviz",
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
                parameters=[str(config)],
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
