"""底盘串口桥接。参数全部来自 config/base.yaml，不在 launch 里硬编码。

只启动 robot_base 的 base_bridge：它独占 /dev/robot_stm32。跑
tools/bench/proto_loop.py 等台架脚本前必须先停掉它，两个进程抢同一个口
的现象是 CRC 全崩。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("robot_bringup")) / "config" / "base.yaml"

    return LaunchDescription(
        [
            Node(
                package="robot_base",
                executable="base_bridge",
                name="base_bridge",
                output="screen",
                parameters=[str(config)],
            ),
        ]
    )
