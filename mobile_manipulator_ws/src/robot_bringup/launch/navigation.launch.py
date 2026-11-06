"""PC 端 Nav2：AMCL 定位 + 全局/局部规划 + DWB 控制 + 恢复行为。

分工（M4 结论）：
  RK3566  board.launch.py    /odom、/scan、TF（odom → base_link → laser_link）
  PC      本 launch          Nav2 + AMCL，输出 /cmd_vel 给板端 base_bridge

前提：
  1. 板端 base.launch.py / board.launch.py 已启动，且 PC 能收到 /odom、/scan；
  2. 底盘已使能，否则 base_bridge 会把 /cmd_vel 替换成零速：
       ros2 service call /set_enable std_srvs/srv/SetBool "{data: true}"
  3. AMCL 默认按建图起点 (0, 0, 0) 自动初始化；如果机器人不在该点，
       启动后先用 RViz 的 "2D Pose Estimate"，或
       python3 tools/bench/nav_goal.py --initial-pose X Y YAW --goal X Y YAW。
       不初始化时 AMCL 不发布 map→odom，Nav2 生命周期会卡在激活阶段，
       RViz 里表现为无法设置目标点。

用法：
  ros2 launch robot_bringup navigation.launch.py
  ros2 launch robot_bringup navigation.launch.py map:=$PWD/maps/room1.yaml
  ros2 launch robot_bringup navigation.launch.py rviz:=false
  ros2 launch robot_bringup navigation.launch.py use_composition:=True

不要在本 launch 里再启动 robot_state_publisher / base_bridge / x2_lidar：
板端已经在发同一套 TF，重复发布会造成 TF 跳变。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("robot_bringup"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))

    default_map = share / "maps" / "room2.yaml"
    params = share / "config" / "nav2_params.yaml"
    rviz_config = share / "config" / "nav.rviz"
    bringup_launch = nav2_share / "launch" / "bringup_launch.py"

    map_yaml = LaunchConfiguration("map")
    rviz = LaunchConfiguration("rviz")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=str(default_map),
                description="Map YAML; defaults to maps/room2.yaml",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz2 with config/nav.rviz",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Let the lifecycle manager configure/activate Nav2",
            ),
            # nav2_bringup 用 PythonExpression('not ' + use_composition) 求值，
            # 必须传 Python 字面量 True/False；小写 true/false 会抛 NameError。
            DeclareLaunchArgument(
                "use_composition",
                default_value="False",
                description="Use a composed container (True/False); False is easier to debug",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="false",
                description="Respawn nav2 nodes if they crash (composition off)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup_launch)),
                launch_arguments={
                    "map": map_yaml,
                    "use_sim_time": "false",
                    "params_file": str(params),
                    "autostart": autostart,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "slam": "False",
                }.items(),
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
