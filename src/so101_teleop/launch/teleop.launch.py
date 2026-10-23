"""teleop.launch.py — il_sim 遥操/采集验收入口（plan/il_sim.md Phase 3）。

用法:
    ros2 launch so101_teleop teleop.launch.py teleop:=expert record:=false [env:=reach] [rviz:=true]
    ros2 launch so101_teleop teleop.launch.py teleop:=expert record:=true \
        repo_id:=il_sim/reach_expert episodes:=10
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    teleop = LaunchConfiguration("teleop").perform(context)
    record = LaunchConfiguration("record").perform(context).lower() in ("true", "1")
    env = LaunchConfiguration("env").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1")
    repo_id = LaunchConfiguration("repo_id").perform(context)
    episodes = LaunchConfiguration("episodes").perform(context)
    reset_time_s = LaunchConfiguration("reset_time_s").perform(context)

    desc_dir = get_package_share_directory("so101_description")
    urdf_path = os.path.join(desc_dir, "urdf", "so101.urdf")
    with open(urdf_path) as f:
        robot_description = f.read()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    teleop_args = ["--teleop", teleop, "--env", env]
    if record:
        teleop_args += ["--record", "--repo-id", repo_id, "--episodes", episodes]
        if reset_time_s != "-1":
            teleop_args += ["--reset-time-s", reset_time_s]
    teleop_node = Node(
        package="so101_teleop",
        executable="teleop_node",
        output="screen",
        arguments=teleop_args,
    )
    actions = [rsp, teleop_node]
    if rviz:
        rviz_cfg = os.path.join(get_package_share_directory("so101_teleop"), "rviz", "teleop.rviz")
        actions.append(Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", rviz_cfg], output="screen"))
    return actions


def generate_launch_description():
    from launch.actions import DeclareLaunchArgument

    return LaunchDescription([
        DeclareLaunchArgument("teleop", default_value="expert"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("env", default_value="reach"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("repo_id", default_value="il_sim/reach_expert"),
        DeclareLaunchArgument("reset_time_s", default_value="-1",
                              description="-1=自动(expert 0 / keyboard 5)"),
        DeclareLaunchArgument("episodes", default_value="10"),
        OpaqueFunction(function=_launch_setup),
    ])
