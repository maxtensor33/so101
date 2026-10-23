"""rl_infer.launch.py — SO101 RL 推理可视化 (workflow/plan/rl.md Phase 4)。

用法:
    ros2 launch so101_rl_infer rl_infer.launch.py env:=reach \
        checkpoint:=outputs/rl/sb3-sac-reach camera:=true rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    env = LaunchConfiguration("env").perform(context)
    checkpoint = LaunchConfiguration("checkpoint").perform(context)
    camera = LaunchConfiguration("camera").perform(context).lower() in ("true", "1")
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1")

    desc_dir = get_package_share_directory("so101_description")
    with open(os.path.join(desc_dir, "urdf", "so101.urdf")) as f:
        robot_description = f.read()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    infer = Node(
        package="so101_rl_infer",
        executable="rl_infer_node",
        parameters=[{"env": env, "checkpoint": checkpoint, "camera": camera}],
        output="screen",
    )
    actions = [rsp, infer]
    if use_rviz:
        rviz_cfg = os.path.join(get_package_share_directory("so101_rl_infer"), "rviz", "rl_infer.rviz")
        actions.append(Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", rviz_cfg], output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_launch_setup),
    ])
