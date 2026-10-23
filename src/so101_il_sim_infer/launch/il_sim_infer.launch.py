"""il_sim_infer.launch.py — IL 推理验收入口（workflow/plan/il_sim.md Phase 5）。

用法:
    ros2 launch so101_il_sim_infer il_sim_infer.launch.py env:=reach checkpoint:=outputs/il_sim/<run>/checkpoints/000010/pretrained_model [camera:=true] [rviz:=true]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    env = LaunchConfiguration("env").perform(context)
    checkpoint = LaunchConfiguration("checkpoint").perform(context)
    camera = LaunchConfiguration("camera").perform(context).lower() in ("true", "1")
    mujoco = LaunchConfiguration("mujoco").perform(context).lower() in ("true", "1")
    rviz = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1")

    desc_dir = get_package_share_directory("so101_description")
    urdf_path = os.path.join(desc_dir, "urdf", "so101.urdf")
    with open(urdf_path) as f:
        robot_description = f.read()

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="so101_il_sim_infer",
            executable="il_sim_infer_node",
            output="screen",
            parameters=[
                {"env": env, "checkpoint": checkpoint, "camera": camera, "mujoco": mujoco},
            ],
        ),
    ]
    if rviz:
        rviz_cfg = os.path.join(
            get_package_share_directory("so101_il_sim_infer"), "rviz", "il_sim_infer.rviz")
        actions.append(Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", rviz_cfg], output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("env", default_value="reach"),
        DeclareLaunchArgument("checkpoint", default_value=""),
        DeclareLaunchArgument("camera", default_value="true"),
        DeclareLaunchArgument("mujoco", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
