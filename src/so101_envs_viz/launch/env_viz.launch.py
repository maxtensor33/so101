"""env_viz.launch.py — SO101 envs RViz 可视化 (workflow/plan/envs.md Phase 5)。

用法:
    ros2 launch so101_envs_viz env_viz.launch.py env:=reach camera:=false rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    env = LaunchConfiguration("env").perform(context)
    camera = LaunchConfiguration("camera").perform(context).lower() in ("true", "1")
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1")

    desc_dir = get_package_share_directory("so101_description")
    urdf_path = os.path.join(desc_dir, "urdf", "so101.urdf")
    with open(urdf_path) as f:
        robot_description = f.read()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    viz = Node(
        package="so101_envs_viz",
        executable="viz_node",
        parameters=[{"env": env, "camera": camera}],
        output="screen",
    )
    actions = [rsp, viz]
    if use_rviz:
        rviz_cfg = os.path.join(
            get_package_share_directory("so101_envs_viz"), "rviz", "env_viz.rviz")
        actions.append(Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", rviz_cfg], output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_launch_setup),
    ])
