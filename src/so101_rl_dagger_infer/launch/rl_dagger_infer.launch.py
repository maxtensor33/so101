"""rl_dagger_infer.launch.py — SO101 DAgger student 推理可视化 (workflow/plan/rl_dagger.md Phase 4 / D8)。

用法:
    ros2 launch so101_rl_dagger_infer rl_dagger_infer.launch.py \
        checkpoint:=outputs/rl/dagger-reach/latest rviz:=true
    # 同一个 env 实例打开 MuJoCo 窗口（按 Enter 重置 episode）:
    ros2 launch so101_rl_dagger_infer rl_dagger_infer.launch.py \
        checkpoint:=outputs/rl/dagger-reach/latest mujoco:=true

env 参数留空 = 用 checkpoint meta 的 env_id 重建（推荐）; 也可给短名
（reach/lift/...）或完整 Camera-v1 id 覆盖。图像话题按 checkpoint 的
camera_keys 实际数量发布: rl_dagger/{fixed,wrist}/image。
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
    rate = float(LaunchConfiguration("rate").perform(context))
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1")
    mujoco = LaunchConfiguration("mujoco").perform(context).lower() in ("true", "1")
    reset_on_enter = LaunchConfiguration("reset_on_enter").perform(context).lower() in ("true", "1")

    desc_dir = get_package_share_directory("so101_description")
    with open(os.path.join(desc_dir, "urdf", "so101.urdf")) as f:
        robot_description = f.read()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    infer = Node(
        package="so101_rl_dagger_infer",
        executable="rl_dagger_infer_node",
        parameters=[{
            "env": env, "checkpoint": checkpoint, "rate": rate,
            "mujoco": mujoco, "reset_on_enter": reset_on_enter,
        }],
        output="screen",
    )
    actions = [rsp, infer]
    if use_rviz:
        # 与 il_sim_infer 一致: 加载包内 rviz 配置（Fixed Frame=world + 双相机 Image）
        rviz_cfg = os.path.join(
            get_package_share_directory("so101_rl_dagger_infer"), "rviz", "rl_dagger_infer.rviz")
        actions.append(Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", rviz_cfg], output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("env", default_value="",
                              description="空 = checkpoint meta 的 env_id; 短名或完整 Camera-v1 id"),
        DeclareLaunchArgument("checkpoint", default_value=""),
        DeclareLaunchArgument("rate", default_value="10.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("mujoco", default_value="false",
                              description="同一 env 实例打开 MuJoCo 交互窗口"),
        DeclareLaunchArgument("reset_on_enter", default_value="true",
                              description="按 Enter 重置 episode（pynput 全局监听）"),
        OpaqueFunction(function=_launch_setup),
    ])
