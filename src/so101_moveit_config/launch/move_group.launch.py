"""SO-ARM101 MoveIt2: move_group + MoveIt RViz.

由 so101_bringup/demo.launch.py (moveit:=true) 或独立启动:
    ros2 launch so101_moveit_config move_group.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    mcfg = get_package_share_directory("so101_moveit_config")

    # MoveItConfigsBuilder 生成 planning_pipelines/kinematics/joint_limits/
    # trajectory_execution 的标准嵌套参数结构
    config = (
        MoveItConfigsBuilder("so101", package_name="so101_moveit_config")
        .robot_description_semantic(file_path="config/so101.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # 与 demo.launch.py 一致: 用 xacro 展开 robot_description
    # (name=so101 与 SRDF 匹配; ros2_control 块对 move_group 无害)
    # sim/real/port/calibration_file 透传 (workflow/plan/setup_hardware.md Phase 4):
    # real:=true 时 MoveIt 规划即作用于实机
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "urdf", "so101.urdf.xacro"]),
            " sim:=",
            LaunchConfiguration("sim"),
            " real:=",
            LaunchConfiguration("real"),
            " port:=",
            LaunchConfiguration("port"),
            " calibration_file:=",
            LaunchConfiguration("calibration_file"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[robot_description, config.to_dict()],
    )

    moveit_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(mcfg, "config", "moveit.rviz")],
        parameters=[robot_description, config.robot_description_semantic],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="none"),
            DeclareLaunchArgument("real", default_value="false"),
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=os.path.join(
                    get_package_share_directory("so101_hardware"),
                    "config",
                    "motor_calibration.yaml",
                ),
            ),
            move_group,
            moveit_rviz,
        ]
    )
