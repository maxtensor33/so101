"""SO-ARM101 统一 demo 入口.

用法:
    ros2 launch so101_bringup demo.launch.py sim:=none     # mock 硬件 + RViz
    ros2 launch so101_bringup demo.launch.py sim:=gazebo   # Gazebo Harmonic + RViz
    ros2 launch so101_bringup demo.launch.py sim:=mujoco   # MuJoCo + RViz
    可选 moveit:=true (默认 false) 附加 move_group + MoveIt RViz
    gazebo 模式下 gz_gui:=false 为 headless server-only
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _setup(context, *args, **kwargs):
    sim = LaunchConfiguration("sim").perform(context)
    gz_gui = LaunchConfiguration("gz_gui").perform(context)
    use_rviz = LaunchConfiguration("rviz").perform(context)
    moveit = LaunchConfiguration("moveit").perform(context)

    controllers_yaml = os.path.join(
        get_package_share_directory("so101_sim"), "config", "controllers.yaml"
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("so101_bringup"), "urdf", "so101.urdf.xacro"]),
            " sim:=",
            sim,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    actions = []

    # robot_state_publisher (gazebo 的 gz_ros2_control 从其参数读取控制器配置)
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description, controllers_yaml],
            output="screen",
        )
    )

    # 仿真后端三选一
    if sim == "none":
        actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controllers_yaml],
                output="screen",
            )
        )
    elif sim == "mujoco":
        actions.append(
            Node(
                package="mujoco_ros2_control",
                executable="ros2_control_node",
                parameters=[robot_description, controllers_yaml],
                output="screen",
            )
        )
    elif sim == "gazebo":
        from launch.actions import SetEnvironmentVariable

        # gz 解析 package:// mesh 资源: <share 根> 下存在 so101_description/meshes
        actions.append(
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.path.join(
                    get_package_share_directory("so101_description"), os.pardir
                ),
            )
        )
        gz_args = "-r empty.sdf" if gz_gui == "true" else "-r -s empty.sdf"
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    )
                ),
                launch_arguments={"gz_args": gz_args}.items(),
            )
        )
        actions.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-name", "so101", "-topic", "robot_description"],
                output="screen",
            )
        )

    # 控制器 spawner (等 controller_manager 就绪)
    actions.append(
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["arm_controller"],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["gripper_controller"],
                    output="screen",
                ),
            ],
        )
    )

    # RViz (moveit:=true 时由 move_group.launch.py 提供带 MotionPlanning 的 RViz)
    if use_rviz == "true" and moveit != "true":
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("so101_description"), "rviz", "view.rviz"
                    ),
                ],
                parameters=[robot_description],
            )
        )

    if moveit == "true":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("so101_moveit_config"),
                        "launch",
                        "move_group.launch.py",
                    )
                )
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim", default_value="none", choices=["none", "gazebo", "mujoco"]
            ),
            DeclareLaunchArgument("moveit", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("gz_gui", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
