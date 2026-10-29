"""SO-ARM101 统一 demo 入口.

用法:
    ros2 launch so101_bringup demo.launch.py sim:=none     # mock 硬件 + RViz
    ros2 launch so101_bringup demo.launch.py sim:=gazebo   # Gazebo Harmonic + RViz
    ros2 launch so101_bringup demo.launch.py sim:=mujoco   # MuJoCo + RViz
    可选 moveit:=true (默认 false) 附加 move_group + MoveIt RViz
    gazebo 模式下 gz_gui:=false 为 headless server-only

实机模式 (workflow/plan/setup_hardware.md):
    ros2 launch so101_bringup demo.launch.py sim:=none   real:=true   # 纯实机
    ros2 launch so101_bringup demo.launch.py sim:=mujoco real:=true   # 数字孪生:
        # ros_control 控实机, MuJoCo 仅镜像显示 (twin:=false 关闭)
    可选 port:=/dev/ttyACM0 calibration_file:=<yaml> 覆盖串口与标定
    real:=true + sim:=gazebo 不支持 (直接报错)
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
    real = LaunchConfiguration("real").perform(context)
    twin = LaunchConfiguration("twin").perform(context)

    # 实机参数 (real:=false 时仅作为 xacro 默认值透传, 无副作用)
    port = LaunchConfiguration("port").perform(context)
    calibration_file = LaunchConfiguration("calibration_file").perform(context)

    if real == "true" and sim == "gazebo":
        raise RuntimeError(
            "real:=true 与 sim:=gazebo 不支持: gz_ros2_control 无法与实机插件共存, "
            "请用 sim:=none (纯实机) 或 sim:=mujoco (数字孪生)"
        )

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
            " real:=",
            real,
            " port:=",
            port,
            " calibration_file:=",
            calibration_file,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    actions = []

    # robot_state_publisher (gazebo 的 gz_ros2_control 从其参数读取控制器配置)
    # URDF 内含 world->base_link 固定关节, rsp 会发布该静态 TF
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description, controllers_yaml],
            output="screen",
        )
    )

    # 控制后端: 标准 controller_manager 用于 sim:=none (mock) 与实机 (real:=true,
    # 含 sim:=mujoco 数字孪生 — MuJoCo 仅显示); mujoco_ros2_control 仅用于纯 MuJoCo 仿真
    if sim == "none" or real == "true":
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
        # /clock 桥接: gz 内嵌 controller_manager (use_sim_time=true) 需要 ROS 侧 /clock
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            )
        )

    # 数字孪生: real+mujoco 时 MuJoCo 仅镜像 /joint_states (ros_control 控实机)
    if real == "true" and sim == "mujoco" and twin == "true":
        actions.append(
            Node(
                package="so101_sim",
                executable="mujoco_twin.py",
                output="screen",
                parameters=[
                    {
                        "model": os.path.join(
                            get_package_share_directory("so101_sim"), "mujoco", "scene.xml"
                        )
                    }
                ],
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
    # 延迟启动: 等控制器/TF 就绪, 避免 Fixed Frame 检查失败
    if use_rviz == "true" and moveit != "true":
        actions.append(
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        output="screen",
                        arguments=[
                            "-d",
                            os.path.join(
                                get_package_share_directory("so101_description"),
                                "rviz",
                                "view.rviz",
                            ),
                        ],
                        parameters=[robot_description],
                    )
                ],
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
                ),
                launch_arguments={
                    "sim": sim,
                    "real": real,
                    "port": port,
                    "calibration_file": calibration_file,
                }.items(),
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
            # 实机模式 (workflow/plan/setup_hardware.md)
            DeclareLaunchArgument("real", default_value="false"),
            DeclareLaunchArgument("twin", default_value="true"),
            DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=os.path.join(
                    get_package_share_directory("so101_hardware"),
                    "config",
                    "motor_calibration.yaml",
                ),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
