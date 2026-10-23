"""SO-ARM101 MTC pick and place (workflow/plan/mtc.md Phase 3).

RViz 规划显示 + 双后端仿真执行:
    ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py sim:=rviz     # 纯 RViz (workflow/plan/mtc_rviz.md)
    ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py sim:=mujoco   # 双后端已暂停, 见 workflow/todo/
    ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py sim:=gazebo
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))


def _xacro_robot_description(sim, mujoco_model_path=None):
    args = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("so101_bringup"), "urdf", "so101.urdf.xacro"]),
        " sim:=",
        sim,
    ]
    if sim == "mujoco" and mujoco_model_path:
        # cube 场景: mujoco_model 是 hardware 参数, 必须经 xacro 传入
        # (节点参数无效 —— mujoco_ros2_control 从 hardware_info 读取)
        args += [" mujoco_model_path:=", mujoco_model_path]
    return {"robot_description": ParameterValue(Command(args), value_type=str)}


def _moveit_config():
    return (
        MoveItConfigsBuilder("so101", package_name="so101_moveit_config")
        .robot_description_semantic(file_path="config/so101.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )


def _setup(context, *args, **kwargs):
    sim = LaunchConfiguration("sim").perform(context)
    use_rviz = LaunchConfiguration("rviz").perform(context)
    execute = LaunchConfiguration("execute").perform(context)
    actions = []

    controllers_yaml = os.path.join(
        get_package_share_directory("so101_mtc_pick_place"), "config",
        "controllers_rviz.yaml" if sim == "rviz" else "controllers_pick.yaml"
    )
    scene_pick = os.path.join(
        get_package_share_directory("so101_sim"), "mujoco", "scene_pick.xml"
    )
    # rviz 模式: xacro sim:=none -> mock_components 硬件, 无 scene_pick.xml
    robot_description = _xacro_robot_description(
        "none" if sim == "rviz" else sim, None if sim == "rviz" else scene_pick)
    moveit_cfg = _moveit_config()
    # R4 (workflow/plan/mtc_fix.md): gz 走 sim time —— joint_states 带仿真时间戳,
    # move_group 轨迹校验 (allowed_start_tolerance) 需同源时钟, 否则
    # "couldn't receive full current joint state within 1s" -> error_code=-4
    use_sim_time = (sim == "gazebo")

    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description, controllers_yaml,
                        {"use_sim_time": use_sim_time}],
            output="screen",
        )
    )

    if sim == "rviz":
        # mock 硬件 + 普通 ros2_control_node (镜像 demo.launch.py sim:=none)
        actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controllers_yaml],
                output="screen",
                on_exit=Shutdown(),
            )
        )
    elif sim == "mujoco":
        actions.append(
            Node(
                package="mujoco_ros2_control",
                executable="ros2_control_node",
                parameters=[robot_description, controllers_yaml],
                output="screen",
                on_exit=Shutdown(),
            )
        )
    elif sim == "gazebo":
        gz_world = os.path.join(
            get_package_share_directory("so101_bringup"), "worlds", "pick_place.sdf"
        )
        # world 文件在 pick_place 包下; gz 解析 so101 mesh 需要 description share
        actions.append(
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                os.pathsep.join([
                    os.path.join(get_package_share_directory("so101_description"), os.pardir),
                    os.path.dirname(os.path.dirname(gz_world)),
                ]),
            )
        )
        gz_args = f"-r {gz_world}"
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("ros_gz_sim"), "launch",
                                 "gz_sim.launch.py")
                ),
                launch_arguments={"gz_args": gz_args}.items(),
            )
        )
        # /clock 桥接 (gz 内嵌 controller_manager 使用 sim time)
        actions.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            )
        )
        # spawn robot (模型名 so101: DetachableJoint child_model 依赖此名)
        actions.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-name", "so101", "-topic", "robot_description"],
                output="screen",
            )
        )
    else:
        raise RuntimeError(f"unsupported sim: {sim}")

    # 控制器 spawner
    actions.append(
        TimerAction(
            period=5.0,
            actions=[
                Node(package="controller_manager", executable="spawner",
                     arguments=["joint_state_broadcaster"], output="screen"),
                Node(package="controller_manager", executable="spawner",
                     arguments=["arm_controller"], output="screen"),
                Node(package="controller_manager", executable="spawner",
                     arguments=["gripper_controller"], output="screen"),
            ],
        )
    )

    # move_group + ExecuteTaskSolution capability (MTC solution 执行)
    moveit_params = moveit_cfg.to_dict()
    # Jazzy move_group 的 capabilities 参数是 string (非 string_array)
    moveit_params["capabilities"] = (
        "move_group/ExecuteTaskSolutionCapability")
    moveit_params["disable_capabilities"] = ""
    actions.append(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[robot_description, moveit_params,
                        {"use_sim_time": use_sim_time}],
        )
    )

    # MTC pick and place 节点 (自动 plan + execute)
    mtc_node = Node(
        package="so101_mtc_pick_place",
        executable="mtc_pick_place_node",
        output="screen",
        arguments=["--log-level", "debug"],
        parameters=[
            robot_description,
            moveit_params,  # planning_pipelines (PipelinePlanner 需要 ompl 插件配置)
            {
                "execute": execute.lower() == "true",
                "auto_exec_delay": 5.0,
                "use_sim_time": use_sim_time,
                # introspection 话题随进程销毁; RViz 面板晚连需要进程存活
                "keep_running": sim == "rviz",
            },
        ],
    )
    # move_group/控制器就绪后再启动 MTC
    actions.append(TimerAction(period=10.0, actions=[mtc_node]))

    # RViz (MotionPlanning + MTC Introspection 面板)
    if use_rviz == "true":
        actions.append(
            TimerAction(
                period=12.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        output="screen",
                        arguments=["-d", os.path.join(
                            get_package_share_directory("so101_mtc_pick_place"),
                            "rviz", "mtc_pick_place.rviz")],
                        parameters=[robot_description,
                                    moveit_cfg.robot_description_semantic],
                        on_exit=Shutdown(),
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="mujoco",
                                  choices=["mujoco", "gazebo", "rviz"]),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("execute", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
