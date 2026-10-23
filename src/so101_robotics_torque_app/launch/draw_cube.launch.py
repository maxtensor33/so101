"""SO-ARM101 draw_cube acceptance launch (workflow/plan/robotics.md Phase 4).

Usage:
    ros2 launch so101_robotics_torque_app draw_cube.launch.py                     # torque (default)
    ros2 launch so101_robotics_torque_app draw_cube.launch.py ctrl_mode:=position # JTC baseline
    Optional: rviz:=true|false (default true), controller:=pd_g|computed_torque
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _find_repo_root():
    """Walk up to the repo root (dev workspace marker: pyproject.toml).

    Works both from install/ (launch file is a colcon COPY, not a symlink) and
    from src/. Returns None outside a dev workspace (pure install).
    """
    for parent in Path(__file__).resolve().parents:
        if (parent / "pyproject.toml").is_file():
            return parent
    return None


def resolve_rviz_config(repo_root=None):
    """Prefer the rviz config in the SOURCE tree over the installed copy.

    ament_python data_files are always COPIES even with colcon
    --symlink-install: RViz File->Save would write the install copy and the
    next build overwrites it. Loading from src/ makes saves land in the
    source tree permanently. Falls back to the package share copy when no
    dev workspace is present.
    """
    if repo_root is None:
        repo_root = _find_repo_root()
    if repo_root is not None:
        p = Path(repo_root) / "src" / "so101_robotics_torque_app" / "rviz" / "draw_cube.rviz"
        if p.is_file():
            return p
    return Path(os.path.join(
        get_package_share_directory("so101_robotics_torque_app"),
        "rviz",
        "draw_cube.rviz",
    ))


def _setup(context, *args, **kwargs):
    ctrl_mode = LaunchConfiguration("ctrl_mode").perform(context)
    use_rviz = LaunchConfiguration("rviz").perform(context)
    controller = LaunchConfiguration("controller").perform(context)
    edge_duration = LaunchConfiguration("edge_duration").perform(context)
    kp = [float(v) for v in LaunchConfiguration("kp").perform(context).split(",")]
    kd = [float(v) for v in LaunchConfiguration("kd").perform(context).split(",")]

    actions = []

    if ctrl_mode == "torque":
        xacro_file = os.path.join(
            get_package_share_directory("so101_robotics_torque_app"),
            "urdf",
            "so101_torque.urdf.xacro",
        )
        controllers_yaml = os.path.join(
            get_package_share_directory("so101_sim"), "config", "controllers_torque.yaml"
        )
    else:
        # position baseline: reuse the proven demo stack (JTC + position MJCF)
        xacro_file = os.path.join(
            get_package_share_directory("so101_bringup"), "urdf", "so101.urdf.xacro"
        )
        controllers_yaml = os.path.join(
            get_package_share_directory("so101_sim"), "config", "controllers.yaml"
        )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # robot_state_publisher: /joint_states -> TF -> RViz sync
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description, controllers_yaml],
            output="screen",
        )
    )

    # mujoco ros2_control node (both modes run mujoco).
    # Closing the MuJoCo window makes this process exit -> Shutdown tears the
    # whole launch down.
    actions.append(
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            parameters=[robot_description, controllers_yaml],
            output="screen",
            on_exit=Shutdown(),
        )
    )

    # controller spawners (controller names identical in both yaml files)
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

    if ctrl_mode == "torque":
        actions.append(
            Node(
                package="so101_robotics_torque_app",
                executable="torque_control_node",
                parameters=[{"controller": controller, "kp": kp, "kd": kd}],
                output="screen",
            )
        )

    # draw_cube_node runs in both modes: it measures tracking error and prints
    # the acceptance stats (position mode drives via follow_joint_trajectory).
    actions.append(
        Node(
            package="so101_robotics_torque_app",
            executable="draw_cube_node",
            parameters=[{"ctrl_mode": ctrl_mode, "edge_duration": float(edge_duration)}],
            output="screen",
        )
    )

    if use_rviz == "true":
        actions.append(
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        output="screen",
                        arguments=["-d", str(resolve_rviz_config())],
                        parameters=[robot_description],
                        # closing the RViz window tears the whole launch down
                        on_exit=Shutdown(),
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("ctrl_mode", default_value="torque", choices=["torque", "position"]),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("controller", default_value="computed_torque", choices=["pd_g", "computed_torque"]),
            DeclareLaunchArgument("edge_duration", default_value="1.5"),
            DeclareLaunchArgument("kp", default_value="50.0,50.0,50.0,50.0,50.0"),
            DeclareLaunchArgument("kd", default_value="5.0,5.0,5.0,5.0,5.0"),
            OpaqueFunction(function=_setup),
        ]
    )
