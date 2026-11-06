"""sim2real 视野对照 launch: leader 同时驱动 envs lift 仿真与真机 follower, RViz 四窗格对比。"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

FOLLOWER_PORT_DEFAULT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14033553-if00"


def generate_launch_description():
    pkg = FindPackageShare("so101_camera_calibration")
    bringup = FindPackageShare("so101_bringup")
    _cfg = Path(__file__).resolve().parent.parent / "config"
    cameras_yaml = str(_cfg / "cameras.local.yaml") if (_cfg / "cameras.local.yaml").is_file() \
        else str(_cfg / "cameras.yaml")

    compare = Node(
        package="so101_camera_calibration",
        executable="compare_views_node",
        name="compare_views",
        output="screen",
        parameters=[{
            "env": LaunchConfiguration("env"),
            "camera_calib": LaunchConfiguration("camera_calib"),
            "leader_port": LaunchConfiguration("leader_port"),
            "leader_id": LaunchConfiguration("leader_id"),
            "leader_invert": LaunchConfiguration("leader_invert"),
            "mujoco": LaunchConfiguration("mujoco"),
        }],
    )

    cameras = [
        Node(
            package="so101_il_sim_develop",
            executable="il_sim_camera_node",
            name=f"il_sim_cam_{key}",
            output="screen",
            parameters=[cameras_yaml, {"camera_key": key}],
        )
        for key in ("fixed", "wrist")
    ]

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([pkg, "config", "compare_views.rviz"])],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("env", default_value="lift"),
        DeclareLaunchArgument("camera_calib", default_value="outputs/calibration/envs_camera_calib.yaml"),
        DeclareLaunchArgument("leader_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("leader_id", default_value="L07252801"),
        DeclareLaunchArgument("leader_invert", default_value=""),
        DeclareLaunchArgument("mujoco", default_value="true"),
        DeclareLaunchArgument("port", default_value=FOLLOWER_PORT_DEFAULT),
        DeclareLaunchArgument(
            "calibration_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("so101_hardware"), "config", "motor_calibration.yaml"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution([bringup, "launch", "demo.launch.py"])]),
            launch_arguments={
                "sim": "none", "real": "true", "rviz": "false", "moveit": "false",
                "twin": "false",
                "port": LaunchConfiguration("port"),
                "calibration_file": LaunchConfiguration("calibration_file"),
            }.items(),
        ),
        *cameras,
        compare,
        rviz,
    ])
