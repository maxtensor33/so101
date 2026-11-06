"""相机标定 launch (plan/setup_camera_calibration.md)。

内参: ros2 run so101_camera_calibration camera_intrinsic_calibration_node（无需底座, 只需相机流）。
手眼: 本 launch（real:=true 底座 + handeye_node; auto 模式驱动臂, 请先清空工作区）。
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

FOLLOWER_PORT_DEFAULT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14033553-if00"


def generate_launch_description():
    pkg = FindPackageShare("so101_camera_calibration")
    bringup = FindPackageShare("so101_bringup")

    handeye = Node(
        package="so101_camera_calibration",
        executable="handeye_node",
        name="handeye_calibration",
        output="screen",
        parameters=[{
            "mode": LaunchConfiguration("mode"),
            "camera": LaunchConfiguration("camera"),
            "intrinsic_yaml": LaunchConfiguration("intrinsic_yaml"),
            "poses_yaml": PathJoinSubstitution([pkg, "config", "calibration_poses.yaml"]),
            "output_dir": LaunchConfiguration("output_dir"),
            "leader_port": LaunchConfiguration("leader_port"),
            "leader_id": LaunchConfiguration("leader_id"),
            "leader_invert": LaunchConfiguration("leader_invert"),
        }],
    )

    # 相机设备 YAML: cameras.local.yaml（本机私有, gitignore）优先, 模板兜底
    _cfg = Path(__file__).resolve().parent.parent / "config"
    cameras_yaml = str(_cfg / "cameras.local.yaml") if (_cfg / "cameras.local.yaml").is_file() \
        else str(_cfg / "cameras.yaml")
    # 节点名必须与 yaml key 匹配（il_sim_cam_fixed/wrist）, 否则 device 参数不生效
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

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="auto"),
        DeclareLaunchArgument("camera", default_value="eye_in_hand"),
        DeclareLaunchArgument("intrinsic_yaml", default_value=""),
        DeclareLaunchArgument("output_dir", default_value="outputs/calibration"),
        DeclareLaunchArgument("with_base", default_value="true"),
        DeclareLaunchArgument("port", default_value=FOLLOWER_PORT_DEFAULT),
        DeclareLaunchArgument("leader_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("leader_id", default_value="L07252801"),
        DeclareLaunchArgument("leader_invert", default_value=""),
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
            condition=IfCondition(LaunchConfiguration("with_base")),
        ),
        *cameras,
        handeye,
    ])
