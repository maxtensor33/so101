"""il_sim real deployment launch (plan/il_sim_develop.md Phase 2/4).

real:=true control base + built-in cv2 cameras (fixed/top) + policy node + optional RViz.
Physical top camera -> training key "fixed"; wrist -> "wrist".
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

BASE_ARGS = {
    "sim": "none",
    "real": "true",
    "rviz": "false",
    "moveit": "false",
    "twin": "false",
    # follower 臂端口（默认 leader=ACM0 会被误控! 用 /dev/serial/by-id 稳定路径, 免 USB 顺序漂移）
    "port": LaunchConfiguration("port"),
    "calibration_file": LaunchConfiguration("calibration_file"),
}

# follower 臂稳定串口路径（leader=5B61037334=ttyACM0, follower=5B14033553; 换机后 ls /dev/serial/by-id 确认）
FOLLOWER_PORT_DEFAULT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14033553-if00"

NODE_PARAMS = {
    "env": LaunchConfiguration("env"),
    "checkpoint": LaunchConfiguration("checkpoint"),
    "camera_keys": LaunchConfiguration("camera_keys"),
    "confirm_step": LaunchConfiguration("confirm_step"),
    "reset_on_enter": LaunchConfiguration("reset_on_enter"),
    "rate": LaunchConfiguration("rate"),
}


# 设备映射在 config/cameras.yaml 按节点名配置（机器相关, 随 USB 插入顺序漂移）；
# camera_key/fps 由 launch 注入, device 只从 yaml 读。
def _camera(key):
    return Node(
        package="so101_il_sim_develop",
        executable="il_sim_camera_node",
        name=f"il_sim_cam_{key}",
        output="screen",
        parameters=[
            [FindPackageShare("so101_il_sim_develop"), "/config/cameras.yaml"],
            {"camera_key": key, "fps": LaunchConfiguration("cam_fps")},
        ],
    )


LAUNCH_ARGS = [
    DeclareLaunchArgument("env", default_value="reach"),
    DeclareLaunchArgument("checkpoint", default_value=""),
    DeclareLaunchArgument("camera_keys", default_value="fixed,wrist"),
    DeclareLaunchArgument("confirm_step", default_value="false"),
    DeclareLaunchArgument("reset_on_enter", default_value="true"),
    DeclareLaunchArgument("rate", default_value="10.0"),
    DeclareLaunchArgument("port", default_value=FOLLOWER_PORT_DEFAULT),
    DeclareLaunchArgument(
        "calibration_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("so101_hardware"), "config", "motor_calibration.yaml"]
        ),
    ),
    DeclareLaunchArgument("cam_fps", default_value="30.0"),
    DeclareLaunchArgument("gui", default_value="true"),
    DeclareLaunchArgument("rviz", default_value="true"),  # 兼容旧命令, 实际开关是 gui
]


def generate_launch_description():
    pkg = FindPackageShare("so101_il_sim_develop")
    bringup = FindPackageShare("so101_bringup")

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([bringup, "launch", "demo.launch.py"])]
        ),
        launch_arguments=BASE_ARGS.items(),
    )

    il_sim_node = Node(
        package="so101_il_sim_develop",
        executable="il_sim_develop_node",
        name="il_sim_develop",
        output="screen",
        parameters=[NODE_PARAMS],
    )

    # 条件用独立名 "gui": include 传给 demo.launch.py 的 "rviz": "false" 会把顶层同名
    # LaunchConfiguration 毒化成 false（launch 参数作用域泄漏, 实证 2026-11-05）, 同名会互相打架。
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("gui")),
        arguments=["-d", PathJoinSubstitution([pkg, "config", "il_sim_develop.rviz"])],
        output="screen",
    )

    return LaunchDescription([
        *LAUNCH_ARGS,
        _camera("fixed"),
        _camera("wrist"),
        base,
        il_sim_node,
        rviz,
    ])
