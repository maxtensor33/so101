"""rl_dagger student real deployment launch (plan/rl_dagger.md Phase 5, 决策 D9).

real:=true control base + built-in cv2 cameras (fixed/top) + student policy node.
Physical top camera -> training key "fixed"; wrist -> "wrist".
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    "checkpoint": LaunchConfiguration("checkpoint"),
    "confirm_step": LaunchConfiguration("confirm_step"),
    "reset_on_enter": LaunchConfiguration("reset_on_enter"),
    "rate": LaunchConfiguration("rate"),
    "max_delta": LaunchConfiguration("max_delta"),
}


# 相机设备 YAML 加载优先级: cameras.local.yaml（gitignore, 本机私有）> cameras.yaml（模板）
# local 在源码树内解析（install 符号链接回源文件, 改动即时生效, 无需重构建）
def _cameras_yaml() -> str:
    _cfg = Path(__file__).resolve().parent.parent / "config"
    return str(_cfg / "cameras.local.yaml") if (_cfg / "cameras.local.yaml").is_file() \
        else str(_cfg / "cameras.yaml")


# 设备映射在 config/cameras.yaml 按节点名配置（机器相关, 随 USB 插入顺序漂移）；
# camera_key/fps 由 launch 注入, device 只从 yaml 读。
def _camera(key, cameras_yaml):
    return Node(
        package="so101_rl_dagger_develop",
        executable="rl_dagger_camera_node",
        name=f"rl_dagger_cam_{key}",
        output="screen",
        parameters=[
            cameras_yaml,
            {"camera_key": key, "fps": LaunchConfiguration("cam_fps")},
        ],
    )


LAUNCH_ARGS = [
    DeclareLaunchArgument("checkpoint", default_value=""),
    DeclareLaunchArgument("confirm_step", default_value="true"),
    DeclareLaunchArgument("reset_on_enter", default_value="true"),
    DeclareLaunchArgument("rate", default_value="10.0"),
    DeclareLaunchArgument("max_delta", default_value="0.1"),
    DeclareLaunchArgument("port", default_value=FOLLOWER_PORT_DEFAULT),
    DeclareLaunchArgument(
        "calibration_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("so101_hardware"), "config", "motor_calibration.yaml"]
        ),
    ),
    DeclareLaunchArgument("cam_fps", default_value="30.0"),
]


def generate_launch_description():
    bringup = FindPackageShare("so101_bringup")

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([bringup, "launch", "demo.launch.py"])]
        ),
        launch_arguments=BASE_ARGS.items(),
    )

    dagger_node = Node(
        package="so101_rl_dagger_develop",
        executable="rl_dagger_develop_node",
        name="rl_dagger_develop",
        output="screen",
        parameters=[NODE_PARAMS],
    )

    return LaunchDescription([
        *LAUNCH_ARGS,
        _camera("fixed", _cameras_yaml()),
        _camera("wrist", _cameras_yaml()),
        base,
        dagger_node,
    ])
