#!/usr/bin/env python3
"""MuJoCo 数字孪生显示节点 (workflow/plan/setup_hardware.md Phase 5, D6)。

real:=true + sim:=mujoco 时: ros_control 走实机插件 (标准 controller_manager),
本节点只订阅 /joint_states 镜像渲染 MuJoCo 场景 — 纯显示, 不参与控制。

用法 (demo.launch.py 自动拉起, 也可手动):
    ros2 run so101_sim mujoco_twin.py --ros-args -p model:=<scene.xml 路径>

依赖: rclpy (系统) + mujoco (uv venv, 脚本自举 sys.path, 同 so101_envs_viz 惯例)。
"""

from __future__ import annotations

import sys
from pathlib import Path


def _find_repo_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "pyproject.toml").is_file():
            return parent
    return None


_REPO = _find_repo_root()
if _REPO is not None:
    for _p in (str(_REPO / ".venv/lib/python3.12/site-packages"),):
        if _p not in sys.path and Path(_p).is_dir():
            sys.path.insert(0, _p)

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

DEFAULT_MODEL = str(Path(__file__).resolve().parent / "mujoco" / "scene.xml")

JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def build_joint_map(model, joint_names: list[str]) -> dict[str, int]:
    """joint 名 → qpos 索引 (MuJoCo hinge/slide 每关节占 1 个 qpos)。"""
    mapping: dict[str, int] = {}
    for name in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid == -1:
            raise KeyError(f"MuJoCo 模型中找不到 joint: {name}")
        mapping[name] = model.jnt_qposadr[jid]
    return mapping


def apply_joint_states(model, data, mapping: dict[str, int], positions: dict[str, float]) -> None:
    """把 /joint_states 位置写入 qpos 并置零对应 qvel (防漂移)。"""
    for name, qadr in mapping.items():
        if name in positions:
            data.qpos[qadr] = positions[name]
            data.qvel[qadr] = 0.0
    mujoco.mj_forward(model, data)


class MujocoTwinNode(Node):
    def __init__(self, model_path: str | None = None):
        super().__init__("mujoco_twin")
        self.declare_parameter("model", model_path or DEFAULT_MODEL)
        model_path = self.get_parameter("model").value
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.mapping = build_joint_map(self.model, JOINT_NAMES)
        self.sub = self.create_subscription(JointState, "joint_states", self._on_joint_states, 10)
        self.get_logger().info(f"mujoco_twin: model={model_path}, 镜像 /joint_states")

    def _on_joint_states(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        apply_joint_states(self.model, self.data, self.mapping, positions)
        if self.viewer.is_running():
            self.viewer.sync()


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = MujocoTwinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
