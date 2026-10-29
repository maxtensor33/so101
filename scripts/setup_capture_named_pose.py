#!/usr/bin/env python3
"""SO-ARM101 命名姿势采集: 手动掰臂 → 回车 → 输入名字 → 存 named_poses.yaml + 更新 SRDF。

workflow/plan/setup_hardware.md Phase 6 (D5):
- 力矩失能后由操作员手动摆姿势, 力矩保持失能 (不会动)
- rad 换算与 C++ 插件同公式同标定 (motor_calibration.yaml), 保证 MoveIt 看到的值一致
- 产出: so101_moveit_config/config/named_poses.yaml (机器可读真源)
        + 幂等更新 so101.srdf 的 <group_state> (arm 5 关节 + gripper 1 关节)
- 同名姿势覆盖 (SRDF 与 YAML 均是), SRDF 由 git 追踪可 diff/回退

用法:
  uv run python scripts/setup_capture_named_pose.py [--port /dev/ttyACM0] \
      [--calibration src/so101_hardware/config/motor_calibration.yaml] \
      [--name my_pose] [--list] [--dry-run]

流程: 运行后力矩立即失能; 掰到目标姿势后按回车 (此时才读数并打印),
再输入名字保存; Ctrl+C/q 退出不保存。
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

# venv 自举 (system python 跑 --monitor 时导入 lerobot/yaml 用, 同 viz_node 惯例)
_REPO = Path(__file__).resolve().parents[1]
_VENV_PKG = _REPO / ".venv/lib/python3.12/site-packages"
if _VENV_PKG.is_dir() and str(_VENV_PKG) not in sys.path:
    sys.path.insert(0, str(_VENV_PKG))

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CALIB = REPO_ROOT / "src/so101_hardware/config/motor_calibration.yaml"
DEFAULT_POSES = REPO_ROOT / "src/so101_moveit_config/config/named_poses.yaml"
DEFAULT_SRDF = REPO_ROOT / "src/so101_moveit_config/config/so101.srdf"
DEFAULT_PORT = "/dev/ttyACM0"

# 与 C++ 插件 so101JointLimits 同源 (D2: 本仓库 URDF 限位)
# shoulder_lift ±1.91986 为手动估计 (todo/to_real_joint_calibration.md), 原导出值 ±1.74533
JOINT_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.91986, 1.91986),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}
JOINTS = list(JOINT_LIMITS)
ARM_JOINTS = JOINTS[:5]
# gripper group_state 与 SRDF open/close 约定一致 (qpos 大 = 张开)
GRIPPER_JOINTS = ["gripper"]

HEADER = "# SO-ARM101 命名姿势 (scripts/setup_capture_named_pose.py 采集, rad, URDF 约定)\n"


def load_calibration(path: Path) -> dict[str, dict]:
    """加载 motor_calibration.yaml → {joint: {id, range_min, range_max, invert}}。"""
    data = yaml.safe_load(Path(path).read_text())
    out = {}
    for joint, m in data.items():
        out[joint] = {
            "id": int(m["id"]),
            "range_min": int(m["range_min"]),
            "range_max": int(m["range_max"]),
            "invert": bool(m.get("invert", False)),
        }
    return out


def raw_to_rad(joint: str, raw: int, calib: dict) -> float:
    """raw→rad 全行程线性映射 (与 so101_system.cpp rawToRadians 同公式, 含 invert)。"""
    lower, upper = JOINT_LIMITS[joint]
    clamped = max(calib["range_min"], min(calib["range_max"], raw))
    progress = (clamped - calib["range_min"]) / (calib["range_max"] - calib["range_min"])
    if calib["invert"]:
        progress = 1.0 - progress
    return progress * (upper - lower) + lower


def update_srdf(srdf_text: str, poses: dict[str, dict[str, float]]) -> str:
    """幂等更新 SRDF group_state: 每个姿势写 arm(5 关节) + gripper(1 关节) 两条。

    同名覆盖; 与已有 group_state 共存 (如 open/closed)。
    文本级插入/删除 (在 </robot> 前插入, 按同名标签块删除旧块):
    其余内容字节不变, 避免整文件重排 (ElementTree 往返会重排自闭合标签,
    且默认丢注释 — SRDF 含 R3 方向纠正记录)。
    """
    for name, joint_values in poses.items():
        # 同名 group_state 块删除 (属性顺序无关, arm 与 gripper 两条一并处理)
        srdf_text = re.sub(
            rf"[ \t]*<group_state\b[^>]*\bname=\"{re.escape(name)}\"[^>]*>.*?</group_state>[ \t]*\n",
            "",
            srdf_text,
            flags=re.DOTALL,
        )
        blocks = []
        for group, joints in (("arm", ARM_JOINTS), ("gripper", GRIPPER_JOINTS)):
            lines = [f'  <group_state name="{name}" group="{group}">']
            for j in joints:
                lines.append(f'    <joint name="{j}" value="{joint_values[j]:.6f}"/>')
            lines.append("  </group_state>")
            blocks.append("\n".join(lines))
        insertion = "\n".join(blocks) + "\n"
        if "</robot>" not in srdf_text:
            raise ValueError("SRDF 缺少 </robot> 结束标签")
        srdf_text = srdf_text.replace("</robot>", insertion + "</robot>")
    return srdf_text


def load_named_poses(path: Path) -> dict[str, dict[str, float]]:
    p = Path(path)
    if not p.is_file():
        return {}
    return yaml.safe_load(p.read_text()) or {}


def save_named_poses(path: Path, poses: dict[str, dict[str, float]]) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    body = yaml.safe_dump(poses, sort_keys=False)
    p.write_text(HEADER + body)


def _open_bus(port: str, calib: dict):
    """打开 Feetech 总线 (力矩失能, 返回 bus)。lerobot API, 真机步骤。"""
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    # norm_mode 不参与换算 (读取走 normalize=False 的 raw 值, 换算用本模块 raw_to_rad)
    motors = {
        joint: Motor(id=c["id"], model="sts3215", norm_mode=MotorNormMode.RANGE_0_100)
        for joint, c in calib.items()
    }
    bus = FeetechMotorsBus(port, motors, protocol_version=0)
    bus.connect()
    for joint in calib:
        bus.write("Torque_Enable", joint, 0)
    return bus


def _read_raw_positions(bus, joints) -> dict[str, int]:
    return bus.sync_read("Present_Position", list(joints), normalize=False)


def capture_pose(bus, calib: dict) -> dict[str, float]:
    """读当前 raw → rad。"""
    raw = _read_raw_positions(bus, list(calib))
    return {j: raw_to_rad(j, int(raw[j]), calib[j]) for j in calib}


def _run_monitor(port: str, calib: dict) -> None:
    """方向校准监控: 力矩失能只读 + 发布 /joint_states (配 mujoco_twin 目视校准)。

    掰动关节时 twin 模型同向跟随 = 映射正确; 反向 = 该关节需 --invert 重新转换标定。
    须用系统 python 跑 (rclpy 在系统环境): python3 scripts/setup_capture_named_pose.py --monitor
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node("pose_monitor")
    pub = node.create_publisher(JointState, "joint_states", 10)
    bus = _open_bus(port, calib)
    print("== 方向校准监控 (只读, 力矩失能) ==")
    print("另一终端运行 `ros2 run so101_sim mujoco_twin.py`, 掰动各关节:")
    print("  模型同向跟随 = 正确; 反向 = 该关节需 --invert <关节> 重新转换标定")
    print("Ctrl+C 退出。")
    try:
        msg = JointState()
        while True:
            raw = _read_raw_positions(bus, list(calib))
            pose = {j: raw_to_rad(j, int(raw[j]), calib[j]) for j in calib}
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = list(pose)
            msg.position = [pose[j] for j in pose]
            pub.publish(msg)
            print("  " + "  ".join(f"{j.split('_')[0][:3]}{j.split('_')[-1][:3]}:{pose[j]:+.2f}" for j in JOINTS), end="\r", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        bus.disconnect()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="SO-ARM101 命名姿势采集 (力矩失能手摆)")
    parser.add_argument("--port", default=DEFAULT_PORT, help="串口设备")
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIB, help="标定 YAML")
    parser.add_argument("--poses", type=Path, default=DEFAULT_POSES, help="named_poses.yaml 路径")
    parser.add_argument("--srdf", type=Path, default=DEFAULT_SRDF, help="SRDF 路径")
    parser.add_argument("--name", default="", help="非交互指定姿势名 (跳过提问)")
    parser.add_argument("--list", action="store_true", help="列出已保存姿势")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="不连总线, 用 --name + JSON 字面量验证 SRDF/YAML 更新 (测试用)")
    parser.add_argument(
        "--monitor", action="store_true",
        help="方向校准: 力矩失能只读 + 发布 /joint_states (配 mujoco_twin 目视检查)")
    parser.add_argument(
        "--positions", type=str, default="",
        help="dry-run 模式的关节位置 JSON, 如 '{\"shoulder_pan\": 0.1, ...}'")
    args = parser.parse_args()

    if args.list:
        for name, pose in load_named_poses(args.poses).items():
            print(f"{name}: {pose}")
        return 0

    calib = load_calibration(args.calibration)

    if args.dry_run:
        given = json.loads(args.positions)
        # 未给的关节按 0.0 补全 (交互路径始终采集全部 6 关节, 此处仅为测试便利)
        pose = {j: float(given.get(j, 0.0)) for j in JOINTS}
        name = args.name or "dry_run_pose"
        poses = load_named_poses(args.poses)
        poses[name] = pose
        save_named_poses(args.poses, poses)
        args.srdf.write_text(update_srdf(args.srdf.read_text(), {name: pose}))
        print(f"[dry-run] 已保存 {name} (未连总线)")
        return 0

    if args.monitor:
        _run_monitor(args.port, calib)
        return 0

    print("== SO-ARM101 命名姿势采集 ==")
    print(f"串口: {args.port} | 标定: {args.calibration}")
    print("即将失能力矩, 之后可手动掰动机械臂; 本脚本只会读位置, 不会驱动电机。")

    bus = _open_bus(args.port, calib)
    try:
        # 修复 (2026-10-29): 读数必须在回车那一刻发生。
        # 旧实现先读后阻塞 input, 回车保存的是"掰臂之前"的旧值 (差一轮滞后),
        # 两个不同姿势会存出几乎相同的数值。
        print("掰到目标姿势后按回车 (此时才读数), q=退出")
        while True:
            cmd = input("回车=读当前姿势并保存, q=退出: ").strip().lower()
            if cmd == "q":
                break
            pose = capture_pose(bus, calib)  # 回车后才读, 保证是当前姿势
            print("当前关节 (rad):")
            for j in JOINTS:
                print(f"  {j:<14} {pose[j]:+.4f}")
            name = args.name or input("姿势名字: ").strip()
            if not name:
                print("名字为空, 跳过 (重掰姿势后再回车)")
                continue
            poses = load_named_poses(args.poses)
            poses[name] = pose
            save_named_poses(args.poses, poses)
            args.srdf.write_text(update_srdf(args.srdf.read_text(), {name: pose}))
            print(
                f"已保存 '{name}': {args.poses.name} 与 "
                f"{args.srdf.name} (同名覆盖; git diff 查看 SRDF 变更)"
            )
            if args.name:
                break
    finally:
        # 力矩保持失能, 直接断开
        bus.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
