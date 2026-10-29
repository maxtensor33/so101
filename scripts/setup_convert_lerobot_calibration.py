#!/usr/bin/env python3
"""LeRobot so101_follower 标定 JSON → so101_hardware motor_calibration.yaml 转换器。

workflow/plan/setup_hardware.md Phase 3 (D2/D4):
- 保留每电机 id/drive_mode/range_min/range_max (C++ 插件唯一真源)
- 丢弃 homing_offset (插件用全行程线性映射, 对齐参考项目决策)
- 新增 per-motor invert 可选字段 (电机安装方向与 URDF 约定相反时镜像, 如 gripper)

用法:
  uv run python scripts/setup_convert_lerobot_calibration.py \
      [--input <calib.json>] [--output src/so101_hardware/config/motor_calibration.yaml] \
      [--invert gripper]

--input 缺省时自动取 ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/ 下最新的 JSON。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import yaml

LEROBOT_CALIB_DIR = Path.home() / ".cache/huggingface/lerobot/calibration/robots"
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "src/so101_hardware/config/motor_calibration.yaml"

HEADER = """\
# SO-ARM101 电机标定 (scripts/setup_convert_lerobot_calibration.py 自动生成, 请勿手改)
# 来源: LeRobot 标定 JSON 全行程标定 (homing_offset 已丢弃, 采用全行程线性映射)
# 消费方: src/so101_hardware (so101_system.cpp) 与 scripts/setup_capture_named_pose.py
"""


def convert(calib: dict, invert_joints: set[str] | None = None) -> dict:
    """LeRobot 标定 dict → motor_calibration dict (每关节 id/drive_mode/range/invert)。"""
    invert_joints = invert_joints or set()
    out: dict[str, dict] = {}
    for joint, motor in calib.items():
        out[joint] = {
            "id": int(motor["id"]),
            "drive_mode": int(motor.get("drive_mode", 0)),
            "range_min": int(motor["range_min"]),
            "range_max": int(motor["range_max"]),
            "invert": joint in invert_joints,
        }
    return out


# 插件映射假设 "标定量程 = URDF 全限位行程" (全行程线性映射)。
# 标定时关节没扫满会静默拉伸所有角度 (2026-10-29 R12552801 教训), 此处按 URDF
# 限位跨度体检 (1 tick = 360/4096 度), 偏差 >10% 判为不完整标定。
JOINT_TRAVEL_DEG = {
    "shoulder_pan": 2 * 110.0,
    # lift ±1.91986 手动估计 (todo/to_real_joint_calibration.md), 原导出 ±1.74533 (200°)
    "shoulder_lift": 2 * 110.0,
    "elbow_flex": 2 * 96.8,
    "wrist_flex": 2 * 95.0,
    "wrist_roll": 157.2 + 162.8,
    "gripper": 10.0 + 100.0,
}


def travel_mismatches(calib: dict, tol: float = 0.1) -> list[str]:
    """返回扫描不足 URDF 限位跨度的关节列表 (空 = 全部扫满)。

    只对**欠扫**告警 (映射会拉伸角度); 超扫是正常钳位, 不报。
    """
    bad = []
    for joint, m in calib.items():
        if joint not in JOINT_TRAVEL_DEG:
            continue
        span_deg = (int(m["range_max"]) - int(m["range_min"])) * 360.0 / 4096.0
        expected = JOINT_TRAVEL_DEG[joint]
        if span_deg < expected * (1 - tol):
            bad.append(
                f"{joint}: 标定跨度 {span_deg:.0f}° < URDF {expected:.0f}° "
                f"(range [{m['range_min']}, {m['range_max']}])")
    return bad


def to_yaml_text(calib: dict) -> str:
    """转 YAML 文本 (保持关节顺序, 顶部带来源注释)。"""
    body = yaml.safe_dump(calib, sort_keys=False, default_flow_style=False)
    return HEADER + "\n" + body


def find_latest_calibration() -> Path | None:
    """自动发现最新的 LeRobot 标定 JSON (扫全部 robot 类型目录, 按修改时间取最新)。

    注意 robot 类型目录名可能是自定义的 (如 so_follower ≠ so101_follower),
    不能写死单个目录 — 2026-10-29 教训: 新标定写进了 so_follower/, 旧标定被误用。
    """
    if not LEROBOT_CALIB_DIR.is_dir():
        return None
    candidates = sorted(LEROBOT_CALIB_DIR.glob("*/*.json"), key=lambda p: p.stat().st_mtime)
    return candidates[-1] if candidates else None


def main() -> int:
    parser = argparse.ArgumentParser(description="LeRobot 标定 JSON 转 motor_calibration.yaml")
    parser.add_argument("--input", type=Path, default=None, help="LeRobot 标定 JSON 路径 (缺省自动发现)")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="输出 YAML 路径")
    parser.add_argument(
        "--invert", type=str, default="", help="需要镜像方向的关节, 逗号分隔 (如 gripper)")
    args = parser.parse_args()

    src = args.input or find_latest_calibration()
    if src is None:
        parser.error(
            f"未找到标定 JSON: 请先运行 LeRobot 全行程标定 (输出目录 {LEROBOT_CALIB_DIR}) "
            "或用 --input 指定路径")
    invert_joints = {j.strip() for j in args.invert.split(",") if j.strip()}

    raw = json.loads(src.read_text())
    calib = convert(raw, invert_joints)
    bad = travel_mismatches(calib)
    if bad:
        print("[警告] 标定行程与 URDF 限位不符, 以下关节可能没扫满全行程:")
        for b in bad:
            print(f"  - {b}")
        print("  全行程线性映射会拉伸角度 → 请重新标定, 每个关节缓慢掰到物理极限两端。")
    args.output.write_text(to_yaml_text(calib))
    print(f"[convert] {src} -> {args.output} ({len(calib)} joints)")
    for joint, m in calib.items():
        flags = f" invert={m['invert']}" if m["invert"] else ""
        print(f"  {joint:<14} id={m['id']} range=[{m['range_min']}, {m['range_max']}]{flags}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
