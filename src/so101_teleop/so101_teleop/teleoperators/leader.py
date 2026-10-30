"""So101LeaderSimTeleop: 真实 leader 臂 → il_sim 仿真从臂 (workflow/plan/il_sim_leader.md Phase 1)。

包装 lerobot SO101Leader (串口/标定/重试复用官方实现), 只做单位语义转换:
  内建 so101_leader 输出 = body 度 (相对其标定 home) + gripper 0-100
  本管线统一语义 = 6 维归一化 [-1,1] 绝对位置 ({m}.pos 键)
度→[-1,1] 走 follower 标定链 (D2, lerobot 0.6.1 量程中点公式):
  raw_f = deg*4095/360 + mid_f → 进度 = (raw_f-lo)/(hi-lo) → norm = 2*进度-1
动作平滑 (D7): 每步 norm 限幅, 抑制 10Hz 手驱噪声 (修"夹得住保持不住")。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np
from pathlib import Path

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.so_leader import SO101Leader, SOLeaderTeleopConfig
from lerobot.teleoperators.teleoperator import Teleoperator

from ..constants import JOINT_NAMES
from ..env_backend import get_active_env_or_none, reset_count

# 与 so101_system.cpp so101JointLimits / setup_capture_named_pose.py 同源
# (shoulder_lift ±1.91986 手动估计, workflow/todo/to_real_joint_calibration.md)
JOINT_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.91986, 1.91986),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}

LEROBOT_CALIB_DIR = Path.home() / ".cache/huggingface/lerobot/calibration/robots"


def find_follower_calibration() -> Path:
    """自动发现最新的 follower 标定 JSON (扫全部 robot 类型目录, 同转换脚本惯例)。"""
    if not LEROBOT_CALIB_DIR.is_dir():
        raise FileNotFoundError(f"标定目录不存在: {LEROBOT_CALIB_DIR}")
    candidates = sorted(LEROBOT_CALIB_DIR.glob("*/*.json"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"标定目录下无 JSON: {LEROBOT_CALIB_DIR}")
    return candidates[-1]


def deg_to_norm(joint: str, deg: float, calib_entry: dict, invert: bool = False) -> float:
    """leader 度值 → 归一化 [-1,1] (D2: follower 标定链)。

    lerobot 0.6.1 DEGREES 归一化 = (raw - 量程中点)*360/4095 (homing_offset 不参与!),
    其 teleoperate 把 leader 的度值 d 交给 follower _unnormalize:
    raw_f = d*4095/360 + mid_f → 进度 = (raw_f-lo)/(hi-lo) → norm = 2*进度-1。
    """
    lo, hi = calib_entry["range_min"], calib_entry["range_max"]
    mid = (lo + hi) / 2.0
    raw = deg * 4095.0 / 360.0 + mid
    clamped = max(lo, min(hi, raw))
    progress = (clamped - lo) / (hi - lo)
    if invert:
        progress = 1.0 - progress
    return 2.0 * progress - 1.0


def grip_to_norm(value: float, invert: bool = False) -> float:
    """leader gripper 0-100 → [-1,1]。"""
    progress = max(0.0, min(1.0, value / 100.0))
    if invert:
        progress = 1.0 - progress
    return 2.0 * progress - 1.0


@TeleoperatorConfig.register_subclass("so101_leader_sim")
@dataclass
class So101LeaderSimTeleopConfig(TeleoperatorConfig):
    port: str = "/dev/ttyACM0"
    id: str = ""
    follower_calibration: str | None = None  # None = 自动发现最新标定
    invert: list[str] = field(default_factory=list)
    # 动作平滑 (D7): 每步最大 norm 变化 — 抑制 10Hz 手驱噪声, 修"夹得住保持不住"
    max_norm_step: float = 0.15
    max_grip_step: float = 0.25


class So101LeaderSimTeleop(Teleoperator):
    config_class = So101LeaderSimTeleopConfig
    name = "so101_leader_sim"

    def __init__(self, config):
        super().__init__(config)
        self.config = config
        self._inner = None
        self._fcalib = {}
        self._prev_norm = None
        self._last_reset_count = 0

    @property
    def action_features(self):
        return {f"{m}.pos": float for m in JOINT_NAMES}

    @property
    def feedback_features(self):
        return {}

    @property
    def is_connected(self):
        return self._inner is not None and self._inner.is_connected

    def is_calibrated(self):
        return True if self._inner is None else self._inner.is_calibrated()

    def _make_inner(self):
        cfg = SOLeaderTeleopConfig(port=self.config.port, id=self.config.id, use_degrees=True)
        return SO101Leader(cfg)

    def _load_follower_calibration(self):
        path = (
            Path(self.config.follower_calibration)
            if self.config.follower_calibration
            else find_follower_calibration()
        )
        data = json.loads(Path(path).read_text())
        missing = [j for j in JOINT_NAMES if j not in data]
        if missing:
            raise ValueError(f"follower 标定缺关节: {missing} ({path})")
        return {j: data[j] for j in JOINT_NAMES}

    def connect(self):
        self._fcalib = self._load_follower_calibration()
        self._inner = self._make_inner()
        self._inner.connect()

    def disconnect(self):
        if self._inner is not None:
            self._inner.disconnect()
        self._inner = None

    def configure(self):
        pass

    def calibrate(self):
        pass

    def send_feedback(self, feedback):
        pass

    def get_action(self):
        if not self.is_connected:
            raise RuntimeError("so101_leader_sim: not connected")

        # env 重置后重新吸附 (对齐 keyboard, 防回弹): 以 env 当前姿态为平滑起点
        rc = reset_count()
        if self._prev_norm is not None and rc != self._last_reset_count:
            env = get_active_env_or_none()
            if env is not None:
                u = env.unwrapped
                self._prev_norm = u.normalize_qpos(u.data.qpos[u._qpos_adr]).copy()
            else:
                self._prev_norm = np.zeros(len(JOINT_NAMES))
        self._last_reset_count = rc

        raw = self._inner.get_action()  # {m.pos: 度 / gripper 0-100}
        out = {}
        for joint in JOINT_NAMES:
            val = raw[f"{joint}.pos"]
            inverted = joint in self.config.invert
            if joint == "gripper":
                out[f"{joint}.pos"] = grip_to_norm(float(val), invert=inverted)
            else:
                out[f"{joint}.pos"] = deg_to_norm(
                    joint, float(val), self._fcalib[joint], invert=inverted)

        # D7 动作平滑: 每步限幅 (首帧 _prev_norm=None 直接吸附)
        if self._prev_norm is None:
            self._prev_norm = np.array([out[f"{j}.pos"] for j in JOINT_NAMES])
        elif self.config.max_norm_step > 0:
            for i, joint in enumerate(JOINT_NAMES):
                max_step = (
                    self.config.max_grip_step if joint == "gripper"
                    else self.config.max_norm_step
                )
                prev = self._prev_norm[i]
                out[f"{joint}.pos"] = max(
                    prev - max_step, min(prev + max_step, out[f"{joint}.pos"]))
            self._prev_norm = np.array([out[f"{j}.pos"] for j in JOINT_NAMES])
        return out
