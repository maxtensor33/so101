"""So101KeyboardTeleop: joint-select + incremental control (workflow/plan/il_sim.md Phase 2).

Keys: 1-6 select joint, w/s increment/decrement, c requests env reset at the
next frame boundary. Key events are injectable (handle_press/release) so the
contract is testable headless; pynput listener attaches in connect() if available.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator

from ..constants import JOINT_NAMES
from ..env_backend import request_reset

JOINT_SELECT_KEYS = {str(i): i - 1 for i in range(1, 7)}
INCR = "w"
DECR = "s"
RESET_KEY = "c"
# 与官方 record 键位同语义: 提前完成/重录 → 同时自动重置场景
NEXT_KEYS = ("n", "right")
RERECORD_KEYS = ("r", "left")


@TeleoperatorConfig.register_subclass("so101_keyboard")
@dataclass
class So101KeyboardTeleopConfig(TeleoperatorConfig):
    step_size: float = 0.05  # normalized increment per keypress


class So101KeyboardTeleop(Teleoperator):
    config_class = So101KeyboardTeleopConfig
    name = "so101_keyboard"

    def __init__(self, config: So101KeyboardTeleopConfig):
        super().__init__(config)
        self.config = config
        self._target = np.zeros(6)
        self._selected = 0
        self._listener = None
        self._connected = False
        self._last_reset_count = -1

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in JOINT_NAMES}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def is_calibrated(self) -> bool:
        return True

    def connect(self) -> None:
        # neutral target = current env qpos (hold), else zeros
        try:
            from ..env_backend import get_active_env

            u = get_active_env().unwrapped
            self._target = np.asarray(u.normalize_qpos(u.data.qpos[u._qpos_adr]))
        except RuntimeError:
            self._target = np.zeros(6)
        self._target[5] = 0.0  # gripper neutral
        self._target = np.clip(self._target, -1.0, 1.0)
        self._connected = True
        self._start_pynput()

    def configure(self) -> None:
        pass

    def disconnect(self) -> None:
        self._connected = False
        if self._listener is not None:
            try:
                self._listener.stop()
            except Exception:
                pass
            self._listener = None

    def calibrate(self) -> None:
        pass

    def send_feedback(self, feedback: dict) -> None:
        pass

    def get_action(self) -> dict[str, float]:
        # env reset 后吸附保持目标到新 qpos（否则机械臂会弹回旧目标位姿）
        from ..env_backend import get_active_env_or_none, reset_count

        if reset_count() != self._last_reset_count:
            env = get_active_env_or_none()
            if env is not None:
                u = env.unwrapped
                self._target = np.clip(
                    np.asarray(u.normalize_qpos(u.data.qpos[u._qpos_adr])), -1.0, 1.0
                )
            self._last_reset_count = reset_count()
        return {f"{m}.pos": float(v) for m, v in zip(JOINT_NAMES, self._target)}

    # ---------- key handling (injectable, testable headless) ----------

    def handle_press(self, key: str) -> None:
        if key in JOINT_SELECT_KEYS:
            self._selected = JOINT_SELECT_KEYS[key]
        elif key == INCR:
            self._target[self._selected] = np.clip(
                self._target[self._selected] + self.config.step_size, -1.0, 1.0
            )
        elif key == DECR:
            self._target[self._selected] = np.clip(
                self._target[self._selected] - self.config.step_size, -1.0, 1.0
            )
        elif key == RESET_KEY:
            request_reset()
        elif key in NEXT_KEYS or key in RERECORD_KEYS:
            request_reset()

    def handle_release(self, key: str) -> None:
        pass

    def _start_pynput(self) -> None:
        try:
            from pynput import keyboard as pynput_keyboard

            self._listener = pynput_keyboard.Listener(
                on_press=self._on_press, on_release=self._on_release
            )
            self._listener.start()
        except Exception:
            self._listener = None  # headless: injectable handlers still work

    def _on_press(self, key) -> None:
        # 字符键走 .char；方向键/ESC 等 pynput 特殊键 char=None, 走 .name（right/left/...）
        ch = getattr(key, "char", None) or getattr(key, "name", None)
        if ch:
            self.handle_press(ch)

    def _on_release(self, key) -> None:
        ch = getattr(key, "char", None) or getattr(key, "name", None)
        if ch:
            self.handle_release(ch)
