"""gamepad / leader 遥操 TODO 占位（workflow/plan/il_sim.md）。

- gamepad: lerobot 自带 KeyboardGamepadTeleop（lerobot.teleoperators.gamepad）可低成本接入，
  只需映射其 6 维动作到 {motor}.pos 键名（TODO(il_real)）。
- leader: lerobot 自带 so101_leader（真机主臂）已注册, il_real 直接使用, 无需占位。
"""

from __future__ import annotations

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator


@TeleoperatorConfig.register_subclass("so101_gamepad")
@dataclass
class GamepadTeleopConfig(TeleoperatorConfig):
    pass


class GamepadTeleopPlaceholder(Teleoperator):
    config_class = GamepadTeleopConfig
    name = "so101_gamepad"

    def __init__(self, config):
        super().__init__(config)
        self.config = config
        self._connected = False

    @property
    def action_features(self) -> dict[str, type]:
        from ..constants import JOINT_NAMES

        return {f"{m}.pos": float for m in JOINT_NAMES}

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    def configure(self) -> None:
        pass

    def connect(self) -> None:
        raise NotImplementedError(
            "gamepad teleop is TODO (lerobot 自带 gamepad 可低成本接入, TODO(il_real))"
        )

    def disconnect(self) -> None:
        pass

    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def send_feedback(self, feedback: dict) -> None:
        pass

    def get_action(self) -> dict[str, float]:
        raise NotImplementedError("gamepad teleop is TODO")
