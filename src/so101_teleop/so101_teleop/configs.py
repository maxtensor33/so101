"""SimSO101Robot 的 draccus config（注册进官方 lerobot-record 的 ChoiceRegistry）。"""

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.robots.config import RobotConfig

from .constants import CONTROL_FPS


@RobotConfig.register_subclass("so101_sim")
@dataclass
class SimSO101RobotConfig(RobotConfig):
    """仿真 SO-ARM101：env 后端（so101_envs 相机版）+ lerobot 协议适配。

    官方 CLI 用法：--robot.type=so101_sim --robot.env_id=SO101ReachCamera-v1
    """

    # gymnasium 环境注册名（so101_envs 相机版）
    env_id: str = "SO101ReachCamera-v1"
    # 控制频率：dataset fps 与 env.step 节拍一致
    fps: int = CONTROL_FPS
    # env.reset 种子（None = 随机）
    seed: int | None = None
    # 相机子集（VQ-BeT 等策略只支持单相机: --robot.camera_keys='["fixed"]'）；
    # 不能叫 cameras —— lerobot 保留给 dict[str, CameraConfig]
    camera_keys: list[str] = field(default_factory=lambda: ["fixed", "wrist"])
    # MuJoCo viewer 窗口（human 模式, step 自动 sync; 无显示环境置 false）
    render: bool = False
    # viewer 左右工具栏（Tab/Shift+Tab 可切）; 最大化（X11 EWMH, best-effort）;
    # viewer 不抢键盘焦点（WM_HINTS.input=False, 键盘留在终端, 鼠标拖视角不受影响）
    viewer_ui: bool = False
    viewer_maximize: bool = True
    viewer_no_input: bool = True
    # keyboard 模式: 成功到达即自动重置场景（成功后下一帧边界 env.reset）
    auto_reset_on_success: bool = False
