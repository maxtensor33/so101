"""域随机化配置（plan/envs.md Phase 3; Phase 1 起供 reset 使用）。

只含 mujoco 运行期安全可变字段的随机化（qpos 噪声 / 出生域 / rgba / 摩擦 / 相机位姿）。
尺寸随机化需处理 geom_size 派生量（rbound），留扩展。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RandomizationConfig:
    # 出生域与初始位形（默认开）
    spawn_box_pos: tuple = (0.25, 0.0)
    spawn_box_half_size: float = 0.1
    initial_qpos_noise_scale: float = 0.02  # rad, rest pose 附近的初始扰动

    # 物体外观/摩擦（默认关, RL 按需开）
    randomize_item_color: bool = False
    item_rgba_jitter: float = 0.3
    item_friction_range: tuple = (0.5, 1.5)

    # 固定相机位姿噪声（默认关）
    fixed_camera_pos_noise: float = 0.0

    # reach 任务: goal 在物体上方的高度 (m) 与成功半径 (m)
    goal_height_above_item: float = 0.02
    target_goal_thresh: float = 0.03
