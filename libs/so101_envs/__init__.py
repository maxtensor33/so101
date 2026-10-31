"""SO101 gymnasium 环境集（workflow/plan/envs.md）。

纯 MuJoCo 内核 + 关节位置控制动作空间（cube 4 任务 + can 4 任务, 各有带相机变体）。
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import gymnasium as gym

from .lift import SO101LiftEnv, SO101LiftCanEnv
from .place import SO101PlaceEnv, SO101PlaceCanEnv
from .reach import SO101ReachEnv, SO101ReachCanEnv
from .stack import SO101StackEnv, SO101StackCanEnv

# Phase 1/2: 无相机 id（max_episode_steps 对齐 squint: 8 个任务均为 50）
gym.register(id="SO101Reach-v1", entry_point="so101_envs.reach:SO101ReachEnv", max_episode_steps=50)
gym.register(id="SO101Lift-v1", entry_point="so101_envs.lift:SO101LiftEnv", max_episode_steps=50)
gym.register(id="SO101Place-v1", entry_point="so101_envs.place:SO101PlaceEnv", max_episode_steps=50)
gym.register(id="SO101Stack-v1", entry_point="so101_envs.stack:SO101StackEnv", max_episode_steps=50)

# can 变体（squint 8 环境对齐: 4 任务 × cube/can; item_type 语义同 squint —
# reach/lift/place 换任务物体, stack 只换底座 itemB）
gym.register(id="SO101ReachCan-v1", entry_point="so101_envs.reach:SO101ReachCanEnv", max_episode_steps=50)
gym.register(id="SO101LiftCan-v1", entry_point="so101_envs.lift:SO101LiftCanEnv", max_episode_steps=50)
gym.register(id="SO101PlaceCan-v1", entry_point="so101_envs.place:SO101PlaceCanEnv", max_episode_steps=50)
gym.register(id="SO101StackCan-v1", entry_point="so101_envs.stack:SO101StackCanEnv", max_episode_steps=50)

# Phase 3: 带相机 id（fixed + wrist 双相机成对启用）
gym.register(id="SO101ReachCamera-v1", entry_point="so101_envs.reach:SO101ReachEnv",
             kwargs={"camera": True}, max_episode_steps=50)
gym.register(id="SO101LiftCamera-v1", entry_point="so101_envs.lift:SO101LiftEnv",
             kwargs={"camera": True}, max_episode_steps=50)
gym.register(id="SO101PlaceCamera-v1", entry_point="so101_envs.place:SO101PlaceEnv",
             kwargs={"camera": True}, max_episode_steps=50)
gym.register(id="SO101StackCamera-v1", entry_point="so101_envs.stack:SO101StackEnv",
             kwargs={"camera": True}, max_episode_steps=50)

# can 带相机变体
gym.register(id="SO101ReachCanCamera-v1", entry_point="so101_envs.reach:SO101ReachCanEnv",
             kwargs={"camera": True}, max_episode_steps=50)
gym.register(id="SO101LiftCanCamera-v1", entry_point="so101_envs.lift:SO101LiftCanEnv",
             kwargs={"camera": True}, max_episode_steps=50)
gym.register(id="SO101PlaceCanCamera-v1", entry_point="so101_envs.place:SO101PlaceCanEnv",
             kwargs={"camera": True}, max_episode_steps=50)
gym.register(id="SO101StackCanCamera-v1", entry_point="so101_envs.stack:SO101StackCanEnv",
             kwargs={"camera": True}, max_episode_steps=50)
