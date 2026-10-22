"""SO101 gymnasium 环境集（plan/envs.md）。

纯 MuJoCo 内核 + 关节位置控制动作空间（无相机 4 id + 带相机 4 id，Phase 3 补齐）。
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import gymnasium as gym

from .lift import SO101LiftEnv
from .place import SO101PlaceEnv
from .reach import SO101ReachEnv
from .stack import SO101StackEnv

# Phase 1/2: 无相机 id（max_episode_steps 对齐 squint: reach/lift 50, place 75, stack 100）
gym.register(id="SO101Reach-v1", entry_point="so101_envs.reach:SO101ReachEnv", max_episode_steps=50)
gym.register(id="SO101Lift-v1", entry_point="so101_envs.lift:SO101LiftEnv", max_episode_steps=50)
gym.register(id="SO101Place-v1", entry_point="so101_envs.place:SO101PlaceEnv", max_episode_steps=75)
gym.register(id="SO101Stack-v1", entry_point="so101_envs.stack:SO101StackEnv", max_episode_steps=100)

# Phase 3: 带相机 id（fixed + wrist 双相机成对启用）
gym.register(id="SO101ReachCamera-v1", entry_point="so101_envs.reach:SO101ReachEnv",
             kwargs={"camera": True}, max_episode_steps=50)
gym.register(id="SO101LiftCamera-v1", entry_point="so101_envs.lift:SO101LiftEnv",
             kwargs={"camera": True}, max_episode_steps=50)
gym.register(id="SO101PlaceCamera-v1", entry_point="so101_envs.place:SO101PlaceEnv",
             kwargs={"camera": True}, max_episode_steps=75)
gym.register(id="SO101StackCamera-v1", entry_point="so101_envs.stack:SO101StackEnv",
             kwargs={"camera": True}, max_episode_steps=100)
