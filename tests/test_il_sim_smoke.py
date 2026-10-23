"""il_sim Phase 0 环境冒烟：锁定 lerobot 0.6.1 实际可用 API 与 so101_envs 相机环境。

这些测试是环境事实的锚点：后续 il_sim 代码只允许依赖本文件验证过的接口。
"""

from __future__ import annotations

import importlib

import gymnasium as gym
import numpy as np
import pytest


def test_lerobot_import_and_version():
    lerobot = importlib.import_module("lerobot")
    assert lerobot.__version__ == "0.6.1"


def test_numpy2_coexistence():
    # 用户实测结论：numpy 2.x 与 lerobot 0.6.1 共存可用（本 venv 实际 2.2.6）
    assert np.__version__.startswith("2.")


def test_core_pipeline_imports():
    # record / train / teleoperate 是采集与训练的官方入口（0.6.1 位于 lerobot.scripts.*）
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.scripts.lerobot_record import record
    from lerobot.scripts.lerobot_teleoperate import teleoperate
    from lerobot.scripts.lerobot_train import train

    assert callable(record) and callable(train) and callable(teleoperate)
    assert LeRobotDataset is not None


def test_robot_teleoperator_abc_and_registry():
    from lerobot.robots.robot import Robot, RobotConfig
    from lerobot.teleoperators.teleoperator import Teleoperator, TeleoperatorConfig

    # 官方 record CLI 经 draccus ChoiceRegistry 解析 --robot.type/--teleop.type，
    # 依赖 config 类上的 register_subclass 类方法
    assert callable(RobotConfig.register_subclass)
    assert callable(TeleoperatorConfig.register_subclass)
    for abc_cls in (Robot, Teleoperator):
        assert inspect_abstract_methods(abc_cls)  # 抽象方法集合非空，确认是 ABC


def inspect_abstract_methods(cls) -> bool:
    return len(getattr(cls, "__abstractmethods__", set())) > 0


@pytest.mark.parametrize("policy", ["act", "diffusion", "vqbet", "smolvla"])
def test_policy_module_import(policy: str):
    mod = importlib.import_module(f"lerobot.policies.{policy}")
    # 每个策略包应可导入（SmolVLA 本阶段仅导入验证，不训练）
    assert mod is not None


def test_so101_reach_camera_env():
    import so101_envs  # noqa: F401 触发 gym 注册

    env = gym.make("SO101ReachCamera-v1")
    obs, info = env.reset(seed=0)
    try:
        assert "state" in obs and "rgb" in obs
        assert set(obs["rgb"].keys()) == {"fixed", "wrist"}
        for img in obs["rgb"].values():
            assert img.dtype == np.uint8 and img.ndim == 3
        obs2, *_ = env.step(np.zeros(6, dtype=np.float32))
        assert obs2["state"].shape == obs["state"].shape
    finally:
        env.close()
