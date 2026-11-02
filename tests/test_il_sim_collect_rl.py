"""RL 专家采集链路契约: teleop 注册 / delta→绝对目标换算 / 脚本 env id 推导。"""

from __future__ import annotations

import numpy as np
import pytest


@pytest.fixture
def sim_robot():
    from so101_teleop.configs import SimSO101RobotConfig
    from so101_teleop.robot import SimSO101Robot

    r = SimSO101Robot(SimSO101RobotConfig(seed=7))
    r.connect()
    yield r
    r.disconnect()


def test_rl_expert_registered():
    from lerobot.teleoperators.teleoperator import TeleoperatorConfig
    from so101_teleop.teleoperators.rl_expert import RLExpertTeleopConfig

    assert "so101_rl_expert" in TeleoperatorConfig.get_known_choices()
    assert TeleoperatorConfig.get_choice_name(RLExpertTeleopConfig) == "so101_rl_expert"


def test_rl_expert_requires_checkpoint():
    from so101_teleop.teleoperators.rl_expert import RLExpertTeleop, RLExpertTeleopConfig

    t = RLExpertTeleop(RLExpertTeleopConfig())
    with pytest.raises(ValueError, match="checkpoint"):
        t.connect()


def test_rl_expert_delta_action_becomes_absolute_target(sim_robot):
    """delta 策略动作 -> target = prev_target + a*DELTA_MAX -> 归一化绝对目标。"""
    from so101_envs.base_env import DELTA_MAX
    from so101_teleop.teleoperators.rl_expert import RLExpertTeleop, RLExpertTeleopConfig

    class _StubAdapter:
        action_mode = "delta"

        def act(self, obs):
            return np.full(6, 0.5)

    t = RLExpertTeleop(RLExpertTeleopConfig(checkpoint="stub"))
    t._adapter = _StubAdapter()
    sim_robot.get_observation()  # 缓存同一帧 state（RL policy 观测）

    u = sim_robot._env.unwrapped
    prev = u.target_qpos.copy()
    act = t.get_action()
    arr = np.array([act[f"{m}.pos"] for m in (
        "shoulder_pan", "shoulder_lift", "elbow_flex",
        "wrist_flex", "wrist_roll", "gripper",
    )])
    expected = np.clip(prev + 0.5 * DELTA_MAX, u._ctrlrange[:, 0], u._ctrlrange[:, 1])
    assert np.allclose(arr, np.clip(u.normalize_qpos(expected), -1.0, 1.0), atol=1e-6)
    assert np.all(arr >= -1.0) and np.all(arr <= 1.0)


def test_rl_expert_rejects_delta_env():
    """采集 env 必须 position 模式（否则记录的动作语义与 il_sim 推理不一致）。"""
    from so101_teleop.configs import SimSO101RobotConfig
    from so101_teleop.robot import SimSO101Robot
    from so101_teleop.teleoperators.rl_expert import RLExpertTeleop, RLExpertTeleopConfig

    class _StubAdapter:
        action_mode = "delta"

        def act(self, obs):
            return np.zeros(6)

    r = SimSO101Robot(SimSO101RobotConfig(env_id="SO101ReachCamera-v1", seed=7))
    # 直接改 env 动作模式模拟误配置
    r.connect()
    try:
        r._env.unwrapped.action_mode = "delta"
        t = RLExpertTeleop(RLExpertTeleopConfig(checkpoint="stub"))
        t._adapter = _StubAdapter()
        with pytest.raises(RuntimeError, match="position"):
            t.get_action()
    finally:
        r.disconnect()


def test_domain_randomization_plumbing():
    from so101_teleop.configs import SimSO101RobotConfig
    from so101_teleop.robot import SimSO101Robot

    r = SimSO101Robot(SimSO101RobotConfig(
        env_id="SO101ReachCamera-v1", seed=7, domain_randomization=True))
    r.connect()
    try:
        assert r._env.unwrapped.domain_randomization is True
    finally:
        r.disconnect()


@pytest.mark.parametrize("train_id,camera_id,short", [
    ("SO101Reach-v1", "SO101ReachCamera-v1", "reach"),
    ("SO101Lift-v1", "SO101LiftCamera-v1", "lift"),
    ("SO101Place-v1", "SO101PlaceCamera-v1", "place"),
    ("SO101Stack-v1", "SO101StackCamera-v1", "stack"),
    ("SO101ReachCan-v1", "SO101ReachCanCamera-v1", "reachcan"),
    ("SO101LiftCan-v1", "SO101LiftCanCamera-v1", "liftcan"),
    ("SO101PlaceCan-v1", "SO101PlaceCanCamera-v1", "placecan"),
    ("SO101StackCan-v1", "SO101StackCanCamera-v1", "stackcan"),
])
def test_collect_supports_all_eight_envs(train_id, camera_id, short):
    """8 个训练环境都能推导出相机版 id / 短名（采集脚本 + RL 专家通用）。"""
    import il_sim_collect_rl as collect

    assert collect.derive_camera_env_id(train_id) == camera_id
    assert collect.short_env_name(camera_id) == short
    assert short in collect.DEFAULT_TASKS


def test_collect_camera_key_subset():
    """VQ-BeT 单相机数据集: camera_keys 透传到 dataset feature（只留 fixed）。"""
    from so101_teleop.configs import SimSO101RobotConfig

    cfg = SimSO101RobotConfig(env_id="SO101ReachCamera-v1", camera_keys=["fixed"])
    assert "fixed" in cfg.camera_keys and "wrist" not in cfg.camera_keys


def test_episode_detail_is_task_agnostic():
    import il_sim_collect_rl as collect

    assert "tcp_to_item_dist" in collect.episode_detail(
        {"tcp_to_item_dist": 0.01, "is_grasped": True, "item_lifted": True})
    assert "tcp_to_goal_dist" in collect.episode_detail({"tcp_to_goal_dist": 0.02})
    assert "a_to_b_xy_dist" in collect.episode_detail(
        {"a_to_b_xy_dist": 0.03, "is_grasped": False})
