"""gymnasium API 合规与行为测试（plan/envs.md Phase 1, TDD）。

- env_checker（gymnasium 官方）对注册 id 全查
- reset(seed) 可复现
- 位置伺服行为：指令位置目标 → 机械臂收敛到目标
- step 返回 5-tuple、reward 数值类型
"""

from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")

import so101_envs  # noqa: F401  注册副作用
import gymnasium as gym
import numpy as np
import pytest

# 全部 8 个注册 id
REGISTERED_IDS = [
    "SO101Reach-v1", "SO101Lift-v1", "SO101Place-v1", "SO101Stack-v1",
    "SO101ReachCamera-v1", "SO101LiftCamera-v1", "SO101PlaceCamera-v1", "SO101StackCamera-v1",
]
# env_checker（含位级确定性检查）仅适用于无相机 id；
# 相机 id 的 rgb 含 EGL 渲染位级噪声（±1），由 test_camera_obs_* 以容差覆盖
STATE_IDS = REGISTERED_IDS[:4]


def _state_of(obs):
    return obs["state"] if isinstance(obs, dict) else obs


def _obs_equiv(o1, o2):
    """state 严格相等; 相机 rgb 允许 ±1 (EGL 位级噪声)。"""
    s1, s2 = _state_of(o1), _state_of(o2)
    if not np.allclose(s1, s2):
        return False
    if isinstance(o1, dict):
        for k in o1["rgb"]:
            if np.abs(o1["rgb"][k].astype(int) - o2["rgb"][k].astype(int)).max() > 1:
                return False
    return True


@pytest.mark.parametrize("env_id", STATE_IDS)
def test_env_checker(env_id):
    """gymnasium 官方 env_checker 通过（render 检查跳过，Phase 3 单独测渲染）。"""
    from gymnasium.utils.env_checker import check_env

    env = gym.make(env_id)
    check_env(env.unwrapped, skip_render_check=True)
    env.close()


@pytest.mark.parametrize("env_id", REGISTERED_IDS)
def test_reset_seed_reproducible(env_id):
    """同 seed 两次 reset 得到相同初始 obs。"""
    env = gym.make(env_id)
    try:
        obs1, _ = env.reset(seed=123)
        obs2, _ = env.reset(seed=123)
        assert _obs_equiv(obs1, obs2)
    finally:
        env.close()


@pytest.mark.parametrize("env_id", REGISTERED_IDS)
def test_step_returns_five_tuple(env_id):
    env = gym.make(env_id)
    try:
        env.reset(seed=0)
        action = env.action_space.sample()
        out = env.step(action)
        assert len(out) == 5
        obs, reward, terminated, truncated, info = out
        assert isinstance(reward, (float, np.floating))
        assert isinstance(terminated, (bool, np.bool_))
        assert isinstance(truncated, (bool, np.bool_))
        assert isinstance(info, dict)
    finally:
        env.close()


@pytest.mark.parametrize("env_id", REGISTERED_IDS)
def test_position_servo_converges(env_id):
    """位置控制核心行为：恒定指令目标 → 各关节收敛到目标（±0.05 rad）。"""
    env = gym.make(env_id)
    try:
        env.reset(seed=0)
        # 目标取动作空间中点附近的一个固定配置
        target_norm = np.array([0.1, -0.3, 0.3, 0.2, -0.1, 0.4])
        for _ in range(100):  # 100 控制步 ≈ 4s 仿真
            obs, _, _, _, _ = env.step(target_norm)
        qpos = np.asarray(_state_of(obs)[:6])
        target = env.unwrapped.denormalize_action(target_norm)
        assert np.allclose(qpos, target, atol=0.05), f"qpos={qpos}, target={target}"
    finally:
        env.close()


@pytest.mark.parametrize("env_id", REGISTERED_IDS)
def test_obs_within_space(env_id):
    env = gym.make(env_id)
    try:
        obs, _ = env.reset(seed=7)
        assert env.observation_space.contains(obs)
        obs, _, _, _, _ = env.step(env.action_space.sample())
        assert env.observation_space.contains(obs)
    finally:
        env.close()


# ---------------- render_mode="human" (gymnasium 标准) ----------------

class _FakeViewer:
    """viewer 替身: 记录 sync/close 调用。"""

    def __init__(self):
        self.syncs = 0
        self.closed = False

    def sync(self):
        self.syncs += 1

    def close(self):
        self.closed = True


def test_human_in_render_modes():
    """metadata 声明 human 与 rgb_array 两种渲染模式。"""
    from so101_envs.base_env import SO101BaseEnv

    assert "human" in SO101BaseEnv.metadata["render_modes"]
    assert "rgb_array" in SO101BaseEnv.metadata["render_modes"]


def test_human_mode_step_syncs_viewer(monkeypatch):
    """render_mode=human: 构造/reset 不开窗, 首次 render 惰性开窗, 每 step sync, close 关闭。"""
    from so101_envs.base_env import SO101BaseEnv
    from so101_envs.reach import SO101ReachEnv

    fake = _FakeViewer()
    opened = []

    def _fake_open(self):
        opened.append(True)
        self._viewer = fake

    monkeypatch.setattr(SO101ReachEnv, "_open_viewer", _fake_open)
    env = SO101ReachEnv(render_mode="human")
    try:
        assert not opened  # 构造不开窗
        env.reset(seed=0)
        assert not opened  # reset 也不开窗（首帧由首次 render 触发）
        env.step(np.zeros(6))
        assert opened and fake.syncs >= 1
        n = fake.syncs
        env.step(np.zeros(6))
        assert fake.syncs == n + 1
    finally:
        env.close()
    assert fake.closed


def test_human_mode_via_gym_make():
    """gym.make(id, render_mode=...) 透传构造参数。"""
    spec_gym = gym.make("SO101Reach-v1", render_mode="human")
    try:
        assert spec_gym.unwrapped.render_mode == "human"
    finally:
        spec_gym.close()


def test_viewer_ui_kwargs(monkeypatch):
    """viewer_ui/viewer_maximize: launch_passive 收到 show_*_ui kwargs（默认保持 True）。"""
    import mujoco.viewer  # noqa: F401  先加载子模块供 monkeypatch
    import mujoco

    from so101_envs.base_env import SO101BaseEnv
    from so101_envs.reach import SO101ReachEnv

    captured = {}
    real = mujoco.viewer.launch_passive

    def fake_launch(model, data, **kwargs):
        captured.update(kwargs)
        return real(model, data, **kwargs) if False else type(
            "H", (), {"sync": lambda s: None, "close": lambda s: None}
        )()

    monkeypatch.setattr(mujoco.viewer, "launch_passive", fake_launch)

    env = SO101ReachEnv(camera=True, render_mode="human", viewer_ui=False, viewer_maximize=True)
    env.render()  # 惰性开 viewer
    assert captured.get("show_left_ui") is False
    assert captured.get("show_right_ui") is False

    captured.clear()
    env2 = SO101ReachEnv(camera=True, render_mode="human")
    env2.render()
    assert captured.get("show_left_ui") is True
    assert captured.get("show_right_ui") is True
    env2.close()
