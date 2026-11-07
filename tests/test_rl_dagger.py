"""DAgger 算法本体契约测试（workflow/plan/rl_dagger.md Phase 1/2, TDD）。

契约（libs/so101_rl/dagger.py）:
- StudentPolicy: N 相机图像 (B,N,3,H,W) float -> 确定性 action (B,6) in [-1,1]
- DAggerBuffer: 按 collect 轮组织, 写满环形覆盖最旧轮; uint8 存 rgb, float32 取出
- train_dagger: student rollout -> expert 逐条标注 -> BC mse 更新; checkpoint = model.pt + meta.json
- load_student_policy: 读 meta 重建网络 + load state_dict
- load_dagger_hparams: 内置默认 < configs/dagger.yaml < overrides
"""

from __future__ import annotations

import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import gymnasium as gym
import numpy as np
import pytest
import torch

import so101_envs  # noqa: F401  注册副作用
from so101_rl.dagger import (
    DEFAULT_HP,
    DAggerBuffer,
    StudentPolicy,
    load_dagger_hparams,
    load_student_policy,
    save_student_policy,
    stack_camera_images,
    train_dagger,
)

ENV_ID = "SO101ReachCamera-v1"
# 图像尺寸必须与 envs 一致（RENDER_SIZE 128x128; apply_camera_calib 只覆写
# cam_pos/quat/fovy, 与分辨率解耦, 标定保持默认开启）:
# 全部走 env 默认构造（不覆写 render_size, 标定保持默认开启）
RENDER_SIZE = (128, 128)
CAMERA_KEYS = ["fixed"]


def _fake_expert(state: np.ndarray) -> np.ndarray:
    """假 expert: 单条 obs state (S,) -> action (6,) in [-1,1]（真 expert 契约同形）。"""
    return np.clip(-0.5 * np.asarray(state, dtype=np.float32)[:6], -1.0, 1.0)


def _make_vec_env(n_envs: int = 2) -> gym.vector.VectorEnv:
    return gym.make_vec(ENV_ID, num_envs=n_envs, vectorization_mode="sync")


def _tiny_hparams(**kw) -> dict:
    hp = {
        "camera_keys": CAMERA_KEYS,
        "collect_size": 2,
        "num_updates": 2,
        "minibatch": 4,
        "lr": 3e-4,
        "bc_loss_th": 0.0,  # 永不早停, 保证跑满轮数
        "learning_starts": 2,
        "buffer_size": 64,
        "total_rounds": 2,
        "seed": 0,
        "progress": False,  # 测试输出静默; 进度条契约见 test_train_progress_flags
    }
    hp.update(kw)
    return hp


# ---------- Phase 1: StudentPolicy ----------


def test_student_policy_output_shape_and_range():
    policy = StudentPolicy(n_cameras=1)  # 缺省 in_hw = env RENDER_SIZE (128, 128)
    policy.eval()
    with torch.no_grad():
        out = policy(torch.rand(2, 1, 3, 128, 128))
    assert out.shape == (2, 6)
    assert torch.all(out >= -1.0) and torch.all(out <= 1.0)


def test_student_policy_two_cameras_forward():
    policy = StudentPolicy(n_cameras=2)
    policy.eval()
    with torch.no_grad():
        out = policy(torch.rand(2, 2, 3, 128, 128))
    assert out.shape == (2, 6)
    assert torch.all(out >= -1.0) and torch.all(out <= 1.0)


def test_student_policy_deterministic():
    policy = StudentPolicy(n_cameras=1)
    policy.eval()
    x = torch.rand(1, 1, 3, 128, 128)
    with torch.no_grad():
        a = policy(x)
        b = policy(x)
    assert torch.allclose(a, b)


def test_stack_camera_images():
    rgb = {"fixed": np.zeros((2, 128, 128, 3), dtype=np.uint8),
           "wrist": np.full((2, 128, 128, 3), 255, dtype=np.uint8)}
    stacked = stack_camera_images(rgb, ["fixed", "wrist"])
    assert stacked.shape == (2, 2, 128, 128, 3)
    assert stacked.dtype == np.uint8
    assert stacked[:, 0].max() == 0 and stacked[:, 1].min() == 255


def test_student_policy_save_load_roundtrip(tmp_path):
    policy = StudentPolicy(n_cameras=2)
    policy.eval()
    save_student_policy(policy, tmp_path, env_id=ENV_ID,
                        env_kwargs={"render_size": [128, 128]},
                        hparams={"camera_keys": ["fixed", "wrist"]}, obs_dim=47)
    assert (tmp_path / "model.pt").exists()
    meta = json.loads((tmp_path / "meta.json").read_text())
    assert meta["backend"] == "dagger"
    assert meta["env_id"] == ENV_ID
    assert meta["env_kwargs"]["render_size"] == [128, 128]
    assert meta["obs_dim"] == 47

    loaded = load_student_policy(tmp_path)
    loaded.eval()
    x = torch.rand(1, 2, 3, 128, 128)
    with torch.no_grad():
        assert torch.allclose(loaded(x), policy(x), atol=1e-6)


# ---------- Phase 1: DAggerBuffer ----------


def _round_obs(steps: int, state_dim: int = 47, hw: int = 8, state_base: float = 0.0,
               rgb_val: int = 0) -> tuple[dict, np.ndarray]:
    obs = {
        "state": np.full((steps, state_dim), state_base, dtype=np.float32),
        "rgb": {"fixed": np.full((steps, hw, hw, 3), rgb_val, dtype=np.uint8)},
    }
    actions = np.tile(np.arange(6, dtype=np.float32) * 0.1, (steps, 1))
    return obs, actions


def test_dagger_buffer_ring_overwrite():
    buffer = DAggerBuffer(collect_size=4, buffer_size=8)  # 容量 = 8//4 = 2 轮
    obs_a, act_a = _round_obs(4, state_base=1.0, rgb_val=1)
    obs_b, act_b = _round_obs(4, state_base=2.0, rgb_val=2)
    obs_c, act_c = _round_obs(4, state_base=3.0, rgb_val=3)

    buffer.append_round(obs_a, act_a)
    buffer.append_round(obs_b, act_b)
    assert len(buffer) == 8
    assert buffer.num_rounds == 2

    buffer.append_round(obs_c, act_c)  # 写满后覆盖最旧轮 A
    assert len(buffer) == 8
    assert buffer.num_rounds == 2
    states = np.concatenate([r["state"] for r in buffer.rounds])
    assert 3.0 in states[:, 0] and 2.0 in states[:, 0]
    assert 1.0 not in states[:, 0]  # A 已被覆盖


def test_dagger_buffer_sample_shapes_and_dtype():
    buffer = DAggerBuffer(collect_size=4, buffer_size=8)
    for i in (1, 2):
        obs, act = _round_obs(4, state_base=float(i), rgb_val=i)
        buffer.append_round(obs, act)

    rng = np.random.default_rng(0)
    batch = buffer.sample(6, rng)
    assert batch["state"].shape == (6, 47)
    assert batch["state"].dtype == np.float32
    assert batch["rgb"]["fixed"].shape == (6, 8, 8, 3)
    assert batch["rgb"]["fixed"].dtype == np.uint8
    assert batch["actions"].shape == (6, 6)
    assert batch["actions"].dtype == np.float32
    # 采样值必须来自已存数据（两轮 state_base=1/2 混合 -> 新旧均匀）
    vals = set(np.unique(batch["state"][:, 0]).tolist())
    assert vals <= {1.0, 2.0} and len(vals) == 2


# ---------- Phase 2: configs/dagger.yaml 超参优先级 ----------


def test_load_dagger_hparams_priority():
    defaults = load_dagger_hparams()
    assert defaults["lr"] == pytest.approx(3e-4)
    assert "bc_loss_th" in defaults and "collect_size" in defaults

    # overrides 覆盖 yaml/内置
    overridden = load_dagger_hparams({"lr": 0.1, "collect_size": 7})
    assert overridden["lr"] == 0.1 and overridden["collect_size"] == 7
    # 未覆盖键保持默认
    assert overridden["minibatch"] == defaults["minibatch"]


def test_train_progress_flags():
    """进度条契约: 默认开启（CLI --no-progress 关闭）; progress=False 时 log_fn
    仍逐轮收到日志（tqdm 进度条走 stderr, 不进 log_fn 通道）。"""
    assert DEFAULT_HP["progress"] is True
    logs: list[str] = []
    env = gym.make_vec(ENV_ID, num_envs=1, vectorization_mode="sync")
    try:
        info = train_dagger(_fake_expert, env, _tiny_hparams(),
                            log_fn=logs.append)
    finally:
        env.close()
    assert info["rounds"] == 2
    assert any("round 2" in m for m in logs)


# ---------- Phase 2: train_dagger 微型 smoke ----------


def test_train_dagger_smoke_single_camera(tmp_path):
    vec_env = _make_vec_env(n_envs=2)
    try:
        info = train_dagger(_fake_expert, vec_env, hparams=_tiny_hparams(),
                            out_dir=tmp_path, env_id=ENV_ID,
                            env_kwargs={"render_size": list(RENDER_SIZE)})
        assert info["rounds"] == 2
        assert len(info["bc_losses"]) > 0
        assert all(np.isfinite(x) for x in info["bc_losses"])

        assert (tmp_path / "model.pt").exists()
        meta = json.loads((tmp_path / "meta.json").read_text())
        assert meta["backend"] == "dagger"
        assert meta["env_id"] == ENV_ID
        assert meta["env_kwargs"]["render_size"] == [128, 128]

        student = load_student_policy(tmp_path)
        obs, _ = vec_env.reset(seed=3)
        rgb = {k: v for k, v in obs["rgb"].items()}  # (n_envs,H,W,3) uint8
        stacked = stack_camera_images(rgb, meta["hparams"]["camera_keys"])
        imgs = torch.from_numpy(stacked).permute(0, 1, 4, 2, 3).float() / 255.0
        with torch.no_grad():
            action = student(imgs)
        assert action.shape == (2, 6)
        assert torch.all(action >= -1.0) and torch.all(action <= 1.0)
    finally:
        vec_env.close()


def test_train_dagger_smoke_two_cameras(tmp_path):
    vec_env = _make_vec_env(n_envs=2)
    try:
        info = train_dagger(_fake_expert, vec_env,
                            hparams=_tiny_hparams(camera_keys=["fixed", "wrist"],
                                                  total_rounds=1),
                            out_dir=tmp_path, env_id=ENV_ID,
                            env_kwargs={"render_size": list(RENDER_SIZE)})
        assert info["rounds"] == 1
        assert len(info["bc_losses"]) > 0
        student = load_student_policy(tmp_path)
        assert student.n_cameras == 2
    finally:
        vec_env.close()


# ---------- Risk R1: 相机标定 x render_size 观察 ----------


def test_camera_calib_at_render_size():
    """R1 风险观察: 渲染分辨率与相机标定的关系（已确认为解耦）。

    实测结论（apply_camera_calib 只覆写 cam_pos/cam_quat/cam_fovy, 见
    libs/so101_envs/camera_calib.py:75-88）:
    - 标定与渲染分辨率完全解耦, 任何分辨率下开启标定都不报错、不崩溃
      （128x128 为 1:1 纵横比, 水平视野比 4:3 窄, 但几何一致）。
    - 结论: 图像尺寸与 envs 渲染尺寸一致（RENDER_SIZE 128x128, 机器性能约束）,
      标定保持默认开启 —— 本文件全链路均用 env 默认构造。
    - 标定确实生效: 同 seed 下开启/关闭标定的渲染像素不同（位姿/视场被覆写）。
    """
    env_on = gym.make(ENV_ID)  # env 默认: render_size=(128,128) + 标定开启
    env_off = gym.make(ENV_ID, camera_calib=False)
    try:
        obs_on, _ = env_on.reset(seed=0)
        obs_off, _ = env_off.reset(seed=0)
        img_on = obs_on["rgb"]["fixed"]
        assert img_on.shape == (128, 128, 3) and img_on.dtype == np.uint8
        assert not np.array_equal(img_on, obs_off["rgb"]["fixed"])
    finally:
        env_on.close()
        env_off.close()
