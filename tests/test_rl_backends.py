"""RL 双后端契约测试（plan/rl.md Phase 1/2, TDD）。

契约：make_env / make_model / train / save / load_policy 形状与目录约定一致。
"""

from __future__ import annotations

import json
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import gymnasium as gym
import numpy as np
import pytest

import so101_envs  # noqa: F401  注册副作用

ENV_ID = "SO101Reach-v1"
OBS_DIM = 25


def _train_hparams(algo):
    """短训练 smoke 超参（覆盖默认的慢参数）。"""
    if algo == "sac":
        return {"learning_starts": 50, "batch_size": 32, "train_freq": 1, "buffer_size": 5000}
    if algo == "td3":
        return {"learning_starts": 50, "batch_size": 32, "train_freq": 1, "buffer_size": 5000}
    return {}  # ppo 默认即可


def _contract(trainer, algo, tmp_path):
    env = trainer.make_env(ENV_ID, seed=0)
    eval_env = trainer.make_env(ENV_ID, seed=100)
    try:
        model = trainer.make_model(env, seed=0, hparams=_train_hparams(algo))
        info = trainer.train(model, total_timesteps=200, eval_env=eval_env,
                             eval_freq=100, out_dir=tmp_path)
        assert "eval_history" in info and isinstance(info["eval_history"], list)

        trainer.save(model, tmp_path)
        meta = json.loads((tmp_path / "meta.json").read_text())
        assert meta["backend"] == trainer.backend
        assert meta["algo"] == algo
        assert meta["env_id"] == ENV_ID
        assert meta["obs_dim"] == OBS_DIM

        policy = trainer.load_policy(tmp_path)
        obs, _ = env.reset(seed=5)
        action = policy(obs)
        arr = np.asarray(action)
        assert arr.shape == (6,)
        assert np.all(arr >= -1.0) and np.all(arr <= 1.0)
    finally:
        env.close()
        eval_env.close()


def test_sb3_contract(tmp_path):
    from so101_rl.algos import get_trainer

    trainer = get_trainer("sb3", "sac")
    assert trainer.backend == "sb3"
    _contract(trainer, "sac", tmp_path / "ckpt_sb3")


def test_registry_knows_planned_algos():
    """5 个计划内 (backend, algo) 组合都在注册表。"""
    from so101_rl.algos import known_algos

    assert set(known_algos()) == {
        "sb3:sac", "sb3:ppo", "sb3:td3", "skrl:ppo", "skrl:sac",
    }


def test_registry_rejects_unknown():
    from so101_rl.algos import load_defaults

    with pytest.raises(KeyError):
        load_defaults("sb3", "nope")


def test_skrl_contract(tmp_path):
    from so101_rl.algos import get_trainer

    trainer = get_trainer("skrl", "ppo")
    assert trainer.backend == "skrl"
    _contract(trainer, "ppo", tmp_path / "ckpt_skrl")


@pytest.mark.parametrize("key", ["sb3:sac", "sb3:ppo", "sb3:td3", "skrl:ppo", "skrl:sac"])
def test_registry_smoke(key, tmp_path):
    """Phase 3: 每个注册组合 构造+短训+save/load/predict 全链路不炸。"""
    backend, algo = key.split(":")
    from so101_rl.algos import get_trainer

    trainer = get_trainer(backend, algo)
    env = trainer.make_env(ENV_ID, seed=0)
    try:
        model = trainer.make_model(env, seed=0, hparams=_train_hparams(algo))
        info = trainer.train(model, total_timesteps=20, eval_env=trainer.make_env(ENV_ID),
                             eval_freq=10, out_dir=tmp_path / "ck")
        trainer.save(model, tmp_path / "ck")
        policy = trainer.load_policy(tmp_path / "ck")
        obs, _ = env.reset(seed=1)
        assert np.asarray(policy(obs)).shape == (6,)
    finally:
        env.close()


def test_backend_algo_identity():
    """skrl:sac 必须构造真正的 skrl SAC agent（而非 PPO 冒名）。"""
    from so101_rl.algos import get_trainer

    env = get_trainer("skrl", "ppo").make_env(ENV_ID, seed=0)
    try:
        skrl_model = get_trainer("skrl", "sac").make_model(env, seed=0, hparams={})
        agent = skrl_model[0] if isinstance(skrl_model, tuple) else skrl_model
        from skrl.agents.torch.sac import SAC as SkrlSAC

        assert isinstance(agent, SkrlSAC), f"got {type(agent)}"

        sb3_model = get_trainer("sb3", "sac").make_model(env, seed=0, hparams={})
        from stable_baselines3 import SAC as Sb3SAC

        assert isinstance(sb3_model, Sb3SAC)
    finally:
        env.close()


def test_deploy_protocol(tmp_path):
    """Phase 5: GymPolicyAdapter 满足 SO101DeployPolicy 协议（零 lerobot 依赖）。"""
    from so101_rl.algos import get_trainer
    from so101_rl.lerobot_deploy import GymPolicyAdapter, SO101DeployPolicy

    trainer = get_trainer("sb3", "sac")
    env = trainer.make_env(ENV_ID, seed=0)
    try:
        model = trainer.make_model(env, seed=0, hparams=_train_hparams("sac"))
        trainer.save(model, tmp_path / "ck")
        adapter = GymPolicyAdapter.from_checkpoint(tmp_path / "ck")
        # 协议: isinstance + reset()/act(obs)
        assert isinstance(adapter, SO101DeployPolicy)
        adapter.reset()
        action = adapter.act(np.zeros(OBS_DIM))
        arr = np.asarray(action)
        assert arr.shape == (6,)
        assert np.all(arr >= -1.0) and np.all(arr <= 1.0)
    finally:
        env.close()


# ---------------- 多环境 (num_envs) + 训练可视化 ----------------

@pytest.mark.parametrize("key", ["sb3:sac", "skrl:ppo"])
def test_multi_env_contract(key, tmp_path):
    """n_envs=2 训练全链路（sb3 SubprocVecEnv / skrl SyncVectorEnv）。"""
    backend, algo = key.split(":")
    from so101_rl.algos import get_trainer

    trainer = get_trainer(backend, algo)
    vec = trainer.make_vec_envs(ENV_ID, seed=0, n_envs=2)
    eval_env = trainer.make_env(ENV_ID, seed=None)
    try:
        model = trainer.make_model(vec, seed=0, hparams=_train_hparams(algo))
        info = trainer.train(model, total_timesteps=20, eval_env=eval_env,
                             eval_freq=10, out_dir=tmp_path / "ck")
        trainer.save(model, tmp_path / "ck")
        policy = trainer.load_policy(tmp_path / "ck")
        obs, _ = eval_env.reset(seed=1)
        assert np.asarray(policy(obs)).shape == (6,)
    finally:
        vec.close() if hasattr(vec, "close") else None
        eval_env.close()


def test_eval_curve_png(tmp_path):
    """训练可视化: eval 曲线绘制为 png。"""
    import matplotlib

    matplotlib.use("Agg")
    from so101_rl.plotting import plot_eval_history

    out = plot_eval_history([(100, 0.0), (200, 0.4), (300, 0.8)], tmp_path)
    assert (tmp_path / "curve.png").exists() and out.name == "curve.png"


@pytest.mark.parametrize("key", ["sb3:sac", "skrl:ppo"])
def test_tensorboard_logging(key, tmp_path):
    """训练写 tensorboard 事件（--tb 默认开启），eval success 一并入 TB。"""
    backend, algo = key.split(":")
    from so101_rl.algos import get_trainer

    trainer = get_trainer(backend, algo)
    env = trainer.make_env(ENV_ID, seed=0)
    try:
        model = trainer.make_model(env, seed=0, hparams=_train_hparams(algo),
                                   tb_dir=tmp_path / "tb")
        trainer.train(model, total_timesteps=20, eval_env=trainer.make_env(ENV_ID),
                      eval_freq=10, out_dir=tmp_path / "ck")
        events = list((tmp_path / "tb").rglob("events.out.tfevents*"))
        assert events, f"no tfevents under {tmp_path / 'tb'}"
    finally:
        env.close()
