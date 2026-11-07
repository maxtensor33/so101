"""DAgger 训练/评估脚本契约测试（workflow/plan/rl_dagger.md Phase 3, TDD）。

契约:
- scripts/rl_dagger_train.py: expert 走 resolve_ckpt_dir->read_meta->get_trainer->load_policy
  链; --env 缺省由 expert meta env id 推导相机版; env_kwargs 落盘 camera=True/render_size=
  [128,128]（env 原生）/camera_calib=True（默认开启, --no-camera-calib 关闭）
  + action_mode/domain_randomization 沿 expert meta（动力学一致）
- scripts/rl_dagger_eval.py: load_student_policy + meta env 重建单 env,
  确定性推理统计 success/平均回报/平均步数
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

import rl_dagger_eval
import rl_dagger_train  # noqa: E402 (conftest 已注入 scripts 路径)

CAMERA_ENV = "SO101ReachCamera-v1"
EXPERT_ENV = "SO101Reach-v1"


def _fake_expert(state: np.ndarray) -> np.ndarray:
    """假 expert: 单条 obs state (S,) -> action (6,) in [-1,1]（真 expert 契约同形）。"""
    return np.clip(-0.5 * np.asarray(state, dtype=np.float32)[:6], -1.0, 1.0)


def _write_fake_expert_meta(tmp_path: Path) -> Path:
    """伪 expert checkpoint 目录（仅 meta.json; expert 本体经 main(expert=...) 注入,
    不依赖 outputs/ 下真实 checkpoint）。"""
    ckpt = tmp_path / "expert"
    ckpt.mkdir()
    (ckpt / "meta.json").write_text(json.dumps({
        "backend": "sb3", "algo": "sac", "env_id": EXPERT_ENV,
        "obs_dim": 47, "action_dim": 6, "normalize": False,
        "env_kwargs": {"action_mode": "delta", "domain_randomization": True},
    }))
    return ckpt


def _parse(argv):
    return rl_dagger_train.build_parser().parse_known_args(argv)[0]


# ---------- 纯逻辑: env id 推导 / 超参组装（无 env, 快） ----------


def test_derive_camera_env_id():
    assert rl_dagger_train.derive_camera_env_id("SO101Lift-v1") == "SO101LiftCamera-v1"
    # 已是相机版原样返回
    assert rl_dagger_train.derive_camera_env_id(CAMERA_ENV) == CAMERA_ENV


def test_short_env_name():
    assert rl_dagger_train.short_env_name(CAMERA_ENV) == "reach"
    assert rl_dagger_train.short_env_name("SO101LiftCanCamera-v1") == "liftcan"


def test_parse_render_size():
    assert rl_dagger_train.parse_render_size("128x128") == (128, 128)
    # HEIGHTxWIDTH 约定（env RENDER_SIZE=(128, 128) 同约定）
    assert rl_dagger_train.parse_render_size("64x96") == (64, 96)
    with pytest.raises(Exception):
        rl_dagger_train.parse_render_size("128")


def test_parse_overrides_types():
    out = rl_dagger_train.parse_overrides(["lr=0.01", "collect_size=8", "bc_loss_th=0.5"])
    assert out == {"lr": 0.01, "collect_size": 8, "bc_loss_th": 0.5}
    assert rl_dagger_train.parse_overrides(["camera_keys=fixed"]) == {"camera_keys": "fixed"}


def test_build_hparams_cli_mapping():
    args = _parse(["--expert-ckpt", "x", "--camera-keys", "fixed,wrist",
                   "--rounds", "3", "--n-envs", "2", "--render-size", "64x96",
                   "--seed", "7",
                   "--overrides", "lr=0.01", "collect_size=8",
                   "--overrides", "bc_loss_th=0.5"])
    hp = rl_dagger_train.build_hparams(args)
    # CLI 旗标映射
    assert hp["camera_keys"] == ["fixed", "wrist"]
    assert hp["total_rounds"] == 3
    assert hp["n_envs"] == 2
    assert hp["render_size"] == [64, 96]
    assert hp["seed"] == 7
    # --overrides 覆盖生效（含多次）
    assert hp["lr"] == 0.01 and hp["collect_size"] == 8 and hp["bc_loss_th"] == 0.5
    # 未覆盖键保持 configs/dagger.yaml 默认
    assert hp["minibatch"] == rl_dagger_train.load_dagger_hparams()["minibatch"]


def test_build_env_kwargs_follows_meta_and_r1():
    meta = {"env_kwargs": {"action_mode": "delta", "domain_randomization": True}}
    args = _parse(["--expert-ckpt", "x"])
    kw = rl_dagger_train.build_env_kwargs(meta, args)
    # 默认 render_size=128x128（env 原生）+ camera_calib 默认开启
    # （env 语义: None = 默认路径标定开启, False = 显式关闭, 传 True 非法）
    assert kw["camera"] is True
    assert kw["render_size"] == [128, 128]
    assert kw["camera_calib"] is None
    # 动力学沿 expert meta
    assert kw["action_mode"] == "delta"
    assert kw["domain_randomization"] is True

    # --no-dr 覆盖 meta / --no-camera-calib 关闭标定
    kw_off = rl_dagger_train.build_env_kwargs(
        meta, _parse(["--expert-ckpt", "x", "--no-dr", "--no-camera-calib"]))
    assert kw_off["domain_randomization"] is False
    assert kw_off["camera_calib"] is False
    # meta 缺字段时回退（action_mode 沿 rl_train 默认 delta, dr 默认 True）
    kw_fallback = rl_dagger_train.build_env_kwargs({}, _parse(["--expert-ckpt", "x"]))
    assert kw_fallback["action_mode"] == "delta"
    assert kw_fallback["domain_randomization"] is True


# ---------- smoke: train -> 落盘 -> eval（微型 Camera env） ----------


def test_train_smoke_fake_expert_then_eval(tmp_path, capsys):
    fake_ckpt = _write_fake_expert_meta(tmp_path)
    out = tmp_path / "dagger"
    rc = rl_dagger_train.main(
        ["--expert-ckpt", str(fake_ckpt), "--out", str(out), "--n-envs", "1",
         "--rounds", "1", "--seed", "0",
         "--overrides", "collect_size=2", "num_updates=2", "minibatch=4",
         "learning_starts=2", "buffer_size=64"],
        expert=_fake_expert)
    assert rc == 0

    assert (out / "model.pt").exists()
    meta = json.loads((out / "meta.json").read_text())
    assert meta["backend"] == "dagger"
    # --env 缺省 -> 由 expert meta env id 推导相机版
    assert meta["env_id"] == CAMERA_ENV
    # 默认 128x128 + camera_calib 默认开启（env 语义 None）+ meta env_kwargs 随存
    assert meta["env_kwargs"]["camera"] is True
    assert meta["env_kwargs"]["render_size"] == [128, 128]
    assert meta["env_kwargs"]["camera_calib"] is None
    assert meta["env_kwargs"]["action_mode"] == "delta"
    assert meta["env_kwargs"]["domain_randomization"] is True
    assert meta["hparams"]["camera_keys"] == ["fixed"]

    # eval 脚本可直接加载该 ckpt 并输出统计
    rc = rl_dagger_eval.main(["--ckpt", str(out), "--episodes", "1"])
    assert rc == 0
    text = capsys.readouterr().out
    assert "success rate" in text
    assert "ep 0" in text


def test_train_real_expert_chain(tmp_path):
    """expert 注入缺省时走 resolve_ckpt_dir->read_meta->get_trainer->load_policy 链
    （D5; 微型 sb3 SAC checkpoint, 镜像 tests/test_rl_backends.py 惯例）。"""
    from so101_rl.algos import get_trainer

    expert_ckpt = tmp_path / "expert_sb3"
    trainer = get_trainer("sb3", "sac")
    env = trainer.make_env(EXPERT_ENV, seed=0,
                           env_kwargs={"action_mode": "delta",
                                       "domain_randomization": True})
    try:
        model = trainer.make_model(env, seed=0,
                                   hparams={"learning_starts": 50, "batch_size": 32,
                                            "train_freq": 1, "buffer_size": 5000})
        trainer.train(model, total_timesteps=200, eval_env=None, eval_freq=10**9,
                      out_dir=expert_ckpt)
        trainer.save(model, expert_ckpt)
    finally:
        env.close()

    out = tmp_path / "dagger"
    rc = rl_dagger_train.main(
        ["--expert-ckpt", str(expert_ckpt), "--out", str(out), "--n-envs", "1",
         "--rounds", "1", "--seed", "0",
         "--overrides", "collect_size=2", "num_updates=2", "minibatch=4",
         "learning_starts=2", "buffer_size=64"])
    assert rc == 0
    meta = json.loads((out / "meta.json").read_text())
    assert meta["backend"] == "dagger"
    assert meta["env_id"] == CAMERA_ENV  # meta env SO101Reach-v1 推导相机版


def test_eval_rejects_non_dagger_checkpoint(tmp_path):
    fake_ckpt = _write_fake_expert_meta(tmp_path)
    with pytest.raises(ValueError, match="backend"):
        rl_dagger_eval.main(["--ckpt", str(fake_ckpt), "--episodes", "1"])
