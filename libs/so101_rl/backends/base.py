"""TrainerSpec 抽象：sb3/skrl 双后端统一接口（workflow/plan/rl.md Phase 1）。

契约（tests/test_rl_backends.py 锁定）:
- make_model(env, **hparams) -> model
- train(model, total_timesteps, out_dir) -> info dict（含最终 eval success）
- save(model, ckpt_dir) -> 写 model.{zip|pt} + meta.json
- load(ckpt_dir) -> policy: 可调用 obs(生产环境 obs) -> action(np.ndarray(6) in [-1,1])
"""

from __future__ import annotations

import abc
import json
import time
from pathlib import Path


def write_meta(ckpt_dir: Path, backend: str, algo: str, env_id: str,
               obs_dim: int, normalize: bool, hparams: dict | None = None,
               env_kwargs: dict | None = None) -> None:
    """checkpoint 元数据（推理侧校验 env/obs 一致性用）。

    env_kwargs 必须随 checkpoint 保存（action_mode=delta / domain_randomization 等）:
    推理侧建 env 时必须使用同一配置，否则动作语义不一致。
    """
    meta = {
        "backend": backend,
        "algo": algo,
        "env_id": env_id,
        "obs_dim": obs_dim,
        "action_dim": 6,
        "normalize": normalize,
        "env_kwargs": dict(env_kwargs or {}),
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    if hparams:
        meta["hparams"] = {k: v for k, v in hparams.items()}
    (Path(ckpt_dir) / "meta.json").write_text(json.dumps(meta, indent=2))


def read_meta(ckpt_dir: Path) -> dict:
    return json.loads((Path(ckpt_dir) / "meta.json").read_text())


class TrainerSpec(abc.ABC):
    """双后端统一训练接口。"""

    backend: str = ""

    @abc.abstractmethod
    def make_model(self, env, seed: int, hparams: dict):
        """用训练 env 构造模型。"""

    @abc.abstractmethod
    def train(self, model, total_timesteps: int, eval_env, eval_freq: int,
              out_dir: Path) -> dict:
        """训练并返回 info（含 eval 历史）；过程中写 checkpoint 到 out_dir。"""

    @abc.abstractmethod
    def save(self, model, ckpt_dir: Path) -> None:
        """保存 model 文件 + meta.json。"""

    @abc.abstractmethod
    def load_policy(self, ckpt_dir: Path):
        """加载 checkpoint 目录 -> 可调用 policy(obs) -> action。"""
