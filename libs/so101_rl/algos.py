"""算法注册表：--backend/--algo -> TrainerSpec + 默认超参（plan/rl.md Phase 3）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class AlgoSpec:
    backend: str          # sb3 | skrl
    algo: str             # sac | ppo | td3
    defaults: dict = field(default_factory=dict)


REGISTRY: dict[str, AlgoSpec] = {}


def register(backend: str, algo: str, defaults: dict) -> None:
    REGISTRY[f"{backend}:{algo}"] = AlgoSpec(backend=backend, algo=algo, defaults=defaults)


def load_defaults(backend: str, algo: str, configs_dir: Path | None = None) -> dict:
    """默认超参：代码内置 < configs/<backend>_<algo>.yaml < CLI overrides。"""
    key = f"{backend}:{algo}"
    if key not in REGISTRY:
        raise KeyError(f"unknown algo {key}; known: {sorted(REGISTRY)}")
    params = dict(REGISTRY[key].defaults)
    if configs_dir is not None:
        import yaml

        cfg_path = Path(configs_dir) / f"{backend}_{algo}.yaml"
        if cfg_path.exists():
            params.update(yaml.safe_load(cfg_path.read_text()) or {})
    return params


# 内置默认（configs/<backend>_<algo>.yaml 覆盖后再由 CLI overrides 覆盖）
register("sb3", "sac", {"learning_rate": 3e-4, "buffer_size": 300_000, "batch_size": 256,
                        "gamma": 0.99, "tau": 0.005, "train_freq": 1, "gradient_steps": 1,
                        "learning_starts": 2000, "ent_coef": "auto"})
register("sb3", "ppo", {"learning_rate": 3e-4, "n_steps": 2048, "batch_size": 64,
                        "n_epochs": 10, "gamma": 0.99, "gae_lambda": 0.95,
                        "clip_range": 0.2, "ent_coef": 0.0})
register("sb3", "td3", {"learning_rate": 1e-3, "buffer_size": 300_000, "batch_size": 256,
                        "gamma": 0.99, "tau": 0.005, "train_freq": 1, "gradient_steps": 1,
                        "learning_starts": 2500})
register("skrl", "ppo", {"learning_rate": 3e-4, "gamma": 0.99, "batch_size": 4096,
                         "rollouts": 16, "learning_epochs": 10, "clip_range": 0.2})
register("skrl", "sac", {"learning_rate": 3e-4, "gamma": 0.99, "batch_size": 256,
                         "buffer_size": 300_000, "learning_starts": 2000})


def get_trainer(backend: str, algo: str):
    """延迟导入后端模块，返回 TrainerSpec 实例。"""
    if backend == "sb3":
        from .backends.sb3_backend import Sb3Trainer

        return Sb3Trainer(algo=algo)
    if backend == "skrl":
        from .backends.skrl_backend import SkrlTrainer

        return SkrlTrainer(algo=algo)
    raise KeyError(f"unknown backend {backend}")


def known_algos() -> list[str]:
    return sorted(REGISTRY)
