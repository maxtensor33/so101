"""SB3 后端（plan/rl.md Phase 1）。

- make_model: SAC/PPO/TD3 直接接受 gymnasium env（sb3 自动包 Monitor+DummyVecEnv）
- train: 自写 SuccessEvalCallback（统计 info["success"]，sb3 内置 EvalCallback 只给 reward）
- save/load: model.zip + meta.json (+ vecnormalize.pkl 若启用 normalize)
"""

from __future__ import annotations

from pathlib import Path

import gymnasium as gym
import numpy as np

from ..normalize import NORM_FILE, save_norm
from .base import TrainerSpec, write_meta

MAX_EP_STEPS = 50  # reach/lift episode 上限（与注册 max_episode_steps 一致）


class SuccessEvalCallback:
    """每 eval_freq 步跑 eval_env 若干 episode 统计 success rate。"""

    def __init__(self, eval_env, eval_freq: int, n_episodes: int = 5, seed_base: int = 1000):
        self.eval_env = eval_env
        self.eval_freq = eval_freq
        self.n_episodes = n_episodes
        self.seed_base = seed_base
        self.history: list[tuple[int, float]] = []
        self._next = eval_freq

    def __call__(self, locals_: dict, globals_: dict) -> bool:
        # 函数式 callback 的返回值 = continue_training, 必须恒返回 True
        model = locals_["self"] if "self" in locals_ else locals_.get("model")
        num_timesteps = model.num_timesteps
        if num_timesteps < self._next:
            return True
        self._next += self.eval_freq
        rate = self._evaluate(model)
        self.history.append((num_timesteps, rate))
        model.logger.record("eval/success_rate", rate)
        return True

    def _evaluate(self, model) -> float:
        n_ok = 0
        for ep in range(self.n_episodes):
            obs, _ = self.eval_env.reset(seed=self.seed_base + ep)
            done = False
            for _ in range(MAX_EP_STEPS):
                action, _ = model.predict(obs, deterministic=True)
                obs, _, term, trunc, info = self.eval_env.step(action)
                done = term or trunc
                if done:
                    break
            n_ok += int(info.get("success", False))
        return n_ok / self.n_episodes


class Sb3Trainer(TrainerSpec):
    backend = "sb3"

    def __init__(self, algo: str = "sac"):
        self.algo = algo

    def make_env(self, env_id: str, seed=None):
        """创建单 env（评估/推理用）。"""
        self._env_id = env_id
        return gym.make(env_id)

    def make_vec_envs(self, env_id: str, seed: int = 0, n_envs: int = 1):
        """CPU 多环境: DummyVecEnv（单进程串行多 env）。

        sb3 官方文档: DummyVecEnv "usually faster than SubprocVecEnv"——
        mujoco 类轻量 step 环境, 进程 IPC 开销反而高于仿真本身, 且无子进程内存/OOM 问题
        （64 envs SubprocVecEnv 在 15GB 机器上曾触发 OOM killer 连带杀掉图形会话）。
        """
        from stable_baselines3.common.monitor import Monitor
        from stable_baselines3.common.vec_env import DummyVecEnv

        self._env_id = env_id
        env_fns = [lambda: Monitor(gym.make(env_id)) for _ in range(n_envs)]
        return DummyVecEnv(env_fns)

    def make_model(self, env, seed: int, hparams: dict, tb_dir=None):
        from stable_baselines3 import SAC, PPO, TD3

        # env 可为单 gym env 或 VecEnv; env_id 优先取 make_env(s) 记录值
        if not hasattr(self, "_env_id"):
            if hasattr(env, "envs"):
                self._env_id = env.envs[0].spec.id
            elif hasattr(env, "spec"):
                self._env_id = env.spec.id
            else:
                self._env_id = ""
        cls = {"sac": SAC, "ppo": PPO, "td3": TD3}[self.algo]
        kwargs = {}
        if tb_dir is not None:
            kwargs["tensorboard_log"] = str(tb_dir)  # sb3 原生 TB（learn 时创建 writer）
        if self.algo == "ppo" and hasattr(env, "num_envs") and env.num_envs > 1:
            # n_steps 语义是"每环境每 rollout 步数"; 多环境时按 env 数缩减,
            # 保持每次更新的样本总量 (n_steps * n_envs) 与单环境一致。
            # 否则 64 envs x 2048 = 131k 样本才更新一次, 小步数训练时网络得不到更新。
            base = int(hparams.get("n_steps", 2048))
            hparams = dict(hparams)
            hparams["n_steps"] = max(8, base // env.num_envs)
        return cls("MlpPolicy", env, seed=seed, verbose=0, **hparams, **kwargs)

    def train(self, model, total_timesteps: int, eval_env, eval_freq: int, out_dir: Path) -> dict:
        from stable_baselines3.common.callbacks import CallbackList

        cb = SuccessEvalCallback(eval_env, eval_freq=eval_freq)
        # progress_bar=True: sb3 内置 tqdm 进度条（ProgressBarCallback）
        model.learn(total_timesteps=total_timesteps, callback=cb, progress_bar=True)
        return {"eval_history": cb.history,
                "final_success": cb.history[-1][1] if cb.history else None}

    def save(self, model, ckpt_dir: Path) -> None:
        ckpt_dir = Path(ckpt_dir)
        ckpt_dir.mkdir(parents=True, exist_ok=True)
        model.save(ckpt_dir / "model")  # -> model.zip
        vec_env = model.get_vec_normalize_env()
        normalized = save_norm(vec_env, ckpt_dir) if vec_env is not None else False
        write_meta(ckpt_dir, self.backend, self.algo, getattr(self, "_env_id", ""),
                   model.observation_space.shape[0], normalized)

    def load_policy(self, ckpt_dir: Path):
        """加载 -> 可调用 policy(obs) -> action(np.ndarray(6), [-1,1])。

        若 checkpoint 含 vecnormalize.pkl，predict 前对 obs 做同样的归一化。
        注意: sb3 load 的 cloudpickle 数据会清空 gymnasium registry
        （自定义注册 env 的已知副作用），load 前后用快照恢复。
        """
        import so101_envs  # noqa: F401  确保 registry 已注册（快照才有内容）

        from gymnasium.envs.registration import registry

        from stable_baselines3 import SAC, PPO, TD3

        ckpt_dir = Path(ckpt_dir)
        registry_snapshot = dict(registry)
        try:
            cls = {"sac": SAC, "ppo": PPO, "td3": TD3}[self.algo]
            model = cls.load(ckpt_dir / "model", device="auto")
        finally:
            registry.clear()
            registry.update(registry_snapshot)
        norm_path = ckpt_dir / NORM_FILE
        normalizer = None
        if norm_path.exists():
            from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

            dummy = DummyVecEnv([lambda: gym.make("SO101Reach-v1")])
            normalizer = VecNormalize.load(str(norm_path), dummy)

        def policy(obs):
            obs = np.asarray(obs, dtype=np.float32)
            if normalizer is not None:
                obs = normalizer.normalize_obs(obs)
            action, _ = model.predict(obs, deterministic=True)
            return np.asarray(action)

        return policy
