"""skrl 后端（plan/rl.md Phase 2）。

- 手写单 env 训练循环（镜像 skrl sequential trainer，eval callback 可嵌入）
- checkpoint: agent.save -> model.pt + meta.json (+hparams)
- load_policy: 重建 agent -> load -> mean_actions（确定性推理）
"""

from __future__ import annotations

from pathlib import Path

import gymnasium as gym
import gymnasium.vector
import numpy as np
import torch

from .base import TrainerSpec, write_meta

MAX_EP_STEPS = 50
DEVICE = "cuda:0" if torch.cuda.is_available() else "cpu"


def _build_models(env, device, hparams, algo: str, obs_space=None, act_space=None):
    from skrl.utils.model_instantiators.torch import deterministic_model, gaussian_model

    if algo == "sac":
        # SAC: deterministic policy(clip) + 双 critic(OBSERVATIONS+ACTIONS -> Q)
        policy = deterministic_model(
            observation_space=obs_space,
            action_space=act_space,
            device=device,
            clip_actions=True,
            network=[{"name": "net", "input": "OBSERVATIONS", "layers": [64, 64],
                      "activations": ["relu", "relu"]}],
            output="ACTIONS",
        )
        def critic():
            return deterministic_model(
                observation_space=obs_space,
                action_space=act_space,
                device=device,
                network=[{"name": "net", "input": "concatenate(OBSERVATIONS, ACTIONS)", "layers": [64, 64],
                          "activations": ["relu", "relu"]}],
                output="Q_VALUES",
            )
        return {"policy": policy, "critic_1": critic(), "critic_2": critic(),
                "target_critic_1": critic(), "target_critic_2": critic()}

    policy = gaussian_model(
        observation_space=obs_space,
        action_space=act_space,
        device=device,
        clip_actions=False,
        clip_mean_actions=True,
        network=[{"name": "net", "input": "OBSERVATIONS", "layers": [64, 64], "activations": ["relu", "relu"]}],
        output="ACTIONS",
    )
    value = deterministic_model(
        observation_space=obs_space,
        device=device,
        network=[{"name": "net", "input": "OBSERVATIONS", "layers": [64, 64], "activations": ["relu", "relu"]}],
        output="VALUES",
    )
    return {"policy": policy, "value": value}


class SkrlTrainer(TrainerSpec):
    backend = "skrl"

    def __init__(self, algo: str = "ppo"):
        self.algo = algo
        self._last_hparams: dict = {}

    def make_env(self, env_id: str, seed=None):
        self._env_id = env_id
        return gym.make(env_id)

    def make_vec_envs(self, env_id: str, seed: int = 0, n_envs: int = 1):
        """CPU 多环境: gymnasium 官方 make_vec（skrl 官方示例同款用法）。"""
        self._env_id = env_id
        if n_envs <= 1:
            return gym.make(env_id)
        return gym.make_vec(env_id, num_envs=n_envs, vectorization_mode="sync")

    def make_model(self, env, seed: int, hparams: dict, tb_dir=None):
        if not hasattr(self, "_env_id"):
            self._env_id = env.spec.id if hasattr(env, "spec") else ""
        return self._make_agent(env, seed, hparams, tb_dir=tb_dir)

    def _make_agent(self, env, seed: int, hparams: dict, tb_dir=None):
        from skrl.memories.torch import RandomMemory

        torch.manual_seed(seed)
        wrapped = self._wrap_env(env)
        # 官方姿势: wrapped.observation_space 在 vectorized 时自动返回 single space
        obs_space = wrapped.observation_space
        act_space = wrapped.action_space
        models = _build_models(env, DEVICE, hparams, self.algo, obs_space, act_space)
        rollouts = int(hparams.get("rollouts", 16))
        num_envs = int(getattr(wrapped, "num_envs", 1))
        if self.algo == "sac":
            agent, wrapped = self._make_sac_agent(env, wrapped, models, hparams, num_envs, obs_space, act_space, tb_dir=tb_dir)
        else:
            agent, wrapped = self._make_ppo_agent(env, wrapped, models, rollouts, hparams, num_envs, obs_space, act_space, tb_dir=tb_dir)
        # tensorboard: experiment.directory 必须在 agent.__init__ 前设置（构造时固化 experiment_dir）。
        # init 内 write_interval=="auto" 依赖 trainer_cfg.timesteps（空则 0 → 不建 writer）,
        # trainer_cfg 必须传 dataclass（dict 会 asdict 报错）。
        if tb_dir is not None:
            import dataclasses

            @dataclasses.dataclass
            class _TrainerCfg:
                timesteps: int = 100_000  # TB 写入间隔 = timesteps/100

            agent.init(trainer_cfg=_TrainerCfg())
        else:
            agent.init()
        return agent, wrapped

    def _make_ppo_agent(self, env, wrapped, models, rollouts, hparams, num_envs=1, obs_space=None, act_space=None, tb_dir=None):
        from skrl.agents.torch.ppo import PPO, PPO_CFG
        from skrl.memories.torch import RandomMemory
        memory = RandomMemory(
            memory_size=rollouts,
            num_envs=num_envs,
            device=DEVICE,
            replacement=False,  # PPO: 每个 rollout batch 用一次
        )
        cfg = PPO_CFG()
        cfg.rollouts = rollouts
        cfg.learning_epochs = int(hparams.get("learning_epochs", 10))
        cfg.mini_batches = int(hparams.get("mini_batches", 2))
        cfg.discount_factor = float(hparams.get("gamma", 0.99))
        cfg.learning_rate = float(hparams.get("learning_rate", 3e-4))
        cfg.clip_range = float(hparams.get("clip_range", 0.2))
        cfg.experiment.checkpoint_interval = 0  # 关闭自动 checkpoint
        if tb_dir is not None:
            cfg.experiment.directory = str(tb_dir)  # agent.__init__ 时固化 experiment_dir, 必须先设
            cfg.experiment.write_interval = "auto"
        else:
            cfg.experiment.write_interval = 0
        agent = PPO(
            models=models,
            memory=memory,
            observation_space=obs_space,
            action_space=act_space,
            device=DEVICE,
            cfg=cfg,
        )
        return agent, wrapped

    def _make_sac_agent(self, env, wrapped, models, hparams, num_envs=1, obs_space=None, act_space=None, tb_dir=None):
        from skrl.agents.torch.sac import SAC, SAC_CFG
        from skrl.memories.torch import RandomMemory

        memory = RandomMemory(
            memory_size=int(hparams.get("buffer_size", 300_000)),
            num_envs=num_envs,
            device=DEVICE,
            replacement=True,  # off-policy replay
        )
        cfg = SAC_CFG()
        cfg.batch_size = int(hparams.get("batch_size", 256))
        cfg.discount_factor = float(hparams.get("gamma", 0.99))
        cfg.learning_rate = float(hparams.get("learning_rate", 3e-4))
        cfg.random_timesteps = int(hparams.get("learning_starts", 2000))
        cfg.experiment.checkpoint_interval = 0
        if tb_dir is not None:
            cfg.experiment.directory = str(tb_dir)  # agent.__init__ 时固化 experiment_dir, 必须先设
            cfg.experiment.write_interval = "auto"
        else:
            cfg.experiment.write_interval = 0
        agent = SAC(
            models=models,
            memory=memory,
            observation_space=obs_space,
            action_space=act_space,
            device=DEVICE,
            cfg=cfg,
        )
        return agent, wrapped

    def _wrap_env(self, env):
        from skrl.envs.wrappers.torch import wrap_env

        return wrap_env(env)

    def train(self, model, total_timesteps: int, eval_env, eval_freq: int, out_dir: Path) -> dict:
        agent, wrapped = model
        num_envs = int(getattr(wrapped, "num_envs", 1))
        self._last_hparams = self._last_hparams or {}
        observations, _ = wrapped.reset()
        history = []
        next_eval = eval_freq
        # num_envs>1 时每 tick 推进 num_envs 个样本, timesteps 按 env 计数
        import tqdm as _tqdm

        pbar = _tqdm.tqdm(total=total_timesteps, desc="train", unit="step")
        for tick in range(max(1, total_timesteps // num_envs)):
            timestep = tick * num_envs
            agent.pre_interaction(timestep=timestep, timesteps=total_timesteps)
            with torch.no_grad():
                actions, _ = agent.act(observations, None, timestep=timestep, timesteps=total_timesteps)
                next_observations, rewards, terminated, truncated, infos = wrapped.step(actions)
            agent.record_transition(
                observations=observations, states=None, actions=actions, rewards=rewards,
                next_observations=next_observations, next_states=None,
                terminated=terminated, truncated=truncated, infos=infos,
                timestep=timestep, timesteps=total_timesteps,
            )
            agent.post_interaction(timestep=timestep, timesteps=total_timesteps)
            observations = next_observations

            if (timestep + num_envs) >= next_eval:
                next_eval += eval_freq
                rate = self._evaluate(agent, eval_env)
                history.append((timestep + num_envs, rate))
            pbar.update(num_envs)
        pbar.close()
        return {"eval_history": history, "final_success": history[-1][1] if history else None}

    def _evaluate(self, agent, eval_env) -> float:
        """评估不经 wrapper（原始 gym env），seed 可固定；确定性动作 = mean_actions。"""
        n_ok = 0
        for ep in range(5):
            obs, _ = eval_env.reset(seed=1000 + ep)
            info = {}
            for _ in range(MAX_EP_STEPS):
                obs_t = torch.as_tensor(np.asarray(obs), dtype=torch.float32, device=DEVICE).unsqueeze(0)
                with torch.no_grad():
                    actions, outputs = agent.act(obs_t, None, timestep=0, timesteps=0)
                    if self.algo == "ppo":
                        raw = outputs["mean_actions"]
                    else:  # sac: deterministic policy 输出
                        raw = actions
                    action = torch.clip(raw, -1.0, 1.0).squeeze(0)
                obs, _, terminated, truncated, info = eval_env.step(action.cpu().numpy())
                if terminated or truncated:
                    break
            n_ok += int(info.get("success", False))
        rate = n_ok / 5
        if getattr(agent, "logger", None) is not None or getattr(agent, "write_interval", 0):
            agent.track_data("eval/success_rate", rate)
        return rate

    def save(self, model, ckpt_dir: Path) -> None:
        agent, _ = model
        ckpt_dir = Path(ckpt_dir)
        ckpt_dir.mkdir(parents=True, exist_ok=True)
        agent.save(ckpt_dir / "model.pt")
        # skrl agent.save 追加 .pt 后缀的路径处理：确认实际文件名
        if not (ckpt_dir / "model.pt").exists():
            found = list(ckpt_dir.glob("model*"))
            if found:
                found[0].rename(ckpt_dir / "model.pt")
        write_meta(ckpt_dir, self.backend, self.algo, getattr(self, "_env_id", ""),
                   int(agent.observation_space.shape[0]), False,
                   hparams=self._last_hparams)

    def load_policy(self, ckpt_dir: Path):
        import json as _json

        import so101_envs  # noqa: F401  确保 env registry 已注册

        meta = _json.loads((Path(ckpt_dir) / "meta.json").read_text())
        env = gym.make(meta["env_id"])
        hparams = meta.get("hparams", {})
        agent, _ = self._make_agent(env, seed=0, hparams=hparams)
        agent.load(Path(ckpt_dir) / "model.pt")

        def policy(obs):
            obs = torch.as_tensor(np.asarray(obs), dtype=torch.float32, device=DEVICE).unsqueeze(0)
            with torch.no_grad():
                actions, outputs = agent.act(obs, None, timestep=0, timesteps=0)
                raw = outputs["mean_actions"] if self.algo == "ppo" else actions
                action = torch.clip(raw, -1.0, 1.0)
            return action.squeeze(0).cpu().numpy()

        return policy
