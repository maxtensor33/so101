"""Image-based DAgger（workflow/plan/rl_dagger.md Phase 1/2）。

方法（适配自 s2v-dagger, AAAI 2025）: student 纯视觉 CNN 确定性 rollout,
expert 用 state 通道事后批量标注, BC mse 蒸馏, 数据按 collect 轮环形混合。

不走 TrainerSpec/sb3/skrl（BC 不属这两后端）: checkpoint = model.pt + meta.json,
meta.json backend="dagger", env_kwargs 随存（推理侧靠它重建同配置 env）。
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from tqdm.auto import tqdm

from .backends.base import read_meta, write_meta

# 内置默认超参（configs/dagger.yaml 覆盖后再由 CLI overrides 覆盖, 沿 algos.py 惯例）
DEFAULT_HP: dict = {
    "camera_keys": ["fixed"],
    "collect_size": 1600,       # 每轮 vec 步数（expert 标注轮的大小）
    "num_updates": 200,         # 每轮 BC 梯度更新次数上限
    "minibatch": 256,
    "lr": 3e-4,
    "bc_loss_th": 1e-3,         # bc_loss < 阈值早停提前转下一轮采集（参考实现核心 trick）
    "learning_starts": 1600,    # 累计过渡数达到后才开训
    "buffer_size": 100_000,     # 过渡容量; 轮数 = buffer_size // collect_size
    "total_rounds": 50,
    "seed": 0,
    "render_size": [128, 128],  # 与 env RENDER_SIZE 一致（apply_camera_calib 只覆写
                                # cam_pos/quat/fovy, 与分辨率无关, 标定保持默认开启）
    "n_envs": 4,
    "progress": True,           # tqdm 进度条（轮级; postfix 实时显示 steps/bc_loss/updates）
}

_ACTION_DIM = 6


# ---------- StudentPolicy ----------


class StudentPolicy(nn.Module):
    """纯视觉确定性策略: N 相机图像 -> PlainConv 风格 encoder -> tanh action(6)。

    encoder（参考 s2v PlainConv）: 每张图 5 组 Conv3x3+MaxPool2d(2)（通道
    16->16->32->64->128）+ 128 通道 1x1 conv, flatten 后 Linear->256 维特征。
    多相机语义（二选一取"特征拼接"）: 每张图独立过**同一个** encoder 得到各自
    256 维特征, 按 camera_keys 顺序拼接后经共享 head Linear(256*N->256) 输出——
    相比通道数 concat, 共享 encoder 参数不随相机数增长, 且天然支持训练/推理
    相机数不同的扩展（此处仍要求一致）。
    state 通道不进网络（仅训练时给 expert 标注用）。
    """

    def __init__(self, n_cameras: int = 1, action_dim: int = _ACTION_DIM,
                 in_hw: tuple[int, int] | None = None):
        super().__init__()
        if n_cameras < 1:
            raise ValueError(f"n_cameras must be >= 1, got {n_cameras}")
        self.n_cameras = n_cameras
        self.action_dim = action_dim
        # 缺省与 env RENDER_SIZE (128, 128) 一致——图像尺寸必须和 envs 渲染尺寸相同
        self.in_hw = tuple(in_hw) if in_hw is not None else (128, 128)

        def block(cin: int, cout: int) -> list[nn.Module]:
            return [nn.Conv2d(cin, cout, kernel_size=3, padding=1), nn.ReLU(),
                    nn.MaxPool2d(2)]

        self.encoder = nn.Sequential(
            *block(3, 16), *block(16, 16), *block(16, 32),
            *block(32, 64), *block(64, 128),
            nn.Conv2d(128, 128, kernel_size=1), nn.ReLU(),
        )
        # 用 dummy 前向求 flatten 维度（不依赖 32 整除的手工推导）
        with torch.no_grad():
            feat_dim = int(self.encoder(torch.zeros(1, 3, *self.in_hw)).numel())
        self.feature = nn.Linear(feat_dim, 256)
        self.head = nn.Sequential(
            nn.Linear(256 * n_cameras, 256), nn.ReLU(),
            nn.Linear(256, action_dim), nn.Tanh(),
        )

    def forward(self, images: torch.Tensor) -> torch.Tensor:
        """images: (B, N, 3, H, W) float（已 /255）; 返回 (B, action_dim) in [-1,1]。"""
        b, n = images.shape[0], images.shape[1]
        if n != self.n_cameras:
            raise ValueError(f"expected {self.n_cameras} cameras, got {n}")
        feats = self.encoder(images.reshape(b * n, *images.shape[2:]))
        feats = self.feature(feats.flatten(1)).reshape(b, n * 256)
        return self.head(feats)


def stack_camera_images(rgb_batch: dict[str, np.ndarray],
                        camera_keys: Sequence[str]) -> np.ndarray:
    """Dict obs 的 rgb 批 -> 网络输入布局。

    rgb_batch: 每键 (B, H, W, 3) uint8; 返回 (B, N, H, W, 3) uint8,
    camera_keys 顺序即网络输入顺序（训练/推理必须一致, 存于 meta hparams）。
    """
    return np.stack([np.asarray(rgb_batch[k]) for k in camera_keys], axis=1)


def images_to_tensor(stacked: np.ndarray, device: str = "cpu") -> torch.Tensor:
    """(B, N, H, W, 3) uint8 -> (B, N, 3, H, W) float32 /255（训练/推理共用预处理）。"""
    return torch.from_numpy(np.ascontiguousarray(stacked)).permute(0, 1, 4, 2, 3).to(
        device).float() / 255.0


# ---------- DAggerBuffer ----------


class DAggerBuffer:
    """按 collect 轮组织的环形 buffer（经典 DAgger agg 模式）。

    容量 = max(1, buffer_size // collect_size) 轮; 写满后从头覆盖最旧轮——
    新旧数据均匀混合。采样 = 均匀随机轮 + 轮内均匀随机过渡（无 priority）。
    rgb 以 uint8 存 CPU（省内存）; 训练 batch 取出时才转 float32（见 images_to_tensor）。
    每轮过渡数 = 实际采集数（n_envs * collect_size 展平）, 轮间可不等。
    """

    def __init__(self, collect_size: int, buffer_size: int):
        if collect_size < 1 or buffer_size < 1:
            raise ValueError("collect_size/buffer_size must be >= 1")
        self.collect_size = collect_size
        self.capacity_rounds = max(1, buffer_size // collect_size)
        self.rounds: list[dict] = []   # 每轮 {"state": (T,S) f32, "rgb": {k: (T,H,W,3) u8}, "actions": (T,6) f32}
        self._next = 0                 # 下一轮写入位置（环形指针）

    @property
    def num_rounds(self) -> int:
        return len(self.rounds)

    def __len__(self) -> int:
        return int(sum(r["state"].shape[0] for r in self.rounds))

    def append_round(self, obs: dict, actions: np.ndarray) -> None:
        """obs: {"state": (T,S) float32, "rgb": {k: (T,H,W,3) uint8}};
        actions: (T, 6) float32（expert 标注）。"""
        state = np.asarray(obs["state"], dtype=np.float32)
        actions = np.asarray(actions, dtype=np.float32)
        if actions.shape != (state.shape[0], _ACTION_DIM):
            raise ValueError(
                f"actions shape {actions.shape} != ({state.shape[0]}, {_ACTION_DIM})")
        rgb = {k: np.asarray(v, dtype=np.uint8) for k, v in obs["rgb"].items()}
        entry = {"state": state, "rgb": rgb, "actions": actions}
        if len(self.rounds) < self.capacity_rounds:
            self.rounds.append(entry)
        else:
            self.rounds[self._next] = entry
        self._next = (self._next + 1) % self.capacity_rounds

    def sample(self, batch_size: int, rng: np.random.Generator) -> dict:
        """均匀随机轮 + 轮内均匀随机过渡; rgb 保持 uint8, state/actions float32。"""
        if not self.rounds:
            raise RuntimeError("sample from empty DAggerBuffer")
        counts = [r["state"].shape[0] for r in self.rounds]
        # 按轮过渡数加权均匀采样（等价于均匀随机轮 + 轮内均匀过渡的合并近似,
        # 轮大小相当时与两段采样一致; 实现简单且无放回偏差可控）
        round_idx = rng.choice(len(self.rounds), size=batch_size, p=np.asarray(
            counts) / sum(counts))
        step_idx = [rng.integers(counts[i]) for i in round_idx]

        states, actions, rgb_rows = [], [], {k: [] for k in self.rounds[0]["rgb"]}
        for ri, si in zip(round_idx, step_idx):
            r = self.rounds[ri]
            states.append(r["state"][si])
            actions.append(r["actions"][si])
            for k in rgb_rows:
                rgb_rows[k].append(r["rgb"][k][si])
        return {
            "state": np.stack(states),
            "rgb": {k: np.stack(v) for k, v in rgb_rows.items()},
            "actions": np.stack(actions),
        }


# ---------- 超参加载（内置默认 < configs/dagger.yaml < overrides） ----------


def load_dagger_hparams(overrides: dict | None = None,
                        configs_dir: Path | None = None) -> dict:
    """简化版 algos.load_defaults 惯例; 返回新 dict, 不改内置默认。"""
    params = dict(DEFAULT_HP)
    configs_dir = Path(configs_dir) if configs_dir is not None else (
        Path(__file__).resolve().parent / "configs")
    yaml_path = configs_dir / "dagger.yaml"
    if yaml_path.exists():
        import yaml

        params.update(yaml.safe_load(yaml_path.read_text()) or {})
    if overrides:
        params.update(overrides)
    return params


# ---------- checkpoint ----------


def save_student_policy(policy: StudentPolicy, ckpt_dir: Path, env_id: str,
                        env_kwargs: dict | None = None,
                        hparams: dict | None = None,
                        obs_dim: int | None = None) -> None:
    """model.pt（state_dict）+ meta.json（backend="dagger", env_kwargs 随存）。"""
    ckpt_dir = Path(ckpt_dir)
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    torch.save(policy.state_dict(), ckpt_dir / "model.pt")
    write_meta(ckpt_dir, backend="dagger", algo="bc", env_id=env_id,
               obs_dim=obs_dim if obs_dim is not None else 0, normalize=False,
               hparams={
                   "camera_keys": list(hparams.get("camera_keys", ["fixed"])) if hparams
                   else ["fixed"],
                   "n_cameras": policy.n_cameras,
                   "in_hw": list(policy.in_hw),
                   **(hparams or {}),
               },
               env_kwargs=env_kwargs)


def load_student_policy(ckpt_dir: Path, device: str = "cpu") -> StudentPolicy:
    """读 meta.json 重建网络（相机数/分辨率来自 hparams）+ load state_dict。"""
    meta = read_meta(ckpt_dir)
    if meta.get("backend") != "dagger":
        raise ValueError(f"not a dagger checkpoint: backend={meta.get('backend')!r}")
    hp = meta.get("hparams", {})
    policy = StudentPolicy(n_cameras=int(hp.get("n_cameras",
                                                len(hp.get("camera_keys", ["fixed"])))),
                           action_dim=int(meta.get("action_dim", _ACTION_DIM)),
                           in_hw=tuple(hp.get("in_hw", (128, 128))))
    state = torch.load(Path(ckpt_dir) / "model.pt", map_location=device)
    policy.load_state_dict(state)
    policy.to(device)
    policy.eval()
    return policy


# ---------- 训练循环 ----------


def train_dagger(expert: Callable[[np.ndarray], np.ndarray], vec_env,
                 hparams: dict, *, out_dir: Path | None = None, env_id: str = "",
                 env_kwargs: dict | None = None, student: StudentPolicy | None = None,
                 device: str = "cpu",
                 log_fn: Callable[[str], None] | None = None) -> dict:
    """两阶段 DAgger 训练（不用 reward）。

    expert 契约: 黑盒可调用, 单条 obs state np.ndarray(S,) -> action np.ndarray(6,)
    in [-1,1]（逐条调用, 与后端无关——Phase 3 用真 expert 走 load_policy 链）。
    vec_env: gymnasium 向量化 env（gym.make_vec sync）或单 env, obs 为
    Dict{"state", "rgb": {key: (H,W,3) uint8}}（向量时各键首维批量）。

    每轮: student 确定性 rollout collect_size 步 -> 展平观测 -> expert 逐条标注
    -> 入 buffer -> （learning_starts 后）至多 num_updates 次 BC mse 更新,
    bc_loss < bc_loss_th 早停提前转下一轮采集。
    """
    log = log_fn or (lambda _: None)
    hp = {**DEFAULT_HP, **(hparams or {})}
    camera_keys: list[str] = list(hp["camera_keys"])
    collect_size = int(hp["collect_size"])
    seed = int(hp["seed"])
    rng = np.random.default_rng(seed)
    torch.manual_seed(seed)

    if student is None:
        # 从一次观测推图像分辨率（env 实际渲染 HxW, 与 hp["render_size"] 一致时同值）
        probe_obs, _ = vec_env.reset(seed=seed)
        sample_img = np.asarray(probe_obs["rgb"][camera_keys[0]])
        single = sample_img[0] if sample_img.ndim == 4 else sample_img
        in_hw = (int(single.shape[0]), int(single.shape[1]))
        vec_env.reset(seed=seed)  # 重置采集起点, 保证 seed 复现语义
        student = StudentPolicy(n_cameras=len(camera_keys), in_hw=in_hw)
    student.to(device)
    opt = torch.optim.Adam(student.parameters(), lr=float(hp["lr"]))
    buffer = DAggerBuffer(collect_size=collect_size, buffer_size=int(hp["buffer_size"]))

    learning_starts = int(hp["learning_starts"])
    minibatch = int(hp["minibatch"])
    num_updates = int(hp["num_updates"])
    bc_loss_th = float(hp["bc_loss_th"])

    obs, _ = vec_env.reset(seed=seed)
    total_steps = 0
    bc_losses: list[float] = []
    rounds_done = 0

    # --- 进度条: 轮级 tqdm, 日志经 bar.write 避免打断进度条; progress=False 全静默 ---
    bar = None
    if bool(hp.get("progress", True)):
        bar = tqdm(total=int(hp["total_rounds"]), desc="dagger", unit="round",
                   dynamic_ncols=True)
        log = (lambda msg: bar.write(msg))

    for _round in range(int(hp["total_rounds"])):
        # --- 采集: student 确定性 rollout（不用 reward, 不管 done 的 episode 边界,
        # 到 max_episode_steps 由 env 自动 reset, 过渡照收——与参考实现一致）---
        states, rgbs = [], {k: [] for k in camera_keys}
        for _ in range(collect_size):
            rgb_batch = {k: np.asarray(obs["rgb"][k]) for k in camera_keys}
            state_batch = np.asarray(obs["state"], dtype=np.float32)
            stacked = stack_camera_images(rgb_batch, camera_keys)
            imgs = images_to_tensor(stacked, device)
            with torch.no_grad():
                action = student(imgs).cpu().numpy()
            states.append(state_batch)
            for k in camera_keys:
                rgbs[k].append(rgb_batch[k])
            obs, *_ = vec_env.step(np.clip(action, -1.0, 1.0))
        total_steps += collect_size * int(np.asarray(states[0]).shape[0])

        # --- 展平 + expert 逐条标注 ---
        flat_state = np.concatenate(states, axis=0)
        flat_rgb = {k: np.concatenate(rgbs[k], axis=0) for k in camera_keys}
        expert_actions = np.stack(
            [np.asarray(expert(s), dtype=np.float32).reshape(-1) for s in flat_state])
        buffer.append_round({"state": flat_state, "rgb": flat_rgb}, expert_actions)
        rounds_done += 1

        # --- BC 更新（learning_starts 后; bc_loss 早停提前转下一轮）---
        if total_steps < learning_starts or len(buffer) < minibatch:
            log(f"[dagger] round {rounds_done}: collected {total_steps} steps "
                f"(warmup)")
            if bar is not None:
                bar.update(1)
            continue
        round_losses: list[float] = []
        for _ in range(num_updates):
            batch = buffer.sample(minibatch, rng)
            stacked = stack_camera_images(batch["rgb"], camera_keys)
            imgs = images_to_tensor(stacked, device)
            target = torch.from_numpy(batch["actions"]).to(device)
            loss = torch.nn.functional.mse_loss(student(imgs), target)
            opt.zero_grad()
            loss.backward()
            opt.step()
            value = float(loss.item())
            bc_losses.append(value)
            round_losses.append(value)
            if bar is not None:
                bar.set_postfix(steps=total_steps, bc_loss=f"{value:.4f}",
                                updates=f"{len(round_losses)}/{num_updates}")
            if value < bc_loss_th:
                break
        log(f"[dagger] round {rounds_done}: steps={total_steps} "
            f"bc_loss={np.mean(round_losses):.6f} ({len(round_losses)}/{num_updates} "
            f"updates)")
        if bar is not None:
            bar.update(1)

    if bar is not None:
        bar.close()

    info: dict = {"bc_losses": bc_losses, "rounds": rounds_done,
                  "steps": total_steps, "out_dir": None}
    if out_dir is not None:
        save_student_policy(student, Path(out_dir), env_id=env_id,
                            env_kwargs=env_kwargs, hparams=hp,
                            obs_dim=int(buffer.rounds[0]["state"].shape[1])
                            if buffer.rounds else 0)
        info["out_dir"] = str(Path(out_dir))
    return info
