"""脚本专家验收（workflow/plan/envs.md Phase 4）。

pinocchio IK + 状态机驱动 4 个 env 端到端，输出 success rate。
用法:
    uv run python scripts/envs_run_expert.py --env all --episodes 10
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import gymnasium as gym
import numpy as np

import so101_envs  # noqa: F401  注册副作用
from so101_envs.ik import SO101IK

REPO_ROOT = Path(__file__).resolve().parents[1]
URDF = REPO_ROOT / "models" / "so101_new_calib.urdf"
MAX_DQ = 0.12  # 每控制步臂关节目标变化上限 (rad)
# tool-down 抓取: 开口方向(site z)朝下, 绕轴旋转自由
TOOL_DOWN_R = np.column_stack([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
SEED_Q = np.array([-0.06, 0.73, -0.62, 1.29, -1.26])  # tool-down 邻近种子位形
SEED_POOL = [
    SEED_Q,
    np.array([-0.46, -0.25, -0.04, 1.58, 1.59]),
    np.array([0.89, -0.16, 0.41, 1.20, 1.21]),
    np.array([0.04, 0.22, 0.28, 1.02, -0.47]),
]


class Expert:
    def __init__(self, env):
        self.env = env
        self.u = env.unwrapped
        self.ik = SO101IK(URDF)
        self.t = 0

    # ---- 基础 ----
    def _arm_action(self, q_ref_arm):
        """5 臂关节目标 -> 动作（夹爪通道保持当前值）。"""
        lo = self.u._ctrlrange[:, 0]
        hi = self.u._ctrlrange[:, 1]
        q_arm = self.u.data.qpos[self.u._qpos_adr][:5]
        dq = np.clip(q_ref_arm - q_arm, -MAX_DQ, MAX_DQ)
        a = 2.0 * (q_arm + dq - lo[:5]) / (hi[:5] - lo[:5]) - 1.0
        return np.concatenate([a, [0.0]])

    def _grip(self, a, open_):
        lo, hi = self.u._ctrlrange[5]
        g = hi if open_ else lo
        a[5] = 2 * (g - lo) / (hi - lo) - 1
        return a

    def _go(self, target_pos, steps, open_=True, R_des=None, tol=0.006):
        """IK 位置伺服; 到位 (TCP 距目标 < tol) 提前结束。"""
        info = {}
        for _ in range(steps):
            q_arm = self.u.data.qpos[self.u._qpos_adr][:5]
            q_ref, _ = self.ik.solve(q_arm, target_pos, R_des=R_des, n_iter=5)
            a = self._arm_action(q_ref)
            a = self._grip(a, open_=open_)
            *_, info = self.env.step(a)
            self.t += 1
            if np.linalg.norm(self.u.tcp_pos - target_pos) < tol:
                break
        return info

    def _wait(self, steps, grip_open=True):
        a = self.u.normalize_qpos(self.u.data.qpos[self.u._qpos_adr])
        a = self._grip(a, open_=grip_open)
        info = {}
        for _ in range(steps):
            *_, info = self.env.step(a)
            self.t += 1
        return info


def run_reach(env, episodes):
    results = []
    for ep in range(episodes):
        ex = Expert(env)
        env.reset(seed=100 + ep)
        ex._wait(10)
        goal = ex.u._goal.copy()
        ex._go(goal, steps=40)
        info = ex._wait(8)
        results.append(info["success"])
    return results


def tool_down_R(from_pos, to_pos):
    """抓取姿态: site x=爪伸方向(水平指向物体), site z=开口方向(朝下)。"""
    x = np.asarray(to_pos[:2]) - np.asarray(from_pos[:2])
    x = np.concatenate([x, [0.0]])
    n = np.linalg.norm(x)
    x = x / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])
    z = np.array([0.0, 0.0, -1.0])
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def walk_to_q(ex, q_ref, max_steps=80, tol=0.02):
    """Rate-limited 直达位形轨迹（不经 IK, 用于跨越位形空间大段）。"""
    info = {}
    for _ in range(max_steps):
        lo = ex.u._ctrlrange[:, 0]
        hi = ex.u._ctrlrange[:, 1]
        q = ex.u.data.qpos[ex.u._qpos_adr][:5]
        dq = np.clip(q_ref - q, -MAX_DQ, MAX_DQ)
        a = 2.0 * (q + dq - lo[:5]) / (hi[:5] - lo[:5]) - 1.0
        a = np.concatenate([a, [ex._grip(np.zeros(6), open_=True)[5]]])
        *_, info = ex.env.step(a)
        ex.t += 1
        if np.linalg.norm(q_ref - ex.u.data.qpos[ex.u._qpos_adr][:5]) < tol:
            break
    return info


def grasp_item(ex, item, pre_h=0.06):
    """tool-down 抓取流程: 多种子 IK 预解 -> 走到位 -> 下探 -> 闭爪。返回 is_grasped。"""
    target = item + [0, 0, pre_h]
    best_q, best_err = None, np.inf
    for seed in SEED_POOL:
        q_ref, err = ex.ik.solve(seed, target, R_des=TOOL_DOWN_R, n_iter=300)
        if err < best_err:
            best_q, best_err = q_ref, err
        if err < 0.005:
            break
    walk_to_q(ex, best_q)
    ex._go(item + [0, 0, pre_h], steps=20, R_des=TOOL_DOWN_R)
    ex._go(item + [0, 0, 0.0], steps=25, R_des=TOOL_DOWN_R, tol=0.004)
    return _close_steps(ex)


def _close_steps(ex, n=30, rate=0.05):
    grasped = False
    for _ in range(n):
        a = ex.u.normalize_qpos(ex.u.data.qpos[ex.u._qpos_adr])
        a = ex._grip(a, open_=False)
        a[5] = np.clip(a[5] + rate, -1.0, 1.0)
        *_, info = ex.env.step(a)
        grasped = grasped or info["is_grasped"]
    return grasped


def run_lift(env, episodes):
    results = []
    for ep in range(episodes):
        ex = Expert(env)
        env.reset(seed=100 + ep)
        ex._wait(10)
        item = ex.u.free_body_pos("item")
        grasped = grasp_item(ex, item)
        ex._go(item + [0, 0, 0.09], steps=15)  # 抬起
        info = ex._wait(8)
        results.append(bool(grasped and info["item_lifted"]))
    return results


def run_place(env, episodes):
    results = []
    for ep in range(episodes):
        ex = Expert(env)
        env.reset(seed=100 + ep)
        ex._wait(10)
        item = ex.u.free_body_pos("item")
        ex._go(item + [0, 0, 0.06], steps=25)
        ex._go(item + [0, 0, 0.005], steps=18)
        for _ in range(30):
            a = ex.u.normalize_qpos(ex.u.data.qpos[ex.u._qpos_adr])
            a = ex._grip(a, open_=False)
            a[5] = np.clip(a[5] + 0.05, -1.0, 1.0)
            *_, info = env.step(a)
        ex._go(item + [0, 0, 0.10], steps=15)           # 抓起提升
        binp = ex.u.free_body_pos("bin")
        ex._go(binp + [0, 0, 0.12], steps=30)           # 移到 bin 上方
        ex._go(binp + [0, 0, 0.05], steps=15)           # 降入 bin
        info = ex._wait(12, grip_open=True)             # 松爪
        results.append(info["success"])
    return results


def run_stack(env, episodes):
    results = []
    for ep in range(episodes):
        ex = Expert(env)
        env.reset(seed=100 + ep)
        ex._wait(10)
        a_pos = ex.u.free_body_pos("itemA")
        ex._go(a_pos + [0, 0, 0.06], steps=25)
        ex._go(a_pos + [0, 0, 0.005], steps=18)
        for _ in range(30):
            a = ex.u.normalize_qpos(ex.u.data.qpos[ex.u._qpos_adr])
            a = ex._grip(a, open_=False)
            a[5] = np.clip(a[5] + 0.05, -1.0, 1.0)
            *_, info = env.step(a)
        b_pos = ex.u.free_body_pos("itemB")
        ex._go(a_pos + [0, 0, 0.10], steps=15)           # 提升
        ex._go(b_pos + [0, 0, 0.11], steps=30)           # B 上方
        ex._go(b_pos + [0, 0, 0.022 + 0.028 + 0.004], steps=15)  # 降到 B 顶
        info = ex._wait(12, grip_open=True)              # 松爪
        results.append(info["success"])
    return results


POLICIES = {
    "SO101Reach-v1": run_reach,
    "SO101Lift-v1": run_lift,
    "SO101Place-v1": run_place,
    "SO101Stack-v1": run_stack,
}


def main():
    parser = argparse.ArgumentParser(description="脚本专家验收")
    parser.add_argument("--env", default="all", choices=["all", *POLICIES])
    parser.add_argument("--episodes", type=int, default=10)
    args = parser.parse_args()
    ids = list(POLICIES) if args.env == "all" else [args.env]
    print(f"{'env':<18} {'success':>8}  detail")
    all_ok = True
    for env_id in ids:
        env = gym.make(env_id)
        try:
            results = POLICIES[env_id](env, args.episodes)
            n_ok = sum(results)
            rate = n_ok / len(results)
            # reach 验收线 >= 0.9; 其余 > 0 且稳定
            print(f"{env_id:<18} {rate:>7.0%}  {n_ok}/{len(results)}")
            if env_id.endswith("Reach-v1"):
                all_ok &= rate >= 0.9
            else:
                all_ok &= rate > 0.0
        finally:
            env.close()
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
