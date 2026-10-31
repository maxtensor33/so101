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

    def _go(self, target_pos, steps, open_=True, R_des=None, tol=0.006,
            position_only=False):
        """IK 位置伺服; 到位 (TCP 距目标 < tol) 提前结束。"""
        info = {}
        for _ in range(steps):
            q_arm = self.u.data.qpos[self.u._qpos_adr][:5]
            q_ref, _ = self.ik.solve(
                q_arm, target_pos, R_des=R_des, n_iter=40,
                q_gripper=self.u.data.qpos[self.u._qpos_adr[5]],
                position_only=position_only)
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
        # squint reach: success 要求闭爪 + TCP=指尖中点。闭爪会移动中点,
        # 因此必须全程闭爪到达 goal（开爪到位再闭爪会把 TCP 移走）。
        ex._wait(10, grip_open=False)
        goal = ex.u._goal.copy()
        ex._go(goal, steps=60, open_=False, position_only=True)
        info = ex._wait(8, grip_open=False)
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


def grasp_item(ex, item, grasp_dz=-0.01, close_fn=None):
    """tool-down 抓取流程: 多种子 IK 预解 -> 走到位 -> 小步逼近 -> 闭爪。返回 is_grasped。

    TCP 目标 = 方块中心下方 |grasp_dz| (-0.01: 指底离桌 1cm 不拖桌, 捏方块上部 2/3)。原因: 本臂 5-DOF
    tool-down 在高目标处不可达 (item 中心及以上 err 17mm~40cm 级, 低目标 0mm),
    误差曲线见 workflow/todo/envs_grasp_tuning_todo.md; 旧版抓中线导致指尖只
    捏到方块上半缘, 抬升即滑落 (envs 已知"抓取保持失败"的真因之一)。
    种子池加 neutral (实测优于部分随机种子, DLS 局部极小问题)。
    """
    target = item + [0, 0, grasp_dz]
    # 停机坪: 方块后方 10cm 同高度 — walk_to_q 的关节空间扫掠会推飞方块
    # (2026-10-29 渲染实测 item 被扫出 5cm), 先走到停机坪再任务空间直线推近
    xy_dir = item[:2] / np.linalg.norm(item[:2])
    staging = target.copy()
    staging[:2] -= xy_dir * 0.10
    best_q, best_err = None, np.inf
    for seed in [np.zeros(5), *SEED_POOL]:
        q_ref, err = ex.ik.solve(
            seed, staging, R_des=TOOL_DOWN_R, n_iter=300,
            q_gripper=ex.u.data.qpos[ex.u._qpos_adr[5]])
        if err < best_err:
            best_q, best_err = q_ref, err
        if err < 0.005:
            break
    walk_to_q(ex, best_q)
    ex._go(staging, steps=20, R_des=TOOL_DOWN_R, tol=0.005)
    # 小步插值逼近 (2026-10-30): 停机坪->目标一次 _go 直线 10cm 会触发 warm-start
    # IK 局部滞点 (TCP 跳到 20cm+ 外不返回, 2026-10-30 逐阶段实测), 改 1.5cm 路点;
    # 收尾自适应爬行兜底 (水平 <5mm)
    start = ex.u.tcp_pos.copy()
    n = max(1, int(np.linalg.norm(target - start) / 0.015))
    for i in range(1, n + 1):
        ex._go(start + (target - start) * i / n, steps=20, R_des=TOOL_DOWN_R, tol=0.004)
    for _ in range(6):
        ex._go(target, steps=20, R_des=TOOL_DOWN_R, tol=0.003)
        if np.linalg.norm(ex.u.tcp_pos[:2] - target[:2]) < 0.005:
            break
    return close_fn(ex, item, grasp_dz=grasp_dz) if close_fn is not None else _close_steps(ex)


def _close_steps(ex, n=30, rate=0.05):
    grasped = False
    for _ in range(n):
        a = ex.u.normalize_qpos(ex.u.data.qpos[ex.u._qpos_adr])
        a = ex._grip(a, open_=False)
        a[5] = np.clip(a[5] + rate, -1.0, 1.0)
        *_, info = ex.env.step(a)
        grasped = grasped or info["is_grasped"]
    return grasped


def _servo_step(ex, env, target_pos, R_des=TOOL_DOWN_R):
    """单帧: 手臂向目标 IK 伺服一步, 夹爪通道保持当前值。返回 info。"""
    u = ex.u
    q_arm = u.data.qpos[u._qpos_adr][:5]
    q_ref, _ = ex.ik.solve(
        q_arm, target_pos, R_des=R_des, n_iter=40,
        q_gripper=u.data.qpos[u._qpos_adr[5]])
    lo, hi = u._ctrlrange[:5, 0], u._ctrlrange[:5, 1]
    dq = np.clip(q_ref - q_arm, -MAX_DQ, MAX_DQ)
    a = 2.0 * (q_arm + dq - lo) / (hi - lo) - 1.0
    a5 = u.normalize_qpos(u.data.qpos[u._qpos_adr])[5]
    *_, info = env.step(np.concatenate([a, [a5]]))
    ex.t += 1
    return info


def servo_close(ex, item, grasp_dz=-0.01, frames=60, rate=0.035, hold=12):
    """伺服闭爪: 手臂跟随 + 夹爪小步闭合交替 (2026-10-30)。

    纯合爪 (_close_steps, 手臂定住) 下, 全开动爪的大角度闭合弧线以切向把方块
    沿桌面推走 (ep2 逐帧实测: 4 帧内被推 4.6cm 后爪闭空); 手臂跟随使抓取
    ~7/10 vs ~3/10。抓住后保持 hold 帧稳定。未根治, 见 todo/envs_grasp_tuning_todo.md。
    """
    env = ex.env
    target = item + [0, 0, grasp_dz]
    grasped = False
    for _ in range(frames):
        info = _servo_step(ex, env, target)
        grasped = grasped or info["is_grasped"]
        a = ex.u.normalize_qpos(ex.u.data.qpos[ex.u._qpos_adr])
        a[5] = max(a[5] - rate, -1.0)
        *_, info = env.step(a)
        ex.t += 1
        grasped = grasped or info["is_grasped"]
        if info["is_grasped"]:
            for _ in range(hold):
                a = ex.u.normalize_qpos(ex.u.data.qpos[ex.u._qpos_adr])
                a = ex._grip(a, open_=False)
                *_, info = env.step(a)
                ex.t += 1
            break
    return grasped


def lift_servo(ex, item, height=0.10, frames=90):
    """缓升伺服抬升: 爪角保持当前值 (open_=False 全闭合命令会把边缘态抓取挤掉),
    TCP 以 ~height/frames 每帧上升, 手臂每帧重解 IK。"""
    u = ex.u
    info = {}
    for i in range(frames):
        info = _servo_step(ex, ex.env, item + [0, 0, height * (i + 1) / frames])
    return info


def run_lift(env, episodes):
    results = []
    for ep in range(episodes):
        ex = Expert(env)
        env.reset(seed=100 + ep)
        ex._wait(10)
        item = ex.u.free_body_pos("item")
        grasped = grasp_item(ex, item, close_fn=servo_close)
        if grasped:
            info = lift_servo(ex, item)
            results.append(bool(info["item_lifted"]))
        else:
            results.append(False)
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
        ex._go(item + [0, 0, 0.10], steps=15, open_=False)           # 抓起提升 (保持闭合!)
        binp = ex.u.free_body_pos("bin")
        ex._go(binp + [0, 0, 0.12], steps=30, open_=False)           # 移到 bin 上方
        ex._go(binp + [0, 0, 0.05], steps=15, open_=False)           # 降入 bin
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
        ex._go(a_pos + [0, 0, 0.10], steps=15, open_=False)           # 提升 (保持闭合!)
        ex._go(b_pos + [0, 0, 0.11], steps=30, open_=False)           # B 上方
        ex._go(b_pos + [0, 0, ex.u._item_a_half_z + ex.u._item_b_half_z + 0.004],
               steps=15, open_=False)  # 降到 B 顶
        info = ex._wait(12, grip_open=True)              # 松爪
        results.append(info["success"])
    return results


# squint 迁移后仅保留 reach 脚本专家（lift/place/stack 专家已失效/暂停,
# 见 workflow/reports/envs_squint_migration.md）
POLICIES = {
    "SO101Reach-v1": run_reach,
    "SO101ReachCan-v1": run_reach,
}


def main():
    parser = argparse.ArgumentParser(description="脚本专家验收")
    parser.add_argument("--env", default="all", choices=["all", *POLICIES])
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--view", action="store_true",
                        help="MuJoCo viewer 可视化单 episode (需图形环境; 弹 viewer 窗口实时观看专家操作)")
    args = parser.parse_args()
    if args.view:
        env_id = args.env if args.env != "all" else "SO101Reach-v1"
        env = gym.make(env_id, render_mode="human")
        try:
            results = POLICIES[env_id](env, 1)
            print(f"{env_id} view episode: success={results[0]}")
        finally:
            env.close()
        return 0
    ids = list(POLICIES) if args.env == "all" else [args.env]
    print(f"{'env':<22} {'success':>8}  detail")
    all_ok = True
    for env_id in ids:
        env = gym.make(env_id)
        try:
            results = POLICIES[env_id](env, args.episodes)
            n_ok = sum(results)
            rate = n_ok / len(results)
            # reach 验收线 >= 0.9（仅保留 reach 专家）
            print(f"{env_id:<22} {rate:>7.0%}  {n_ok}/{len(results)}")
            all_ok &= rate >= 0.9
        finally:
            env.close()
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
