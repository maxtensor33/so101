"""Trajectory 模块 (plan/robotics.md Phase 3, 对标 lab-2 B1)。"""

import numpy as np


def quintic(q0, q1, T, t):
    """五次多项式点到位 (零速零加速度边界)。返回 (q, dq, ddq)。"""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    s = t / T
    s1 = 1 - s
    p = 10*s**3 - 15*s**4 + 6*s**5
    pd = 30*s**2 - 60*s**3 + 30*s**4
    pdd = 60*s - 180*s**2 + 120*s**3
    q = q0 + p * (q1 - q0)
    dq = pd * (q1 - q0) / T
    ddq = pdd * (q1 - q0) / T**2
    return q, dq, ddq


def min_jerk(q0, q1, T, t):
    """minimum-jerk 轨迹 (五次多项式同形)。"""
    return quintic(q0, q1, T, t)


def trapezoidal_duration(dq_total, v_max, a_max):
    """梯形速度规划总时长 (每关节独立, 取最长; 短行程自动退化为三角形)。"""
    a = np.abs(dq_total)
    t_acc_full = v_max / a_max
    t_acc = np.minimum(t_acc_full, np.sqrt(a / a_max))
    v_peak = np.minimum(v_max, np.sqrt(a * a_max))
    t_flat = np.maximum((a - v_peak * t_acc) / v_peak, 0.0)
    return float(np.max(2 * t_acc + t_flat))


def trapezoidal_vel(dq_total, v_max, a_max, t):
    """梯形速度曲线在时刻 t 的关节速度 (每关节独立; 短行程退化三角形)。"""
    dq_total = np.asarray(dq_total, dtype=float)
    sgn = np.sign(dq_total)
    a = np.abs(dq_total)
    t_acc_full = v_max / a_max
    t_acc = np.minimum(t_acc_full, np.sqrt(a / a_max))
    v_peak = np.minimum(v_max, np.sqrt(a * a_max))
    t_flat = np.maximum((a - v_peak * t_acc) / v_peak, 0.0)
    T = 2 * t_acc + t_flat  # 加速 + 匀速 + 减速
    t_eff = np.minimum(t, T)
    v = np.where(
        t_eff < t_acc,
        v_peak / t_acc * t_eff,
        np.where(
            t_eff < t_acc + t_flat,
            v_peak,
            np.maximum(v_peak - v_peak / t_acc * (t_eff - t_acc - t_flat), 0.0),
        ),
    )
    return sgn * v


class JointTrajectory:
    """多段 quintic 拼接的关节轨迹 (相邻段之间零速停顿)。"""

    def __init__(self, waypoints, durations):
        self.waypoints = [np.asarray(w, dtype=float) for w in waypoints]
        self.durations = list(durations)
        self.duration = float(sum(self.durations))

    def sample(self, t):
        """返回 (q, dq, ddq)。t 超界抛 ValueError。"""
        if t < 0.0 or t > self.duration:
            raise ValueError(f"t={t} 超出轨迹时长 [0, {self.duration}]")
        acc = 0.0
        for w0, w1, T in zip(self.waypoints[:-1], self.waypoints[1:], self.durations):
            if t <= acc + T or w1 is self.waypoints[-1]:
                return quintic(w0, w1, T, t - acc)
            acc += T
        return quintic(self.waypoints[-2], self.waypoints[-1], self.durations[-1], self.duration - acc)