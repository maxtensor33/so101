"""stack 任务: 抓起 itemA 堆到 itemB 上（squint Stack 镜像, mujoco 实现）。"""

from __future__ import annotations

import mujoco
import numpy as np

from .base_env import SO101BaseEnv
from .utils import is_body_static, is_grasping, is_static

ITEM_A = "itemA"
ITEM_B = "itemB"
XY_TOL = 0.03      # A/B 中心 xy 距离容差
Z_TOL = 0.008      # A 高度容差


class SO101StackEnv(SO101BaseEnv):
    SCENE_XML = "scene_stack.xml"
    TASK_OBS_DIM = 17  # itemA_pose(7)+tcp_to_A(3)+B_pos(3)+A_to_B_xy(2)+grasped(1)+on_top(1)
    HALF_A = 0.022     # itemA 半高（永远是 cube, squint 语义）
    HALF_B = 0.028     # itemB 半高（can 子类覆盖为罐半高）

    def _reset_task(self, rng):
        x = 0.25 + rng.uniform(-0.08, 0.08)
        y = 0.0 + rng.uniform(-0.08, 0.08)
        self.set_free_body_pose(ITEM_A, (x, y, self.HALF_A), yaw=rng.uniform(0, 2 * np.pi))
        self._randomize_item("itemA_geom", rng)
        for _ in range(50):
            bx = 0.25 + rng.uniform(-0.08, 0.08)
            by = 0.12 + rng.uniform(-0.05, 0.05)
            if np.hypot(bx - x, by - y) >= 0.12:
                break
        self.set_free_body_pose(ITEM_B, (bx, by, self.HALF_B), yaw=rng.uniform(0, 2 * np.pi))
        self._randomize_item("itemB_geom", rng)
        mujoco.mj_forward(self.model, self.data)

    def _task_obs(self):
        qadr = self.model.jnt_qposadr[self.model.body_jntadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, ITEM_A)]]
        a_pose = self.data.qpos[qadr:qadr + 7]
        a = a_pose[:3]
        b = self.free_body_pos(ITEM_B)
        grasped = float(is_grasping(self.model, self.data, ITEM_A))
        on_top = float(self._on_top(a, b))
        return np.concatenate([a_pose, a - self.tcp_pos, b,
                               a[:2] - b[:2], [grasped], [on_top]]).astype(np.float64)

    def _get_obs(self):
        state = np.concatenate([self._get_state_obs(), self._task_obs()])
        if self.camera:
            return {"state": state, "rgb": self._render_cameras()}
        return state

    def _on_top(self, a, b):
        return (float(np.hypot(*(a[:2] - b[:2]))) <= XY_TOL
                and abs(a[2] - b[2] - self.HALF_A - self.HALF_B) <= Z_TOL)

    def evaluate(self):
        a = self.free_body_pos(ITEM_A)
        b = self.free_body_pos(ITEM_B)
        grasped = is_grasping(self.model, self.data, ITEM_A)
        static_a = is_body_static(self.model, self.data, ITEM_A)
        robot_static = is_static(self.data, self._qvel_adr)
        return {
            "tcp_to_item_dist": float(np.linalg.norm(a - self.tcp_pos)),
            "a_to_b_xy_dist": float(np.hypot(*(a[:2] - b[:2]))),
            "is_grasped": grasped,
            "on_top": bool(self._on_top(a, b)),
            "item_static": static_a,
            "robot_static": robot_static,
            "is_static": static_a,
            "success": bool(self._on_top(a, b) and not grasped and static_a and robot_static),
        }

    def compute_dense_reward(self, action, info):
        r = 1.0 - np.tanh(5.0 * info["tcp_to_item_dist"])
        if info["is_grasped"]:
            r += 1.0 + np.exp(-2.0 * info["a_to_b_xy_dist"])
        if info["on_top"]:
            r += 2.0
        if info["success"]:
            r += 2.0
        return r


class SO101StackCanEnv(SO101StackEnv):
    """stack-can: 底座 itemB 换圆柱罐（squint StackCan 镜像; itemA 永远是 cube）。

    罐立放: itemB spawn z 与 on_top 判定基准 = 罐半高 0.03。
    """

    SCENE_XML = "scene_stack_can.xml"
    HALF_B = 0.03
