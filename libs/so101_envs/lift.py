"""lift 任务: 抓起物体并抬离桌面（squint Lift 镜像, mujoco 实现）。"""

from __future__ import annotations

import mujoco
import numpy as np

from .base_env import SO101BaseEnv, REST_QPOS
from .utils import is_grasping, is_static, is_touching

ITEM_BODY = "item"
ITEM_GEOM = "item_geom"
ITEM_HALF = 0.022
LIFT_HEIGHT = 0.005  # 物体中心高于 half 多少算"抬起"


class SO101LiftEnv(SO101BaseEnv):
    """success = 抓住 & 物体抬离桌面 & 机械臂接近 rest 位形。"""

    SCENE_XML = "scene_lift.xml"
    TASK_OBS_DIM = 10  # item_pose(7) + tcp_to_item(3)

    # tool-down 可抓工作空间半径上限 (todo/envs_grasp_tuning_todo.md):
    # 半径 >0.28 时 wrist_flex 触限 + 固定指拖桌, 无法抓取 — 超出部分钳回可抓带
    GRASPABLE_RADIUS = 0.26

    def _reset_task(self, rng):
        cx, cy = self.cfg.spawn_box_pos
        h = self.cfg.spawn_box_half_size
        x = cx + rng.uniform(-h, h)
        y = cy + rng.uniform(-h, h)
        r = float(np.hypot(x, y))
        if r > self.GRASPABLE_RADIUS:
            x, y = x / r * self.GRASPABLE_RADIUS, y / r * self.GRASPABLE_RADIUS
        self.set_free_body_pose(ITEM_BODY, (x, y, ITEM_HALF), yaw=rng.uniform(0, 2 * np.pi))
        self._randomize_item(ITEM_GEOM, rng)
        mujoco.mj_forward(self.model, self.data)

    def _task_obs(self):
        qadr = self.model.jnt_qposadr[self.model.body_jntadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, ITEM_BODY)]]
        item_pose = self.data.qpos[qadr:qadr + 7]
        rel = item_pose[:3] - self.tcp_pos
        return np.concatenate([item_pose, rel]).astype(np.float64)

    def _get_obs(self):
        state = np.concatenate([self._get_state_obs(), self._task_obs()])
        if self.camera:
            return {"state": state, "rgb": self._render_cameras()}
        return state

    def evaluate(self):
        item_p = self.free_body_pos(ITEM_BODY)
        d_tcp = float(np.linalg.norm(item_p - self.tcp_pos))
        grasped = is_grasping(self.model, self.data, ITEM_BODY)
        lifted = item_p[2] >= ITEM_HALF + LIFT_HEIGHT
        arm_q = self.data.qpos[self._qpos_adr][:5]
        d_rest = float(np.linalg.norm(arm_q - REST_QPOS[:5]))
        return {
            "tcp_to_item_dist": d_tcp,
            "is_grasped": grasped,
            "item_lifted": bool(lifted),
            "dist_to_rest": d_rest,
            "robot_touching_table": is_touching(self.model, self.data, "table_top"),
            "success": bool(grasped and lifted and d_rest < 0.5),
        }

    def compute_dense_reward(self, action, info):
        r = 1.0 - np.tanh(5.0 * info["tcp_to_item_dist"])
        if info["is_grasped"]:
            r += 1.0 + np.exp(-2.0 * info["dist_to_rest"])
        r -= 3.0 * float(info["robot_touching_table"])
        if not info["item_lifted"]:
            r -= 1.0
        if info["success"]:
            r += 2.0
        return r
