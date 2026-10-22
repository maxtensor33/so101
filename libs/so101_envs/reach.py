"""reach 任务: TCP 到达物体上方目标点（squint Reach 镜像, mujoco 实现）。"""

from __future__ import annotations

import mujoco
import numpy as np

from .base_env import SO101BaseEnv
from .utils import is_static, is_touching

ITEM_BODY = "item"
ITEM_GEOM = "item_geom"
ITEM_HALF = 0.024


class SO101ReachEnv(SO101BaseEnv):
    """TCP 到达物体上方 goal（<= target_goal_thresh 且静止 -> success）。"""

    SCENE_XML = "scene_reach.xml"
    TASK_OBS_DIM = 6  # goal_pos(3) + tcp_to_goal(3)

    def _reset_task(self, rng):
        cfg = self.cfg
        cx, cy = cfg.spawn_box_pos
        h = cfg.spawn_box_half_size
        x = cx + rng.uniform(-h, h)
        y = cy + rng.uniform(-h, h)
        self.set_free_body_pose(ITEM_BODY, (x, y, ITEM_HALF), yaw=rng.uniform(0, 2 * np.pi))
        self._randomize_item(ITEM_GEOM, rng)
        self._randomize_fixed_cam(rng)
        self._goal = np.array([x, y, 2 * ITEM_HALF + cfg.goal_height_above_item])
        # 同步 goal_site mocap（viewer/RViz 显示与判定点一致）
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "goal_site")
        self.data.mocap_pos[self.model.body_mocapid[bid]] = self._goal
        mujoco.mj_forward(self.model, self.data)

    def _task_obs(self):
        goal = self._goal
        return np.concatenate([goal, goal - self.tcp_pos]).astype(np.float64)

    def _get_obs(self):
        state = np.concatenate([self._get_state_obs(), self._task_obs()])
        if self.camera:
            return {"state": state, "rgb": self._render_cameras()}
        return state

    def evaluate(self):
        dist = float(np.linalg.norm(self._goal - self.tcp_pos))
        is_reached = dist <= self.cfg.target_goal_thresh
        static = is_static(self.data, self._qvel_adr)
        touching_table = is_touching(self.model, self.data, "table_top")
        return {
            "tcp_to_goal_dist": dist,
            "is_reached": bool(is_reached),
            "is_static": static,
            "robot_touching_table": touching_table,
            "success": bool(is_reached and static),
        }

    def compute_dense_reward(self, action, info):
        r = 1.0 - np.tanh(5.0 * info["tcp_to_goal_dist"])
        r += 1.0 * float(info["is_reached"])
        r -= 1.0 * float(info["robot_touching_table"])
        if info["success"]:
            r += 2.0
        return r
