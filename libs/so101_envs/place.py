"""place 任务: 抓起物体放入 bin（squint Place 镜像, mujoco 实现）。"""

from __future__ import annotations

import mujoco
import numpy as np

from .base_env import SO101BaseEnv
from .utils import is_grasping, is_static, is_touching

ITEM_BODY = "item"
ITEM_GEOM = "item_geom"
ITEM_HALF = 0.022
BIN_HALF_INNER = (0.055, 0.075)  # bin 内框半长 (x, y)
BIN_MARGIN = 0.006               # 物体完全落入内框余量
BIN_WALL_TOP = 0.048             # bin 壁顶高度


class SO101PlaceEnv(SO101BaseEnv):
    SCENE_XML = "scene_place.xml"
    TASK_OBS_DIM = 15  # item_pose(7)+tcp_to_item(3)+bin_xy(2)+item_to_bin_xy(2)+in_bin(1)

    def _reset_task(self, rng):
        # cube 出生域
        x = 0.25 + rng.uniform(-0.08, 0.08)
        y = -0.1 + rng.uniform(-0.06, 0.06)
        self.set_free_body_pose(ITEM_BODY, (x, y, ITEM_HALF), yaw=rng.uniform(0, 2 * np.pi))
        self._randomize_item(ITEM_GEOM, rng)
        # bin 出生域（与 cube 保持 >=12cm 间距）
        for _ in range(50):
            bx = 0.42 + rng.uniform(-0.06, 0.06)
            by = 0.10 + rng.uniform(-0.05, 0.05)
            if np.hypot(bx - x, by - y) >= 0.12:
                break
        self.set_free_body_pose("bin", (bx, by, 0.0), yaw=0.0)
        mujoco.mj_forward(self.model, self.data)

    def _task_obs(self):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, ITEM_BODY)
        qadr = self.model.jnt_qposadr[self.model.body_jntadr[bid]]
        item_pose = self.data.qpos[qadr:qadr + 7]
        ip = item_pose[:3]
        bp = self.free_body_pos("bin")
        in_bin = float(self._in_bin(ip))
        return np.concatenate([item_pose, ip - self.tcp_pos, bp[:2],
                               ip[:2] - bp[:2], [in_bin]]).astype(np.float64)

    def _get_obs(self):
        state = np.concatenate([self._get_state_obs(), self._task_obs()])
        if self.camera:
            return {"state": state, "rgb": self._render_cameras()}
        return state

    def _in_bin(self, item_p):
        bp = self.free_body_pos("bin")
        dx = abs(item_p[0] - bp[0])
        dy = abs(item_p[1] - bp[1])
        return (dx <= BIN_HALF_INNER[0] - ITEM_HALF - BIN_MARGIN
                and dy <= BIN_HALF_INNER[1] - ITEM_HALF - BIN_MARGIN
                and item_p[2] <= BIN_WALL_TOP)

    def evaluate(self):
        ip = self.free_body_pos(ITEM_BODY)
        d_tcp = float(np.linalg.norm(ip - self.tcp_pos))
        grasped = is_grasping(self.model, self.data, ITEM_BODY)
        in_bin = bool(self._in_bin(ip))
        return {
            "tcp_to_item_dist": d_tcp,
            "is_grasped": grasped,
            "in_bin": in_bin,
            "is_static": is_static(self.data, self._qvel_adr),
            "robot_touching_table": is_touching(self.model, self.data, "table_top"),
            "success": bool(in_bin and not grasped and is_static(self.data, self._qvel_adr)),
        }

    def compute_dense_reward(self, action, info):
        r = 1.0 - np.tanh(5.0 * info["tcp_to_item_dist"])
        if info["is_grasped"]:
            r += 1.0
            bp = self.free_body_pos("bin")
            r += np.exp(-2.0 * float(np.linalg.norm(self.free_body_pos(ITEM_BODY)[:2] - bp[:2])))
        if info["in_bin"]:
            r += 2.0
        r -= 1.0 * float(info["robot_touching_table"])
        if info["success"]:
            r += 2.0
        return r
