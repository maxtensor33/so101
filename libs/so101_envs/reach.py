"""reach 任务: TCP 到达物体上方目标点（严格对齐 squint `envs/reach.py`）。

squint 语义对照:
- 物体: cube half [0.011, 0.014] / can r [0.014, 0.019] half-h [0.025, 0.035],
  密度 200, 摩擦 [0.1, 0.5]（DR 开时逐 episode 采样, 关时取中点）
- 出生域: [0.3, 0] ± 0.1
- goal: item 正上方 item_half + 0.02
- success = 到达 goal (阈值 0.02) & 机器人静止 & 夹爪闭合
- reward（归一化 /4）:
  (1-tanh(5d)) + gripper_closeness + is_reached - 3*touch_table - 1*touch_item
"""

from __future__ import annotations

import mujoco
import numpy as np

from .base_env import SO101BaseEnv
from .utils import is_static, is_touching

ITEM_BODY = "item"
ITEM_GEOM = "item_geom"
FINGER_BODIES = frozenset(["gripper", "moving_jaw_so101_v1"])


class SO101ReachEnv(SO101BaseEnv):
    """TCP 到达物体上方 goal（<= 阈值 & 静止 & 夹爪闭合 -> success）。"""

    SCENE_XML = "scene_reach.xml"
    # goal_pos(3)+tcp_to_goal(3)+item_pose(7)+tcp_to_item(3)+is_reached(1)
    # +clean_qpos(6)+item_dimensions(3)+item_friction(1)+item_density(1)
    TASK_OBS_DIM = 28
    TERMINATE_ON_SUCCESS = False  # squint ManiSkillVectorEnv ignore_terminations=True

    ITEM_SHAPE = "box"
    CUBE_HALF_RANGE = (0.011, 0.014)
    CAN_RADIUS_RANGE = (0.014, 0.019)
    CAN_HALF_HEIGHT_RANGE = (0.025, 0.035)

    def _sample_geometry(self, rng):
        if self.ITEM_SHAPE == "box":
            return self.sample_box_geometry(rng, self.CUBE_HALF_RANGE)
        if self.ITEM_SHAPE == "cylinder":
            return self.sample_cylinder_geometry(
                rng, self.CAN_RADIUS_RANGE, self.CAN_HALF_HEIGHT_RANGE)
        raise NotImplementedError(f"未知 ITEM_SHAPE: {self.ITEM_SHAPE}")

    def _item_dimensions(self):
        if self.ITEM_SHAPE == "box":
            return np.full(3, self._item_half_z)
        r, hh = self._item_size
        return np.array([r, r, hh])

    def _reset_task(self, rng):
        cfg = self.cfg
        size, half_z, mass, inertia = self._sample_geometry(rng)
        self._item_size = size
        self._item_half_z = half_z
        self.apply_free_geometry(ITEM_BODY, ITEM_GEOM, size, mass, inertia)

        cx, cy = cfg.spawn_box_pos
        h = cfg.spawn_box_half_size
        x = cx + rng.uniform(-h, h)
        y = cy + rng.uniform(-h, h)
        self.set_free_body_pose(ITEM_BODY, (x, y, half_z), yaw=rng.uniform(0, 2 * np.pi))
        self._randomize_item(ITEM_GEOM, rng)

        self._goal = np.array([x, y, 2 * half_z + cfg.goal_height_above_item])
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "goal_site")
        self.data.mocap_pos[self.model.body_mocapid[bid]] = self._goal
        mujoco.mj_forward(self.model, self.data)

    def _task_obs(self):
        qadr = self.model.jnt_qposadr[self.model.body_jntadr[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, ITEM_BODY)]]
        item_pose = self.data.qpos[qadr:qadr + 7]
        goal = self._goal
        dims = self._item_dimensions()
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, ITEM_GEOM)
        friction = float(self.model.geom_friction[gid][0])
        return np.concatenate([
            goal, goal - self.tcp_pos, item_pose, item_pose[:3] - self.tcp_pos,
            [float(np.linalg.norm(goal - self.tcp_pos) <= self.cfg.target_goal_thresh)],
            self.data.qpos[self._qpos_adr],
            dims, [friction], [self.cfg.item_density],
        ]).astype(np.float64)

    def _get_obs(self):
        state = np.concatenate([self._get_state_obs(), self._task_obs()])
        if self.camera:
            return {"state": state, "rgb": self._render_cameras()}
        return state

    def _gripper_closeness(self):
        lo, hi = self._ctrlrange[5]
        return float((hi - self.data.qpos[self._qpos_adr[5]]) / (hi - lo))

    def evaluate(self):
        dist = float(np.linalg.norm(self._goal - self.tcp_pos))
        is_reached = dist <= self.cfg.target_goal_thresh
        static = is_static(self.data, self._qvel_adr)
        closeness = self._gripper_closeness()
        gripper_closed = closeness > 0.9  # squint (1 - closeness) < 0.1
        touching_table = is_touching(
            self.model, self.data, "table_top", min_force=1e-2, bodies=FINGER_BODIES)
        touching_item = is_touching(
            self.model, self.data, ITEM_GEOM, min_force=1e-2, bodies=FINGER_BODIES)
        return {
            "tcp_to_goal_dist": dist,
            "is_reached": bool(is_reached),
            "is_static": static,
            "is_robot_static": static,
            "gripper_closeness": closeness,
            "is_gripper_closed": gripper_closed,
            "robot_touching_table": touching_table,
            "robot_touching_item": touching_item,
            "success": bool(is_reached and static and gripper_closed),
        }

    def compute_dense_reward(self, action, info):
        # squint compute_normalized_dense_reward / 4
        r = 1.0 - np.tanh(5.0 * info["tcp_to_goal_dist"])
        r += info["gripper_closeness"]
        r += float(info["is_reached"])
        if info["success"]:
            r = 4.0  # squint success 覆盖，而非叠加
        r -= 3.0 * float(info["robot_touching_table"])
        r -= 1.0 * float(info["robot_touching_item"])
        return r / 4.0


class SO101ReachCanEnv(SO101ReachEnv):
    """reach-can: 任务物体换圆柱罐（squint ReachCan 镜像）。"""

    SCENE_XML = "scene_reach_can.xml"
    ITEM_SHAPE = "cylinder"
