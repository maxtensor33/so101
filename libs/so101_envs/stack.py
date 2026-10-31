"""stack 任务: 抓起 itemA 堆到 itemB 上（严格对齐 squint `envs/stack.py`）。

squint 语义对照:
- itemA 永远是 cube: half [0.011,0.014], 密度 200
- itemB: cube half [0.0125,0.0175] 或 can r [0.014,0.019] half-h [0.025,0.035]
- 出生域: 两者均在 [0.3,0] ± 0.1 内且不重叠, 随机 yaw
- success = A 在 B 上（xy<=0.02, |z-(hA+hB)|<=0.01）& A 静止 & 未抓 A & 机器人静止
- reward: squint 原式 /9
"""

from __future__ import annotations

import mujoco
import numpy as np

from .base_env import SO101BaseEnv
from .utils import is_grasping, is_static, is_touching

ITEM_A = "itemA"
ITEM_B = "itemB"
ITEM_A_GEOM = "itemA_geom"
ITEM_B_GEOM = "itemB_geom"
XY_TOL = 0.02
Z_TOL = 0.01
FINGER_BODIES = frozenset(["gripper", "moving_jaw_so101_v1"])


class SO101StackEnv(SO101BaseEnv):
    SCENE_XML = "scene_stack.xml"
    # itemA_pose(7)+tcp_to_A(3)+itemB_pose(7)+tcp_to_B(3)+B-A(3)+grasped(1)
    # +clean_qpos(6)+itemA_dim(3)+itemB_dim(3)+fric(1)+dens(1)
    TASK_OBS_DIM = 38
    TERMINATE_ON_SUCCESS = False  # squint ManiSkillVectorEnv ignore_terminations=True

    ITEM_B_SHAPE = "box"
    ITEM_A_HALF_RANGE = (0.011, 0.014)
    ITEM_B_CUBE_HALF_RANGE = (0.0125, 0.0175)
    CAN_RADIUS_RANGE = (0.014, 0.019)
    CAN_HALF_HEIGHT_RANGE = (0.025, 0.035)

    def _sample_item_b(self, rng):
        if self.ITEM_B_SHAPE == "box":
            return self.sample_box_geometry(rng, self.ITEM_B_CUBE_HALF_RANGE)
        if self.ITEM_B_SHAPE == "cylinder":
            return self.sample_cylinder_geometry(
                rng, self.CAN_RADIUS_RANGE, self.CAN_HALF_HEIGHT_RANGE)
        raise NotImplementedError(f"未知 ITEM_B_SHAPE: {self.ITEM_B_SHAPE}")

    def _item_b_dimensions(self):
        if self.ITEM_B_SHAPE == "box":
            return np.full(3, self._item_b_half_z)
        r, hh = self._item_b_size
        return np.array([r, r, hh])

    def _reset_task(self, rng):
        cfg = self.cfg
        sizeA, halfA, massA, inertiaA = self.sample_box_geometry(rng, self.ITEM_A_HALF_RANGE)
        self._item_a_half_z = halfA
        self.apply_free_geometry(ITEM_A, ITEM_A_GEOM, sizeA, massA, inertiaA)

        sizeB, halfB, massB, inertiaB = self._sample_item_b(rng)
        self._item_b_size = sizeB
        self._item_b_half_z = halfB
        self.apply_free_geometry(ITEM_B, ITEM_B_GEOM, sizeB, massB, inertiaB)

        # 两者均在 [0.3,0] ± 0.1，且不重叠
        cx, cy = cfg.spawn_box_pos
        h = cfg.spawn_box_half_size
        radiusA = float(halfA + 0.01)
        radiusB = float(max(self._item_b_dimensions()[:2]) + 0.01)
        for _ in range(100):
            ax = cx + rng.uniform(-h, h)
            ay = cy + rng.uniform(-h, h)
            bx = cx + rng.uniform(-h, h)
            by = cy + rng.uniform(-h, h)
            if np.hypot(bx - ax, by - ay) >= radiusA + radiusB:
                break

        self.set_free_body_pose(ITEM_A, (ax, ay, halfA), yaw=rng.uniform(0, 2 * np.pi))
        self._randomize_item(ITEM_A_GEOM, rng)
        self.set_free_body_pose(ITEM_B, (bx, by, halfB), yaw=rng.uniform(0, 2 * np.pi))
        # itemB 用同一摩擦采样（squint 两物体共享 frictions[i]）
        self.model.geom_friction[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, ITEM_B_GEOM)] = \
            self.model.geom_friction[mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, ITEM_A_GEOM)]
        mujoco.mj_forward(self.model, self.data)

    def _body_linear_speed(self, body):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
        vadr = self.model.jnt_dofadr[self.model.body_jntadr[bid]]
        return float(np.linalg.norm(self.data.qvel[vadr:vadr + 3]))

    def _on_b_flags(self):
        a = self.free_body_pos(ITEM_A)
        b = self.free_body_pos(ITEM_B)
        xy = float(np.linalg.norm(a[:2] - b[:2]))
        z = float(abs((a[2] - b[2]) - (self._item_a_half_z + self._item_b_half_z)))
        return xy, z, bool(xy <= XY_TOL and z <= Z_TOL)

    def _touching(self, geom_name):
        return is_touching(self.model, self.data, geom_name,
                           min_force=1e-2, bodies=FINGER_BODIES)

    def _task_obs(self):
        a_pose = self.free_body_pose(ITEM_A)
        b_pose = self.free_body_pose(ITEM_B)
        grasped = float(is_grasping(self.model, self.data, ITEM_A))
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, ITEM_A_GEOM)
        friction = float(self.model.geom_friction[gid][0])
        return np.concatenate([
            a_pose, a_pose[:3] - self.tcp_pos,
            b_pose, b_pose[:3] - self.tcp_pos,
            b_pose[:3] - a_pose[:3],
            [grasped], self.data.qpos[self._qpos_adr],
            np.full(3, self._item_a_half_z), self._item_b_dimensions(),
            [friction], [self.cfg.item_density],
        ]).astype(np.float64)

    def _get_obs(self):
        state = np.concatenate([self._get_state_obs(), self._task_obs()])
        if self.camera:
            return {"state": state, "rgb": self._render_cameras()}
        return state

    def evaluate(self):
        a = self.free_body_pos(ITEM_A)
        b = self.free_body_pos(ITEM_B)
        xy, z, on_top = self._on_b_flags()
        grasped = is_grasping(self.model, self.data, ITEM_A)
        speed_a = self._body_linear_speed(ITEM_A)
        static_a = speed_a <= 0.02
        robot_static = is_static(self.data, self._qvel_adr)
        touching_a = self._touching(ITEM_A_GEOM)
        return {
            "tcp_to_item_dist": float(np.linalg.norm(a - self.tcp_pos)),
            "a_to_b_xy_dist": xy,
            "z_dist": z,
            "is_grasped": grasped,
            "is_itemA_grasped": grasped,
            "on_top": on_top,
            "is_itemA_on_itemB": on_top,
            "item_static": static_a,
            "is_itemA_static": static_a,
            "robot_static": robot_static,
            "is_static": static_a,
            "item_lifted": bool(a[2] >= self._item_a_half_z + 1e-3),
            "is_itemA_lifted": bool(a[2] >= self._item_a_half_z + 1e-3),
            "robot_touching_table": self._touching("table_top"),
            "robot_touching_itemA": touching_a,
            "success": bool(on_top and static_a and not touching_a and robot_static),
        }

    def compute_dense_reward(self, action, info):
        # squint compute_normalized_dense_reward / 9
        a = self.free_body_pos(ITEM_A)
        b = self.free_body_pos(ITEM_B)
        halfA, halfB = self._item_a_half_z, self._item_b_half_z

        tcp_to_a = np.linalg.norm(self.tcp_pos - a)
        r = 2.0 * (1.0 - np.tanh(5.0 * tcp_to_a))

        goal_xyz = b.copy()
        goal_xyz[2] = b[2] + halfB + halfA
        item_to_goal = np.linalg.norm(goal_xyz - a)
        place_final = 1.0 - np.tanh(5.0 * item_to_goal)

        d_xy = np.linalg.norm(goal_xyz[:2] - a[:2])
        d_z_far = abs((goal_xyz[2] + 0.03) - a[2])
        d_z_close = abs(goal_xyz[2] - a[2])
        d_z = d_z_close if d_xy <= 0.04 else d_z_far
        place_z = 1.0 - np.tanh(10.0 * d_z)
        place = place_final + place_z

        lo, hi = self._ctrlrange[5]
        ungrasp = float((self.data.qpos[self._qpos_adr[5]] - lo) / (hi - lo))
        speed_a = self._body_linear_speed(ITEM_A)
        robot_v = float(np.linalg.norm(self.data.qvel[self._qvel_adr][:5]))
        static_a = 1.0 - np.tanh(speed_a * 10.0)
        static_robot = 1.0 - np.tanh(robot_v * 10.0)

        if info["is_itemA_grasped"]:
            r = 3.0 + place
        if info["is_itemA_on_itemB"] and info["robot_touching_itemA"]:
            r = 4.0 + place + ungrasp
        if info["is_itemA_on_itemB"] and not info["robot_touching_itemA"]:
            r = 7.0 + (static_a + static_robot) / 2.0
        if info["success"]:
            r = 9.0
        r -= 6.0 * float(info["robot_touching_table"])
        r -= 1.0 * float(not info["is_itemA_lifted"])
        return r / 9.0


class SO101StackCanEnv(SO101StackEnv):
    """stack-can: 底座 itemB 换圆柱罐（squint StackCan 镜像；itemA 永远是 cube）。"""

    SCENE_XML = "scene_stack_can.xml"
    ITEM_B_SHAPE = "cylinder"
