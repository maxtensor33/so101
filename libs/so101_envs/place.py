"""place 任务: 抓起物体放入 bin（严格对齐 squint `envs/place.py`）。

squint 语义对照:
- 物体: cube half [0.011, 0.014] / can r [0.014, 0.019] half-h [0.025, 0.035],
  密度 200, 摩擦 [0.1, 0.5]
- bin: half x [0.035,0.045] / y [0.045,0.055] / z [0.012,0.018], 壁厚 0.005,
  随机 yaw
- 出生域: item/bin 均在 [0.3,0] ± 0.1 内且不重叠
- success = item 中心在 bin 本地 xy 内 & 未接触 item & 机器人静止 & 未接触 bin
- reward: squint 原式 /9
"""

from __future__ import annotations

import mujoco
import numpy as np

from .base_env import SO101BaseEnv
from .utils import is_grasping, is_static, is_touching, is_touching_any

ITEM_BODY = "item"
ITEM_GEOM = "item_geom"
BIN_BODY = "bin"
BIN_GEOMS = ("bin_bottom", "bin_wall_px", "bin_wall_nx", "bin_wall_py", "bin_wall_ny")
BIN_THICKNESS = 0.005
FINGER_BODIES = frozenset(["gripper", "moving_jaw_so101_v1"])


class SO101PlaceEnv(SO101BaseEnv):
    SCENE_XML = "scene_place.xml"
    # item_pose(7)+tcp_to_item(3)+bin_pose(7)+tcp_to_bin(3)+item_to_bin(3)
    # +grasped(1)+clean_qpos(6)+item_dimensions(3)+bin_dimensions(3)
    # +item_friction(1)+item_density(1)
    TASK_OBS_DIM = 38
    TERMINATE_ON_SUCCESS = False  # squint ManiSkillVectorEnv ignore_terminations=True

    ITEM_SHAPE = "box"
    CUBE_HALF_RANGE = (0.011, 0.014)
    CAN_RADIUS_RANGE = (0.014, 0.019)
    CAN_HALF_HEIGHT_RANGE = (0.025, 0.035)
    BIN_HALF_X_RANGE = (0.035, 0.045)
    BIN_HALF_Y_RANGE = (0.045, 0.055)
    BIN_HALF_Z_RANGE = (0.012, 0.018)

    # ---------- 物体/ bin 几何 ----------
    def _sample_item_geometry(self, rng):
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

    def _sample_bin(self, rng):
        if self.domain_randomization:
            bx = rng.uniform(*self.BIN_HALF_X_RANGE)
            by = rng.uniform(*self.BIN_HALF_Y_RANGE)
            bz = rng.uniform(*self.BIN_HALF_Z_RANGE)
        else:
            bx = sum(self.BIN_HALF_X_RANGE) / 2.0
            by = sum(self.BIN_HALF_Y_RANGE) / 2.0
            bz = sum(self.BIN_HALF_Z_RANGE) / 2.0
        return bx, by, bz, BIN_THICKNESS

    def _apply_bin_geometry(self, bx, by, bz, t):
        m = self.model
        def gid(name):
            return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
        m.geom_size[gid("bin_bottom")] = [bx, by, t / 2]
        m.geom_pos[gid("bin_bottom")] = [0, 0, t / 2]
        for name, sign, axis in (("bin_wall_px", 1, 0), ("bin_wall_nx", -1, 0),
                                 ("bin_wall_py", 1, 1), ("bin_wall_ny", -1, 1)):
            if axis == 0:
                m.geom_size[gid(name)] = [t / 2, by, bz]
                m.geom_pos[gid(name)] = [sign * bx, 0, bz]
            else:
                m.geom_size[gid(name)] = [bx, t / 2, bz]
                m.geom_pos[gid(name)] = [0, sign * by, bz]
        self._recompute_constants()

    def _reset_task(self, rng):
        cfg = self.cfg
        size, half_z, mass, inertia = self._sample_item_geometry(rng)
        self._item_size = size
        self._item_half_z = half_z
        self.apply_free_geometry(ITEM_BODY, ITEM_GEOM, size, mass, inertia)

        bx, by, bz, t = self._sample_bin(rng)
        self._bin_dims = np.array([bx, by, bz])
        self._bin_thickness = t
        self._apply_bin_geometry(bx, by, bz, t)

        # item/bin 均在 [0.3,0] ± 0.1，且不重叠（squint UniformPlacementSampler）
        cx, cy = cfg.spawn_box_pos
        h = cfg.spawn_box_half_size
        item_r = float(np.max(self._item_dimensions()) + 0.01)
        bin_r = float(np.linalg.norm([bx, by]) + 0.01)
        for _ in range(100):
            ix = cx + rng.uniform(-h, h)
            iy = cy + rng.uniform(-h, h)
            bxp = cx + rng.uniform(-h, h)
            byp = cy + rng.uniform(-h, h)
            if np.hypot(bxp - ix, byp - iy) >= item_r + bin_r:
                break

        self.set_free_body_pose(ITEM_BODY, (ix, iy, half_z), yaw=rng.uniform(0, 2 * np.pi))
        self._randomize_item(ITEM_GEOM, rng)
        self.set_free_body_pose(BIN_BODY, (bxp, byp, t / 2), yaw=rng.uniform(0, 2 * np.pi))
        mujoco.mj_forward(self.model, self.data)

    def _item_to_bin_xy(self):
        """item 相对 bin 的本地 xy（bin 有随机 yaw）。"""
        item = self.free_body_pos(ITEM_BODY)
        binp = self.free_body_pos(BIN_BODY)
        bq = self.free_body_pose(BIN_BODY)[3:7]
        w, x, y, z = bq
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        c, s = np.cos(yaw), np.sin(yaw)
        off = item - binp
        return c * off[0] + s * off[1], -s * off[0] + c * off[1]

    def _in_bin(self):
        ox, oy = self._item_to_bin_xy()
        bx, by, _ = self._bin_dims
        return bool(abs(ox) < bx and abs(oy) < by)

    def _touching(self, geom_name):
        return is_touching(self.model, self.data, geom_name,
                           min_force=1e-2, bodies=FINGER_BODIES)

    def _task_obs(self):
        item_pose = self.free_body_pose(ITEM_BODY)
        bin_pose = self.free_body_pose(BIN_BODY)
        grasped = float(is_grasping(self.model, self.data, ITEM_BODY))
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, ITEM_GEOM)
        friction = float(self.model.geom_friction[gid][0])
        item_dim = self._item_dimensions()
        return np.concatenate([
            item_pose, item_pose[:3] - self.tcp_pos,
            bin_pose, bin_pose[:3] - self.tcp_pos,
            bin_pose[:3] - item_pose[:3],
            [grasped], self.data.qpos[self._qpos_adr],
            item_dim, self._bin_dims, [friction], [self.cfg.item_density],
        ]).astype(np.float64)

    def _get_obs(self):
        state = np.concatenate([self._get_state_obs(), self._task_obs()])
        if self.camera:
            return {"state": state, "rgb": self._render_cameras()}
        return state

    def evaluate(self):
        ip = self.free_body_pos(ITEM_BODY)
        grasped = is_grasping(self.model, self.data, ITEM_BODY)
        above_bin = self._in_bin()
        item_static = is_static(self.data, self._qvel_adr)  # 仅 info, success 不用
        robot_static = is_static(self.data, self._qvel_adr)
        touching_item = self._touching(ITEM_GEOM)
        touching_bin = is_touching_any(self.model, self.data, BIN_GEOMS,
                                       min_force=1e-2, bodies=FINGER_BODIES)
        return {
            "tcp_to_item_dist": float(np.linalg.norm(ip - self.tcp_pos)),
            "is_grasped": grasped,
            "is_item_grasped": grasped,
            "in_bin": above_bin,
            "is_item_above_bin": above_bin,
            "is_item_static": item_static,
            "is_robot_static": robot_static,
            "item_lifted": bool(ip[2] >= self._item_half_z + 1e-3),
            "robot_touching_table": self._touching("table_top"),
            "robot_touching_item": touching_item,
            "robot_touching_bin": touching_bin,
            "success": bool(above_bin and not touching_item and robot_static
                            and not touching_bin),
        }

    def compute_dense_reward(self, action, info):
        # squint compute_normalized_dense_reward / 9
        ip = self.free_body_pos(ITEM_BODY)
        bp = self.free_body_pos(BIN_BODY)
        bx, by, bz = self._bin_dims
        item_half = self._item_half_z
        bin_radius = float(np.linalg.norm([bx, by]))

        tcp_to_item = np.linalg.norm(self.tcp_pos - ip)
        r = 2.0 * (1.0 - np.tanh(5.0 * tcp_to_item))

        goal_xyz = bp.copy()
        goal_xyz[2] = self._bin_thickness + item_half
        item_to_goal = np.linalg.norm(goal_xyz - ip)
        place_final = 1.0 - np.tanh(5.0 * item_to_goal)

        dist_xy = np.linalg.norm(goal_xyz[:2] - ip[:2])
        target_z_far = goal_xyz[2] + 2.0 * bz + 0.03
        d_z_far = abs(target_z_far - ip[2])
        d_z_close = abs(goal_xyz[2] - ip[2])
        d_z = d_z_close if dist_xy <= bin_radius else d_z_far
        place_z = 1.0 - np.tanh(10.0 * d_z)
        place = place_final + place_z

        lo, hi = self._ctrlrange[5]
        gripper_openness = float((self.data.qpos[self._qpos_adr[5]] - lo) / (hi - lo))
        robot_v = float(np.linalg.norm(self.data.qvel[self._qvel_adr][:5]))
        static_robot = 1.0 - np.tanh(robot_v * 10.0)

        if info["is_item_grasped"]:
            r = 3.0 + place
        if info["is_item_above_bin"]:
            is_item_dropped = float(not info["robot_touching_item"])
            r = 4.0 + place + is_item_dropped + gripper_openness + static_robot
        if info["success"]:
            r = 9.0
        r -= 6.0 * float(info["robot_touching_table"])
        r -= 3.0 * float(info["robot_touching_bin"])
        r -= 1.0 * float(not info["item_lifted"])
        return r / 9.0


class SO101PlaceCanEnv(SO101PlaceEnv):
    """place-can: 任务物体换圆柱罐（squint PlaceCan 镜像）。"""

    SCENE_XML = "scene_place_can.xml"
    ITEM_SHAPE = "cylinder"
