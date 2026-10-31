"""lift 任务: 抓起物体并抬离桌面（严格对齐 squint `envs/lift.py`）。

squint 语义对照（ManiSkill/SAPIEN -> MuJoCo）:
- 动作: pd_joint_target_delta_pos（base_env action_mode="delta"）
- 物体: cube half [0.011, 0.014] / can r [0.014, 0.019] half-h [0.025, 0.035],
  密度 200 kg/m^3, 摩擦 [0.1, 0.5]（DR 开时逐 episode 采样, 关时取中点）
- 出生域: [0.3, 0] ± 0.1, 不做可达性裁剪
- success = 抓住 & item z >= half_z + 1e-3 & ||target_qpos[:-1]-rest[:-1]|| < 0.2
- dense reward（归一化 /3）:
  (1-tanh(5*d_tcp)) + grasped + exp(-2*d_rest)*grasped
  - 3*touching_table - 1*(not lifted)
"""

from __future__ import annotations

import mujoco
import numpy as np

from .base_env import SO101BaseEnv, REST_QPOS
from .utils import is_grasping, is_touching

ITEM_BODY = "item"
ITEM_GEOM = "item_geom"
LIFT_HEIGHT = 1e-3          # squint: item z >= half_size + 1e-3
REACH_THRESH = 0.03         # squint reached_object: tcp_to_obj_dist < 0.03
REST_THRESH = 0.2           # squint reached_rest_qpos: target qpos[:-1] 距 rest < 0.2
ITEM_DENSITY = 200.0        # squint item_density_range = (200, 200)
FINGER_BODIES = frozenset(["gripper", "moving_jaw_so101_v1"])


class SO101LiftEnv(SO101BaseEnv):
    """success = 抓住 & 物体抬离桌面 & 控制器目标接近 rest 位形。"""

    SCENE_XML = "scene_lift.xml"
    # TCP: squint 用两指尖中点; MuJoCo 刚体接触下实测真实夹持中心是 gripperframe
    # site（尖中点在张爪时偏离夹持中心 24-50mm, 会导致放偏滑落）。沿用 base 的 site。
    # item_pose(7)+tcp_to_item(3)+target_qpos(6)+dist_to_rest(5)+grasped(1)
    # +item_dimensions(3)+item_friction(1)+item_density(1)+clean_qpos(6)
    # （squint DR 开时额外提供 clean_qpos/item 尺寸/摩擦/密度等 privileged obs）
    TASK_OBS_DIM = 33
    TERMINATE_ON_SUCCESS = False  # squint ManiSkillVectorEnv ignore_terminations=True

    ITEM_SHAPE = "box"
    CUBE_HALF_RANGE = (0.011, 0.014)
    CAN_RADIUS_RANGE = (0.014, 0.019)
    CAN_HALF_HEIGHT_RANGE = (0.025, 0.035)

    # ---------- 物体几何/惯性（squint 尺寸/密度语义） ----------
    def _sample_geometry(self, rng):
        """返回 (geom_size, half_z, mass, inertia(3))。DR 关时取 range 中点。"""
        if self.ITEM_SHAPE == "box":
            h = (rng.uniform(*self.CUBE_HALF_RANGE)
                 if self.domain_randomization else sum(self.CUBE_HALF_RANGE) / 2.0)
            side = 2.0 * h
            mass = ITEM_DENSITY * side ** 3
            inertia = np.full(3, mass * side ** 2 / 6.0)
            return np.array([h, h, h]), float(h), float(mass), inertia
        if self.ITEM_SHAPE == "cylinder":
            r = (rng.uniform(*self.CAN_RADIUS_RANGE)
                 if self.domain_randomization else sum(self.CAN_RADIUS_RANGE) / 2.0)
            hh = (rng.uniform(*self.CAN_HALF_HEIGHT_RANGE)
                  if self.domain_randomization else sum(self.CAN_HALF_HEIGHT_RANGE) / 2.0)
            mass = ITEM_DENSITY * np.pi * r ** 2 * (2.0 * hh)
            inertia = np.array([
                mass * (3.0 * r ** 2 + (2.0 * hh) ** 2) / 12.0,
                mass * (3.0 * r ** 2 + (2.0 * hh) ** 2) / 12.0,
                mass * r ** 2 / 2.0,
            ])
            return np.array([r, hh]), float(hh), float(mass), inertia
        raise NotImplementedError(f"未知 ITEM_SHAPE: {self.ITEM_SHAPE}")

    def _apply_geometry(self, geom_name, size, mass, inertia):
        """运行期改写 geom_size/body 质量/惯性，并重算 rbound 等常量。

        注意: mj_setConst 会把 data 重置为模型默认状态（qpos/ctrl=0），
        这里先保存再恢复，否则机器人初始位形会被清掉。
        """
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        bid = int(self.model.geom_bodyid[gid])
        self.model.geom_size[gid][: len(size)] = size
        self.model.body_mass[bid] = mass
        self.model.body_inertia[bid] = inertia
        qpos, qvel, ctrl, sim_time = (
            self.data.qpos.copy(), self.data.qvel.copy(),
            self.data.ctrl.copy(), self.data.time)
        mujoco.mj_setConst(self.model, self.data)
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        self.data.ctrl[:] = ctrl
        self.data.time = sim_time

    def _item_dimensions(self):
        """squint item_dimensions: cube [h,h,h], can [r,r,half_h]（观测用）。"""
        if self.ITEM_SHAPE == "box":
            return np.full(3, self._item_half_z)
        r, hh = self._item_size
        return np.array([r, r, hh])

    # ---------- 任务 hook ----------
    def _reset_task(self, rng):
        size, half_z, mass, inertia = self._sample_geometry(rng)
        self._item_size = size
        self._item_half_z = half_z
        self._apply_geometry(ITEM_GEOM, size, mass, inertia)

        # squint spawn: robot.pos + spawn_box_pos ± half_size（不裁剪）
        cx, cy = self.cfg.spawn_box_pos
        h = self.cfg.spawn_box_half_size
        x = cx + rng.uniform(-h, h)
        y = cy + rng.uniform(-h, h)
        self.set_free_body_pose(ITEM_BODY, (x, y, half_z), yaw=rng.uniform(0, 2 * np.pi))
        self._randomize_item(ITEM_GEOM, rng)
        mujoco.mj_forward(self.model, self.data)

    def _task_obs(self):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, ITEM_BODY)
        qadr = self.model.jnt_qposadr[self.model.body_jntadr[bid]]
        item_pose = self.data.qpos[qadr:qadr + 7]
        rel = item_pose[:3] - self.tcp_pos
        target = self.target_qpos
        dist_rest = target[:5] - REST_QPOS[:5]
        grasped = float(is_grasping(self.model, self.data, ITEM_BODY))
        dims = self._item_dimensions()
        friction = float(self.model.geom_friction[
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, ITEM_GEOM)][0])
        return np.concatenate([
            item_pose, rel, target, dist_rest,
            [grasped], dims, [friction], [ITEM_DENSITY],
            self.data.qpos[self._qpos_adr],  # clean_qpos（squint privileged obs）
        ]).astype(np.float64)

    def _get_obs(self):
        state = np.concatenate([self._get_state_obs(), self._task_obs()])
        if self.camera:
            return {"state": state, "rgb": self._render_cameras()}
        return state

    def evaluate(self):
        item_p = self.free_body_pos(ITEM_BODY)
        d_tcp = float(np.linalg.norm(item_p - self.tcp_pos))
        grasped = is_grasping(self.model, self.data, ITEM_BODY)
        lifted = item_p[2] >= self._item_half_z + LIFT_HEIGHT
        target = self.target_qpos
        d_rest = float(np.linalg.norm(target[:5] - REST_QPOS[:5]))
        return {
            "tcp_to_item_dist": d_tcp,
            "reached_object": bool(d_tcp < REACH_THRESH),
            "is_item_grasped": grasped,
            "is_grasped": grasped,
            "item_lifted": bool(lifted),
            "distance_to_rest_qpos": d_rest,
            "dist_to_rest": d_rest,
            "robot_touching_table": is_touching(
                self.model, self.data, "table_top",
                min_force=1e-2, bodies=FINGER_BODIES),
            "success": bool(grasped and lifted and d_rest < REST_THRESH),
        }

    def compute_dense_reward(self, action, info):
        # squint compute_normalized_dense_reward / 3
        reaching = 1.0 - np.tanh(5.0 * info["tcp_to_item_dist"])
        r = reaching + float(info["is_item_grasped"])
        r += np.exp(-2.0 * info["distance_to_rest_qpos"]) * float(info["is_item_grasped"])
        r -= 3.0 * float(info["robot_touching_table"])
        r -= 1.0 * float(not info["item_lifted"])
        return r / 3.0


class SO101LiftCanEnv(SO101LiftEnv):
    """lift-can: 任务物体换圆柱罐（squint LiftCan 镜像）。"""

    SCENE_XML = "scene_lift_can.xml"
    ITEM_SHAPE = "cylinder"
