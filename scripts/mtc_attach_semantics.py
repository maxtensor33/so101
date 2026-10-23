"""attach 语义 spike (workflow/plan/mtc_fix.md R3) — 纯 mujoco 镜像 grasp_attach_plugin。

在无 ROS 的 mujoco 里复现插件语义并验收:
  - ideal 模式: attach 后 cube 刚体跟随动爪 (ideal_rel_pose, 与
    controllers_pick.yaml 货值一致)
  - legacy 模式: 世界系平移偏移 (规划位姿 - jaw), cube 保持水平 (旧行为)
  - 释放: gripper ctrl 张开越阈 -> 停止覆盖, cube 贴回物理仿真

符号约定 (与插件相反, 因插件读 JTC reference 的方向约定):
  mujoco gripper ctrl: 大 = 张开 (R1 实测), 小 = 闭合。
  插件 (R3 纠正后): attach 当 JTC ref < close_threshold,
  release 当 ref > release_threshold。

用法:
    uv run python scripts/attach_semantics.py   # 打印验收指标
"""

import importlib.util
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "libs"))

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[1]
SCENE_PICK = REPO / "src" / "so101_sim" / "mujoco" / "scene_pick.xml"

# R2 重探测关节解 (probe_pick_poses.probe_pair, pick/place 与节点默认一致)
PRE_JOINTS = np.array([0.77387, -0.71416, 0.95882, 1.32613, 0.82254])
GRASP_JOINTS = np.array([0.77386, -0.44185, 1.19005, 0.82260, 0.82253])
LIFT_JOINTS = np.array([0.77386, -0.64436, 1.07257, 1.14258, 0.82253])
PREPLACE_JOINTS = np.array([-0.63090, -0.74622, 0.97778, 1.33914, -0.58222])
PLACE_JOINTS = np.array([-0.63090, -0.47374, 1.21428, 0.83027, -0.58222])

GRIP_OPEN = 1.0        # SRDF "open" (R3 纠正后: span 56mm > cube 20mm)
GRIP_CLOSED = -0.02    # R1 close_angle
GRIP_CLOSE_THR = 0.1   # attach: ctrl 降到此值以下 (近全闭)
GRIP_RELEASE_THR = 0.6  # release: ctrl 升到此值以上 (张开中)

# 两指面测量函数来自 probe_grasp_geometry (同一模型结构, 可用于场景 model)
_spec = importlib.util.spec_from_file_location(
    "mtc_probe_grasp_geometry",
    Path(__file__).resolve().parent / "mtc_probe_grasp_geometry.py")
pgg = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pgg)

ARM_ACTUATOR_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
                      "wrist_flex", "wrist_roll"]


def load_pick_scene():
    model = mujoco.MjModel.from_xml_path(str(SCENE_PICK))
    data = mujoco.MjData(model)
    return model, data


class AttachController:
    """镜像 grasp_attach_plugin 的 kinematic attach 语义 (ctrl 阈值版)。"""

    def __init__(self, model, mode="ideal", ideal_rel_pose=None,
                 planned_cube=(0.117, -0.077, 0.01)):
        self.model = model
        self.mode = mode
        self.cube_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
        self.jaw_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,
                                         "moving_jaw_so101_v1")
        self.cube_jid = model.body_jntadr[self.cube_bid]
        self.qadr = model.jnt_qposadr[self.cube_jid]
        self.vadr = model.jnt_dofadr[self.cube_jid]
        self.attached = False
        self.attach_time = None
        self.release_time = None
        self.rel_samples = []  # [(t, rel_p_world, rel_q_world)]
        self.max_penetration = 0.0
        self.cube_size = 0.02
        if mode == "ideal":
            q = np.array(ideal_rel_pose[3:], dtype=float)
            q /= np.linalg.norm(q)
            self.rel_p = np.array(ideal_rel_pose[:3], dtype=float)
            self.rel_q = np.zeros(4)
            R9 = np.zeros(9)
            mujoco.mju_quat2Mat(R9, q)
            mujoco.mju_mat2Quat(self.rel_q, R9)  # 归一化后的四元数 -> 严格正交 R
        else:
            self.planned = np.asarray(planned_cube, dtype=float)

    def update(self, data, grip_ctrl):
        if self.attached:
            if grip_ctrl > GRIP_RELEASE_THR:
                self.attached = False
                self.release_time = data.time
                return
            self._write_attached(data)
            self._record(data)
        else:
            if grip_ctrl < GRIP_CLOSE_THR:
                self._capture(data)

    def _capture(self, data):
        self.attached = True
        self.attach_time = data.time
        if self.mode == "legacy":
            pj = data.xpos[self.jaw_bid]
            self.rel_w = self.planned - pj

    def _write_attached(self, data):
        pj = data.xpos[self.jaw_bid]
        if self.mode == "ideal":
            Rj = data.xmat[self.jaw_bid].reshape(3, 3)
            wp = pj + Rj @ self.rel_p
            wq = np.zeros(4)
            mujoco.mju_mulQuat(wq, data.xquat[self.jaw_bid], self.rel_q)
        else:
            wp = pj + self.rel_w
            wq = np.array([1.0, 0.0, 0.0, 0.0])
        qa = self.qadr
        data.qpos[qa:qa + 3] = wp
        data.qpos[qa + 3:qa + 7] = wq
        data.qvel[self.vadr:self.vadr + 6] = 0.0

    def _record(self, data):
        if self.rel_samples and data.time - self.rel_samples[-1][0] < 0.1:
            return
        pj = data.xpos[self.jaw_bid]
        rel_p = data.xpos[self.cube_bid] - pj
        self.rel_samples.append(
            (data.time, rel_p.copy(), data.xquat[self.cube_bid].copy()))

    def _scan_contacts(self, data):
        cube_g = self._cube_geoms()
        finger_g = self._finger_geoms()
        for i in range(data.ncon):
            c = data.contact[i]
            if ({c.geom1, c.geom2} & cube_g) and ({c.geom1, c.geom2} & finger_g):
                self.max_penetration = min(self.max_penetration, float(c.dist))

    def _cube_geoms(self):
        if not hasattr(self, "_cube_geoms_cache"):
            self._cube_geoms_cache = set(self._geoms_of_body(self.cube_bid))
        return self._cube_geoms_cache

    def _finger_geoms(self):
        if not hasattr(self, "_finger_geoms_cache"):
            gb = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
            self._finger_geoms_cache = set(self._geoms_of_body(gb)) | set(
                self._geoms_of_body(self.jaw_bid))
        return self._finger_geoms_cache

    def _geoms_of_body(self, bid):
        return [self.model.body_geomadr[bid] + i
                for i in range(self.model.body_geomnum[bid])]

    def center_error(self, data):
        """cube 中心与两指面中点偏差 (probe_grasp_geometry 实测函数)。"""
        faces = pgg._finger_face_positions(self.model, data)
        if faces is None:
            return float("inf")
        axis, fpt, mpt, _span = faces
        mid = 0.5 * (fpt + mpt)
        return float(np.linalg.norm(mid - data.xpos[self.cube_bid]))

    def penetration_now(self, data):
        """碰撞凸包接触深度 (信息性指标: 凸包向指间隙鼓出, 超近似)。"""
        cube_g = self._cube_geoms()
        finger_g = self._finger_geoms()
        best = 0.0
        for i in range(data.ncon):
            c = data.contact[i]
            if ({c.geom1, c.geom2} & cube_g) and ({c.geom1, c.geom2} & finger_g):
                best = min(best, float(c.dist))
        return best

    def visual_clearance(self, data):
        """视觉网格间隙: 两指面间距 (带中点切片) - cube 沿闭合轴投影宽度。

        > 0 即渲染网格无穿透 (cube 已由 attach 旋转至与 gripper 对齐,
        沿轴宽度 = cube 边长)。
        """
        faces = pgg._finger_face_positions(self.model, data)
        if faces is None:
            return float("-inf")
        axis, _fpt, _mpt, span = faces
        R_c = data.xmat[self.cube_bid].reshape(3, 3)
        half = 0.5 * self.cube_size
        extent = 2.0 * half * sum(abs(float(axis @ R_c[:, i])) for i in range(3))
        return float(span - extent)


def run_pick_place(ideal_rel_pose, mode="ideal", verbose=False):
    """完整 pick->place 仿真, 返回验收指标 dict。"""
    model, data = load_pick_scene()
    ctrl_idx = {n: i for i, n in enumerate(
        [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
         for i in range(model.nu)])}
    a = [ctrl_idx[n] for n in ARM_ACTUATOR_NAMES]
    g = ctrl_idx["gripper"]

    def arm_ctrl(vals):
        c = np.zeros(model.nu)
        for i, v in zip(a, vals):
            c[i] = v
        c[g] = GRIP_OPEN
        return c

    phases = [
        ("settle", 0.3, None, arm_ctrl(np.zeros(5))),
        ("to_pre", 1.2, None, arm_ctrl(PRE_JOINTS)),
        ("to_grasp", 0.8, None, arm_ctrl(GRASP_JOINTS)),
        ("close", 0.6, None, None),
        ("hold", 0.4, None, None),
        ("to_lift", 0.8, None, arm_ctrl(LIFT_JOINTS)),
        ("to_preplace", 1.2, None, arm_ctrl(PREPLACE_JOINTS)),
        ("to_place", 0.8, None, arm_ctrl(PLACE_JOINTS)),
        ("open", 0.4, None, None),
        ("settle_drop", 0.6, None, None),
        ("retreat", 0.8, None, arm_ctrl(PREPLACE_JOINTS)),
    ]

    controller = AttachController(model, mode=mode,
                                  ideal_rel_pose=ideal_rel_pose,
                                  planned_cube=(0.117, -0.077, 0.01))

    center_error_at_hold = None
    max_pen_at_hold = 0.0
    ctrl = arm_ctrl(np.zeros(5))  # home + open

    dt = model.opt.timestep
    for name, dur, c0, c1 in phases:
        # arm 相位保持相位开始时的 gripper 目标 (arm_ctrl 默认张开会把搬运段
        # 的 ctrl[g] 拉回 open -> 搬运中误释放, R3 修复)
        if c1 is not None and name not in ("close", "open"):
            c1 = np.array(c1)
            c1[g] = ctrl[g]
        steps = int(round(dur / dt))
        for k in range(steps):
            alpha = (k + 1) / steps
            if name == "close":
                ctrl[g] = GRIP_OPEN + alpha * (GRIP_CLOSED - GRIP_OPEN)
            elif name == "open":
                ctrl[g] = GRIP_CLOSED + alpha * (GRIP_OPEN - GRIP_CLOSED)
            elif c1 is not None:
                ctrl = (1 - alpha) * ctrl + alpha * c1
            data.ctrl[:] = ctrl  # 必须写入 data.ctrl, 局部数组不生效
            controller.update(data, ctrl[g])
            mujoco.mj_step(model, data)

        if name == "hold":
            center_error_at_hold = controller.center_error(data)
            max_pen_at_hold = controller.penetration_now(data)
            visual_clear_at_hold = controller.visual_clearance(data)

    final_p = data.xpos[controller.cube_bid].copy()
    return {
        "attach_time": controller.attach_time,
        "release_time": controller.release_time,
        "rel_samples": controller.rel_samples,
        "center_error": center_error_at_hold,
        "max_penetration": max_pen_at_hold,
        "visual_clearance": visual_clear_at_hold,
        "final_xy": (float(final_p[0]), float(final_p[1])),
        "final_z": float(final_p[2]),
    }


def main():
    import yaml
    cfg = yaml.safe_load((REPO / "src" / "so101_mtc_pick_place" / "config" /
                          "controllers_pick.yaml").read_text())
    rel = cfg["/**"]["ros__parameters"]["mujoco_plugins"]["grasp_attach"]["ideal_rel_pose"]
    out = run_pick_place(ideal_rel_pose=rel)
    print("=" * 60)
    print("attach 语义 spike (ideal 模式, 参数与 yaml 货值一致)")
    print("=" * 60)
    print(f"  attach @ t={out['attach_time']:.2f}s  release @ t={out['release_time']:.2f}s")
    print(f"  居中偏差 (两指面中点): {out['center_error']*1000:.1f} mm")
    print(f"  最大穿透: {out['max_penetration']*1000:.2f} mm")
    print(f"  落点 xy: ({out['final_xy'][0]:+.4f}, {out['final_xy'][1]:+.4f})"
          f"  z: {out['final_z']*1000:.1f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
