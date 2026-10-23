"""sim 模块: MuJoCo 仿真 wrapper (workflow/plan/robotics.md Phase 3, 对标 lab-2 mujoco_sim.py)。"""

import mujoco
import numpy as np

from . import common, constraints


class So101Sim:
    """力矩版 MJCF 独立仿真 wrapper (算法层闭环验证用, 不依赖 ROS)。"""

    def __init__(self, xml_path=None):
        if xml_path is None:
            xml_path = common.DEFAULT_XML
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self._joint_adrs = None

    def _qpos_adrs(self):
        if self._joint_adrs is None:
            self._joint_adrs = [
                self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                for n in common.JOINT_NAMES
            ]
        return self._joint_adrs

    def reset(self, q=None):
        mujoco.mj_resetData(self.model, self.data)
        if q is not None:
            for k, adr in enumerate(self._qpos_adrs()):
                self.data.qpos[adr] = q[k]
        mujoco.mj_forward(self.model, self.data)

    def step(self, tau, n_substeps=1):
        """设置力矩并推进仿真 (默认力矩饱和在 ±3.35 Nm)。"""
        self.data.ctrl[:] = constraints.saturate_torque(tau)
        for _ in range(n_substeps):
            mujoco.mj_step(self.model, self.data)
        return self.q, self.dq, self.time

    @property
    def q(self):
        return self.data.qpos[:6].copy()

    @property
    def time(self):
        return self.data.time

    @property
    def dq(self):
        return self.data.qvel[:6].copy()

    def ee_pos(self):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, common.EE_BODY)
        return self.data.xpos[bid].copy()
