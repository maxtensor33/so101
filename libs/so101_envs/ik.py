"""pinocchio 数值 IK（workflow/plan/envs.md Phase 4）。

仅供脚本专家策略与未来任务空间接口使用；env 核心不依赖本模块。
DLS (damped least squares) + 关节限位裁剪 + 温启动。
5 臂关节求解（gripper 锁定），TCP = URDF gripper_frame_link。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin

TCP_FRAME = "gripper_frame_link"
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
DAMPING = 1e-4
MAX_STEP = 0.2  # 每次迭代 dq 上限 (rad)


class SO101IK:
    def __init__(self, urdf_path):
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = pin.Data(self.model)
        self.frame_id = self.model.getFrameId(TCP_FRAME)
        # 5 臂关节在 q 向量中的索引
        self.idx = [self.model.joints[self.model.getJointId(n)].idx_q for n in ARM_JOINTS]
        self.q_lower = np.array([self.model.lowerPositionLimit[i] for i in self.idx])
        self.q_upper = np.array([self.model.upperPositionLimit[i] for i in self.idx])

    def fk(self, q_arm):
        """5 臂关节角 -> (tcp_pos, R)。"""
        q = pin.neutral(self.model)
        for k, iq in enumerate(self.idx):
            q[iq] = q_arm[k]
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        oMf = self.data.oMf[self.frame_id]
        return oMf.translation.copy(), oMf.rotation.copy()

    def solve(self, q_init, target_pos, R_des=None, n_iter=100, tol=1e-3):
        """位置(+姿态) IK。R_des=None 保持 q_init 的 TCP 朝向 -> (q_arm, err)。"""
        q = np.clip(np.asarray(q_init, dtype=float), self.q_lower, self.q_upper)
        if R_des is None:
            _, R_des = self.fk(q)
            e_ori_full = True
        else:
            e_ori_full = False  # 5 DOF: 只锁 R_des z 轴方向, 绕轴旋转自由
        z_axis = R_des[:, 2]
        err = np.inf
        for _ in range(n_iter):
            pos, R = self.fk(q)
            e_ori = pin.log3(R_des @ R.T)
            if not e_ori_full:
                e_ori = e_ori - (e_ori @ z_axis) * z_axis
            e = np.concatenate([target_pos - pos, e_ori])
            err = float(np.linalg.norm(e[:3]))
            if err < tol:
                break
            J = self._jacobian(q)
            dq = J.T @ np.linalg.solve(J @ J.T + DAMPING * np.eye(6), e)
            dq = np.clip(dq, -MAX_STEP, MAX_STEP)
            q = np.clip(q + dq, self.q_lower, self.q_upper)
        return q, err

    def _jacobian(self, q_arm):
        q = pin.neutral(self.model)
        for k, iq in enumerate(self.idx):
            q[iq] = q_arm[k]
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J = pin.getFrameJacobian(self.model, self.data, self.frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return J[:, self.idx]
