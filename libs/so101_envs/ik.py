"""pinocchio 数值 IK（workflow/plan/envs.md Phase 4）。

仅供脚本专家策略与未来任务空间接口使用；env 核心不依赖本模块。
DLS (damped least squares) + 关节限位裁剪 + 温启动。
5 臂关节求解（gripper 角作为已知参数），TCP = 两指尖中点（squint 同款）。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pinocchio as pin

FINGER1_LINK = "gripper_link"
FINGER2_LINK = "moving_jaw_so101_v1_link"
FINGER1_TIP_LOCAL = np.array([-0.002, 0.0, -0.092])
FINGER2_TIP_LOCAL = np.array([-0.01, -0.077, 0.02])
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
DAMPING = 1e-4
MAX_STEP = 0.2  # 每次迭代 dq 上限 (rad)


def _skew(r):
    return np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])


class SO101IK:
    def __init__(self, urdf_path):
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = pin.Data(self.model)
        self.f1_id = self.model.getFrameId(FINGER1_LINK)
        self.f2_id = self.model.getFrameId(FINGER2_LINK)
        self.gripper_idx = self.model.joints[self.model.getJointId("gripper")].idx_q
        # 5 臂关节在 q 向量中的索引
        self.idx = [self.model.joints[self.model.getJointId(n)].idx_q for n in ARM_JOINTS]
        self.q_lower = np.array([self.model.lowerPositionLimit[i] for i in self.idx])
        self.q_upper = np.array([self.model.upperPositionLimit[i] for i in self.idx])

    def _placements(self, q_arm, q_gripper):
        q = pin.neutral(self.model)
        for k, iq in enumerate(self.idx):
            q[iq] = q_arm[k]
        q[self.gripper_idx] = q_gripper
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.f1_id], self.data.oMf[self.f2_id], q

    def fk(self, q_arm, q_gripper=0.0):
        """5 臂关节角 + 夹爪角 -> (两指尖中点, gripper_link 朝向)。"""
        o1, o2, _ = self._placements(q_arm, q_gripper)
        p1 = o1.translation + o1.rotation @ FINGER1_TIP_LOCAL
        p2 = o2.translation + o2.rotation @ FINGER2_TIP_LOCAL
        return (p1 + p2) / 2.0, o1.rotation.copy()

    def solve(self, q_init, target_pos, R_des=None, n_iter=100, tol=1e-3,
              q_gripper=0.0, position_only=False):
        """位置(+姿态) IK（夹爪角固定）。

        position_only=True 时只约束位置（reach 任务不要求朝向）；
        否则 R_des=None 保持 q_init 的 TCP 朝向。
        """
        q = np.clip(np.asarray(q_init, dtype=float), self.q_lower, self.q_upper)
        if position_only:
            R_des = None
            e_ori_full = False
        elif R_des is None:
            _, R_des = self.fk(q, q_gripper)
            e_ori_full = True
        else:
            e_ori_full = False  # 5 DOF: 只锁 R_des z 轴方向, 绕轴旋转自由
        z_axis = R_des[:, 2] if R_des is not None else np.array([0.0, 0.0, 1.0])
        err = np.inf
        for _ in range(n_iter):
            pos, R = self.fk(q, q_gripper)
            if position_only:
                e = target_pos - pos
                err = float(np.linalg.norm(e))
                if err < tol:
                    break
                J = self._jacobian(q, q_gripper)[:3]
                dq = J.T @ np.linalg.solve(J @ J.T + DAMPING * np.eye(3), e)
            else:
                e_ori = pin.log3(R_des @ R.T)
                if not e_ori_full:
                    e_ori = e_ori - (e_ori @ z_axis) * z_axis
                e = np.concatenate([target_pos - pos, e_ori])
                err = float(np.linalg.norm(e[:3]))
                if err < tol:
                    break
                J = self._jacobian(q, q_gripper)
                dq = J.T @ np.linalg.solve(J @ J.T + DAMPING * np.eye(6), e)
            dq = np.clip(dq, -MAX_STEP, MAX_STEP)
            q = np.clip(q + dq, self.q_lower, self.q_upper)
        return q, err

    def _jacobian(self, q_arm, q_gripper):
        """两指尖中点 + gripper 朝向对 5 臂关节的 6x5 雅可比（夹爪关节列剔除）。"""
        o1, o2, q = self._placements(q_arm, q_gripper)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        J1 = pin.getFrameJacobian(self.model, self.data, self.f1_id,
                                  pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J2 = pin.getFrameJacobian(self.model, self.data, self.f2_id,
                                  pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        r1 = (o1.translation + o1.rotation @ FINGER1_TIP_LOCAL) - o1.translation
        r2 = (o2.translation + o2.rotation @ FINGER2_TIP_LOCAL) - o2.translation
        p1_lin = J1[:3] - _skew(r1) @ J1[3:]
        p2_lin = J2[:3] - _skew(r2) @ J2[3:]
        J = np.vstack([(p1_lin + p2_lin) / 2.0, J1[3:]])
        return J[:, self.idx]
