"""力矩控制器节点 (plan/robotics.md Phase 4)。

100 Hz 控制环: /joint_states 反馈 → τ → effort 命令。
订阅 /draw_cube/arm_target (Float64MultiArray, 布局 [q_d(5), dq_d(5), ddq_d(5)]);
无目标时纯重力补偿保持当前位姿。控制器经 ROS 参数选择 (pd_g | computed_torque)。
输出: /arm_controller/commands (5 effort), /gripper_controller/commands (0 保持)。
"""

import sys
from pathlib import Path


def _find_repo_root():
    """Walk up to the repo root (uv + colcon hybrid: so101_robotics lives in libs/)."""
    for parent in Path(__file__).resolve().parents:
        if (parent / "pyproject.toml").is_file():
            return parent
    return None


_REPO = _find_repo_root()
if _REPO is not None:
    for _p in (str(_REPO / "libs"), str(_REPO / ".venv/lib/python3.12/site-packages")):
        if _p not in sys.path and Path(_p).is_dir():
            sys.path.insert(0, _p)

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from so101_robotics import control, dynamics

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
KP = np.full(6, 50.0)  # 6-dof for the model; gripper slot padded with 0
KD = np.full(6, 5.0)


class TorqueControlNode(Node):
    def __init__(self):
        super().__init__("torque_control_node")
        self.declare_parameter("controller", "computed_torque")
        self.declare_parameter("kp", [50.0] * 5)
        self.declare_parameter("kd", [5.0] * 5)
        self.controller = self.get_parameter("controller").get_parameter_value().string_value
        kp = list(self.get_parameter("kp").value)
        kd = list(self.get_parameter("kd").value)
        self.KP = np.array([*kp, 0.0])  # 6-dof; gripper slot unused
        self.KD = np.array([*kd, 0.0])

        self.q = None
        self.dq = None
        self.target = None
        self.q_hold = None  # latched hold pose: pure g-comp is neutrally stable,
        # so regulate to a fixed pose instead of chasing the drifting current one

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Float64MultiArray, "/draw_cube/arm_target", self._on_target, 10)
        self.arm_cmd_pub = self.create_publisher(Float64MultiArray, "/arm_controller/commands", 10)
        self.gripper_cmd_pub = self.create_publisher(
            Float64MultiArray, "/gripper_controller/commands", 10
        )
        self.create_timer(0.01, self._on_timer)
        self.get_logger().info(f"torque_control_node ready (controller={self.controller})")

    def _on_joint_states(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        try:
            self.q = np.array([msg.position[idx[n]] for n in ARM_JOINTS])
            self.dq = np.array([msg.velocity[idx[n]] for n in ARM_JOINTS])
        except KeyError:
            return  # 等待包含全部关节的完整消息

    def _on_target(self, msg):
        d = np.asarray(msg.data, dtype=float)
        if d.size == 15:
            self.target = (d[0:5], d[5:10], d[10:15])
        elif d.size == 5:
            self.target = (d, np.zeros(5), np.zeros(5))

    def _compute_tau(self, q_d, dq_d, ddq_d):
        # 模型是 6 维 (含 gripper); arm 只控 5 维, gripper 位补 0
        q6 = np.concatenate([self.q, [0.0]])
        dq6 = np.concatenate([self.dq, [0.0]])
        qd6 = np.concatenate([q_d, [0.0]])
        dqd6 = np.concatenate([dq_d, [0.0]])
        ddqd6 = np.concatenate([ddq_d, [0.0]])
        if self.controller == "pd_g":
            tau = control.pd_gravity(q6, dq6, qd6, dqd6, self.KP, self.KD)
        else:
            tau = control.computed_torque(q6, dq6, qd6, dqd6, ddqd6, self.KP, self.KD)
        return tau[:5]

    def _on_timer(self):
        if self.q is None:
            return
        if self.target is None:
            if self.q_hold is None:
                self.q_hold = self.q.copy()
            q_d, dq_d, ddq_d = self.q_hold, np.zeros(5), np.zeros(5)
        else:
            q_d, dq_d, ddq_d = self.target
        tau = self._compute_tau(q_d, dq_d, ddq_d)
        arm = Float64MultiArray()
        arm.data = [float(x) for x in tau]
        self.arm_cmd_pub.publish(arm)
        g = Float64MultiArray()
        g.data = [0.0]
        self.gripper_cmd_pub.publish(g)


def main(args=None):
    rclpy.init(args=args)
    node = TorqueControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
