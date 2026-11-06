"""sim2real 视野对照节点 (标定验收工具)。

leader 臂同时驱动: ① envs lift 仿真环境（标定后相机）② 真机 follower（JTC 跟随）。
RViz 四窗格: /compare/sim/{fixed,wrist} + /real/{fixed,wrist}/image_raw。
"""

import sys
from pathlib import Path


def _find_repo_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "pyproject.toml").is_file():
            return parent
    return None


_REPO = _find_repo_root()
if _REPO is not None:
    for _p in (str(_REPO / ".venv/lib/python3.12/site-packages"),
               str(_REPO / "libs"),
               str(_REPO / "src/so101_teleop")):
        if _p not in sys.path and Path(_p).is_dir():
            sys.path.insert(0, _p)

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
JOINTS6 = JOINTS + ["gripper"]
LIMITS6 = np.array([
    [-1.91986, 1.91986], [-1.91986, 1.91986], [-1.69, 1.69],
    [-1.65806, 1.65806], [-2.74385, 2.84121], [-0.17453, 1.74533],
])


class CompareViewsNode(Node):
    def __init__(self):
        super().__init__("compare_views_node")
        self.declare_parameter("env", "lift")
        self.declare_parameter("camera_calib", "outputs/calibration/envs_camera_calib.yaml")
        self.declare_parameter("leader_port", "/dev/ttyACM0")
        self.declare_parameter("leader_id", "L07252801")
        self.declare_parameter("leader_invert", "")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("mujoco", True)  # 弹 MuJoCo viewer 窗口（拖动 leader 同步观看）
        env_id = str(self.get_parameter("env").value)
        calib = str(self.get_parameter("camera_calib").value)
        show_mj = bool(self.get_parameter("mujoco").value)

        import gymnasium as gym
        import so101_envs  # noqa: F401

        self._env = gym.make(f"SO101{env_id.capitalize()}-v1", camera=True, camera_calib=calib,
                             render_size=(480, 640),  # 实机分辨率, 视野对比同尺寸
                             render_mode="human" if show_mj else None)
        self._env.reset(seed=0)
        self._u = self._env.unwrapped
        if show_mj:
            self._env.render()  # 打开 MuJoCo viewer 并同步首帧
        self.get_logger().info(f"sim env: SO101{env_id.capitalize()}-v1 (camera_calib={calib}, mujoco={show_mj})")

        from so101_teleop.teleoperators.leader import (
            So101LeaderSimTeleop,
            So101LeaderSimTeleopConfig,
        )

        self._leader = So101LeaderSimTeleop(So101LeaderSimTeleopConfig(
            port=str(self.get_parameter("leader_port").value),
            id=str(self.get_parameter("leader_id").value),
            invert=[j.strip() for j in
                    str(self.get_parameter("leader_invert").value).split(",") if j.strip()],
            max_norm_step=0.15,
        ))
        self._leader.connect()
        self.get_logger().info("leader 已连接（拖动即同步驱动 sim+real）")

        self._pub_sim = {k: self.create_publisher(Image, f"/compare/sim/{k}", 10)
                         for k in ("fixed", "wrist")}
        self._prev = None
        self.create_subscription(JointState, "joint_states", lambda m: None, 10)

        rate = float(self.get_parameter("rate").value)
        self.create_timer(1.0 / rate, self.tick)

    def _publish_sim(self, name, img_rgb):
        msg = Image()
        msg.height, msg.width = img_rgb.shape[:2]
        msg.encoding = "rgb8"
        msg.step = 3 * msg.width
        msg.data = img_rgb.tobytes()
        self._pub_sim[name].publish(msg)

    def tick(self):
        action = self._leader.get_action()
        norm = np.array([float(action[f"{j}.pos"]) for j in JOINTS6])
        # ① sim: envs 动作空间就是 [-1,1] 归一化位置目标
        self._env.step(norm)
        if self._env.render_mode == "human":
            self._env.render()  # 同步 MuJoCo 窗口
        for k in ("fixed", "wrist"):
            self._publish_sim(k, self._u._render_camera(f"{k}_cam"))
        # ② real: JTC 跟随（无订阅者时无害）
        target = LIMITS6[:, 0] + (norm + 1.0) / 2.0 * (LIMITS6[:, 1] - LIMITS6[:, 0])
        if self._prev is not None:
            target = np.clip(target, self._prev - 0.1, self._prev + 0.1)
        target = np.clip(target, LIMITS6[:, 0], LIMITS6[:, 1])
        self._prev = target
        for pub, names, idx in (
            (self.pub_arm, JOINTS, slice(0, 5)),
            (self.pub_gripper, ["gripper"], slice(5, 6)),
        ):
            traj = JointTrajectory()
            traj.joint_names = list(names)
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in target[idx]]
            pt.time_from_start.nanosec = int(0.2 * 1e9)
            traj.points = [pt]
            pub.publish(traj)

    def init_pubs(self):
        self.pub_arm = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.pub_gripper = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10)


def main(args=None):
    rclpy.init(args=args)
    node = CompareViewsNode()
    node.init_pubs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
