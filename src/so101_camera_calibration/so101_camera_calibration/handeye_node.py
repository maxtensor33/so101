"""手眼标定节点 (workflow/plan/setup_camera_calibration.md Phase 3)。

采集: mode=auto（poses YAML → JTC 驱动 + 质量门控）/ manual（Enter 采样, 配合 leader 遥操）。
求解: 内参 YAML(K/D) + solvePnP(target→cam) + TF(gripper→base) → handeye_core 双解。
输出: outputs/calibration/handeye_<camera>.yaml（含 optical→link 换算与静态 TF 片段）。

真机前置: real:=true 底座（auto 模式需 controller 运行; manual 配合 leader 遥操）。
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
                  str(_REPO / "src/so101_teleop"),):
        if _p not in sys.path and Path(_p).is_dir():
            sys.path.insert(0, _p)  # 自举必须在 numpy/cv2 导入前

import threading

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from so101_camera_calibration.handeye_core import (
    consistency_ok,
    optical_to_link,
    solve_handeye,
    solve_variants,
)
from so101_camera_calibration.intrinsics import decode_image

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ARM_LIMITS = np.array([
    [-1.91986, 1.91986], [-1.91986, 1.91986], [-1.69, 1.69],
    [-1.65806, 1.65806], [-2.74385, 2.84121],
])
JOINTS6 = JOINTS + ["gripper"]
JOINT_LIMITS6 = np.array([
    [-1.91986, 1.91986], [-1.91986, 1.91986], [-1.69, 1.69],
    [-1.65806, 1.65806], [-2.74385, 2.84121], [-0.17453, 1.74533],
])


class HandeyeNode(Node):
    def __init__(self):
        super().__init__("handeye_calibration_node")
        self.declare_parameter("mode", "auto")  # auto | manual | leader
        self.declare_parameter("camera", "eye_in_hand")  # eye_in_hand(wrist) | eye_to_hand(top)
        self.declare_parameter("image_topic", "")
        self.declare_parameter("intrinsic_yaml", "")
        self.declare_parameter("poses_yaml", "")
        self.declare_parameter("output_dir", "outputs/calibration")
        self.declare_parameter("base_frame", "world")
        self.declare_parameter("gripper_frame", "gripper_frame_link")
        self.declare_parameter("settle_time", 1.5)
        self.declare_parameter("min_corners", 8)
        self.declare_parameter("max_reproj", 1.5)
        self.declare_parameter("leader_port", "/dev/ttyACM0")
        self.declare_parameter("leader_id", "L07252801")
        self.declare_parameter("leader_invert", "")  # 方向镜像关节, 探针实测后填（逗号分隔）
        self.declare_parameter("viser_port", 8017)

        self._mode = str(self.get_parameter("mode").value)
        self._camera = str(self.get_parameter("camera").value)
        img_topic = str(self.get_parameter("image_topic").value)
        if not img_topic:
            img_topic = "/real/wrist/image_raw" if self._camera == "eye_in_hand" else "/real/fixed/image_raw"
        self._intrinsic_yaml = str(self.get_parameter("intrinsic_yaml").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._gripper_frame = str(self.get_parameter("gripper_frame").value)
        self._settle = float(self.get_parameter("settle_time").value)
        self._min_corners = int(self.get_parameter("min_corners").value)
        self._max_reproj = float(self.get_parameter("max_reproj").value)
        self._out_dir = Path(str(self.get_parameter("output_dir").value))
        self._out_dir.mkdir(parents=True, exist_ok=True)

        # 板规格按模式自动选: eye_in_hand=大板(内参板规格), eye_to_hand=小板(手眼板规格);
        # 特殊安装仍可用参数覆盖
        if self._camera == "eye_in_hand":
            _sq, _mk, _dict = (8, 6), 0.025, "DICT_5X5_250"
        else:
            _sq, _mk, _dict = (4, 5), 0.015, "DICT_4X4_50"
        self.declare_parameter("squares_x", _sq[0])
        self.declare_parameter("squares_y", _sq[1])
        self.declare_parameter("square_length", _mk)
        self.declare_parameter("marker_length", _mk * 0.72)
        self.declare_parameter("aruco_dict", _dict)
        import cv2.aruco as aruco

        _d = str(self.get_parameter("aruco_dict").value)
        _attr = _d if _d.startswith("DICT_") else f"DICT_{_d}"
        self.board = cv2.aruco.CharucoBoard(
            (int(self.get_parameter("squares_x").value), int(self.get_parameter("squares_y").value)),
            float(self.get_parameter("square_length").value),
            float(self.get_parameter("marker_length").value),
            aruco.getPredefinedDictionary(getattr(aruco, _attr)),
        )
        self.board_obj = np.asarray(self.board.getChessboardCorners())
        K, D = self._load_intrinsics()
        self._K, self._D = K, D

        self._estop = False
        self._latest_bgr = None
        self._qpos = None
        self.samples = []  # (T_g2b, T_t2c, reproj, n_corners)

        self.create_subscription(Image, img_topic, self._on_img, 10)
        self.create_subscription(JointState, "joint_states", self._on_js, 10)
        self.create_subscription(Empty, "camera_calibration/estop", self._on_estop, 10)
        self.create_subscription(Empty, "camera_calibration/sample", self._on_sample, 10)
        self.create_subscription(Empty, "camera_calibration/solve", self._on_solve, 10)
        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, self)
        self.pub_arm = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.pub_gripper = self.create_publisher(
            JointTrajectory, "/gripper_controller/joint_trajectory", 10)

        self._init_viser()

        if self._mode == "auto":
            poses_path = str(self.get_parameter("poses_yaml").value)
            if not poses_path:
                from ament_index_python.packages import get_package_share_directory

                poses_path = str(Path(get_package_share_directory("so101_camera_calibration"))
                                 / "config" / "calibration_poses.yaml")
            self._poses = self._load_poses(poses_path)
            self._pose_idx = 0
            self._state = "move"  # move | settle | done
            self._state_t = self.get_clock().now()
            self._timer = self.create_timer(0.1, self._tick_auto)
            self.get_logger().info(f"auto mode: {len(self._poses)} poses")
        elif self._mode == "leader":
            self._init_leader()
            self._poses = []
            self._prev_target = None
            self._start_key_listener()
            self._timer = self.create_timer(0.05, self._tick_leader)  # 20Hz 遥操跟随
            self._load_samples()
            self.get_logger().info("leader mode: Enter=采样, q=求解保存, Ctrl-C=退出")
        else:
            self._poses = []
            self._start_key_listener()
            self._timer = self.create_timer(0.2, self._tick_manual)
            self._load_samples()
            self.get_logger().info("manual mode: Enter=采样, q=求解保存, Ctrl-C=退出")

    # ---------- 数据加载 ----------

    def _load_intrinsics(self):
        if not self._intrinsic_yaml:
            raise RuntimeError("必须传 intrinsic_yaml（先跑内参标定）")
        data = yaml.safe_load(Path(self._intrinsic_yaml).read_text())
        K = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        D = np.array(data["distortion_coefficients"]["data"], dtype=np.float64)
        return K, D

    def _load_poses(self, path):
        data = yaml.safe_load(Path(path).read_text())
        poses = np.array(data["poses"], dtype=np.float64)
        assert poses.shape[1] == 5, "每姿势 5 关节"
        return np.clip(poses, ARM_LIMITS[:, 0], ARM_LIMITS[:, 1])

    # ---------- 回调 ----------

    def _on_img(self, msg):
        try:
            self._latest_bgr = decode_image(str(msg.encoding), bytes(msg.data), msg.height, msg.width)
        except ValueError:
            pass

    def _on_js(self, msg):
        qmap = dict(zip(msg.name, msg.position))
        try:
            self._qpos = np.array([qmap[j] for j in JOINTS])
        except KeyError:
            pass

    def _on_estop(self, msg):
        self._estop = True
        self.get_logger().warn("estop! 停止驱动")

    def _on_sample(self, msg):
        self._sample_requested = True

    def _on_solve(self, msg):
        self._solve_requested = True

    def _start_key_listener(self):
        """ros2 launch 不转发 stdin 给子进程——pynput 全局 Enter 为主, stdin 兜底（rl_infer_node 惯例）。"""
        try:
            from pynput import keyboard

            self._kb_listener = keyboard.Listener(on_press=self._on_key)
            self._kb_listener.start()
        except Exception as exc:
            self.get_logger().warn(f"pynput 不可用（回退 stdin）: {exc}")
            self._kb_listener = None
        threading.Thread(target=self._stdin_loop, daemon=True).start()

    def _on_key(self, key):
        try:
            self.get_logger().info(f"key event: {key}", throttle_duration_sec=1.0)
            from pynput.keyboard import Key

            if key == Key.enter:
                self._sample_requested = True
            elif key == keyboard.KeyCode.from_char("q"):
                self._solve_requested = True
        except Exception:
            pass

    def _stdin_loop(self):
        try:
            for _ in sys.stdin:
                self._sample_requested = True
        except Exception:
            pass

    # ---------- 采样与检测 ----------

    def _detect(self, bgr):
        """ChArUco 检测 + solvePnP → (T_t2c, reproj, n_corners) 或 None。"""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.CharucoDetector(self.board)
        corners, ids, _, _ = detector.detectBoard(gray)
        if ids is None or len(ids) < self._min_corners:
            return None
        obj = self.board_obj[ids.ravel()].astype(np.float32)
        pts = corners.astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj, pts, self._K, self._D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None
        proj, _ = cv2.projectPoints(obj, rvec, tvec, self._K, self._D)
        reproj = float(np.sqrt(np.mean(np.sum((pts.reshape(-1, 2) - proj.reshape(-1, 2)) ** 2, axis=1))))
        if reproj > self._max_reproj:
            return None
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = tvec.ravel()
        return T, reproj, len(ids)

    def _lookup_g2b(self):
        try:
            tf = self._tf.lookup_transform(self._base_frame, self._gripper_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup 失败: {e}", throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        from scipy.spatial.transform import Rotation

        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def _try_sample(self) -> bool:
        """采样一次（质量门控）; 成功返回 True。"""
        if self._latest_bgr is None or self._qpos is None:
            self.get_logger().warn(
                f"采样拒绝: 无图像({self._latest_bgr is None})/无关节状态({self._qpos is None})",
                throttle_duration_sec=1.0)
            return False
        det = self._detect(self._latest_bgr)
        if det is None:
            self.get_logger().warn("采样拒绝: 检测不足/重投影超标", throttle_duration_sec=2.0)
            return False
        T_t2c, reproj, n = det
        T_g2b = self._lookup_g2b()
        if T_g2b is None:
            return False
        self.samples.append((T_g2b, T_t2c, reproj, n))
        self.get_logger().info(f"采样 {len(self.samples)}: 角点 {n}, 重投影 {reproj:.2f}px")
        self._save_samples()  # 每组即落盘, 任何崩溃都不丢样本
        return True

    def _send_arm(self, q5):
        self._send_joints(np.concatenate([q5, [0.5]]))

    def _send_joints(self, q6):
        # arm_controller 收 5 关节, gripper_controller 收 gripper——关节名必须与各自控制器一致
        for pub, names, idx in (
            (self.pub_arm, JOINTS, slice(0, 5)),
            (self.pub_gripper, ["gripper"], slice(5, 6)),
        ):
            traj = JointTrajectory()
            traj.joint_names = list(names)
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in q6[idx]]
            pt.time_from_start.nanosec = int(0.2 * 1e9)
            traj.points = [pt]
            pub.publish(traj)

    # ---------- auto 状态机 ----------

    def _tick_auto(self):
        if self._estop:
            return
        now = self.get_clock().now()
        if self._state == "move":
            if self._pose_idx >= len(self._poses):
                self._state = "done"
                self._solve_and_save()
                return
            self._send_arm(self._poses[self._pose_idx])
            self._state = "settle"
            self._state_t = now
        elif self._state == "settle":
            if (now - self._state_t).nanoseconds / 1e9 < self._settle:
                return
            self._try_sample()
            self._pose_idx += 1
            self._state = "move"

    def _tick_manual(self):
        if getattr(self, "_sample_requested", False):
            self._sample_requested = False
            self._try_sample()
        if getattr(self, "_solve_requested", False):
            self._solve_requested = False
            self._solve_and_save()

    # ---------- Viser 采样面板（:8017; 参考项目同款反馈, 去掉其 IK gizmo 依赖） ----------

    def _init_viser(self):
        import viser

        self._viser = viser.ViserServer(port=int(self.get_parameter("viser_port").value))
        self._ui_img = self._viser.gui.add_image(
            np.zeros((480, 640, 3), dtype=np.uint8), label="相机视图（绿框=检测到的 ChArUco）")
        self._ui_status = self._viser.gui.add_text("状态", "等待图像…")
        self._sample_requested = False
        self._solve_requested = False
        self._clear_requested = False
        self._viser.gui.add_button("📷 采样 (Enter)").on_click(
            lambda _: setattr(self, "_sample_requested", True))
        self._viser.gui.add_button("🧮 求解 (q)").on_click(
            lambda _: setattr(self, "_solve_requested", True))
        self._viser.gui.add_button("🗑 清空重采").on_click(
            lambda _: setattr(self, "_clear_requested", True))
        self._ui_timer = self.create_timer(0.5, self._tick_ui)
        self.get_logger().info("Viser 采样面板: http://localhost:8017")

    def _tick_ui(self):
        if self._latest_bgr is None:
            return
        view = self._latest_bgr.copy()
        status = f"模式={self._mode} 相机={self._camera} 已采样={len(self.samples)}"
        gray = cv2.cvtColor(view, cv2.COLOR_BGR2GRAY)
        corners, ids, _, _ = cv2.aruco.CharucoDetector(self.board).detectBoard(gray)
        if ids is not None and len(ids) >= 4:
            cv2.aruco.drawDetectedCornersCharuco(view, corners, ids, (0, 255, 0))
            n = len(ids)
            if n >= self._min_corners:
                ok, rvec, tvec = cv2.solvePnP(
                    self.board_obj[ids.ravel()].astype(np.float32), corners.astype(np.float32),
                    self._K, self._D, flags=cv2.SOLVEPNP_ITERATIVE)
                if ok:
                    proj, _ = cv2.projectPoints(
                        self.board_obj[ids.ravel()].astype(np.float32), rvec, tvec, self._K, self._D)
                    reproj = float(np.sqrt(np.mean(np.sum(
                        (corners.reshape(-1, 2) - proj.reshape(-1, 2)) ** 2, axis=1))))
                    status += (f" | 当前帧: 角点 {n}, 重投影 {reproj:.2f}px"
                               + (" ✓ 可采样" if reproj <= self._max_reproj
                                  else f" ✗ 超 {self._max_reproj}px（重投影差: 查内参/板规格是否匹配）"))
                else:
                    status += f" | 角点 {n}, solvePnP 失败"
            else:
                status += f" | 角点 {n} < {self._min_corners}（靠近些/补光/增大板在画面中占比）"
        else:
            status += " | 完全未检出: 光照/反光/板不在视野或太小"
        self._ui_img.image = view
        self._ui_status.value = status
        try:
            cv2.imwrite("/tmp/he_last_view.png", view)  # 最后帧落盘, 便于排查
        except Exception:
            pass

    def _ui_show_result(self, result: dict):
        try:
            self._viser.gui.add_text(
                "求解结果",
                "—— 求解结果 ——\n"
                f"PARK  t = {np.round(result['optical_frame']['t'], 4).tolist()}\n"
                f"HORAUD t = 见 YAML\n"
                f"一致性: {'✓ <1cm 可信' if result['consistent'] else '✗ ≥1cm, 建议补采'}\n"
                f"camera_link xyz = {np.round(result['camera_link']['xyz'], 4).tolist()}\n"
                f"结果文件含静态 TF 片段（终端已打印）")
        except Exception:
            pass

    # ---------- leader 遥操跟随 ----------

    def _init_leader(self):
        from so101_teleop.teleoperators.leader import (
            So101LeaderSimTeleop,
            So101LeaderSimTeleopConfig,
        )

        self._leader = So101LeaderSimTeleop(So101LeaderSimTeleopConfig(
            port=str(self.get_parameter("leader_port").value),
            id=str(self.get_parameter("leader_id").value),
            invert=[j.strip() for j in
                    str(self.get_parameter("leader_invert").value).split(",") if j.strip()],
            max_norm_step=0.15,  # 每拍限幅, 安全
        ))
        self._leader.connect()
        self.get_logger().info(
            f"leader 已连接: {self.get_parameter('leader_port').value} "
            f"(invert={self.get_parameter('leader_invert').value})")

    def _tick_leader(self):
        """leader 归一化关节 → follower 目标 rad → JTC（estop 即停）; Enter 触发采样。"""
        if self._estop:
            return
        if getattr(self, "_sample_requested", False):
            self._sample_requested = False
            self._try_sample()
        if getattr(self, "_solve_requested", False):
            self._solve_requested = False
            self._solve_and_save()
        action = self._leader.get_action()  # {j.pos: [-1,1]}（follower 坐标, 已平滑）
        norm = np.array([float(action[f"{j}.pos"]) for j in JOINTS6])
        target = JOINT_LIMITS6[:, 0] + (norm + 1.0) / 2.0 * (JOINT_LIMITS6[:, 1] - JOINT_LIMITS6[:, 0])
        if self._prev_target is not None:
            target = np.clip(target, self._prev_target - 0.1, self._prev_target + 0.1)
        target = np.clip(target, JOINT_LIMITS6[:, 0], JOINT_LIMITS6[:, 1])
        self._send_joints(target)

    # ---------- 求解与保存 ----------

    # ---------- 样本持久化 ----------

    def _samples_path(self) -> Path:
        return self._out_dir / f"samples_{self._camera}.npz"

    def _save_samples(self):
        arr_g = np.array([s[0] for s in self.samples])
        arr_t = np.array([s[1] for s in self.samples])
        np.savez(self._samples_path(), T_g2b=arr_g, T_t2c=arr_t,
                 reproj=np.array([s[2] for s in self.samples]),
                 n_corners=np.array([s[3] for s in self.samples]))
        self.get_logger().info(f"样本已保存: {self._samples_path()} ({len(self.samples)} 组)")

    def _load_samples(self):
        path = self._samples_path()
        if not path.is_file():
            return
        data = np.load(path)
        for g, t, r, n in zip(data["T_g2b"], data["T_t2c"],
                              data["reproj"], data["n_corners"]):
            self.samples.append((g, t, float(r), int(n)))
        self.get_logger().info(f"载入历史样本 {len(self.samples)} 组 ({path}); 面板清空按钮可重采")

    def _clear_samples(self):
        self.samples.clear()
        if self._samples_path().is_file():
            self._samples_path().unlink()
        self.get_logger().info("样本已清空, 重新采集")

    def _solve_and_save(self):
        n = len(self.samples)
        if n < 5:
            self.get_logger().error(f"样本不足 ({n}), 无法求解")
            return
        self._save_samples()
        T_g2b = [s[0] for s in self.samples]
        T_t2c = [s[1] for s in self.samples]
        # 权威解: TF/solvePnP 的文档约定（g2b 原始 × t2c 原始）; cv2 直接给出相机位姿。
        # 变体列表仅作诊断: 错误约定也能解出矩阵, 用板刚性区分（见 handeye_core）。
        out = solve_handeye(T_g2b, T_t2c, mode=self._camera)
        consistent = consistency_ok(out)
        T = out["PARK"]["T"]
        cands = solve_variants(T_g2b, T_t2c, mode=self._camera)
        self.get_logger().info("== 诊断候选（原始约定为权威解; 按板刚性排序）==")
        for c in cands:
            mark = "★" if c["name"].startswith("g2b×t2c 原始") and c["name"].endswith("/PARK") else " "
            self.get_logger().info(
                f"{mark} {c['name']}: t={np.round(c['t'], 3).tolist()} "
                f"板std={c['board_std'] * 1000:.1f}mm "
                f"({'物理合理' if c['plausible'] else '不合理'})")
        self.get_logger().info(
            f"PARK/HORAUD 平移差={np.linalg.norm(out['PARK']['t'] - out['HORAUD']['t']) * 1000:.2f}mm "
            f"({'✓ <1cm 可信' if consistent else '✗ ≥1cm, 建议补采'})")
        R = T[:3, :3]
        from scipy.spatial.transform import Rotation

        rpy_opt = Rotation.from_matrix(R).as_euler("xyz", degrees=True)
        xyz_link, rpy_link = optical_to_link(T[:3, 3], rpy_opt)
        # eye_in_hand 的位姿相对 gripper_frame_link（随臂动）; eye_to_hand 相对 base/world
        tf_parent = self._gripper_frame if self._camera == "eye_in_hand" else self._base_frame
        tf_snippet = (
            f'<node pkg="tf2_ros" exec="static_transform_publisher" args="'
            f'{xyz_link[0]:.4f} {xyz_link[1]:.4f} {xyz_link[2]:.4f} '
            f'{rpy_link[0]:.6f} {rpy_link[1]:.6f} {rpy_link[2]:.6f} '
            f'{tf_parent} camera_{self._camera}_link" />'
        )
        result = {
            "camera": self._camera,
            "method": "PARK/g2b×t2c 原始（权威约定）",
            "consistent": bool(consistent),
            "n_samples": n,
            "parent_frame": tf_parent,
            "reproj_per_sample": [s[2] for s in self.samples],
            "optical_frame": {"t": T[:3, 3].tolist(), "rpy_deg": rpy_opt.tolist()},
            "camera_link": {"xyz": xyz_link.tolist(), "rpy_deg": rpy_link.tolist()},
            "static_tf_snippet": tf_snippet,
            "candidates": [
                {"name": c["name"], "t": c["t"].tolist(),
                 "plausible": bool(c["plausible"]),
                 "board_std_m": float(c["board_std"])} for c in cands
            ],
        }
        # 样本先落盘（历史教训: YAML 写入失败不应连带丢样本）
        self._save_samples()
        path = self._out_dir / f"handeye_{self._camera}.yaml"
        try:
            path.write_text(yaml.safe_dump(result, allow_unicode=True, sort_keys=False))
            self.get_logger().info(f"已保存 {path}")
        except Exception as exc:
            self.get_logger().error(f"YAML 保存失败: {exc}")
        self.get_logger().info(f"静态 TF 片段:\n{tf_snippet}")
        self._ui_show_result(result)


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C = 纯退出不求解; 求解用 q 键或 /camera_calibration/solve 话题
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
