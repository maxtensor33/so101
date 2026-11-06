"""so101_camera_calibration 包测试 (workflow/plan/setup_camera_calibration.md, TDD)。

覆盖: ChArUco A4 板生成器（几何/可检测性）→ 内参节点检测纯函数 → 手眼换算函数。
契约风格对齐 tests/test_il_sim_develop.py。
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
PKG = REPO / "src" / "so101_camera_calibration"

if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))


class TestPackageLayout:
    def test_package_xml(self):
        root = ET.parse(PKG / "package.xml").getroot()
        assert root.findtext("name") == "so101_camera_calibration"
        assert "ament_python" in (root.findtext("export/build_type") or "")

    def test_scripts_exist(self):
        for name in ("gen_charuco_intrinsic.py", "gen_charuco_handeye.py"):
            assert (PKG / "scripts" / name).is_file(), f"缺生成器 {name}"

    def test_attribution_preserved(self):
        """尊重原项目: Apache-2.0 版权头保留 + 来源注明（需求 1）。"""
        for rel in ("scripts/gen_charuco_intrinsic.py", "scripts/gen_charuco_handeye.py"):
            text = (PKG / rel).read_text()
            assert "Apache License" in text or "Apache-2.0" in text, f"{rel} 缺许可头"
        readme = (PKG / "README.md").read_text()
        assert "so101-ros-physical-ai" in readme


BOARD_SPECS = {
    "intrinsic": dict(squares=(8, 6), square_mm=25.0, dict_name="DICT_5X5_250"),
    "handeye": dict(squares=(4, 5), square_mm=15.0, dict_name="DICT_4X4_50"),
}


@pytest.mark.parametrize("kind", ["intrinsic", "handeye"])
class TestBoardGenerators:
    def test_generate_outputs(self, kind, tmp_path):
        from so101_camera_calibration.board_gen import generate_board

        generate_board(kind, out_dir=tmp_path)
        assert (tmp_path / f"charuco_{kind}_A4.png").is_file()
        assert (tmp_path / f"charuco_{kind}_A4.pdf").is_file()
        assert (tmp_path / f"charuco_{kind}_A4.pdf").stat().st_size > 10_000

    def test_a4_page_size_at_300dpi(self, kind, tmp_path):
        import cv2

        from so101_camera_calibration.board_gen import DPI, generate_board

        generate_board(kind, out_dir=tmp_path)
        img = cv2.imread(str(tmp_path / f"charuco_{kind}_A4.png"))
        mm2px = DPI / 25.4
        page = {int(210 * mm2px), int(297 * mm2px)}
        assert img.shape[0] in page and img.shape[1] in page

    def test_charuco_corners_detectable(self, kind, tmp_path):
        """生成板必须能被 cv2 检出全部角点, 且相邻角点间距 = 方格边长（物理尺度锚定）。"""
        import cv2
        import cv2.aruco as aruco

        from so101_camera_calibration.board_gen import BOARD_CONFIGS, generate_board, make_board

        cfg = BOARD_CONFIGS[kind]
        generate_board(kind, out_dir=tmp_path)
        img = cv2.imread(str(tmp_path / f"charuco_{kind}_A4.png"))
        board = make_board(kind)
        detector = aruco.CharucoDetector(board)
        corners, ids, _, _ = detector.detectBoard(img)
        expected = (cfg["squares"][0] - 1) * (cfg["squares"][1] - 1)
        assert ids is not None and len(ids) == expected, (
            f"{kind}: 角点 {0 if ids is None else len(ids)}/{expected}")
        pts = corners.reshape(-1, 2)
        step_px = np.median(np.linalg.norm(np.diff(pts, axis=0), axis=1))
        step_mm = step_px * 25.4 / 300.0
        assert abs(step_mm - cfg["square_mm"]) < cfg["square_mm"] * 0.05, (
            f"{kind}: 角点间距 {step_mm:.2f}mm != {cfg['square_mm']}mm")

    def test_pdf_prints_actual_size(self, kind, tmp_path):
        """PDF MediaBox 必须 A4 (pt), 打印对话框才可选实际尺寸。"""
        from so101_camera_calibration.board_gen import generate_board

        generate_board(kind, out_dir=tmp_path)
        pdf = (tmp_path / f"charuco_{kind}_A4.pdf").read_bytes()
        assert b"MediaBox" in pdf
        assert b"595" in pdf and b"841" in pdf


class TestIntrinsics:
    """内参标定纯函数（intrinsics 模块, 无 ROS 依赖）+ 合成数据闭环。"""

    def test_attribution_node_file(self):
        text = (PKG / "so101_camera_calibration" / "camera_intrinsic_calibration_node.py").read_text()
        assert "Apache License" in text
        assert "so101-ros-physical-ai" in text  # 来源注明

    def test_resolve_aruco_dict(self):
        from so101_camera_calibration.intrinsics import resolve_aruco_dict

        d1 = resolve_aruco_dict("DICT_5X5_250")
        d2 = resolve_aruco_dict("5X5_250")
        assert d1.bytesList.shape == d2.bytesList.shape
        with pytest.raises(ValueError):
            resolve_aruco_dict("DICT_NOPE")

    def test_param_distance(self):
        from so101_camera_calibration.intrinsics import param_distance

        assert param_distance([0, 0, 0, 0], [1, 2, 3, 4]) == pytest.approx(10.0)

    def test_rgb8_decode(self):
        """rgb8 → BGR（红蓝通道互换）; bgr8 原样; 与 numpy2 栈无 cv_bridge。"""
        import numpy as np

        from so101_camera_calibration.intrinsics import decode_image

        rgb = np.zeros((4, 6, 3), dtype=np.uint8)
        rgb[..., 0] = 255  # 红色 (RGB)
        bgr = decode_image("rgb8", rgb.tobytes(), 4, 6)
        assert bgr.shape == (4, 6, 3) and bgr[..., 2].all() and not bgr[..., 0].any()
        bgr2 = decode_image("bgr8", rgb.tobytes(), 4, 6)
        assert bgr2[..., 0].all()

    def _synth_view(self, board, cfg, K, rvec, tvec, size=(640, 480)):
        """已知 K/位姿 → 透视变换合成标定板视图 → ChArUco 检测。"""
        import cv2
        import cv2.aruco as aruco

        from so101_camera_calibration.board_gen import DPI

        sx, sy = cfg["squares"]
        sq_m = cfg["square_mm"] / 1000.0
        mm2px = DPI / 25.4
        bw_px = int(sx * cfg["square_mm"] * mm2px)
        bh_px = int(sy * cfg["square_mm"] * mm2px)
        flat = board.generateImage((bw_px, bh_px), marginSize=0, borderBits=1)
        ppm_x, ppm_y = bw_px / (sx * sq_m), bh_px / (sy * sq_m)

        obj = np.asarray(board.getChessboardCorners(), dtype=np.float64)
        flat_pts = obj[:, :2] * np.array([ppm_x, ppm_y])
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, None)
        H, _ = cv2.findHomography(flat_pts.astype(np.float32), proj.reshape(-1, 2))
        warped = cv2.warpPerspective(flat, H, size)  # generateImage 产物为单通道灰度
        detector = aruco.CharucoDetector(board)
        corners, ids, _, _ = detector.detectBoard(warped)
        return corners, ids

    def test_board_params_synthetic_tilt(self):
        from so101_camera_calibration.intrinsics import board_params_charuco
        from so101_camera_calibration.board_gen import BOARD_CONFIGS, make_board

        board = make_board("intrinsic")
        K = np.array([[800.0, 0, 320], [0, 800.0, 240], [0, 0, 1]])
        corners, ids = self._synth_view(
            board, BOARD_CONFIGS["intrinsic"], K,
            rvec=np.array([0.25, -0.2, 0.1]), tvec=np.array([-0.05, -0.03, 0.55]))
        assert ids is not None and len(ids) >= 12
        params = board_params_charuco(corners, ids, (640, 480), 7, 5)
        assert params is not None and all(0.0 <= v <= 1.0 for v in params)
        assert params[2] > 0.1 and params[3] > 0.02  # 有尺寸且有倾斜

    def test_calibrate_recovers_known_K(self):
        """合成数据闭环: 已知 K 的 6 个视图 → calibrateCamera 恢复 K（±5%）, 重投影 <1px。"""
        from so101_camera_calibration.intrinsics import calibrate_charuco
        from so101_camera_calibration.board_gen import BOARD_CONFIGS, make_board

        board = make_board("intrinsic")
        K_true = np.array([[800.0, 0, 320], [0, 800.0, 240], [0, 0, 1]])
        all_c, all_i = [], []
        rng = np.random.default_rng(7)
        for _ in range(6):
            rvec = rng.uniform(-0.35, 0.35, 3)
            tvec = np.array([rng.uniform(-0.06, 0.06), rng.uniform(-0.04, 0.04), 0.55])
            corners, ids = self._synth_view(
                board, BOARD_CONFIGS["intrinsic"], K_true, rvec, tvec)
            assert ids is not None
            all_c.append(corners)
            all_i.append(ids)
        err, K, D, view_errors = calibrate_charuco(all_c, all_i, board, (640, 480))
        assert err < 1.0, f"重投影误差 {err:.3f}px"
        assert abs(K[0, 0] - 800) / 800 < 0.05 and abs(K[1, 1] - 800) / 800 < 0.05
        assert abs(K[0, 2] - 320) / 320 < 0.05 and abs(K[1, 2] - 240) / 240 < 0.05


class TestHandeye:
    """手眼标定核心: 合成数据闭环 + 一致性判定 + 光学系换算（无 ROS 依赖）。"""

    def _make_samples(self, X_true, mode, n=8, rng=None):
        """构造物理正确的采样集。

        X_true = 相机在标定父系中的位姿（T_父←相机）:
          eye_in_hand: T_gripper←cam（相机随臂, 板固定桌面）
          eye_to_hand: T_base←cam（相机固定, 板贴夹爪）
        观测 T_t2c = T_cam←board（solvePnP 输出）。
        """
        from scipy.spatial.transform import Rotation

        rng = rng or np.random.default_rng(3)
        T_b_board = np.eye(4)
        T_b_board[:3, 3] = [0.25, 0.0, 0.02]  # 板在基座前方桌面
        T_g_board = np.eye(4)                 # 板贴夹爪（TCP 前方）
        T_g_board[:3, :3] = Rotation.from_euler("xyz", [0.2, -0.3, 0.5]).as_matrix()
        T_g_board[:3, 3] = [0.0, 0.02, 0.08]
        T_g2b_list, T_t2c_list = [], []
        for _ in range(n):
            r = Rotation.from_euler("xyz", [rng.uniform(-0.5, 0.5), rng.uniform(-0.4, 0.4), rng.uniform(-1.2, 1.2)])
            t = np.array([rng.uniform(0.15, 0.3), rng.uniform(-0.12, 0.12), rng.uniform(0.15, 0.35)])
            T_g2b = np.eye(4)
            T_g2b[:3, :3] = r.as_matrix()
            T_g2b[:3, 3] = t
            if mode == "eye_in_hand":
                T_b_cam = T_g2b @ X_true                       # 相机随夹爪
                T_c_board = np.linalg.inv(T_b_cam) @ T_b_board
            else:
                T_c_board = np.linalg.inv(X_true) @ T_g2b @ T_g_board
            T_g2b_list.append(T_g2b)
            T_t2c_list.append(T_c_board)
        return T_g2b_list, T_t2c_list

    @staticmethod
    def _check_recovers(out, X_true):
        from scipy.spatial.transform import Rotation

        for method in ("PARK", "HORAUD"):
            t_err = np.linalg.norm(out[method]["t"] - X_true[:3, 3])
            assert t_err < 0.005, f"{method}: 平移误差 {t_err*1000:.1f}mm"
            angle = Rotation.from_matrix(out[method]["R"] @ X_true[:3, :3].T).magnitude()
            assert angle < 0.01, f"{method}: 旋转误差 {np.degrees(angle):.2f}°"

    def test_synthetic_eye_in_hand(self):
        """合成闭环: 已知相机在夹爪系位姿 → PARK/HORAUD 双解均恢复。"""
        from scipy.spatial.transform import Rotation

        from so101_camera_calibration.handeye_core import solve_handeye

        rng = np.random.default_rng(11)
        X_true = np.eye(4)
        X_true[:3, :3] = Rotation.from_euler("xyz", [0.1, -2.9, 0.2]).as_matrix()
        X_true[:3, 3] = [0.03, -0.02, 0.06]
        T_g2b, T_t2c = self._make_samples(X_true, "eye_in_hand", n=10, rng=rng)
        out = solve_handeye(T_g2b, T_t2c, mode="eye_in_hand")
        self._check_recovers(out, X_true)

    def test_synthetic_eye_to_hand(self):
        from scipy.spatial.transform import Rotation

        from so101_camera_calibration.handeye_core import solve_handeye

        rng = np.random.default_rng(13)
        X_true = np.eye(4)
        X_true[:3, :3] = Rotation.from_euler("xyz", [-0.2, 1.45, 0.3]).as_matrix()
        X_true[:3, 3] = [0.55, 0.1, 0.75]
        T_g2b, T_t2c = self._make_samples(X_true, "eye_to_hand", n=10, rng=rng)
        out = solve_handeye(T_g2b, T_t2c, mode="eye_to_hand")
        self._check_recovers(out, X_true)

    @pytest.mark.parametrize("mode", ["eye_in_hand", "eye_to_hand"])
    def test_variants_prefer_documented_convention(self, mode):
        """板刚性排序必须选中 TF/solvePnP 文档约定（g2b 原始 × t2c 原始）。"""
        from scipy.spatial.transform import Rotation

        from so101_camera_calibration.handeye_core import solve_variants

        X_true = np.eye(4)
        X_true[:3, :3] = Rotation.from_euler("xyz", [0.2, -0.4, 0.7]).as_matrix()
        X_true[:3, 3] = [0.03, -0.02, 0.06] if mode == "eye_in_hand" else [0.55, 0.1, 0.75]
        T_g2b, T_t2c = self._make_samples(
            X_true, mode, n=10, rng=np.random.default_rng(17))
        cands = solve_variants(T_g2b, T_t2c, mode=mode)
        assert cands[0]["name"].startswith("g2b×t2c 原始")
        assert cands[0]["board_std"] < 1e-6

    def test_consistency_check(self):
        from so101_camera_calibration.handeye_core import consistency_ok, solve_handeye

        rng = np.random.default_rng(5)
        X_true = np.eye(4)
        X_true[:3, 3] = [0.05, 0.0, 0.04]
        T_g2b, T_t2c = self._make_samples(X_true, "eye_in_hand", n=8, rng=rng)
        out = solve_handeye(T_g2b, T_t2c, mode="eye_in_hand")
        assert consistency_ok(out, tol_m=0.01)
        # 篡改 HORAUD 平移 5cm → 不一致
        out["HORAUD"]["t"] = out["HORAUD"]["t"] + np.array([0.05, 0, 0])
        assert not consistency_ok(out, tol_m=0.01)

    def test_optical_to_link_matches_reference(self):
        """换算函数与参考 README 数值对齐: xyz 不变（T_opt 零平移）, rpy 变化。"""
        from so101_camera_calibration.handeye_core import optical_to_link

        t = [0.1752, 0.0255, 0.5618]
        rpy_deg = [-176.0, -1.0, -87.2]
        xyz, rpy = optical_to_link(t, rpy_deg)
        assert np.allclose(xyz, t, atol=1e-6)
        # camera_link pitch 接近 90°（gimbal lock 区, 参考项目实测 ~86°）
        assert 80 < abs(rpy[1]) < 100


class TestEnvsCameraCalib:
    """标定结果 → envs 相机定制（Phase 4）: 汇总 YAML → model 相机字段覆写。"""

    def _calib_yaml(self, tmp_path):
        import yaml

        calib = {
            "fixed": {"pos": [0.7, -0.6, 0.8], "quat_wxyz": [0.5, 0.5, 0.5, 0.5], "fovy": 55.0},
            "wrist": {"pos": [0.04, -0.045, -0.02], "quat_wxyz": [0.866, 0.0, 0.5, 0.0], "fovy": 71.0},
        }
        path = tmp_path / "envs_camera_calib.yaml"
        path.write_text(yaml.safe_dump(calib))
        return path, calib

    def test_fovy_from_fy(self):
        from so101_envs.camera_calib import fovy_from_fy

        # fy=160px, H=128 → fovy = 2*atan(128/(2*160)) = 2*atan(0.4) ≈ 43.6°
        assert fovy_from_fy(160.0, 128) == pytest.approx(43.6, abs=0.1)

    def test_apply_overrides_model(self, tmp_path):
        import mujoco
        import so101_envs

        from so101_envs.camera_calib import apply_camera_calib

        path, calib = self._calib_yaml(tmp_path)
        env = so101_envs.make_env_for_test(camera_calib=str(path))
        cid_fixed = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, "fixed_cam")
        cid_wrist = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_CAMERA, "wrist_cam")
        assert np.allclose(env.model.cam_pos[cid_fixed], calib["fixed"]["pos"])
        assert np.allclose(env.model.cam_quat[cid_fixed], calib["fixed"]["quat_wxyz"])
        assert env.model.cam_fovy[cid_fixed] == pytest.approx(55.0)
        assert np.allclose(env.model.cam_pos[cid_wrist], calib["wrist"]["pos"])
        assert env.model.cam_fovy[cid_wrist] == pytest.approx(71.0)

    def test_none_keeps_default(self, tmp_path):
        """camera_calib=None 与现行为逐字段一致（回归保证）。"""
        import mujoco
        import so101_envs

        from so101_envs.camera_calib import apply_camera_calib

        e0 = so101_envs.make_env_for_test()
        e1 = so101_envs.make_env_for_test(camera_calib=None)
        for name in ("fixed_cam", "wrist_cam"):
            c0 = mujoco.mj_name2id(e0.model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            c1 = mujoco.mj_name2id(e1.model, mujoco.mjtObj.mjOBJ_CAMERA, name)
            assert np.allclose(e0.model.cam_pos[c0], e1.model.cam_pos[c1])
            assert np.allclose(e0.model.cam_quat[c0], e1.model.cam_quat[c1])
            assert e0.model.cam_fovy[c0] == e1.model.cam_fovy[c1]

    def test_load_validates_keys(self, tmp_path):
        import yaml

        from so101_envs.camera_calib import load_camera_calib

        bad = tmp_path / "bad.yaml"
        bad.write_text(yaml.safe_dump({"fixed": {}}))
        with pytest.raises((KeyError, ValueError)):
            load_camera_calib(str(bad))


class TestExportEnvsCalib:
    """标定 YAML → envs 汇总 YAML 桥接（Phase 4）。"""

    def _inputs(self, tmp_path, camera):
        import yaml

        h = {"camera": camera,
             "optical_frame": {"t": [0.55, 0.1, 0.75], "rpy_deg": [0.0, 0.0, 0.0]}}
        hy = tmp_path / f"handeye_{camera}.yaml"
        hy.write_text(yaml.safe_dump(h))
        iy = tmp_path / "camera_cal.yaml"
        iy.write_text(yaml.safe_dump({
            "camera_matrix": {"data": [160.0, 0, 64, 0, 160.0, 64, 0, 0, 1]},
            "height": 128, "width": 128,
        }))
        return hy, iy

    def test_export_entry(self, tmp_path):
        from scripts.export_envs_camera_calib import export_entry

        hy, iy = self._inputs(tmp_path, "eye_to_hand")
        out = export_entry(hy, iy)
        assert "fixed" in out  # eye_to_hand → fixed_cam
        e = out["fixed"]
        assert e["pos"] == [0.55, 0.1, 0.75]
        # rpy=0 → R_opt=I → R_mj=diag(1,-1,-1) → 180°绕x → quat wxyz=(0,1,0,0)
        assert np.allclose(e["quat_wxyz"], [0, 1, 0, 0], atol=1e-9)
        # fy=160, H=128 → fovy≈43.6
        assert e["fovy"] == pytest.approx(43.6, abs=0.1)

    def test_export_maps_wrist(self, tmp_path):
        sys.path.insert(0, str(PKG / "scripts"))
        from export_envs_camera_calib import export_entry  # noqa: E402

        hy, iy = self._inputs(tmp_path, "eye_in_hand")
        out = export_entry(hy, iy)
        assert "wrist" in out

    def test_export_quat_uses_right_multiplication(self, tmp_path):
        """回归: R_mj = R_opt @ CONV（右乘）。rpy=0 两种顺序相同, 必须用非对称旋转区分。"""
        import yaml
        from scipy.spatial.transform import Rotation

        sys.path.insert(0, str(PKG / "scripts"))
        from export_envs_camera_calib import CONV, export_entry  # noqa: E402

        rpy = [10.0, 20.0, 30.0]
        hy = tmp_path / "handeye_eye_to_hand.yaml"
        hy.write_text(yaml.safe_dump({
            "camera": "eye_to_hand",
            "optical_frame": {"t": [0.55, 0.1, 0.75], "rpy_deg": rpy},
        }))
        iy = tmp_path / "camera_cal.yaml"
        iy.write_text(yaml.safe_dump({
            "camera_matrix": {"data": [160.0, 0, 64, 0, 160.0, 64, 0, 0, 1]},
            "height": 128, "width": 128,
        }))
        out = export_entry(hy, iy)
        R_opt = Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()
        expect = Rotation.from_matrix(R_opt @ CONV).as_quat()[[3, 0, 1, 2]]
        assert np.allclose(out["fixed"]["quat_wxyz"], expect, atol=1e-12)

    def test_gripper_tcp_constant_matches_urdf(self):
        """wrist 父系换算常量必须与 URDF gripper_frame_joint 一致。"""
        sys.path.insert(0, str(PKG / "scripts"))
        from export_envs_camera_calib import T_GRIPPER_LINK_FROM_FRAME  # noqa: E402
        from scipy.spatial.transform import Rotation

        root = ET.parse(REPO / "src/so101_description/urdf/so101.urdf").getroot()
        joint = next(j for j in root.findall("joint") if j.get("name") == "gripper_frame_joint")
        origin = joint.find("origin")
        xyz = [float(v) for v in origin.get("xyz").split()]
        rpy = [float(v) for v in origin.get("rpy").split()]
        assert np.allclose(T_GRIPPER_LINK_FROM_FRAME[:3, 3], xyz, atol=1e-9)
        assert np.allclose(
            T_GRIPPER_LINK_FROM_FRAME[:3, :3],
            Rotation.from_euler("xyz", rpy).as_matrix(), atol=1e-5)

    def test_export_wrist_composes_tcp_to_body(self, tmp_path):
        """回归: wrist 手眼结果是相对 gripper_frame_link, 必须换到 wrist_cam 的父 body。"""
        from scipy.spatial.transform import Rotation

        sys.path.insert(0, str(PKG / "scripts"))
        from export_envs_camera_calib import (  # noqa: E402
            CONV, T_GRIPPER_LINK_FROM_FRAME, export_entry)
        import yaml

        hy = tmp_path / "handeye_eye_in_hand.yaml"
        hy.write_text(yaml.safe_dump({
            "camera": "eye_in_hand",
            "optical_frame": {"t": [0.0, 0.0, 0.0], "rpy_deg": [0.0, 0.0, 0.0]},
        }))
        iy = tmp_path / "camera_cal.yaml"
        iy.write_text(yaml.safe_dump({
            "camera_matrix": {"data": [160.0, 0, 64, 0, 160.0, 64, 0, 0, 1]},
            "height": 128, "width": 128,
        }))
        out = export_entry(hy, iy)["wrist"]
        assert np.allclose(out["pos"], T_GRIPPER_LINK_FROM_FRAME[:3, 3], atol=1e-12)
        R_link_cam = T_GRIPPER_LINK_FROM_FRAME[:3, :3]  # R_opt=I
        expect = Rotation.from_matrix(R_link_cam @ CONV).as_quat()[[3, 0, 1, 2]]
        assert np.allclose(out["quat_wxyz"], expect, atol=1e-12)

    def test_render_changes_with_calib(self, tmp_path):
        """集成: 标定覆写后 fixed_cam 渲染应与默认不同（EGL headless）。"""
        import so101_envs

        from so101_envs.camera_calib import apply_camera_calib

        e0 = so101_envs.make_env_for_test()
        e0.reset(seed=0)
        img0 = e0._render_camera("fixed_cam")
        e1 = so101_envs.make_env_for_test()
        e1.reset(seed=0)
        apply_camera_calib(e1.model, {
            "fixed": {"pos": [0.3, 0.0, 0.5], "quat_wxyz": [0.0, 1.0, 0.0, 0.0], "fovy": 80.0},
            "wrist": {"pos": [0.04, -0.045, -0.02], "quat_wxyz": [0.866, 0.0, 0.5, 0.0], "fovy": 71.0},
        })
        img1 = e1._render_camera("fixed_cam")
        assert np.abs(img0.astype(int) - img1.astype(int)).mean() > 5.0
