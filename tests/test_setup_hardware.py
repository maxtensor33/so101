"""so101_hardware 包结构测试 (workflow/plan/setup_hardware.md Phase 1)。

契约: 实机插件包 = ament_cmake C++ SystemInterface + vendor SCServo SDK。
"""

import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1] / "src" / "so101_hardware"
SDK = PKG / "scservo_sdk"
CPP = PKG / "src" / "so101_system.cpp"
HPP = PKG / "include" / "so101_hardware" / "so101_system.hpp"

JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


class TestPackageLayout:
    def test_package_xml(self):
        root = ET.parse(PKG / "package.xml").getroot()
        assert root.findtext("name") == "so101_hardware"
        assert root.findtext("export/build_type") == "ament_cmake"
        deps = {d.text for d in root.iter() if d.tag in ("depend", "exec_depend", "build_depend")}
        assert {"hardware_interface", "pluginlib", "rclcpp", "rclcpp_lifecycle"} <= deps

    def test_plugin_registration(self):
        root = ET.parse(PKG / "so101_hardware.xml").getroot()
        classes = {c.get("name"): c.get("type") for c in root.iter("class")}
        assert "so101_hardware/SO101SystemHardware" in classes
        assert classes["so101_hardware/SO101SystemHardware"] == "so101_hardware::SO101SystemHardware"

    def test_cmake_exports_plugin_and_builds_sdk(self):
        text = (PKG / "CMakeLists.txt").read_text()
        assert "pluginlib_export_plugin_description_file(hardware_interface so101_hardware.xml)" in text
        # vendor SDK 源码参与编译
        for src in ["SCSerial.cpp", "SCS.cpp", "SMS_STS.cpp"]:
            assert src in text
        assert "so101_system.cpp" in text
        assert "cxx_std_17" in text

    def test_sdk_files_vendored(self):
        for f in ["SCSerial.h", "SCSerial.cpp", "SCS.h", "SCS.cpp", "SMS_STS.h", "SMS_STS.cpp"]:
            assert (SDK / f).is_file(), f"缺少 SDK 文件: {f}"

    def test_header_exists(self):
        text = (PKG / "include" / "so101_hardware" / "so101_system.hpp").read_text()
        assert "class SO101SystemHardware : public hardware_interface::SystemInterface" in text


class TestSystemCpp:
    def test_calibration_required(self):
        """D4: 标定文件必填 — calibration_file 为空时 on_configure 报 ERROR。"""
        text = CPP.read_text()
        assert "calibration_file_.empty()" in text

    def test_no_hardcoded_motor_ids(self):
        """D4: 电机 ID 不再硬编码, 由标定 YAML 的 joint 键驱动。"""
        text = CPP.read_text()
        assert 'motor_ids_["shoulder_pan"] = 1' not in text

    def test_limits_match_repo_urdf(self):
        """D2: rad↔raw 映射限位 = 本仓库 URDF 限位, 拒绝参考项目数值。"""
        text = CPP.read_text()
        # 本仓库特有数值必须出现
        assert "-1.69" in text      # elbow_flex lower/upper
        assert "2.84121" in text    # wrist_roll upper (参考项目是 ±2.79253)
        assert "-2.74385" in text   # wrist_roll lower
        assert "1.74533" in text    # gripper upper (参考项目 1.4483)
        # 参考项目限位特征必须消失 (elbow 上下限对 + gripper upper)
        assert "urdf_upper = 1.5708" not in text
        assert "1.4483" not in text

    def test_limits_keyed_by_joint_name(self):
        """D2: 映射按 joint 名索引 (而非 calib.id), 电机 ID 来自标定。"""
        text = CPP.read_text()
        assert "calib.id ==" not in text

    def test_initial_move_lock_preserved(self):
        """D3: 可选 initial_positions_file 时保留 initial-move 锁 read 机制。"""
        text = CPP.read_text()
        assert "initial_move_cycles_remaining_" in text
        assert "needs_initial_move_" in text

    def test_sync_write_and_speeds(self):
        """write() 用 SyncWritePosEx 一次写全部电机; initial-move 低速 1000 / 正常 2400。"""
        text = CPP.read_text()
        assert "SyncWritePosEx" in text
        assert "2400" in text
        assert "115200" in text

    def test_plugin_export_macro(self):
        assert "PLUGINLIB_EXPORT_CLASS" in CPP.read_text()

    def test_activate_fails_loudly_on_read_error(self):
        """激活时任一关节读失败必须 ERROR, 不得静默保持零位
        (2026-10-29 教训: 连错串口时 6 关节 FeedBack 全超时,
        hw_positions_ 保持 on_configure 的 0 → RViz 显示 home 假象)。"""
        text = CPP.read_text()
        assert "Failed to read joint" in text
        # 读失败计数检查: 读到的关节数 != 总数 → 拒绝激活
        assert "read_ok != info_.joints.size()" in text

    def test_read_warns_throttled_on_error(self):
        """运行期 read 失败要打节流告警 (100Hz 下不能刷屏)。"""
        assert "RCLCPP_WARN_THROTTLE" in CPP.read_text()

    def test_deactivate_parks_to_start_pose(self):
        """退出保护: on_deactivate 分步缓速回启动位姿, 严禁一步猛冲。"""
        text = CPP.read_text()
        # 记录激活位姿
        assert "start_pose_" in text
        # on_deactivate 里插值回程 (多步), 不是直接写目标
        assert "parkToStart" in text
        # 回程用低速 + SyncWrite
        assert "kParkSpeed" in text
        assert "SyncWritePosEx" in text
        # 可通过 hardware 参数关闭
        assert "return_to_start" in text
        # park 期间 read/write 必须让路 (write 旧命令会与低速插值打架 → 半路被拽停)
        assert "parking_" in text


# ---------------- Phase 3: 标定转换脚本 ----------------
import setup_convert_lerobot_calibration as conv  # noqa: E402 (conftest 已注入 scripts 路径)

REAL_CALIB = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower/R12552801.json"

SAMPLE_CALIB = {
    "shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": 2043, "range_min": 838, "range_max": 2588},
    "gripper": {"id": 6, "drive_mode": 0, "homing_offset": 1381, "range_min": 2006, "range_max": 3098},
}


class TestConvertCalibration:
    def test_convert_drops_homing_offset(self):
        """D2: 丢弃 homing_offset, 保留 id/drive_mode/range; 默认 invert=False。"""
        out = conv.convert(SAMPLE_CALIB)
        assert out["shoulder_pan"] == {
            "id": 1, "drive_mode": 0, "range_min": 838, "range_max": 2588, "invert": False,
        }

    def test_convert_invert_flag(self):
        """风险缓解: per-motor invert (gripper 方向镜像)。"""
        out = conv.convert(SAMPLE_CALIB, invert_joints={"gripper"})
        assert out["gripper"]["invert"] is True
        assert out["shoulder_pan"]["invert"] is False

    def test_yaml_roundtrip(self):
        import yaml
        text = conv.to_yaml_text(conv.convert(SAMPLE_CALIB))
        data = yaml.safe_load(text)
        assert data["gripper"]["range_min"] == 2006
        assert set(data) == {"shoulder_pan", "gripper"}

    def test_real_calibration_file(self):
        """真实标定 (R12552801): 6 关节, id 1..6。"""
        if not REAL_CALIB.is_file():
            pytest.skip("本机无真实标定文件")
        raw = json.loads(REAL_CALIB.read_text())
        out = conv.convert(raw)
        assert set(out) == set(JOINTS)
        assert [out[j]["id"] for j in JOINTS] == [1, 2, 3, 4, 5, 6]
        assert all(c["range_max"] > c["range_min"] > 0 for c in out.values())

    def test_travel_mismatch_detection(self):
        """行程体检: 欠扫要被检出 (R12552801 旧标定教训), 满行程/超扫通过。"""
        partial = {
            "shoulder_pan": {"id": 1, "drive_mode": 0, "range_min": 838, "range_max": 2588},
            "elbow_flex": {"id": 3, "drive_mode": 0, "range_min": 856, "range_max": 3081},
        }
        bad = conv.travel_mismatches(conv.convert(partial))
        assert any(b.startswith("shoulder_pan") for b in bad)   # 154° < 220° → 检出
        assert not any(b.startswith("elbow_flex") for b in bad)  # 196° vs 194° → 通过
        # 参考项目量程 (满行程) 全部通过
        full = {
            "shoulder_pan": {"id": 1, "drive_mode": 0, "range_min": 847, "range_max": 3371},
            "wrist_roll": {"id": 5, "drive_mode": 0, "range_min": 52, "range_max": 3911},
        }
        assert conv.travel_mismatches(conv.convert(full)) == []
        # 超扫 (如新标定 wrist_roll 360° vs URDF 320°) 是正常钳位, 不告警
        overscan = {
            "wrist_roll": {"id": 5, "drive_mode": 0, "range_min": 0, "range_max": 4095},
        }
        assert conv.travel_mismatches(conv.convert(overscan)) == []


# ---------------- Phase 4: URDF / launch 接线 ----------------
BRINGUP = Path(__file__).resolve().parents[1] / "src" / "so101_bringup"
MOVEIT_CFG = Path(__file__).resolve().parents[1] / "src" / "so101_moveit_config"


class TestLaunchWiring:
    def test_urdf_xacro_real_arg(self):
        """so101.urdf.xacro: real arg (默认 false) + real 宏分支, sim 分支保持。"""
        text = (BRINGUP / "urdf" / "so101.urdf.xacro").read_text()
        assert '<xacro:arg name="real" default="false"/>' in text
        assert "so101.ros2_control_real.xacro" in text
        assert "so101_ros2_control_real" in text

    def test_real_macro(self):
        """real 宏: 自定义插件 + port/calibration_file 参数。"""
        text = (PKG / "urdf" / "so101.ros2_control_real.xacro").read_text()
        assert '<xacro:macro name="so101_ros2_control_real"' in text
        assert "<plugin>so101_hardware/SO101SystemHardware</plugin>" in text
        assert 'name="port"' in text
        assert 'name="calibration_file"' in text

    def test_demo_launch_real_param(self):
        """demo.launch.py: real 参数默认 false, xacro 透传 real/port/calibration_file。"""
        text = (BRINGUP / "launch" / "demo.launch.py").read_text()
        assert '"real"' in text and 'default_value="false"' in text
        assert '" real:="' in text
        assert '" port:="' in text
        assert '" calibration_file:="' in text

    def test_demo_launch_rejects_real_gazebo(self):
        """real+gazebo 组合必须显式报错退出 (计划需求 2)。"""
        text = (BRINGUP / "launch" / "demo.launch.py").read_text()
        assert 'real == "true" and sim == "gazebo"' in text

    def test_demo_launch_real_mujoco_standard_cm(self):
        """real+mujoco: 标准 controller_manager (非 mujoco_ros2_control) 控实机。"""
        text = (BRINGUP / "launch" / "demo.launch.py").read_text()
        assert 'real == "true" and sim == "mujoco"' in text

    def test_move_group_forwards_sim_real(self):
        """move_group.launch.py: sim/real 透传, 去掉硬编码 sim:=none。"""
        text = (MOVEIT_CFG / "launch" / "move_group.launch.py").read_text()
        assert '" sim:="' in text
        assert '" real:="' in text
        assert '" sim:=none"' not in text


class TestXacroRender:
    """xacro 渲染冒烟 (需要 ROS 环境; 无 xacro 时跳过)。"""

    XACRO = BRINGUP / "urdf" / "so101.urdf.xacro"

    def _render(self, *args: str) -> str:
        import os
        import shutil
        import subprocess

        if shutil.which("xacro") is None:
            pytest.skip("xacro 不在 PATH (需 source ROS)")
        env = dict(os.environ)
        # 强制本项目 install 优先: 用户 shell 可能同时挂着参考工作区
        # (lerobot-ros-moveit 与本项目有同名包 so101_description/so101_hardware,
        #  AMENT_PREFIX_PATH 顺序会让 $(find) 解析到参考项目, 见 pitfalls/setup-hardware.md)
        # 注意 ament 索引按 per-package 前缀搜索, 必须逐包列出 (裸 install/ 无索引)
        repo = Path(__file__).resolve().parents[1]
        our_prefixes = [
            str(repo / "install" / d)
            for d in sorted((repo / "install").iterdir())
            if (repo / "install" / d / "share" / "ament_index").is_dir()
        ]
        env["AMENT_PREFIX_PATH"] = os.pathsep.join(our_prefixes) + os.pathsep + env.get(
            "AMENT_PREFIX_PATH", ""
        )
        out = subprocess.run(
            ["xacro", str(self.XACRO), *args],
            capture_output=True, text=True, timeout=60, env=env,
        )
        if out.returncode != 0 and "PackageNotFoundError" in out.stderr:
            pytest.skip("ROS 环境未 source (AMENT_PREFIX_PATH 缺工作区包)")
        assert out.returncode == 0, out.stderr
        return out.stdout

    def test_real_true_uses_hardware_plugin(self):
        xml = self._render("sim:=mujoco", "real:=true")
        assert "so101_hardware/SO101SystemHardware" in xml
        assert "mujoco_ros2_control/MujocoSystemInterface" not in xml

    def test_real_false_regression(self):
        xml = self._render("sim:=mujoco", "real:=false")
        assert "mujoco_ros2_control/MujocoSystemInterface" in xml

    def test_real_none_mock_regression(self):
        xml = self._render("sim:=none", "real:=false")
        assert "mock_components/GenericSystem" in xml


# ---------------- Phase 5: mujoco_twin 数字孪生 ----------------
SO101_SIM = Path(__file__).resolve().parents[1] / "src" / "so101_sim"
sys.path.insert(0, str(SO101_SIM))

import mujoco_twin  # noqa: E402


class TestMujocoTwin:
    def test_joint_map_matches_urdf(self):
        """scene.xml 关节名与 URDF 一致, build_joint_map 全部命中。"""
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(SO101_SIM / "mujoco" / "scene.xml"))
        mapping = mujoco_twin.build_joint_map(model, JOINTS)
        assert set(mapping) == set(JOINTS)
        assert len(set(mapping.values())) == 6  # qpos 索引互不重复

    def test_apply_joint_states(self):
        """apply_joint_states 写 qpos 并置零 qvel。"""
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(SO101_SIM / "mujoco" / "scene.xml"))
        data = mujoco.MjData(model)
        mapping = mujoco_twin.build_joint_map(model, JOINTS)
        positions = {j: 0.1 * i for i, j in enumerate(JOINTS)}
        mujoco_twin.apply_joint_states(model, data, mapping, positions)
        for j in JOINTS:
            assert data.qpos[mapping[j]] == pytest.approx(positions[j])
            assert data.qvel[mapping[j]] == 0.0

    def test_demo_launch_twin_wiring(self):
        """real+mujoco 时启动 twin 节点 (mujoco_twin.py), twin:=false 可关。"""
        text = (BRINGUP / "launch" / "demo.launch.py").read_text()
        assert 'real == "true" and sim == "mujoco"' in text
        assert "mujoco_twin.py" in text


# ---------------- Phase 6: 命名姿势采集 ----------------
import setup_capture_named_pose as cap  # noqa: E402 (conftest 注入 scripts 路径)

SRDF_PATH = Path(__file__).resolve().parents[1] / "src" / "so101_moveit_config" / "config" / "so101.srdf"
CALIB_YAML = PKG / "config" / "motor_calibration.yaml"


class TestCapturePose:
    def test_raw_to_rad_linear(self):
        """raw→rad 全行程线性映射 (与 C++ 插件同公式)。"""
        calib = {"id": 1, "range_min": 1000, "range_max": 3000, "invert": False}
        # 量程中点 → 限位中点
        assert cap.raw_to_rad("shoulder_pan", 2000, calib) == pytest.approx(0.0)
        # range_min → lower 限位
        assert cap.raw_to_rad("shoulder_pan", 1000, calib) == pytest.approx(-1.91986)

    def test_raw_to_rad_clamp_and_invert(self):
        """超量程 clamp 到限位; invert=True 时方向取反。"""
        calib = {"id": 6, "range_min": 2000, "range_max": 3000, "invert": True}
        # invert: raw=range_min → progress 0 → 取反 1.0 → upper 限位
        assert cap.raw_to_rad("gripper", 2000, calib) == pytest.approx(1.74533)
        # 超上限 clamp 到 upper
        assert cap.raw_to_rad("gripper", 9999, calib) == pytest.approx(-0.174533)

    def test_load_calibration(self):
        """加载 Phase 3 生成的 motor_calibration.yaml (6 关节, id 1-6)。"""
        calib = cap.load_calibration(CALIB_YAML)
        assert set(calib) == set(JOINTS)
        assert [calib[j]["id"] for j in JOINTS] == [1, 2, 3, 4, 5, 6]

    def test_update_srdf_new_pose(self):
        """新姿势 → arm group 5 关节 + gripper group 1 关节两条 group_state。"""
        pose = dict(zip(JOINTS, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]))
        out = cap.update_srdf(SRDF_PATH.read_text(), {"wave": pose})
        root = ET.fromstring(out)
        arm = [g for g in root.iter("group_state") if g.get("name") == "wave" and g.get("group") == "arm"]
        grip = [g for g in root.iter("group_state") if g.get("name") == "wave" and g.get("group") == "gripper"]
        assert len(arm) == 1 and len(grip) == 1
        vals = {j.get("name"): float(j.get("value")) for j in arm[0].iter("joint")}
        assert vals == {
            "shoulder_pan": 0.1, "shoulder_lift": 0.2, "elbow_flex": 0.3,
            "wrist_flex": 0.4, "wrist_roll": 0.5,
        }
        assert {j.get("name"): float(j.get("value")) for j in grip[0].iter("joint")} == {"gripper": 0.6}
        # 原有姿势保留
        assert any(g.get("name") == "home" for g in root.iter("group_state"))

    def test_update_srdf_idempotent_and_overwrite(self):
        """重复应用结果不变; 同名姿势覆盖 home 且各 group 只剩一条。"""
        pose = dict(zip(JOINTS, [0.0] * 6))
        once = cap.update_srdf(SRDF_PATH.read_text(), {"home": pose})
        twice = cap.update_srdf(once, {"home": pose})
        assert once == twice
        root = ET.fromstring(once)
        for grp in ("arm", "gripper"):
            homes = [g for g in root.iter("group_state") if g.get("name") == "home" and g.get("group") == grp]
            assert len(homes) == 1

    def test_update_srdf_preserves_comments_and_declaration(self):
        """R3 注释与 XML 声明原样保留 (文本级插入, 不重排其他内容)。"""
        text = SRDF_PATH.read_text()
        out = cap.update_srdf(text, {"wave": dict(zip(JOINTS, [0.0] * 6))})
        assert "纠正方向" in out  # SRDF 中的 R3 注释
        assert out.startswith(text[:40])  # 头部字节不变 (含原始 XML 声明)
        # 除新增块外其余内容字节不变
        stripped = re.sub(
            r"[ \t]*<group_state\b[^>]*\bname=\"wave\"[^>]*>.*?</group_state>[ \t]*\n",
            "", out, flags=re.DOTALL)
        assert stripped == text
