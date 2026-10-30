"""so101_leader_sim teleoperator 测试 (workflow/plan/il_sim_leader.md Phase 1)。

契约: 包装 lerobot SO101Leader, 把其输出 (body=度, gripper=0-100) 换算为
本管线统一的 6 维归一化 [-1,1] 绝对位置; 度→[-1,1] 走 follower 标定链
(homing_offset + deg*2048/180 → range 线性映射, 计划 D2)。
"""

import json
from pathlib import Path

import pytest
from lerobot.teleoperators.config import TeleoperatorConfig

import so101_teleop.teleoperators.leader as leader_mod
from so101_teleop.teleoperators.leader import (
    So101LeaderSimTeleop,
    So101LeaderSimTeleopConfig,
)

# 合成 follower 标定: homing=0, 量程 [0,4096] → raw = deg*2048/180, 进度=raw/4096
SYNTH_CALIB = {
    j: {"id": i + 1, "drive_mode": 0, "homing_offset": 0, "range_min": 0, "range_max": 4096}
    for i, j in enumerate(
        ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    )
}

JOINTS = list(SYNTH_CALIB)


def _write_calib(tmp_path):
    p = tmp_path / "follower_calib.json"
    p.write_text(json.dumps(SYNTH_CALIB))
    return p


class _StubInner:
    """替代 lerobot SO101Leader: 返回固定原始读数 (度 / gripper 0-100)。"""

    def __init__(self, raw_action):
        self._raw = raw_action

    @property
    def is_connected(self):
        return True

    def get_action(self):
        return dict(self._raw)

    def connect(self):
        pass

    def disconnect(self):
        pass


def _make_teleop(tmp_path, raw_action, invert=None):
    calib_path = _write_calib(tmp_path)
    cfg = So101LeaderSimTeleopConfig(
        port="/dev/null",
        id="L07252801",
        follower_calibration=str(calib_path),
        invert=invert or [],
    )
    t = So101LeaderSimTeleop(cfg)
    t._make_inner = lambda: _StubInner(raw_action)  # 屏蔽串口, 保留标定加载路径
    t.connect()
    return t


def _raw_action(deg=90.0, grip=25.0):
    raw = {f"{j}.pos": deg for j in JOINTS}
    raw["gripper.pos"] = grip
    return raw


class TestRegistration:
    def test_registered(self):
        assert "so101_leader_sim" in TeleoperatorConfig.get_known_choices()

    def test_choice_name_roundtrip(self):
        cfg = So101LeaderSimTeleopConfig(port="/dev/null")
        assert TeleoperatorConfig.get_choice_name(type(cfg)) == "so101_leader_sim"


class TestConversion:
    def test_deg_to_norm_midpoint(self):
        """deg=0 → raw=量程中点 → 进度 0.5 → norm 0.0。

        lerobot 0.6.1 DEGREES 公式: deg=(raw-mid)*360/4095 (homing_offset 不参与),
        raw = deg*4095/360 + mid (真机实测修正, 见 report)。
        """
        v = leader_mod.deg_to_norm("shoulder_pan", 0.0, SYNTH_CALIB["shoulder_pan"])
        assert v == pytest.approx(0.0)

    def test_deg_to_norm_quarter(self):
        """合成标定 span=4096: deg=±90 → raw 偏移 ±1023.75 → 进度 0.75/0.25 → norm ±0.5。"""
        v = leader_mod.deg_to_norm("shoulder_pan", 90.0, SYNTH_CALIB["shoulder_pan"])
        assert v == pytest.approx(0.5, abs=1e-3)
        v = leader_mod.deg_to_norm("shoulder_pan", -90.0, SYNTH_CALIB["shoulder_pan"])
        assert v == pytest.approx(-0.5, abs=1e-3)

    def test_deg_clamped(self):
        """超量程 clamp 到 [-1,1] 边界。"""
        v = leader_mod.deg_to_norm("shoulder_pan", -400.0, SYNTH_CALIB["shoulder_pan"])
        assert v == pytest.approx(-1.0)
        v = leader_mod.deg_to_norm("shoulder_pan", 400.0, SYNTH_CALIB["shoulder_pan"])
        assert v == pytest.approx(1.0)

    def test_invert_negates(self):
        v = leader_mod.deg_to_norm(
            "shoulder_pan", 90.0, SYNTH_CALIB["shoulder_pan"], invert=True)
        assert v == pytest.approx(-0.5, abs=1e-3)

    def test_gripper(self):
        assert leader_mod.grip_to_norm(0.0) == pytest.approx(-1.0)
        assert leader_mod.grip_to_norm(50.0) == pytest.approx(0.0)
        assert leader_mod.grip_to_norm(100.0) == pytest.approx(1.0)
        assert leader_mod.grip_to_norm(25.0, invert=True) == pytest.approx(0.5, abs=1e-3)

    def test_homing_offset_ignored(self):
        """lerobot 0.6.1 归一化只用量程中点, homing_offset 不参与 (真机实测教训)。"""
        calib = dict(SYNTH_CALIB["shoulder_pan"], homing_offset=1024)
        v = leader_mod.deg_to_norm("shoulder_pan", 0.0, calib)
        assert v == pytest.approx(0.0)  # 与 homing=0 时一致

    def test_real_scale_sanity(self):
        """真实量纲 sanity: follower span=2762 时 leader ±110° ≈ norm ±0.9 (全行程可达)。"""
        calib = {"homing_offset": -1889, "range_min": 617, "range_max": 3379}
        v = leader_mod.deg_to_norm("shoulder_pan", 110.0, calib)
        assert 0.85 < v <= 1.0


class TestActionContract:
    def test_six_keys_normalized(self, tmp_path):
        t = _make_teleop(tmp_path, _raw_action(deg=180.0, grip=50.0))
        act = t.get_action()
        assert set(act) == {f"{j}.pos" for j in JOINTS}
        assert all(-1.0 <= v <= 1.0 for v in act.values())

    def test_values_converted(self, tmp_path):
        t = _make_teleop(tmp_path, _raw_action(deg=90.0, grip=25.0))
        act = t.get_action()
        assert act["shoulder_pan.pos"] == pytest.approx(0.5, abs=1e-3)
        assert act["gripper.pos"] == pytest.approx(-0.5)

    def test_invert_per_joint(self, tmp_path):
        t = _make_teleop(tmp_path, _raw_action(deg=90.0, grip=25.0), invert=["shoulder_pan"])
        act = t.get_action()
        assert act["shoulder_pan.pos"] == pytest.approx(-0.5, abs=1e-3)
        assert act["shoulder_lift.pos"] == pytest.approx(0.5, abs=1e-3)  # 其余不受影响

    def test_not_connected_raises(self, tmp_path):
        cfg = So101LeaderSimTeleopConfig(
            port="/dev/null", follower_calibration=str(_write_calib(tmp_path)))
        t = So101LeaderSimTeleop(cfg)
        with pytest.raises(RuntimeError, match="not connected"):
            t.get_action()

    def test_is_connected_delegates(self, tmp_path):
        t = _make_teleop(tmp_path, _raw_action())
        assert t.is_connected is True

    def test_disconnect_resets(self, tmp_path):
        t = _make_teleop(tmp_path, _raw_action())
        t.disconnect()
        assert t.is_connected is False


# ---------------- 动作平滑 (D7: 10Hz 手驱噪声抑制, 修"抓得住保持不住") ----------------
class TestSmoothing:
    def test_first_call_snaps(self, tmp_path):
        """首帧直接吸附到 leader 姿态 (无历史可限幅)。"""
        t = _make_teleop(tmp_path, _raw_action(deg=270.0, grip=100.0))
        act = t.get_action()
        assert act["shoulder_pan.pos"] == pytest.approx(1.0, abs=1e-3)

    def test_rate_limited(self, tmp_path):
        """后续帧每关节每步最多变化 max_norm_step (0.15)。"""
        t = _make_teleop(tmp_path, _raw_action(deg=270.0, grip=0.0))
        t.get_action()  # 吸附: pan=1.0, grip=-1.0
        t._inner._raw = _raw_action(deg=-270.0, grip=100.0)  # 手瞬间打到另一端
        act = t.get_action()
        assert act["shoulder_pan.pos"] == pytest.approx(1.0 - 0.15, abs=1e-3)
        assert act["gripper.pos"] == pytest.approx(-1.0 + 0.25, abs=1e-3)

    def test_reset_re_snaps(self, tmp_path):
        """env 重置后重新吸附到 env 当前姿态 (对齐 keyboard 行为, 防回弹)。"""
        from so101_teleop import env_backend

        t = _make_teleop(tmp_path, _raw_action(deg=270.0, grip=0.0))
        t.get_action()
        env_backend.request_reset()
        # robot 侧消费 reset → reset_count 变化; leader 下一帧应吸附 env 姿态而非继续限幅爬行
        env_backend.bump_reset_count()  # 模拟 robot 消费后的计数
        act = t.get_action()
        # env 初始姿态 norm≈0 → 重置后吸附到 0 附近, 而不是从 1.0 慢慢爬
        assert abs(act["shoulder_pan.pos"]) < 0.2

    def test_smoothing_off(self, tmp_path):
        """max_norm_step=0 关闭平滑 (兼容旧行为)。"""
        calib_path = _write_calib(tmp_path)
        cfg = So101LeaderSimTeleopConfig(
            port="/dev/null", follower_calibration=str(calib_path), max_norm_step=0.0)
        t = So101LeaderSimTeleop(cfg)
        t._make_inner = lambda: _StubInner(_raw_action(deg=270.0, grip=0.0))
        t.connect()
        t.get_action()
        t._inner._raw = _raw_action(deg=-270.0, grip=100.0)
        act = t.get_action()
        assert act["shoulder_pan.pos"] == pytest.approx(-1.0, abs=1e-3)


# ---------------- Phase 2: CLI 接线 ----------------
class TestCliWiring:
    def test_factory_leader_branch(self):
        from so101_teleop.cli_config import make_teleop_config

        cfg = make_teleop_config("leader", leader_port="/dev/ttyACM1", leader_id="X1",
                                 invert=["gripper"])
        assert isinstance(cfg, So101LeaderSimTeleopConfig)
        assert cfg.port == "/dev/ttyACM1"
        assert cfg.id == "X1"
        assert cfg.invert == ["gripper"]

    def test_factory_unknown_still_raises(self):
        from so101_teleop.cli_config import make_teleop_config

        with pytest.raises(ValueError, match="unknown teleop"):
            make_teleop_config("leader_xxx")

    def test_record_config_reset_policy(self):
        """leader 采 reset 策略同 keyboard: 成功自动重置 + 5s 缓冲 (D4)。"""
        from so101_teleop.cli_config import build_record_config

        cfg = build_record_config(
            teleop="leader", env="reach", repo_id="il_sim/leader_smoke", episodes=2,
        )
        assert cfg.dataset.reset_time_s == 5.0
        assert cfg.robot.auto_reset_on_success is True

    def test_teleop_mode_config_leader(self):
        from so101_teleop.cli_config import build_teleop_mode_config

        cfg = build_teleop_mode_config("leader", env="reach")
        assert cfg.teleop.port == "/dev/ttyACM0"
        assert cfg.teleop.id == "L07252801"
