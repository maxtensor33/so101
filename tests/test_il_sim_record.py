"""il_sim Phase 2: Teleoperator 契约 + 官方 record() 采集冒烟（plan/il_sim.md）。

- ScriptExpertTeleop: IK 状态机（WAIT→GO→SETTLE），经 env_backend 共享 env 闭环
- So101KeyboardTeleop: 关节选择 + 增量控制（键事件可注入，headless 可测）
- 占位: gamepad/leader TODO（注册存在，connect 报 NotImplementedError）
- record 冒烟: 官方 record() → LeRobotDataset v3 结构校验
"""

from __future__ import annotations

import pytest


@pytest.fixture
def sim_robot():
    from so101_teleop.configs import SimSO101RobotConfig
    from so101_teleop.robot import SimSO101Robot

    r = SimSO101Robot(SimSO101RobotConfig(seed=7))
    r.connect()
    yield r
    r.disconnect()


@pytest.fixture
def expert(sim_robot):
    from so101_teleop.teleoperators.expert import ScriptExpertTeleop, ScriptExpertTeleopConfig

    t = ScriptExpertTeleop(ScriptExpertTeleopConfig())
    t.connect()
    yield t
    t.disconnect()


# ---------- ScriptExpertTeleop ----------

def test_expert_registered():
    from lerobot.teleoperators.teleoperator import TeleoperatorConfig
    from so101_teleop.teleoperators.expert import ScriptExpertTeleopConfig

    assert "so101_expert" in TeleoperatorConfig.get_known_choices()
    assert TeleoperatorConfig.get_choice_name(ScriptExpertTeleopConfig) == "so101_expert"


def test_expert_action_contract(expert):
    act = expert.get_action()
    assert set(act) == {
        "shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos",
        "wrist_flex.pos", "wrist_roll.pos", "gripper.pos",
    }
    assert all(-1.0 <= v <= 1.0 for v in act.values())


def test_expert_drives_tcp_to_goal(sim_robot, expert):
    """闭环冒烟：专家经共享 env 驱动 TCP 到达 reach 目标（≤80 帧）。"""
    u = sim_robot._env.unwrapped
    for _ in range(80):
        sim_robot.send_action(expert.get_action())
        info = u.evaluate()
        if info["success"]:
            break
    assert info["success"], f"tcp_to_goal_dist={info['tcp_to_goal_dist']:.4f}"


def test_reset_handshake(sim_robot, expert):
    """专家 SETTLE 结束请求重置 → robot.send_action 消费并执行 env.reset。"""
    u = sim_robot._env.unwrapped
    for _ in range(80):
        sim_robot.send_action(expert.get_action())
        if u.evaluate()["success"]:
            break
    assert u.evaluate()["success"]
    from so101_teleop.env_backend import reset_pending
    from so101_teleop.teleoperators.expert import SETTLE_STEPS

    for _ in range(SETTLE_STEPS + 2):
        sim_robot.send_action(expert.get_action())
    assert not reset_pending(), "reset 请求应已被 send_action 消费"
    assert u._step_count < SETTLE_STEPS + 2, "env 应已 reset（step 计数回零）"


# ---------- So101KeyboardTeleop ----------

def test_keyboard_registered():
    from lerobot.teleoperators.teleoperator import TeleoperatorConfig
    from so101_teleop.teleoperators.keyboard import So101KeyboardTeleopConfig

    assert "so101_keyboard" in TeleoperatorConfig.get_known_choices()
    assert TeleoperatorConfig.get_choice_name(So101KeyboardTeleopConfig) == "so101_keyboard"


def test_keyboard_action_contract():
    from so101_teleop.teleoperators.keyboard import So101KeyboardTeleop, So101KeyboardTeleopConfig

    t = So101KeyboardTeleop(So101KeyboardTeleopConfig())
    t.connect()
    try:
        base = t.get_action()
        assert set(base) == {
            "shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos",
            "wrist_flex.pos", "wrist_roll.pos", "gripper.pos",
        }
        assert all(-1.0 <= v <= 1.0 for v in base.values())
    finally:
        t.disconnect()


def test_keyboard_increment_and_clamp():
    from so101_teleop.teleoperators.keyboard import So101KeyboardTeleop, So101KeyboardTeleopConfig

    t = So101KeyboardTeleop(So101KeyboardTeleopConfig())
    t.connect()
    try:
        base = t.get_action()
        t.handle_press("1"); t.handle_press("w"); t.handle_release("w")
        after = t.get_action()
        assert after["shoulder_pan.pos"] == pytest.approx(
            base["shoulder_pan.pos"] + t.config.step_size, abs=1e-5
        )
        for _ in range(30):
            t.handle_press("w"); t.handle_release("w")
        assert t.get_action()["shoulder_pan.pos"] <= 1.0
    finally:
        t.disconnect()


def test_keyboard_reset_key():
    from so101_teleop.env_backend import take_reset_request
    from so101_teleop.teleoperators.keyboard import So101KeyboardTeleop, So101KeyboardTeleopConfig

    t = So101KeyboardTeleop(So101KeyboardTeleopConfig())
    t.connect()
    try:
        t.handle_press("c")
        assert take_reset_request() is True
    finally:
        t.disconnect()


# ---------- 占位 ----------

def test_placeholder_teleops_registered():
    import lerobot.scripts.lerobot_record  # noqa: F401  触发 lerobot 内建 teleop 注册（含 so101_leader）
    import so101_teleop  # noqa: F401  我们的 teleop 注册

    from lerobot.teleoperators.teleoperator import TeleoperatorConfig

    # so101_gamepad 为 TODO 占位；so101_leader 是 lerobot 自带真机主臂（il_real 直接用）
    choices = TeleoperatorConfig.get_known_choices()
    assert "so101_gamepad" in choices
    assert "so101_leader" in choices


# ---------- 官方 record() 采集冒烟 ----------

def test_record_smoke(tmp_path):
    import so101_teleop  # noqa: F401  触发 draccus 注册

    from lerobot.configs.dataset import DatasetRecordConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.scripts.lerobot_record import RecordConfig, record
    from so101_teleop.configs import SimSO101RobotConfig
    from so101_teleop.teleoperators.expert import ScriptExpertTeleopConfig

    root = tmp_path / "dataset"
    rec_cfg = RecordConfig(
        robot=SimSO101RobotConfig(seed=7),
        teleop=ScriptExpertTeleopConfig(),
        dataset=DatasetRecordConfig(
            repo_id="il_sim/smoke",
            single_task="reach the goal above the item",
            root=root,
            fps=10,
            episode_time_s=9,
            reset_time_s=0,
            num_episodes=1,
            video=False,
            push_to_hub=False,
            num_image_writer_threads_per_camera=1,
        ),
        play_sounds=False,
    )
    record(rec_cfg)

    ds = LeRobotDataset("il_sim/smoke", root=root)
    assert ds.meta.total_episodes == 1
    assert ds.meta.total_frames >= 20
    assert ds.features["observation.state"]["shape"] == (6,)
    assert ds.features["action"]["shape"] == (6,)
    for cam in ("fixed", "wrist"):
        assert f"observation.images.{cam}" in ds.features
    assert (root / "meta" / "info.json").exists()


def test_record_cli_renders_default():
    """il_sim_record.py 默认注入 --robot.render=true；显式 false 尊重用户。"""
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path("/home/maxtensor/Third/Projects/local/so101/scripts")))
    import il_sim_record

    # 未指定 → 注入 true
    assert "--robot.render=true" in il_sim_record.ensure_render_default(
        ["--robot.type=so101_sim", "--teleop.type=so101_expert"])
    # 显式 true/false → 不重复注入
    assert il_sim_record.ensure_render_default(["--robot.render=false"]) == [
        "--robot.render=false"]
    assert il_sim_record.ensure_render_default(["--robot.render=true"]) == [
        "--robot.render=true"]


def test_keyboard_nr_auto_reset(sim_robot):
    """n/r 键 = 提前完成/重录 → 自动请求环境重置（帧边界执行）。"""
    from so101_teleop.teleoperators.keyboard import So101KeyboardTeleop, So101KeyboardTeleopConfig

    t = So101KeyboardTeleop(So101KeyboardTeleopConfig())
    t.connect()
    try:
        t.handle_press("n")
        from so101_teleop.env_backend import reset_pending

        assert reset_pending(), "n 键应请求 reset"
        sim_robot.send_action(t.get_action())
        assert sim_robot._env.unwrapped._step_count < 2, "send_action 应消费 reset"
    finally:
        t.disconnect()


def test_auto_reset_on_success(sim_robot, expert):
    """keyboard 模式 auto_reset_on_success: 成功后下一帧自动重置场景。"""
    from so101_teleop.configs import SimSO101RobotConfig
    from so101_teleop.robot import SimSO101Robot

    u = sim_robot._env.unwrapped
    for _ in range(80):
        sim_robot.send_action(expert.get_action())
        if u.evaluate()["success"]:
            break
    assert u.evaluate()["success"]
    steps_before = u._step_count

    r2 = SimSO101Robot(SimSO101RobotConfig(seed=7, auto_reset_on_success=True))
    r2.connect()
    try:
        u2 = r2._env.unwrapped
        from so101_teleop.teleoperators.expert import ScriptExpertTeleop, ScriptExpertTeleopConfig

        ex = ScriptExpertTeleop(ScriptExpertTeleopConfig())
        ex.connect()
        # 成功帧内即重置(evaluate 的 success 是瞬时的), 用 step_count 归零次数验证
        resets, prev = 0, None
        for _ in range(120):
            r2.send_action(ex.get_action())
            cur = u2._step_count
            if prev is not None and cur < prev:
                resets += 1
            prev = cur
        assert resets >= 2, f"成功触发自动重置应周期性归零 step_count, resets={resets}"
        ex.disconnect()
    finally:
        r2.disconnect()


def test_keyboard_arrow_keys_auto_reset():
    """←/→ 方向键（pynput 特殊键）同样触发自动重置。"""
    from pynput.keyboard import Key

    from so101_teleop.env_backend import reset_pending, take_reset_request
    from so101_teleop.teleoperators.keyboard import So101KeyboardTeleop, So101KeyboardTeleopConfig

    t = So101KeyboardTeleop(So101KeyboardTeleopConfig())
    t.connect()
    try:
        t._on_press(Key.right)  # 模拟 pynput 特殊键分发路径
        assert reset_pending(), "→ 应请求 reset"
        take_reset_request()

        t._on_press(Key.left)
        assert reset_pending(), "← 应请求 reset"
    finally:
        t.disconnect()


def test_keyboard_target_resnaps_after_env_reset(sim_robot):
    """env reset 后 keyboard teleop 的保持目标应重新对齐新 qpos（否则机械臂弹回旧位姿）。"""
    from so101_teleop.env_backend import reset_pending, reset_count
    from so101_teleop.teleoperators.keyboard import So101KeyboardTeleop, So101KeyboardTeleopConfig

    t = So101KeyboardTeleop(So101KeyboardTeleopConfig())
    t.connect()
    try:
        # 把目标推离当前位形
        t.handle_press("1"); t.handle_press("w")

        # reset 发生（robot 帧边界消费）
        t.handle_press("c")
        assert reset_pending()
        sim_robot.send_action(t.get_action())
        assert not reset_pending()

        # 下一帧目标应吸附到 reset 后的 qpos, 而非保持旧目标
        fresh = t.get_action()
        qpos_norm = sim_robot._env.unwrapped.normalize_qpos(
            sim_robot._env.unwrapped.data.qpos[sim_robot._env.unwrapped._qpos_adr])
        assert fresh["shoulder_pan.pos"] == pytest.approx(float(qpos_norm[0]), abs=1e-5)
    finally:
        t.disconnect()
