# Plan: il_sim_leader — 真实 leader 臂遥操 il_sim（采集第三通道）

**日期**: 2026-10-29
**状态**: 代码与测试已完成（377 passed）；Phase 3 探针实测 / Phase 4 真机遥操与采集由用户执行，
证据见 workflow/tdd/il_sim_leader.tdd.md + workflow/reports/il_sim_leader_report.md
**复杂度**: Medium
**分支**: il_sim（已快进合并 main 至 14ba7d7）
**已验证前提**: 用户已跑通真机对真机
  `lerobot-teleoperate --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=R12552801 --teleop.type=so101_leader --teleop.port=/dev/ttyACM0 --teleop.id=L07252801`
  （leader 标定 L07252801、follower 标定 R12552801 均已存在）

## 需求重述

1. 在 il_sim 现有采集体系（脚本专家 so101_expert / 键盘 so101_keyboard）基础上，新增**真实 leader 臂**作为第三种 teleoperator，驱动 `so101_sim` 仿真从臂（MuJoCo env），支持纯遥操与**录制采集**（这是本需求的核心目的：leader 臂采集模仿学习演示）。
2. 继续在 il_sim 分支开发；计划输出本文件。

## 环境事实（已勘察确认）

| 项 | 事实 |
|---|---|
| 动作语义链（全管线统一） | teleop `get_action()` → `dict{f"{m}.pos": float}` **6 维归一化 [-1,1] 绝对位置** → `SimSO101Robot.send_action` clip 后 `env.step`；env 动作空间 = `normalize_qpos`（ctrlrange 线性映射，`libs/so101_envs/base_env.py:104`） |
| lerobot 内建 leader | `lerobot.teleoperators.so_leader`：注册名 `"so101_leader"`（0.6.1），`SO101Leader` + config 字段 `port/use_degrees(默认 True)/num_read_retries/id/calibration_dir`；`get_action()` 返回 `{m}.pos`，**body 单位=度（相对其标定 home），gripper 单位=0-100** —— 与 env 期望的 [-1,1] 不同，必须转换 |
| 电机名 | leader 6 电机名与 `constants.JOINT_NAMES` 完全一致（shoulder_pan…gripper） |
| 本仓库注册机制 | draccus `@TeleoperatorConfig.register_subclass`，import 副作用注册（`teleoperators/__init__.py`）；占位文件 `placeholders.py` docstring 明确"leader 无需占位"（届时更新） |
| CLI 工厂 | `cli_config.py:make_teleop_config`（if/elif，L34-43）+ `il_sim_teleop.py` argparse choices（L24）；`il_sim_record.py` 走官方 draccus 原生透传（注册即生效，零改动） |
| reset 耦合 | `cli_config.py:64-67`：`reset_time_s`/`auto_reset_on_success` 按 `teleop == "keyboard"` 分派；expert 用 `request_reset()` 握手 |
| 真机对真机语义（映射基准） | lerobot teleoperate：leader 度值 d 直接作为 follower 目标 → follower 库算 target raw = `homing_offset_f + d*2048/180` → follower 按自身标定解释。**follower 标定 R12552801.json（新全行程档，`~/.cache/.../calibration/robots/so_follower/`）含 homing_offset/range** |
| 已有测试锚点 | `test_il_sim_record.py:142-151` 已断言 `"so101_leader" in get_known_choices()`；动作契约断言模板 = 6 键 + [-1,1]（`test_il_sim_teleop.py:45-51`）；`test_il_sim_teleop_run.py:40-44` 断言未知 teleop 报 ValueError（工厂需扩分支） |
| 串口 | follower=ACM1，leader=ACM0（与 setup_hardware 会话结论一致） |

## 关键设计决策

- **D1 注册名 `so101_leader_sim`**：自有包装类（`teleoperators/leader.py`），不占用 lerobot 内建名 `so101_leader`——内建输出度/0-100，本类输出 [-1,1]，语义不同必须区分；`test_il_sim_record.py` 既有断言不受影响。
- **D2 度→[-1,1] 映射走 follower 标定链**（与"leader 遥控真机从臂"在 lerobot 下行为一致）：
  `rad = linear_map(homing_offset_f + deg*2048/180; range_f → JOINT_LIMITS)`，
  再 `norm = 2*(rad-lo)/(hi-lo)-1`。follower 标定（homing_offset/range）从 R12552801.json 读取；
  复用/镜像 `setup_capture_named_pose.py` 的线性映射与 `--invert` 经验。caveat：本仓库真机插件与
  envs 均用"range→URDF 限位"映射（home=量程中点），leader home 不必落在 rad 0——
  映射探针阶段实测确认并记录偏差。
- **D3 gripper**：leader 0-100 → [-1,1] 直接线性；方向（leader 开/闭 ↔ env qpos 大=张开）由探针实测，必要时 per-joint `--invert`。
- **D4 reset 策略**：`auto_reset_on_success=True` + `reset_time_s=5.0`（同 keyboard 分支）；leader 无按键，不做手动 reset（后续可加"张开夹爪保持 2s 触发 reset"手势，本期不做）。
- **D5 内部组合 lerobot `SO101Leader`**：connect/disconnect/get_action 委托其官方实现（复用串口重试/标定加载），本类只做单位换算——不直接碰 FeetechMotorsBus。
- **D6 record 路径零改动**：`il_sim_record.py` 的 draccus 原生透传天然支持
  `--teleop.type=so101_leader_sim --teleop.port=/dev/ttyACM0 --teleop.id=L07252801`。
- **D7 动作平滑**（真机实测新增）：`get_action` 每步 norm 限幅（关节 0.15 / 夹爪 0.25，
  config 可调/可关）+ env 重置后重吸附（对齐 keyboard）。背景：10Hz 手驱噪声让摩擦
  保持失效（"夹得住保持不住"）；探针+MuJoCo 接触实验证实静态碰撞正常（双爪同时接触、
  穿透 6-8mm 为软化执行器正常形变量）。另 D2 公式修正：lerobot 0.6.1 DEGREES 归一化
  只用量程中点（`deg=(raw-mid)*360/4095`），homing_offset 不参与——实现初版误用 0.5.x
  公式导致臂关节恒被钳在 -1（"只有 gripper 能动"的根因）。

## 项目结构（目标）

```
src/so101_teleop/so101_teleop/teleoperators/leader.py   # 新增: So101LeaderSimTeleop + Config
src/so101_teleop/so101_teleop/teleoperators/__init__.py # UPDATE: import 注册
src/so101_teleop/so101_teleop/teleoperators/placeholders.py  # UPDATE: docstring 决策更新
src/so101_teleop/so101_teleop/cli_config.py             # UPDATE: make_teleop_config 分支 + leader 参数
scripts/il_sim_teleop.py                                # UPDATE: argparse choices/参数
src/so101_teleop/launch/teleop.launch.py                # UPDATE: 注释/用法（choices 本就自由）
tests/test_il_sim_teleop_leader.py                      # 新增: TDD 契约/换算/record 冒烟
workflow/pitfalls/il_sim.md                             # UPDATE: leader 语义转换坑
workflow/tdd/il_sim_leader.tdd.md                       # 新增: TDD 证据
workflow/reports/il_sim_leader_report.md                # 新增: 验收报告
CLAUDE.md                                               # UPDATE: il_sim 命令区
```

## 实施阶段

### Phase 0：分支与基线
- il_sim 已快进合并 main（14ba7d7）；`clean_ros.sh` + `colcon build` + `uv run pytest` 基线全绿。
- **验证**: 基线通过（含 test_il_sim_* 既有测试）。

### Phase 1：leader.py 包装类（TDD）
- 先写 `tests/test_il_sim_teleop_leader.py`：
  1) 注册契约（`"so101_leader_sim" in get_known_choices()`）；
  2) 动作契约（stub 内部 leader：`get_action` 返回已知度值/0-100 → 断言输出 6 键、[-1,1]、换算公式正确——用合成标定 fixture 验证 homing/range 数学）；
  3) invert 生效断言；未连接时 get_action 报错语义（对齐 expert/keyboard）。
- 实现 `leader.py`：Config（`port/id/follower_calibration 路径/invert 列表`）+ 换算函数（纯函数，便于单测）+ 组合 `SO101Leader`。
- **验证**: 新测试 RED→GREEN + 既有 il_sim 测试不回归。

### Phase 2：CLI / launch 接线（TDD）
- `make_teleop_config` 加 leader 分支（读取 `--leader-port/--leader-id/--invert` 顶层参数）；
  `cli_config.py:64-67` reset 策略加 leader（同 keyboard）。
- `il_sim_teleop.py` argparse choices 加 `leader` + 参数；launch 用法注释更新。
- 测试：工厂分支断言（leader → 正确 config 类型/字段；未知类型仍 ValueError）。
- **验证**: `uv run pytest tests/test_il_sim_*` 全绿。

### Phase 3：真机映射探针（用户执行，只读）
- 新增小工具（或 `--probe` 模式）：连接 leader 打印 `deg → rad → [-1,1]` 三列实时值。
- 操作员校验：①leader 回 home → 仿真臂姿态记录（允许非零偏差，记录之）；②各关节方向一致性；
  ③夹爪开合方向。发现反向 → `--invert <关节>` 重新采集记录。
- **验证**: 探针输出 + 用户确认方向表（写入 report）。

### Phase 4：端到端遥操与采集验收
- 纯遥操：`uv run python scripts/il_sim_teleop.py --teleop leader --env reach`（MuJoCo 窗口 + RViz），leader 带动仿真臂。
- 录制：`--teleop leader --record --episodes 10`（或 `il_sim_record.py --teleop.type=so101_leader_sim ...`），
  数据落 `outputs/datasets/il_sim/<repo_id>`，`LeRobotDataset` 回读校验 features。
- 可选：`il_sim_train.py` 冒烟确认新数据可训练。
- **验证**: 数据集回读断言 + 用户实操确认。

### Phase 5：文档工件
- CLAUDE.md il_sim 命令区（leader 遥操/采集命令）、pitfalls/il_sim.md（单位语义链坑）、
  tdd 证据、report（含方向表与遗留）。
- **验证**: 全量 pytest + 文档齐备。

## 依赖关系

Phase 0 → 1 → 2 → 3 → 4 → 5（线性；Phase 3 依赖真机在场，1/2 可先行完成）。

## 风险

| 风险 | 概率 | 缓解 |
|---|---|---|
| leader home ↔ env home 不对齐（D2 caveat：标定 home ≠ 量程中点，各关节偏差不同） | 中 | Phase 3 探针实测记录偏差；若影响操作体验，备选方案改用"leader 自身标定 range→URDF 限位"映射（等同放弃 homing，与真机插件约定一致），决策点留待探针后 |
| leader 与 follower 安装镜像导致个别关节反向 | 中 | per-joint `--invert`（沿用 setup_hardware 惯例），探针确认 |
| gripper 方向反 | 中 | 同上，探针实测 |
| 10Hz 下 leader sync_read 延迟/掉帧 | 低 | lerobot 自带 num_read_retries=2；record fps=10 已验证可行（真机对真机已跑通） |
| lerobot 内建注册名冲突/版本差异 | 低 | 自有注册名 `so101_leader_sim` 隔离；契约测试锁定 |
| 真机不在场导致 Phase 3/4 无法本会话完成 | 中 | Phase 1/2 全部 TDD 完成；Phase 3/4 写成 checklist 由用户执行，报告如实标注 |

## 验收

- [ ] `uv run pytest` 全绿（含新增 leader 测试）
- [ ] `--teleop.type=so101_leader_sim` 在 il_sim_teleop.py 与 il_sim_record.py 两条路径均可用
- [ ] 探针确认关节方向表并记录（含 invert 决策）
- [ ] leader 遥操 reach 任务可驱动仿真臂；录制 ≥10 episodes 并回读校验
- [ ] 文档（CLAUDE.md/pitfalls/tdd/report）齐备
