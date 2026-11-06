# Plan: setup_camera_calibration — 相机标定（内参 + 手眼）迁移

**日期**: 2026-11-05
**状态**: 待确认
**复杂度**: Medium-High
**参考项目**: `/home/maxtensor/Third/Projects/github/so101/so101-ros-physical-ai`（Apache-2.0, © 2026 Dmitri Manajev）— `so101_camera_calibration` 包
**关联**: `workflow/todo/setup_hand_eye_calibration_todo.md`（本计划落定其方案选型, 验收后关闭该 todo）

## 需求重述

1. **迁移参考项目的相机标定**到本项目：内参标定（ChArUco + Viser Web UI）+ 手眼标定（eye-on-base, Park/Horaud 双解验证）。
2. **尊重原项目**: 保留源文件 Apache-2.0 版权头；项目 README 与 docs 添加引用说明（来源、许可、改动范围）。
3. **详细中文标定说明**: 输出到 `docs/hardware/`（docs 站单源惯例）——给标定零基础读者讲清"内参/外参/手眼各是什么、为什么需要、每一步在做什么"。
4. **A4 打印文件**: 生成 ChArUco 标定板 A4 PDF×2（内参板 8×6@25mm、手眼板 4×5@15mm），告知打印要求（实际尺寸/亚光/卡尺校验）；**用户打印完成后再做真机验收**。
5. **完整闭环流程（标定的最终消费者是 envs）**: 不同开发者的 fixed/wrist 相机安装有差异 → **先跑标定 → 标定结果定制 envs 相机（sim 渲染 = 真机视野）→ 在 envs 里训练学习算法 → 回到部署**。本计划覆盖全链路三处落地: envs 相机覆写（训练侧）、ROS 静态 TF（部署侧）、camera_info（感知侧）。

## 环境事实（已勘察确认）

| 项 | 事实 |
|---|---|
| 参考项目结构 | `so101_camera_calibration`（ament_python）: `camera_intrinsic_calibration_node.py`（内参, ChArUco+Viser Web UI, 输出 ROS camera_info YAML + npz）、`handeye_calibration_node.py`（手眼, Viser UI + IK gizmo 手动采样 + Park/Horaud 双解对比 + 自动标定姿势）、`scripts/gen_charuco_{intrinsic,handeye}.py`（A4 300DPI 生成器）、`config/so101_eye_on_base.yaml`（结果格式） |
| 内参节点依赖 | cv2 + viser + rclpy——**干净可整体迁移**，只需指向我们的图像话题 |
| 手眼节点依赖（重） | 除上述外还耦合 `so101_kinematics_msgs`（GoToJoints IK 服务）、`robokin.ui.ViserRobotUI`、`yourdfpy`——是参考项目自有生态，**不整体迁移** |
| 本机大坑（已实证） | `cv_bridge` 系统可用但按 numpy 1.x 编译，与 .venv numpy2 栈冲突（同 system cv2 崩溃模式）→ **迁移代码一律手动解码 rgb8 Image**（camera_node 已验证的模式），弃用 cv_bridge |
| 本项目现状 | 图像: `il_sim_camera_node` 发布 `/real/{fixed,wrist}/image_raw`（rgb8, 无 camera_info/TF）；手臂: `real:=true` 底座有 `/joint_states` + robot_state_publisher TF（URDF 根为 world）；姿态采集: **leader 臂遥操已打通**（lerobot-teleoperate 实测）；`libs/so101_robotics` 有 FK |
| 换算链 | 参考项目: 标定输出 optical-frame 外参 → 脚本换算成 camera_link 的 xyz/rpy → launch 参数。本项目对应: 相机 TF 挂到 URDF/静态 TF（wrist=eye-in-hand 挂 gripper_link, top=eye-to-hand 挂 world） |
| viser | 纯 Python Web UI（localhost:8080），pip 可装，无系统 GUI 依赖——Web UI 保留 |
| scipy | .venv 有 scipy（lerobot 带入）→ 用 `scipy.spatial.transform.Rotation` 替代 tf_transformations |
| 测试惯例 | `tests/test_setup_hardware.py` 契约风格 + TDD（先 RED） |

## 关键设计决策

- **D1 新包 `src/so101_camera_calibration`**（沿用参考包名, 保留版权头注明来源与修改）: 三部分迁移——①内参节点（基本原样, 参数指向我们话题）；②ChArUco 生成器×2（原样）；③手眼标定**重写采集端、复用求解端**。
- **D2 手眼采集双模式（自动为主, 手动兜底）**（不迁 robokin/so101_kinematics）:
  - **自动（主路径）**: `config/calibration_poses.yaml` 预置 20-30 关节姿势（为本项目相机安装重新设计, 强制旋转多样性——纯平移是退化解; 每姿势经安全限幅后 JTC 单点驱动, 复用 il_sim_develop 模式）→ 等稳定 → ChArUco 检测（角点数/重投影误差合格才收样, 不合格跳过并记录）→ 求解。带 estop。姿势集质量由覆盖率指标反馈, 不合格姿势给出调整建议。
  - **手动（兜底）**: leader 臂遥操摆位（已打通）+ Enter 采样确认——自动姿势集双解不一致时补采。
  - 求解用 `cv2.calibrateHandEye`（PARK+HORAUD 双解, 对齐参考项目验证准则：两解平移差 <1cm 视为可信）；检测质量终端日志 + 可视化图落盘 `/tmp`。
- **D3 eye-in-hand + eye-to-hand 两种模式都支持**: 参考项目只做 eye-on-base（static camera）；本项目 top 相机=eye-to-hand（静态, 求解 base→cam），wrist 相机=eye-in-hand（随臂动, 求解 gripper→cam）。`cv2.calibrateHandEye` 原生支持两种输入排布。
- **D4 结果应用（部署侧）**: 输出 YAML（对齐参考 `so101_eye_on_base.yaml` 字段）+ 自动换算脚本（optical→link 的 rpy 逆变换, 移植参考 README 的 scipy 换算代码）→ 生成静态 TF 发布 launch 片段 / URDF 片段, 接入 `il_sim_develop`（相机节点补 camera_info/TF 后 RL/IL 真机可用）。
- **D7 结果应用（训练侧, envs 相机定制）**: 标定 YAML 作为**单一真源**, envs 增加构造参数 `camera_calib`（缺省 None=现行为不变）——加载后运行时覆写 MuJoCo 相机（复用 `_randomize_fixed_cam` 已验证的 model 字段写法）:
  - `fixed_cam`: 外参 base→cam（eye-to-hand 结果）→ `model.cam_pos/cam_quat`; fovy 由内参 fy 换算 `fovy = 2·atan(H/(2·fy))`（度）
  - `wrist_cam`: 外参 gripper→cam（eye-in-hand 结果）+ 内参 fovy, 同法覆写
  - DR 固定相机位姿噪声在标定基位之上继续生效（`_randomize_fixed_cam` 基位改为标定值）
  - **近似性如实记录**: MuJoCo 不支持畸变与主点偏移——只对齐 fovy+位姿, 畸变残差写入文档; 不同开发者各自跑标定得各自 YAML, envs 按文件自适应
- **D5 内参先行**: 内参标定独立可交付（不依赖手眼），用户打印内参板即可先验收第一部分。
- **D6 文档**: `docs/hardware/camera_calibration.md`（中文详解: 标定是什么/为什么/怎么做/怎么判好坏, 含原理科普）+ README 引用节 + 更新 `setup_hand_eye_calibration_todo.md` 为已立项状态。

## 项目结构（目标）

```
src/so101_camera_calibration/                    # 新增（ament_python, Apache-2.0 沿用）
├── package.xml / setup.py / resource/ / wrapper/   # wrapper 模式（setuptools>=80 libexec 坑）
├── config/
│   └── calibration_poses.yaml                  # 自动采样姿势（移植, 可选用途）
├── scripts/
│   ├── gen_charuco_intrinsic.py                # 内参板 A4 300DPI 8×6@25mm DICT_5X5_250（原样迁移）
│   └── gen_charuco_handeye.py                  # 手眼板 A4 300DPI 4×5@15mm DICT_4X4_50（原样迁移）
├── launch/
│   └── camera_calibration.launch.py            # 内参/手眼模式选择 + 图像话题参数
└── so101_camera_calibration/
    ├── camera_intrinsic_calibration_node.py    # 内参（原样迁移, 弃 cv_bridge, 参数适配）
    ├── handeye_node.py                         # 手眼（重写: 遥操采样 CLI + Park/Horaud 求解 + 结果换算）
    └── charuco.py                              # 共用: ChArUco 检测（rgb8 手动解码 + 角点/重投影误差）

libs/so101_envs/base_env.py                     # UPDATE: cfg.camera_calib 参数 + 相机覆写逻辑
libs/so101_envs/randomize.py 或 cfg             # UPDATE: fixed_cam DR 基位取标定值

docs/hardware/camera_calibration.md             # 新增: 中文详细标定说明（原理科普 + 操作指南）
outputs/calibration/                            # 生成物: A4 PDF/PNG ×2 + 标定结果 YAML
README.md                                       # UPDATE: 引用与致谢节
tests/test_camera_calibration.py                # 新增: TDD（生成器输出几何正确性/检测纯函数/换算函数）
workflow/todo/setup_hand_eye_calibration_todo.md # UPDATE: 标注已立项→本计划
CLAUDE.md / workflow/{tdd,reports,pitfalls}/setup_camera_calibration.md
```

## 实施阶段

### Phase 0：基线 + 依赖
- 基线构建/测试全绿；`uv add viser`（唯一新增 Python 依赖, 纯 Python）。
- **验证**: 基线通过。

### Phase 1：ChArUco 生成器 + A4 文件（TDD）
- 迁移两个生成器脚本 → 产出 `outputs/calibration/charuco_{intrinsic,handeye}_A4.{png,pdf}`。
- 单测: 生成图像几何断言（棋盘格边长换算像素数 = mm×DPI/25.4, 角点可被 cv2 检出）。
- **⚠️ 交付给用户**: 告知打印要求（实际尺寸/100% 缩放、亚光纸、卡尺量 25mm/15mm 校验）→ **用户打印完成后才进入 Phase 5 真机验收**。
- **验证**: `uv run pytest`。

### Phase 2：内参标定迁移（TDD）
- `camera_intrinsic_calibration_node.py` 迁移: 弃 cv_bridge（rgb8 手动解码）、话题参数默认 `/real/fixed/image_raw`、输出到 `outputs/calibration/`。
- 单测: 检测纯函数（合成 ChArUco 图像 → 角点检出数/重投影误差）、camera_info YAML 格式断言。
- **验证**: pytest + 仿真冒烟（假图像流 → 节点不崩、覆盖网格 UI 可开）。

### Phase 3：手眼标定（重写采集端, TDD）
- `handeye_node.py`: 订阅图像+`/joint_states`+TF → 每次采样存 (图像角点 2D, gripper→base 4×4)；`cv2.calibrateHandEye` PARK/HORAUD 双解 + 一致性判定；结果 YAML + optical→link 换算 + 静态 TF/URDF 片段输出。
- **双采集模式**: `mode:=auto`（逐姿势 JTC 驱动 + 质量门控采样, 姿势集来自 calibration_poses.yaml）/ `mode:=manual`（leader 遥操 + Enter 采样）。
- eye-in-hand（wrist）与 eye-to-hand（top）两种排布。
- 单测: 合成数据闭环——已知真值外参生成虚拟样本 → 求解误差 < 容差；换算函数与参考 README 数值对齐。
- **验证**: pytest + 合成数据验收（无需真机）。

### Phase 4：envs 相机定制接入（TDD）
- `so101_envs` 增加 `camera_calib` 构造参数: 读标定 YAML → 构造/reset 时覆写 `model.cam_pos/cam_quat/cam_fovy`（fixed_cam 与 wrist_cam）; DR 基位跟随标定值; 缺省 None 行为完全不变（回归保证）。
- 单测: 已知 YAML → model 相机字段断言; None 时与现行为逐字段一致; fovy 换算数值正确。
- 对照验收: 同一场景 sim 渲染（标定前后）与真机抓帧并排对比, 存图入报告。
- **验证**: `uv run pytest` + `envs_check_env.py --render` 出对照图。

### Phase 5：真机标定验收（用户执行, 阻塞于打印）
- 前置: 用户打印 A4 板×2 并卡尺校验。
- 流程（中文文档逐步指引）: 内参标定双相机 → 手眼自动采样（calibration_poses.yaml 驱动, 一键跑完; 双解不一致时手动模式补采）→ Park/Horaud 一致性 → 结果三处落地: envs 相机定制（训练侧）+ il_sim_develop 静态 TF（部署侧）→ 闭环验证: sim 渲染 vs 真机图像对照 + RViz TF 投影对照。
- **验证**: `workflow/reports/setup_camera_calibration.md` 如实记录（内参重投影误差、手眼双解差、TF 对照截图）。

### Phase 6：文档与工件
- `docs/hardware/camera_calibration.md` 中文详解（原理: 针孔模型/畸变/手眼 AX=XB 问题；操作: 每步截图级指引；判定: 误差指标含义）；README 引用与致谢；todo 回链关闭；CLAUDE.md 命令区。
- **验证**: pytest 全绿 + 全量构建 + docs 构建（若站点可构建）。

## 依赖关系

Phase 0 → 1 → 2 → 3 → 4 → 6 线性；Phase 5（真机验收）阻塞于用户打印（Phase 1 完成即可开始打印），内参验收与手眼验收可分开做；Phase 4（envs 接入）不依赖真机，可与 Phase 5 并行。

## A4 打印交付清单（回答需求 3）

| 文件 | 用途 | 打印要求 |
|---|---|---|
| `outputs/calibration/charuco_intrinsic_A4.pdf` | 内参标定板（8×6 方格, 25mm 方格/18mm 标记） | **实际尺寸打印**（打印对话框选 100%/无缩放）, **亚光纸**, 打印后卡尺量任一方格边长 = 25.0mm（±0.2mm）, 平整粘贴（硬纸板防翘曲） |
| `outputs/calibration/charuco_handeye_A4.pdf` | 手眼标定板（4×5 方格, 15mm 方格, 贴夹爪） | 同上; 方格边长 = 15.0mm; 建议硬卡纸裁小块贴夹爪正/侧面 |

## 风险

| 风险 | 概率 | 缓解 |
|---|---|---|
| 手眼精度受舵机重复性限制（参考项目明示 ~1-2° 硬下限, 自标实验性） | 高 | 沿用其验证准则（Park/Horaud 双解一致才可信）; 文档如实标注精度预期; 结果仅作"起点" |
| cv_bridge/numpy ABI（同 system cv2 崩溃模式） | 高（已知） | 全程手动 rgb8 解码, 不引入 cv_bridge; 测试覆盖 |
| viser 与 .venv numpy2 兼容性 | 低 | viser 纯 Python; Phase 2 冒烟即验 |
| 自动姿势集质量不佳（遮挡/退化/覆盖差） | 中 | 姿势集带质量门控与统计反馈; 不合格自动跳过; 手动模式兜底补采 |
| wrist 相机装夹松动导致眼在手标定漂移 | 中 | 文档强调标定前紧固; 验收含 TF 对照步骤 |
| MuJoCo 不支持畸变/主点偏移, sim 与真机存在视觉残差 | 中 | fovy+位姿对齐为主; 残差量化写入文档; 后续如需可评估 MJCF 自定义畸变（超出本期） |
| 打印缩放/纸张变形导致角点尺度错误 | 中 | 卡尺校验步骤强制; 文档给量化判据 |
| 原项目 Apache-2.0 合规 | 低 | 保留版权头 + LICENSE 引用 + README 致谢（需求 1） |

## 验收

- [ ] `colcon build --symlink-install` + `uv run pytest` 全绿（含新增 test_camera_calibration.py）
- [ ] A4 板×2 已生成并交付打印要求（用户打印完成后进入真机项）
- [ ] 内参标定双相机跑通, 重投影误差记录（通常 <0.5px 可用）
- [ ] 手眼标定 eye-in-hand/eye-to-hand 各完成一次, Park/Horaud 平移差 <1cm
- [ ] envs `camera_calib` 参数接入: 标定 YAML 驱动 sim 相机（None 回归不变）, sim 渲染 vs 真机图像对照图入报告
- [ ] 结果接入 il_sim_develop（静态 TF）, RViz 图像-TF 对照确认
- [ ] `docs/hardware/camera_calibration.md` 中文详解 + README 引用节
- [ ] `setup_hand_eye_calibration_todo.md` 关闭（已立项落地）
- [ ] 文档工件（tdd/reports/pitfalls/CLAUDE.md）齐备
