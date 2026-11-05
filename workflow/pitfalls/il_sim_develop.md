# 坑: il_sim_develop（实机 IL 部署）

> 按 pitfalls 惯例: 单源进 docs 站, 实施中踩坑逐条追加。

## 已确认

- **lerobot 0.6.1 robot type**: `lerobot-rollout --robot.type so101_follower` 合法（CLI help 实证）；但 squint `robot_config.py` 的 `from lerobot.robots.robot import make_robot_from_config` 在 0.6.1 **已失效**（ImportError），借 squint deploy 脚本时需按 0.6.1 API 改写。
- **usb_cam 未安装**: 本机无 `ros-jazzy-usb-cam`（apt 装）。so101_il_sim_develop 改用包内内置 cv2 相机节点（`il_sim_camera_node`, BGR→RGB→rgb8, 对标 lerobot OpenCVCamera 语义）。
- **lerobot policy 动作带 batch 维**: `policy.select_action` 经 post-processor 后返回 shape (1,6)——下发前必须 `reshape(-1)`，否则 JTC positions 构造报 "only length-1 arrays"。
- **estop 判定的 in-flight 语义**: 单线程 executor 中 estop 前已发布、未投递到计数方的消息会计入"estop 后"窗口——判定须先 spin 排空（本实现约 0.3s 宽限）。真实急停物理延迟同理存在，安全靠 estop 保持 + homing，不追求零延迟。
- **test_docs_links 强校验**: workflow/*.md 里引用的 tdd/reports/pitfalls 路径必须真实存在（计划落地时先建占位 stub，否则 pytest 基线 RED）。

## 待真机确认（Phase 1/4/5）

- cv2 相机 index 漂移与 BGR/RGB、640×480@30fps 实测
- JTC 100Hz 插值与 10Hz 决策的滞后
- LeRobot 标定（lerobot-rollout 用）与 motor_calibration.yaml（so101_hardware 用）两套来源的数值一致性

## 2026-11-05 追加（首轮真机 launch 实测）

- **setuptools>=80 console_scripts 落 bin/**: ros2 run/launch 找 `lib/<pkg>/` libexec——新 ament_python 包必须带 wrapper 脚本进 `lib/<pkg>/`（仓库 il_sim_infer 已有先例, 新包漏抄即 "libexec directory does not exist"）。已在契约测试固化。
- **参数类型严格匹配**: launch 传 "/dev/video0"（string）而 `declare_parameter("device", 0)`（int）→ InvalidParameterTypeException。设备参数一律 string, 节点内 `_resolve_device` 兼容纯数字索引。
- **相机设备映射机器相关**: /dev/videoN 与 USB 插入顺序有关（本机 4 个 video 口, fixed=0 / wrist=2）——设备路径放 `config/cameras.yaml` 按节点名配置, launch 不硬编码（契约测试断言 launch 内无 /dev/video 字样）。
- **~/.local pip numpy 2.x 遮蔽系统栈**: 系统 python3 下 user-site 的 numpy 2.3.5 压过 dist-packages numpy 1.26.4, 系统 cv2 4.6.0（numpy1.x 编译）import 即崩且**无输出**（进程静默死）。排查口诀: `ros2 topic hz` 无数据时先查进程是否还活着。
  - **最终解（2026-11-05）**: 不再剔除 user-site, 改为**自举 .venv 统一 numpy2 栈**——.venv 已有 numpy 2.2.6 + cv2 4.14.0 + torch 2.11 互相匹配（cv2 由 lerobot 依赖带入, 勿再单独 pip 装）。所有项目 ROS python 节点统一此模式: rclpy 用系统, numpy/cv2/torch 用 .venv。系统 cv2 4.6.0 被 apt 绑死 Ubuntu 24.04, 不升级也不被用到。
  - 曾用"剔除 user-site 回退系统 numpy1.26+cv2 4.6"方案（可用但非统一路线）, 已被上述自举方案取代。
- **实测帧率**: /dev/video0 640×480 实测 ~12.7Hz（自动曝光决定, 非 fps 参数能强拉）, 对 10Hz 决策足够。

## 2026-11-05 追加二（实机臂接入）

- **launch 已内置 follower 端口默认值**（`/dev/serial/by-id/...5B14033553-if00`）+ `port`/`calibration_file` 参数透传 demo.launch.py；`calibration_file` 默认指向安装的 `so101_hardware/config/motor_calibration.yaml`（插件对空值直接报错）。leader=ACM0 的插件缺省端口陷阱见 setup_hardware pitfalls 2026-11-05 节。

## 2026-11-05 追加三（实机首跑修复）

- **ROS 节点自举顺序**: `import numpy` 必须在 .venv 自举**之后**——ros2 run 环境下 user-site 的损坏 numpy 2.3.5 会先加载, accelerate/transformers 导入链即崩（`numpy._core has no attribute multiarray`）。纯 `python3 -c` 测试不复现（user-site 未启用）, 必须 `ros2 run` 同路径复现。
- **launch 参数名跨 include 泄漏**: 顶层 `DeclareLaunchArgument("rviz")` + include 传 `"rviz": "false"` → 顶层同名 LaunchConfiguration 被毒化为 false, 本包 RViz 节点静默不启动（无任何报错）。修法: 内部条件用独立名 `gui`（默认 true）, `rviz` 仅作兼容别名。增量对照实验法: 最小 launch → +Node → +include 逐层二分。
- **checkpoint 参数必须显式传**: 缺省空串时 load_policy 报 `config.json not found in .` 难排查。
- **实机 read 耗时**: 1M 波特率下 6 电机同步读 ~15ms, 超过 controller_manager 100Hz 周期 → 周期性 Overrun 警告。无害但若要消除可调低 `update_rate`（暂不动）。
