# TDD 证据报告: setup_camera_calibration（plan/setup_camera_calibration.md）

日期：2026-11-05 · 分支：main · 测试运行器：`uv run pytest`

## 来源计划

`workflow/plan/setup_camera_calibration.md`（用户 `/ecc:tdd-workflow` 触发）。

## 用户旅程

1. 作为开发者，我要生成 ChArUco A4 标定板并按规范打印，以便进行标定。
2. 作为开发者，我要用 Viser Web UI 对 fixed/wrist 双相机做内参标定，以便后续手眼与感知使用。
3. 作为操作员，我要一键自动采集手眼样本（或 leader 遥操手动补采），以便不必手工摆 20-30 个姿势。
4. 作为开发者，我要标定结果自动导出为 envs 相机定制 YAML，以便 sim 渲染对齐我的真机视野后再训练。
5. 作为操作员，我要标定结果生成 ROS 静态 TF 片段，以便部署侧 RViz/感知直接使用。

## 任务报告

| 任务 | 结果 |
|---|---|
| Phase 0 | `uv add viser`（1.1.1, 纯 Python）; 基线回归 459 passed |
| Phase 1 RED→GREEN | `TestBoardGenerators`×8 + 包契约×3 首跑 11 failed（模块缺失, 编译期 RED）→ `board_gen.py` 共享模块 + 2 CLI → 11 passed; 产出 `outputs/calibration/charuco_{intrinsic,handeye}_A4.{png,pdf}`（几何/角点可检/物理尺度/_pdf A4 pt 全部断言） |
| Phase 2 RED→GREEN | `TestIntrinsics`×6 首跑 6 failed → `intrinsics.py` 纯函数模块（提取自参考节点）+ 节点迁移（弃 cv_bridge/手动解码/output_dir 参数/自举）→ 17 passed; **合成数据闭环: 已知 K=800 的 6 视图 → K 恢复 ±5%、重投影 <1px**；冒烟: 伪板图流 → 检测 28 角点 + viser HTTP 200 |
| Phase 3 RED→GREEN | `TestHandeye`×4 首跑 4 failed → `handeye_core.py`（cv2.calibrateHandEye PARK/HORAUD + eye-in-hand/eye-to-hand 双排布 + 光学系换算）→ **合成闭环: 已知真值 X 双解恢复 <5mm; 一致性判定; 换算与参考 README 数值对齐** → 21 passed; ROS 壳 `handeye_node.py`（auto 状态机/manual Enter 采样/estop/质量门控）+ poses YAML×24 + launch |
| Phase 4 RED→GREEN | `TestEnvsCameraCalib`×5 + `TestExportEnvsCalib`×3 首跑 7 failed → `libs/so101_envs/camera_calib.py` + `base_env camera_calib 参数` + 导出脚本 → **29 passed; 全量回归 488 passed（+29, 无回归）**; 集成: 标定覆写后 fixed_cam 渲染可见差异 |
| Phase 5 | 真机项阻塞于 A4 打印——用户执行（checklist 见 reports） |
| Phase 6 | docs 中文详解 + README 引用 + todo 回链 + CLAUDE.md |

## 测试保证表

| # | 保证 | 测试 | 类型 | 结果 |
|---|---|---|---|---|
| 1 | 包结构/署名（Apache 头 + 来源注明） | `TestPackageLayout` | 契约 | PASS |
| 2 | A4 板: 输出文件/A4 几何/角点全检出/物理尺度/PDF 实际尺寸 | `TestBoardGenerators`×8 | 单元 | PASS |
| 3 | 内参: 字典解析/参数距离/rgb8 解码（无 cv_bridge） | `TestIntrinsics` | 单元 | PASS |
| 4 | 内参: 已知 K 合成闭环恢复 ±5%、重投影 <1px | `test_calibrate_recovers_known_K` | 单元(闭环) | PASS |
| 5 | 手眼: eye-in-hand/eye-to-hand 合成闭环双解 <5mm | `TestHandeye` | 单元(闭环) | PASS |
| 6 | 手眼: Park/Horaud 一致性判定 | `test_consistency_check` | 单元 | PASS |
| 7 | 换算: optical→link 与参考 README 数值对齐 | `test_optical_to_link_matches_reference` | 单元 | PASS |
| 8 | envs: camera_calib 覆写 model 字段; None 回归一致; fovy 换算 | `TestEnvsCameraCalib` | 单元/集成 | PASS |
| 9 | 桥接: 手眼+内参 YAML → envs 汇总 YAML（相机映射/四元数/fovy） | `TestExportEnvsCalib` | 单元 | PASS |
| 10 | 集成: 覆写后渲染可见差异 | `test_render_changes_with_calib` | 集成 | PASS |
| 11 | 手眼: 物理正确约定下合成闭环（平移 <5mm + 旋转 <0.01rad）, 双向 | `test_synthetic_eye_{in,to}_hand` | 单元(闭环) | PASS |
| 12 | 手眼: 板刚性排序选中文档约定, 不选弱先验 | `test_variants_prefer_documented_convention` | 单元 | PASS |
| 13 | 导出: 四元数必须右乘 `R_opt @ CONV`（非对称旋转回归） | `test_export_quat_uses_right_multiplication` | 单元 | PASS |
| 14 | 导出: wrist 父帧换算常量与 URDF `gripper_frame_joint` 一致 + 组合结果 | `test_gripper_tcp_constant_matches_urdf` / `test_export_wrist_composes_tcp_to_body` | 单元 | PASS |

> 2026-11-06 补丁: 真机对照发现 sim 相机整体反向, 根因是手眼结果多取逆、MuJoCo 四元数转换左乘、wrist 父帧未换算（详见 pitfall）。以上 #11–#14 为回归保证; 现有 20 组真机样本重求解后 `envs_camera_calib.yaml` 已更新, 包内 33 passed。

## 覆盖率与已知缺口

- 真机路径（相机实拍/串口驱动/真机求解）无法本会话验证——Phase 5 留用户，合成数据已闭环算法正确性。
- `handeye_node.py` ROS 壳（状态机/TF 回调）未做单测（与 il_sim_develop_node 同等对待, 真机冒烟覆盖）; `intrinsics.py`/`handeye_core.py` 纯函数全测。
- 发现并修复: OpenCV 4.14 移除 `aruco.calibrateCameraCharuco` → 双 API 兼容; 节点迁移时 docstring 吞代码事故 → 重构文件头（详见 pitfalls）。

## 合并证据

checkpoint commits 待用户批准。建议: `test(camera_calibration): 标定包 TDD 测试先行（RED）` / `feat(camera_calibration): 内参+手眼标定迁移与 envs 相机定制接入（GREEN）`。
