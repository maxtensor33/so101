# 相机标定详解（内参 + 手眼）

> 来源: 内参/手眼方法学迁移自 [so101-ros-physical-ai](https://github.com/dmlevi/so101-ros-physical-ai)
> （Apache-2.0, © 2026 Dmitri Manajev），采集端为本项目重写。包: `src/so101_camera_calibration`。

## 1. 标定是什么？为什么需要？

**内参（intrinsic）**：描述相机自身的成像参数——焦距（fx, fy）、光心（cx, cy）、镜头畸变。
它回答"图像上的一个像素对应现实中哪个方向"。**畸变**是镜头的物理缺陷（直线被拍成弯曲）。

**外参（extrinsic）/手眼（hand-eye）**：描述相机相对机械臂的安装位置和朝向。
它回答"相机看到的东西在机械臂坐标系里的位置"。名字来历: 相机（眼）装在手上/桌上。

**为什么本项目需要**:

1. **sim2real 闭环**（核心动机）: 每位开发者的 fixed（顶部）/wrist（腕部）相机安装都不同。
   先标定 → 把标定结果喂给 envs 仿真（`camera_calib` 参数）→ 仿真渲染 = 你的真机视野 →
   在 envs 里训练 → 部署回真机。视野一致，学习算法才能无缝迁移。
2. **感知基础**: 知道像素和机械臂坐标的关系，才能从图像算出物体在臂坐标系的位姿
   （数字孪生物体同步、RL state 观测的 item_pose 都依赖它）。
3. **RViz 可视化**: 相机 TF 正确后，图像内容可以在 3D 场景中投影显示。

## 2. 标定原理（30 秒版）

- **内参**: 拿一块已知几何尺寸的棋盘格（ChArUco = 棋盘格 + 二维码标记），从不同角度拍它。
  每张图里角点的像素位置已知、物理位置已知，解方程反推出相机参数。张数越多、角度越杂、
  覆盖画面越满，结果越准。**判据: 重投影误差 < 0.5px 很好，< 1px 可用**。
- **手眼**: 经典的 AX = XB 问题。机械臂动 N 次（每次位姿已知），相机看固定板（每次板相对相机的
  位姿可由内参算出），两组相对运动联立解出相机-机械臂的固定变换。
  **判据: PARK 与 HORAUD 两种解法的结果一致（平移差 < 1cm）才可信**——这是原项目的经验准则。

## 3. 第一步：打印标定板（阻塞后续所有步骤）

生成（已生成在 `outputs/calibration/`，也可重新生成）:

```bash
uv run python src/so101_camera_calibration/scripts/gen_charuco_intrinsic.py  # 内参板
uv run python src/so101_camera_calibration/scripts/gen_charuco_handeye.py    # 手眼板
```

| 文件 | 规格 | 用途 |
|---|---|---|
| `charuco_intrinsic_A4.pdf` | 8×6 方格，方格 25mm / 标记 18mm，DICT_5X5_250 | 内参（手持移动） |
| `charuco_handeye_A4.pdf` | 4×5 方格，方格 15mm / 标记 11mm，DICT_4X4_50 | 手眼（贴夹爪，裁小） |

**打印要求（缺一不可）**:

1. 打印对话框选 **实际尺寸 / 100%**——绝不能"适应页面"（尺寸错=标定错）。
2. **亚光纸**（哑光）——光面纸反光会毁掉检测。
3. 打印后用**卡尺量任一方格边长**: 内参板 25.0mm、手眼板 15.0mm（±0.2mm）。
4. 内参板贴硬纸板/铝板**压平**（翘曲=误差）；手眼板裁小块**紧固**在夹爪上。

## 4. 第二步：内参标定（两路相机各做一次）

```bash
# 底座 + 双相机起起来（real:=true; 不驱动臂）
ros2 launch so101_il_sim_develop il_sim_develop.launch.py checkpoint:=<不需要> gui:=true
# 或单独起相机流后:

ros2 run so101_camera_calibration camera_intrinsic_calibration_node \
  --ros-args -p image_topic:=/real/fixed/image_raw \
             -p output_dir:=outputs/calibration/fixed
# wrist 相机: image_topic:=/real/wrist/image_raw output_dir:=outputs/calibration/wrist
```

浏览器打开 `http://localhost:8080`（Viser Web UI，无桌面也能用）:

1. 手持内参板在相机前**缓慢移动**，覆盖画面中心/四角/边缘（UI 有覆盖网格）。
2. 每个位置**改变倾角**（前后俯仰、左右旋转）。
3. 采样 ~25-40 张后点 **Calibrate**，看重投影误差；点 **Save**。

产出: `outputs/calibration/<相机>/camera_cal.yaml`（ROS camera_info 格式）+ `.npz`。

## 5. 第三步：手眼标定（自动模式一键跑）

前置: 内参已完成；夹爪上贴好手眼板（正对相机）；工作区清空。

```bash
ros2 launch so101_camera_calibration camera_calibration.launch.py \
  camera:=eye_in_hand \
  intrinsic_yaml:=outputs/calibration/wrist/camera_cal.yaml
```

- `camera:=eye_in_hand`（wrist 相机, 板放桌上固定）/ `camera:=eye_to_hand`（top 相机固定, 板贴夹爪）。
- **auto 模式**: 节点按 `config/calibration_poses.yaml` 逐姿势驱动机械臂（24 个姿势，
  强制旋转多样性）→ 每姿势稳定后检测板（角点数/重投影质量门控，不合格自动跳过）→ 全部完成自动求解。
  随时可停: `ros2 topic pub --once /camera_calibration/estop std_msgs/msg/Empty`。
- **manual 模式**（自动姿势集不适用时兜底）: `mode:=manual`，用 leader 臂遥操摆姿势，按 Enter 采样。
- 求解判据: **PARK/HORAUD 平移差 < 1cm** 才可信；不一致就补样本（更多旋转多样性）。

产出: `outputs/calibration/handeye_<camera>.yaml`。关键字段:

| 字段 | 含义 |
|---|---|
| `parent_frame` | 相机位姿的父系: eye_in_hand → `gripper_frame_link`; eye_to_hand → `world` |
| `optical_frame.t/rpy_deg` | **相机在父系中的位姿**（不是它的逆）; envs 导出的输入 |
| `camera_link.xyz/rpy_deg` | 换算到 camera_link 的位姿（静态 TF 用） |
| `consistent` | PARK/HORAUD 平移差 <1cm 的质量判据 |
| `candidates` | 4 种输入方向约定的诊断解; 权威解固定为 `g2b×t2c 原始`, 板刚性 std 越小越可信 |

> **方向约定（最容易错）**: `cv2.calibrateHandEye` 返回的 `cam2gripper / cam2base`
> 已经是"相机在父系中的位姿", **不要再取逆**。eye_in_hand 的父系是 TCP
> `gripper_frame_link`, 不是 MJCF 的 `gripper` body（二者差一个固定关节）。

> 精度预期（如实）: SO-101 舵机重复性 ~1-2°，手眼结果有硬下限（原项目标注为实验性）。
> 把它当"起点"而非"真值"；对照 RViz 中 TF 与图像投影再微调。

### 5.1 视野对照验收（compare_views）

标定 + 导出后，用 leader 同时驱动 **真机 follower + envs 仿真**，在 RViz 四窗格并排看视野：

```bash
# 前置: follower 上电、双相机已接、leader 在 /dev/ttyACM0; 工作区清空
ros2 launch so101_camera_calibration compare_views.launch.py \
  camera_calib:=outputs/calibration/envs_camera_calib.yaml [env:=lift] [mujoco:=true]
```

- 拖动 leader: 真机 follower 与仿真同步运动（JTC 跟随 `/arm_controller/joint_trajectory`）。
- RViz 窗格: `/compare/sim/{fixed,wrist}` 与 `/real/{fixed,wrist}/image_raw`。
- 验收标准（**同一 leader 位姿**下）:
  - fixed: 机器人/桌面在画面中的位置、大小、俯视角一致（本机安装在左前，俯视）;
  - wrist: 夹爪与桌面的相对朝向一致，不能出现上下颠倒/镜像;
  - 拖动过程中 sim 与 real 的关节方向一致（方向反了先查 leader `invert`，不是相机标定）。
- **不一致时不要手调相机**: 按顺序检查 ① 手眼变换方向（不能取逆）② MuJoCo 四元数右乘
  ③ wrist 的 `gripper_frame_link→gripper` 换算; 用已有样本重新求解/导出即可，无需重采。

## 6. 第四步：结果应用

### 6.1 训练侧——envs 相机定制（闭环关键）

```bash
# 汇总两路标定结果为 envs 格式
uv run python src/so101_camera_calibration/scripts/export_envs_camera_calib.py \
  --handeye outputs/calibration/handeye_eye_in_hand.yaml --intrinsic outputs/calibration/wrist/camera_cal.yaml \
  --handeye outputs/calibration/handeye_eye_to_hand.yaml  --intrinsic outputs/calibration/fixed/camera_cal.yaml
```

```python
import gymnasium as gym, so101_envs

# sim 渲染 = 你的真机视野
env = gym.make("SO101Lift-v1", camera=True,
               camera_calib="outputs/calibration/envs_camera_calib.yaml")
```

原理: 覆写 MuJoCo 相机的位置/朝向（手眼外参）与视场角（内参 fy 换算 fovy）。
**近似性**: MuJoCo 不支持镜头畸变与主点偏移——只对齐视场+位姿，畸变残差通常可接受。

坐标系注意（2026-11-06 修复）:
- 手眼 YAML 的 `optical_frame` 是**相机在标定父系中的位姿**（eye-in-hand 父系
  `gripper_frame_link`；eye-to-hand 父系 `world`），不是它的逆。
- `fixed_cam` 在 MuJoCo 中挂 world，位姿可直接使用；`wrist_cam` 挂 body `gripper`
  （= URDF `gripper_link`），而手眼父系是 TCP `gripper_frame_link`——导出脚本会自动
  左乘二者间的固定关节变换，**不要手工把 handeye 的 t/rpy 直接写进 wrist 段**。
- 若 sim 相机位置/朝向与真机成镜像或反向，先检查这三处约定（详见
  `workflow/pitfalls/setup_camera_calibration.md`）。

### 6.2 部署侧——ROS 静态 TF

标定 YAML 里的 `static_tf_snippet` 直接粘进 launch，或用 `camera_link` 字段写 URDF。

- `eye_to_hand`(fixed): snippet 父系是 `world`，静态 TF 成立（相机不随臂动）。
- `eye_in_hand`(wrist): snippet 父系是 `gripper_frame_link`（随臂动）。
  静态 TF 的父系必须是这个运动链节点，**不要改成 world**——否则相机不会跟臂走。
- `camera_link` 与 optical 的固定旋转差 `rpy=(-π/2,0,-π/2)` 已由 `optical_to_link` 换算。
- 之后 RViz 中相机 TF 与图像投影一致，感知节点可查 `base→camera_optical` TF。
  MuJoCo 训练侧不需要这段 TF，直接使用 6.1 的 `envs_camera_calib.yaml`。

## 7. 故障排查

| 现象 | 原因/处理 |
|---|---|
| 检测不到角点（绿点） | 光照不足/反光/板翘曲——换亚光+压平+补光 |
| 重投影误差 > 1px | 样本角度太单一/张数不够——加倾角多样性再采 |
| PARK/HORAUD 差 > 1cm | 旋转多样性不足——补不同倾角样本; 或改 manual 模式 |
| auto 采样大量被拒 | 板被遮挡/出视野——微调 poses_yaml 对应姿势; 参考每姿势日志 |
| 标定结果每次差很多 | 相机松动——紧固后重标; Euler 角在 pitch≈90° 附近会万向锁（用 optical 系数值对比） |
| `numpy._core`/`cv_bridge` 报错 | 本仓库相机链路已全部规避 cv_bridge——不要在节点里引 cv_bridge |
| sim fixed 在右后、真机在左前 | 手眼结果被多取逆了——`optical_frame` 必须是"相机在父系"的位姿; 用现有样本重求解即可 |
| sim wrist 朝上/镜像 | ① `R_mj = R_opt @ CONV` 是否右乘 ② 是否做了 `gripper_frame_link→gripper` 换算 ③ 重跑 `export_envs_camera_calib.py` |
| compare_views 中 sim/real 不同步 | 先查 leader `port`/`invert` 与 follower 底座; 相机标定只影响"视野内容", 不会导致关节不跟随 |

## 8. 本机实测结果（2026-11-06）

供后续对照/回归参考（换机、换相机安装后必须重标，不能沿用）:

| 项 | 数值 |
|---|---|
| fixed 内参 | fx=440.66, fy=441.15, cx=316.54, cy=243.63（640×480） |
| wrist 内参 | fx=683.10, fy=684.12, cx=327.29, cy=255.93（640×480） |
| eye_to_hand (fixed) | t=(0.3057, 0.1459, 0.7088)m, rpy=(-174.27, -4.02, 88.84)° |
| eye_in_hand (wrist) | t=(-0.0303, 0.0723, -0.0871)m, rpy=(-27.24, -3.01, -168.81)° |
| 手眼质量 | 20 样本/路, 重投影 0.09–0.41px, PARK/HORAUD 一致 |
| envs 最终值 | fixed `(0.3057,0.1459,0.7088)` fovy 57.1°; wrist `(0.0224,0.0720,-0.0110)` fovy 38.7° |

对应本机生成物 `outputs/calibration/envs_camera_calib.yaml`（gitignore，不入库）。
完整验收记录见 `workflow/reports/setup_camera_calibration.md`；坐标系事故详见
`workflow/pitfalls/setup_camera_calibration.md`。
