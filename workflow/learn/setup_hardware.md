# 实机控制复盘：收获与坑（plan/setup_hardware.md 实施）

> 作者：**glm-5.3-flash**（Claude Code 会话内运行）· 2026-10-29
> 背景：参考 lerobot-ros-moveit 为本项目添加实机 ros2_control 插件 + real:=true
> 数字孪生 + 命名姿势采集，全程 TDD（38 新增测试），经历多轮真机实测反馈
> （标定错误、RViz home 假象、plan 被拒、退出半路 ×2）后真机全链路跑通。

## 一、最大的收获

### 1. "明显准确"的对照物是最快的定位工具
用户一句"参考项目明显准确得多"把排查从猜测带入对照：
对比两份标定的 **raw tick 跨度换算角度 vs URDF 限位跨度**（4096 ticks = 360°），
立刻看出我们那份标定大部分关节只扫了一半行程（154° vs 220°），全行程线性
映射把它拉伸到全限位——所有角度放大 ~1.4 倍。
**教训：怀疑映射时，先验证映射的前提假设（量程=行程），而不是映射代码本身。**
该检查已固化进 `setup_convert_lerobot_calibration.py`（欠扫告警）。

### 2. 沉默的失败最危险，插件必须在激活时"验明正身"
连错串口（默认 ttyACM0 ≠ 实机 ttyACM1）时 6 关节 FeedBack 全部 100ms 超时，
参考代码的 `if (FeedBack(...) != -1)` 静默跳过 → `hw_positions_` 保持
on_configure 的 0 → `/joint_states` 全零 → RViz 显示 home 假象、MoveIt 从
错误起始状态规划。修复后读不全直接拒绝激活。
**教训：真机插件里"读不到"必须是 ERROR，任何静默降级都会变成显示层谎言。**

### 3. "命令到位 ≠ 电机到位"——退出保护的三轮迭代
on_deactivate 缓速回启动位姿，真机实测两轮失败才收敛：
| 迭代 | 现象 | 根因 | 修复 |
|---|---|---|---|
| v1 → v2 | 回程半路被拽停 | 主循环 write()（旧命令 speed 2400）与 park 低速插值并发打架 | `parking_` 原子标志，park 期间 read/write 让路 |
| v2 → v3 | 命令发完了臂松在半路 | 低速下伺服物理运动滞后于插值，最后一条命令刚发就失能 | 轮询实际位置（0.06 rad 容差）确认到位再失能 |
外加：回程上限 2s（第二次 Ctrl+C 会被 launch 强杀）、析构兜底失能。
**教训：写硬件的"到达"必须以传感器反馈为准，命令队列清空不是完成。**

### 4. MoveIt 的仿真默认值在真机上全是坑
- `allowed_start_tolerance` 0.01 rad：仿真零噪声通过，真机编码器噪声/齿隙
  直接 abort（"start point deviates"）→ 放宽 0.05；
- 折叠起始姿态下 URDF 碰撞 mesh 轻微相交 → CheckStartStateCollision 拒绝规划
  → SRDF 加 disable_collisions（reason=User，注明物理可达无害）。
**教训：每个"仿真里没暴露"的校验阈值，都要在真机清单里复核一遍。**

### 5. 不走 MoveIt 的退出归位是对的
用户最初希望"用 moveit 规划回初始位姿"，但 Ctrl+C 时 launch 同时杀
move_group 与 controller_manager——轨迹半路夭折、力矩半途断掉比匀速回程
危险得多。插件内插值不依赖任何外部节点，且回程目标=启动位姿（保证可达，
无需规划）。**关机路径上的行为必须不依赖"别的进程还活着"。**

## 二、踩过的坑（按主题）

### 标定与映射
| 坑 | 现象 | 规则 |
|---|---|---|
| 部分行程标定被全映射拉伸 | 两个明显不同的姿势存出相近数值、动作幅度放大 | 标定必须每关节掰满物理极限；转换脚本体检欠扫 |
| LeRobot 标定目录名不固定 | 重标定写进 `so_follower/`，脚本只找 `so101_follower/` 拿到 4 月旧档 | 自动发现扫 `calibration/robots/*/` 全部类型目录取最新 |
| 全行程映射的方向对应无先验 | range_min↔URDF lower 是假设，电机镜像安装即反向 | per-motor `--invert` 字段 + `--monitor`+twin 目视校准流程 |

### C++ 插件 / ros2_control
| 坑 | 现象 | 规则 |
|---|---|---|
| FeedBack 静默跳过 | 连错串口 → RViz home 假象 | 激活读不全 → ERROR 拒绝激活 |
| park 与 write 并发 | 回程被 speed 2400 旧命令拽停 | park 期间 read/write 让路（原子标志） |
| 插值完成即失能 | 臂松在半路 | 轮询到位（传感器反馈）再失能 |

### MoveIt / launch
| 坑 | 现象 | 规则 |
|---|---|---|
| allowed_start_tolerance 0.01 | 真机执行全部 abort | 真机 0.05 起 |
| SRDF 碰撞 mesh 余量 | 折叠姿态无法作为起始状态 | 实测无害的对用 reason=User 禁用并留痕 |
| 第二次 Ctrl+C = 强杀 | park 腰斩 | 回程 ≤2s；按一次 Ctrl+C 等它走完 |

### 工具链 / 环境
| 坑 | 现象 | 规则 |
|---|---|---|
| xacro property 把 "true" 转 bool | `${real == 'true'}` 恒 False 且静默失效 | 布尔 arg 用 `${real}` 判断（pitfalls H1） |
| 双工作区同名包冲突 | AMENT_PREFIX_PATH 挂着参考工作区，`$(find so101_description)` 解析到对方 | 测试显式按 per-package 前缀把本项目 install 排最前；同名包工作区不要同时 source |
| ament 索引是 per-package 前缀 | 裸 `install/` 加进 AMENT_PREFIX_PATH 无效 | 测试环境构造逐包列出 |
| venv 激活 + colcon | CMake 缓存 `.venv/bin/python3`，干净 shell 也 `No module named catkin_pkg` | 新窗口重跑即可（用户已确认）；必要时 `-DPython3_EXECUTABLE=/usr/bin/python3` |
| 交互采集先读后问 | 回车保存的是上一轮旧读数（off-by-one） | 读数发生在"确认"那一刻，不能在阻塞输入之前 |

## 三、遗留与后续

- `workflow/todo/to_real_joint_calibration.md`：lift ±1.91986 是手动估计，
  其余关节软限位未逐关节真机验证（6 处同步位置已列）
- move_group 退出时偶发 segfault（MoveIt 2.12 析构竞态）——不影响控制，未处理
- gripper 方向未经真机确认，`--invert` 开关备用
- 所有改动待 commit（用户批准制）
