# 踩坑日志

<!-- 汇总各方向实战踩坑; 新坑追加到对应分节表末。深度复盘见 learn/rl.md -->

## 通用 / 环境

| 症状 | 原因与处理 |
|---|---|
| gz 加载了错误的硬件插件（mujoco 字样）或 RViz 报 "Frame base_link does not exist" | 残留进程发布旧 `/robot_description`——`bash scripts/clean_ros.sh` 后重来 |
| gz 中机械臂倾倒 | URDF 缺 world 固定关节（本项目已修，勿删 `world_joint`） |
| 反复 "No clock received" | /clock 桥接缺失（demo.launch.py 已含 parameter_bridge） |
| `import pinocchio` 段错误 | venv 装了 numpy 2.x——保持 `numpy<2` + `--system-site-packages` |
| move_group 启动即崩 "plugin name is empty" | MoveIt 2.12 参数名是 `planning_plugins`（复数），用 MoveItConfigsBuilder |
| ros2 CLI 没反应 | daemon 假死：`ros2 daemon stop && ros2 daemon start` |
| mujoco headless 崩溃 | `export MUJOCO_GL=egl` |
| ROS 节点报 `No module named 'so101_robotics'` | uv+colcon 混合环境 sys.path 自举（向上找 pyproject.toml）；launch 的 `additional_env` 在 Jazzy 实测不可靠 |
| ament_python 包改了 launch/urdf/rviz 后行为没变 | data_files 是拷贝非 symlink，改动后必须 `colcon build`；或代码里直接解析 src 源文件 |

## robotics（力矩）

| 症状 | 原因与处理 |
|---|---|
| ForwardCommandController 报 interface 类型错误 | `interface_name` 必须是标量 `effort`，YAML 数组会被拒 |
| 力矩保持时机械臂缓慢下垂 | 纯重力补偿是中性稳定——`torque_control_node` 锁存 `q_hold` 做 PD+重力补偿 |
| t=0 时 MuJoCo 报一次 "NaN in CTRL at ACTUATOR 0" | mujoco_ros2_control 启动瞬态，首帧命令后消失 |
| 轨迹执行中力矩饱和失控 | IK 冗余分支翻转导致相邻航点关节跳变——段时长按关节距离自适应（已实现） |

## envs / rl

| 症状 | 原因与处理 |
|---|---|
| **RL 训练结束后 wezterm/vscode 集体闪退** | 多进程 vec env 内存爆 → OOM killer 连坐杀 GUI（15GB 机器 spawn 64 envs 必现）。已改单进程 DummyVecEnv/gym.make_vec；仍闪退先 `free -g`、`dmesg \| grep -i oom`，并降 `--n-envs`（内存红线 ~170MB/env） |
| RL 训练"没动静"（进度条不动、无报错） | ① sb3 函数式 callback 返回值必须 True（None = 静默停止）；② PPO 多环境 n_steps×n_envs 预算过大网络零更新（已自动缩减）；③ TB 无事件是 skrl 的 experiment.directory 未在 agent 构造前设置 |
| `model.load(...)` 后 `gym.make` 全部 NameNotFound | sb3/skrl load 反序列化清空 gymnasium registry——load_policy 内 import so101_envs + registry 快照恢复（已实现） |
| tensorboard 打开无数据 | skrl `experiment.directory` 必须在 agent 构造前设置（构造时固化）；`write_interval="auto"` 依赖 init 的 trainer_cfg.timesteps |
| rl_eval 报 `SO101Reach doesn't exist` | 进程未 import so101_envs（registry 空）——load_policy 内已有守卫，自定义脚本需自行 import |
| 相机版 obs 断言确定性失败（±1 差异） | EGL 渲染位级噪声，断言留容差 |
| 训练产物覆盖 | 已由时间戳分层 + latest 指针解决（`libs/so101_rl/runs.py`）；显式 `--out` 仍由用户自管 |

完整深度复盘（含"重来一次"改进清单）：`learn/rl.md`。
