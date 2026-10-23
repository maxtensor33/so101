# il_sim（模仿学习仿真管线，lerobot 0.6.1）

| 症状 | 原因与处理 |
|---|---|
| lerobot record 报 `ModuleNotFoundError: No module named 'lerobot.record'` | 0.6.1 入口在 `lerobot.scripts.lerobot_record`（HEAD 路径不同）——代码只依赖 `tests/test_il_sim_smoke.py` 冒烟验证过的 API |
| dataset 创建报 FileExistsError | 官方 record 不覆盖已有数据集；**重录 = 手动删 `workflow/../outputs/datasets/il_sim/<repo_id>`**，或 `--resume` 续录 |
| features 键名 | `{motor}.pos: float` → observation.state/action；相机键 → `observation.images.<key>`；`cameras` 是 lerobot RobotConfig 保留名，相机子集用 `camera_keys`（VQ-BeT 只支持 1 路图像） |
| episode 间不重置 | 官方 record 无 reset 概念——env_backend 握手（teleop 请求 → send_action 帧边界执行，边界帧 action 是对旧目标计算的）；keyboard 录制 `←/→/n/r`/成功到达均自动重置（reset 计数 → 保持目标吸附新 qpos，防机械臂弹回旧位姿） |
| RViz 看不到图像/joint_states | ros_viz 线程不渲染（EGL 上下文线程绑定）——主线程渲后经 env_backend 缓存转发；确认 launch 里 ros_viz 已随环境进程启动 |
| colcon 树下 import 失败 | 包 `__init__` 须自举 repo libs/ + .venv site-packages + cmeel.prefix（pinocchio .pth 不随手动 sys.path 生效）——**自举必须在包 `__init__` 顶部**（节点自己的 bootstrap 太晚） |
| policy 推理设备不匹配（cuda vs cpu） | 以预处理器 to_device 步骤为准（checkpoint 存于 GPU 时 to cuda），`load_policy` 内对齐；**节点侧勿再 `policy.to("cpu")`** |
| SmolVLA | TODO（`workflow/todo/il_sim_smolvla_todo.md`） |
| mujoco 版本被压 | gym-aloha（lerobot[all] 带入）会把 mujoco 压到 <3.9 破坏 envs 抓取物理——pyproject 已 override-dependencies 锁 mujoco>=3.9，勿删 |
| MuJoCo 窗口/工具栏 | viewer 由 SimSO101Robot config `render` 开关（launch `mujoco:=true` 默认开，无显示环境置 false）；默认隐藏左右工具栏+最大化（`SO101*Env(viewer_ui=, viewer_maximize=)`，maximize 走 X11 EWMH，Wayland 无效静默跳过）；viewer_no_input（不抢键盘焦点）KWin 下无效=TODO，workaround=焦点留终端 |
| keyboard 录制键位 | `←/→/n/r` 取消/提前完成（lerobot 语义）+ 自动重置；`c` 手动重置；`1-6`+`w/s` 选关节/增减；`ESC/q` 停止+编码 |

完整复盘：`workflow/learn/il_sim.md`。
