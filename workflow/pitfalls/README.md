# pitfalls — 按方向分类的坑与关键约定

CLAUDE.md 只保留全局纪律（commit 纪律）；其余坑**按方向**放本目录，
新坑追加到对应文件末尾的表格里。docs 站点的 `docs/pitfalls.md` 通过
myst include 汇总本目录（单源，勿在 docs 里直接加坑）。

| 文件 | 范围 |
|---|---|
| [general.md](general.md) | 通用/环境：残留进程、GL、sys.path 自举、colcon/pytest、清理脚本 |
| [setup.md](setup.md) | URDF/SRDF 结构约定、模型对齐验证 |
| [robotics.md](robotics.md) | 力矩控制 / draw cube |
| [mtc.md](mtc.md) | MTC 纯 RViz 模式（KDL tip、调度深水区、planning scene 共享） |
| [envs.md](envs.md) | gymnasium 环境集（夹爪语义、EGL 噪声、OOM 闪退） |
| [rl.md](rl.md) | RL 训练/推理（静默停止、registry、TB） |
| [il_sim.md](il_sim.md) | lerobot 管线（入口路径、features 键名、重置握手、设备对齐） |

补充资源：skill `ros2-jazzy-sim-integration-pitfalls`（ros2 集成通用）、
各方向 `workflow/learn/<方向>.md`（深度复盘）。
