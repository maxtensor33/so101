# envs（gymnasium 环境集）

| 症状 | 原因与处理 |
|---|---|
| 抓取行为不符合直觉 | env MJCF 是**单动爪**夹爪，爪口沿 gripper 系 x_g 上下开合；gripper actuator 已软化（kp=400, ±1.2Nm） |
| 相机版 obs 断言确定性失败（±1 差异） | EGL 渲染位级噪声，env_checker/断言需留容差 |
| lift/place/stack 抓取保持失败 | 已知问题（`workflow/reports/envs_report.md`）；reach 100% |
| 相机版 obs 结构 | Dict{state(25), rgb{fixed,wrist}}，128×128 RGB 双视角成对输出 |
| RL 训练结束后 wezterm/vscode 集体闪退 | 多进程 vec env 内存爆 → OOM killer 连坐杀 GUI（15GB 机器 spawn 64 envs 必现）。已改单进程 DummyVecEnv/gym.make_vec；仍闪退先 `free -g`、`dmesg \| grep -i oom`，并降 `--n-envs`（内存红线 ~170MB/env） |
| 相机版环境注册名 | 无相机版与 Camera 版是不同 id（SO101Reach-v1 / SO101ReachCamera-v1），il_sim 用后者 |
