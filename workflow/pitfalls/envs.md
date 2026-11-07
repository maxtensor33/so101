# envs（gymnasium 环境集）

| 症状 | 原因与处理 |
|---|---|
| 抓取行为不符合直觉 | env MJCF 是**单动爪**夹爪，爪口沿 gripper 系 x_g 上下开合；gripper actuator 已软化（kp=400, ±1.2Nm） |
| 相机版 obs 断言确定性失败（±1 差异） | EGL 渲染位级噪声，env_checker/断言需留容差 |
| 抓取保持失败（历史） | **已被 2026-10-31 squint 迁移取代**：condim=4 硬接触 + 指尖 pad box 后 lift 保持 30/30、穿透 <1mm；诊断旧档见 `workflow/reports/envs_report.md`（文末更新节），迁移结论见 `workflow/reports/lift_squint_migration.md` |
| 相机版 obs 结构 | Dict{state(47), rgb{fixed,wrist}}，128×128 RGB 双视角成对输出（squint 迁移后 state 维度：reach 47 / lift 52 / place 57 / stack 57） |
| RL 训练结束后 wezterm/vscode 集体闪退 | 多进程 vec env 内存爆 → OOM killer 连坐杀 GUI（15GB 机器 spawn 64 envs 必现）。已改单进程 DummyVecEnv/gym.make_vec；仍闪退先 `free -g`、`dmesg \| grep -i oom`，并降 `--n-envs`（内存红线 ~170MB/env） |
| 相机版环境注册名 | 无相机版与 Camera 版是不同 id（SO101Reach-v1 / SO101ReachCamera-v1），il_sim 用后者 |
| RENDER_SIZE 改动是全链路单源变更 | `base_env.RENDER_SIZE` 一处定义、多处消费：envs 渲染 / teleop `_SimCamera` 占位形状 / rl dagger `render_size`+`in_hw`。改默认尺寸必须全 repo grep `RENDER_SIZE\|480, 640` 对齐（2026-11-07 曾 128→480x640→回退 128x128：640x480 软渲染 ~0.7s/帧，机器扛不住；连带 record 冒烟帧数阈值放宽、buffer_size 内存换算）。保持 tuple 形式 `(128, 128)`，teleop 用 `(*RENDER_SIZE, 3)` 单源引用 |
| 相机标定与渲染分辨率解耦 | `apply_camera_calib` 只覆写 cam_pos/quat/fovy（外参+垂直 FOV），**分辨率无关**——128x128 下标定默认开启依然成立；但非 4:3 纵横比会改变水平视野（1:1 比 4:3 窄），标定几何本身仍精确 |
