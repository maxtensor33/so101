# 文档图片资源清单（待补）

> 使用者后期补充；生成方式已注明。补充后把对应 md 里的占位引用指向 `docs/img/` 下的实际文件。

| # | 建议路径 | 用途 | 引用位置 | 生成方式 | 状态 |
|---|---|---|---|---|---|
| 1 | `docs/img/arch_overview.png` | 全景架构图（envs / rl / robotics 三层 + ROS 边界） | docs/index.md「方向总览」+ docs/robotics/architecture.md | draw.io / mermaid → png（可先用 mermaid live 渲染） | 待补 |
| 2 | `docs/img/rl_pipeline.png` | RL 动线：训练→评估→推理→RViz（含 checkpoint/latest 指针流转） | docs/rl/index.md「Quick Start」 | draw.io / mermaid → png | 待补 |
| 3 | `docs/img/tb_scalars.png` | TensorBoard scalars 截图（eval/success_rate 曲线） | docs/rl/index.md「Tensorboard 指标」 | 训练一个 run 后 `tensorboard --logdir outputs/rl/<tag>/latest/tb` 截图 | 待补 |
| 4 | `docs/img/curve_example.png` | curve.png 示例（训练结束自动生成的 eval 曲线） | docs/rl/index.md「评估与可视化」 | 从 `outputs/rl/*/…/curve.png` 拷贝改名（真实产物已存在） | 待补 |
| 5 | `docs/img/rviz_rl_infer.png` | RViz RL 推理截图（机械臂执行策略 + Image 面板） | docs/rl/index.md「推理」 | `ros2 launch so101_rl_infer … camera:=true rviz:=true` 后截屏 | 待补 |
| 6 | `docs/img/envs_viewer.png` | envs mujoco viewer 窗口（fixed 视角） | docs/usage.md「envs 环境集」 | `scripts/envs_view_env.py` 截屏 | 待补 |
| 7 | `docs/img/envs_camera_views.png` | fixed/wrist 双相机 obs 示例（上下拼接） | docs/rl_guide.md 或 usage | `scripts/envs_check_env.py --render` 产 /tmp/envs_{fixed,wrist}.png 后拼接 | 待补 |
| 8 | `docs/img/multi_env_mem.png` | 内存 vs n_envs 实测曲线（佐证 170MB/env 红线） | docs/rl/index.md「多环境」 | 脚本循环 `--n-envs 8/16/32/48` 记录 `free -m` 峰值后 matplotlib 绘制 | 待补 |

## 约定

- 统一放 `docs/img/`（gitignore 未排除，会入库；体积大的截图建议压一压或降分辨率）。
- md 引用格式：`![说明](img/<file>.png)`（相对 docs/ 路径）。
- mermaid 图也可直接内嵌 md（无需图片文件），二选一。
