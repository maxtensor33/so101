# so101_camera_calibration

SO-101 相机内参标定与手眼标定工具。

**来源**: 迁移自 [so101-ros-physical-ai](https://github.com/dmlevi/so101-ros-physical-ai)（本地路径
`/home/maxtensor/Third/Projects/github/so101/so101-ros-physical-ai`）的 `so101_camera_calibration` 包,
Apache License 2.0, © 2026 Dmitri Manajev。

**改动范围**: 生成器共享模块化（board_gen）、弃 cv_bridge（numpy2 兼容）、输出路径 outputs/calibration、
手眼节点采集端重写（leader 遥操/JTC 自动模式替代原 IK 服务）。求解与验证方法学沿用原项目
（ChArUco + Park/Horaud 双解一致性）。

详细中文说明见 `docs/hardware/camera_calibration.md`。
