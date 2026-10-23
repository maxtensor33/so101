# TODO: il_sim SmolVLA 接入（已降级, plan/il_sim.md）

前置（本机/目标机器）：
- [ ] HF hub 下载 `smolvla_base` 预训练权重（~1GB+，离线机器需镜像）
- [ ] 显存评估：SmolVLA finetune 的 batch_size × 分辨率组合（目标机器 GPU 表）
- [ ] lerobot 0.6.1 SmolVLA 输入特征核对（需 language instruction 字段，
  采集侧 `--dataset.single_task` 已有文本，需确认 pipeline 是否要 task 图像 token）

接入点：
- `so101_teleop/train_config.py::small_policy_config("smolvla")` 当前 raise
  NotImplementedError —— 实现时替换为 SmolVLAConfig 小网络模板
- `scripts/il_sim_train.py --policy` choices 增加 smolvla
- 推理侧 `so101_il_sim_infer.core.load_policy` 无需改动（通用加载链路）
