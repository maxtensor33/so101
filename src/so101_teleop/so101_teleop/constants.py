"""so101_teleop 共享常量（features 键名集中定义，双端引用防漂移 — plan/il_sim.md）。"""

# SO-ARM101 六关节，与 env 动作向量顺序一一对应（qpos/ctrl 同序）
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# 相机键名：env obs["rgb"] 的键 = dataset `observation.images.<key>`
CAMERA_KEYS = ("fixed", "wrist")

# 控制频率（Hz）：send_action 一次调用 = 一个控制步，dataset fps 与此一致
CONTROL_FPS = 10
