# SO-ARM101 URDF ↔ MuJoCo XML 对齐报告

| 检查项 | 结果 | 详情 |
|---|---|---|
| joint_list | PASS | 6 joints aligned: ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'] |
| joint_limits | PASS | max |d| = 0.000004 rad at 'wrist_roll' (tol 0.001) |
| inertials | PASS | max |dm| = 0.000000 kg, max |dc| = 0.000000 m, worst '' |
| forward_kinematics | PASS | n_samples=100 max_pos=0.0025 mm (tol 1.0), max_rot=0.0009 deg (tol 0.50), worst 'moving_jaw_so101_v1_link' |
