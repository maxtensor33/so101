# robotics：力矩控制学习栈（教学目的）

**定位**：以力矩控制为主线的机器人学学习栈——FK/Jacobian/IK/动力学/轨迹/控制
→ draw cube 验收。对标 [mujoco-robotics-lab lab-2 (UR5e)](https://github.com/ozkannceylan/mujoco-robotics-lab)
的 A2~A5 / B1~B3 / C3 结构；**力矩控制是教学重点，非生产链路**。

## 构成

- `libs/so101_robotics/`——无 ROS 依赖学习库：
  `common/fk/jacobian/ik/dynamics/trajectory/control/constraints/sim`
- `so101_robotics_torque_app/`——ROS 链路（torque_control_node 100Hz +
  draw_cube_node + launch/rviz）
- `scripts/robotics_draw_cube_viewer.py`——MuJoCo viewer 版（500Hz 内环，无 ROS）

## 页面

- {doc}`architecture`——模型体系（xxx_torque）、力矩控制分层、实测基准、lab-2 对照
- {doc}`usage`——draw_cube launch 全参数、viewer 用法、运行时检查

## 与其他方向的关系

- `libs/so101_robotics` 仅供本方向与 torque 学习使用；**envs/rl 方向不 import 它**
  （envs 用 mujoco 位置伺服 + pinocchio）。
- 位置控制版本模型（`so101_new_calib.xml/urdf`）由 setup 方向的对齐脚本维护，
  envs 方向的 MJCF 从同源派生。
