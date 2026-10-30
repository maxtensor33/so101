# il_sim Lift 专家调试问题交接 (glm-5.3 会话)

日期: 2026-10-30 · 计划: `plan/envs.md` / `plan/il_sim_leader.md` · 分支: il_sim · 相关 todo: `todo/envs_grasp_tuning_todo.md`

## 任务背景

用户要求 lift 专家在两条链路可用：`scripts/envs_run_expert.py`（uv 无头/view）与
`ros2 launch so101_teleop teleop.launch.py teleop:=expert env:=lift`（ROS2 链路, RViz+MuJoCo 查看）。
reach 早已 100%；lift 历史验收线（todo 定）为**连续 5 episode ≥80%**，当前远未达到。

## 当前状态（截至交接）

### 已完成且验证 ✅（与 lift 专家耦合但独立可交付）

| 项 | 说明 |
|---|---|
| 抓取物理修复 | 指尖碰撞网格迁移 squint 凸分解（`Fixed_part_1/2.obj` + `moving_jaw_so101_v1_convex.obj`），替换 dffb7fd 错位 4–14cm 的 pad box。闭爪平衡穿透 0.2mm；此前全量唯一失败测试 `test_lift_grasp_teleported_item` 转绿；全量 pytest exit 0 |
| can 8 环境补全 | 4 任务 × cube/can（含 Camera 变体），squint 语义（stack 只换底座）；全包短名映射补齐（cli_config/ros_viz/viz_node/rl_infer/il_sim_infer）；envs 测试 65/65 |
| envs_run_expert lift 管线 | 修复 `run_lift/place/stack` 抬升段 `_go` 默认 `open_=True` **松爪** 的重大 bug（修复前 lift 0%）；`grasp_item` 改 1.5cm 路点增量逼近（修 IK 长距离丢失，对准 1/3→4/5）；`run_stack` 降落高度读 `u.HALF_A+u.HALF_B`；新增 `--view` 可视化模式；POLICIES 接入 4 个 can id |

### 卡点 ❌：lift 成功率低 + teleop 移植版与 uv 版行为不一致

**uv 版 run_lift（`scripts/envs_run_expert.py`）**: ~25–40% 波动（多次实测 3/8、3/12、4/10、5/10）。
用户 `--view` 看到过成功 episode（~35% 概率事件）。

**teleop 版（`src/so101_teleop/.../teleoperators/expert.py`）**: 抓取 1–5 次/12 episode，
**完整成功 0 次**——抓住后全部在抬升段掉落。reach 状态机不受影响（无头 1200 帧 32 次成功）。

expert.py 现状 = **第三版**："逐字转录 envs_run_expert 为逐动作生成器"
（`env.step(a)` → `yield a`，resume 后用 `u.evaluate()`/`u.tcp_pos` 等价读原 info），
设计目的是从构造上排除转录误差。**该版未完成有效验证**：最后一次 A/B 的 A 侧
episode 计数逻辑有 bug（相位检测漏计，报 0/0 无效），仅知 16k 帧内 HOLD 出现 1 次
（异常偏低，但测量本身不可信）。**下一会话第一件事：修正计数后重跑生成器版 A/B。**

## 状态更新（2026-10-31）：lift 专家代码已整体移除 → 转 todo

按要求将 `src/so101_teleop/.../teleoperators/expert.py` 中的 lift 抓取管线
（第三版生成器 + 相关常量/分派）**整体移除**；`ScriptExpertTeleop` 恢复为
reach-only 状态机（非 reach 环境为零动作保持兜底）。后续重做、根因记录与
验收计划归档至 `workflow/todo/il_sim_expert_lift.md`。

交接前定位的关键根因（留档，重做时勿重蹈）：teleop 转录版 `servo_step_action`
把夹爪通道写死 **0.0**，而 envs `_servo_step` 读当前夹爪角 `normalize_qpos(qpos)[5]`；
0.0 = ctrlrange 中点 ≈ 半开，闭爪阶段反复弹回半开、抬升阶段抓住必掉。
修正后同进程逐帧动作序列与 uv 版 bit 级一致（seed 100/101/105/106，maxdiff=0）。
（下文"已确证事实 4"把 0.0 归到 uv 版，与代码实际相反，以本条为准。）

## 已确证事实（实测证据，勿重蹈）

1. **对准是充分条件**：闭爪前 TCP 水平残差 <5mm 时，闭爪+抬升几乎必成功（多轮一致）。
   所有失败都在"到达"环节或"保持"环节。
2. **失败模式 A——切向推飞**：全开动爪伸出 site 前方 ~9.5cm，闭合大角度段走切向，
   把方块沿桌面推走。ep2 逐帧实锤：闭爪 4 帧内方块被推 4.6cm，随后爪对空气全闭合
   （零接触）。水平逼近（低高度爬行）必经方块，推力 40N 级 >> 桌面摩擦 0.25N。
   已试无效：预压紧 8/12mm（向固定指侧偏移）、逼近途中半收爪/全收爪（全收爪改为
   指尖平推，同样推）。
3. **失败模式 B——边缘态抬升掉落**：`is_grasped` 刚过 0.5N 阈值的抓取，抬升扰动即掉
   （多处实测 grasped=True → lifted=False）。
4. **`_servo_step` 夹爪通道硬编码 0.0**（= ctrlrange 中点 ≈ 0.785 rad 半开）：
   uv 版抬升时爪是"半开搭着"方块而非"夹紧"。曾把手写移植版的抬升改为"保持当前
   抓紧角"（更紧）→ 全部掉落；改为 0.0 后闭爪阶段又抓不住（0/12）。
   **抬升段爪压的合理值是未解问题**，与失败模式 B 直接相关。
5. **IK 局部滞点**：偶发对准残差 21cm（tcp 跳走不返回）。增量逼近已大幅缓解但未根除。
6. **tool-down 可达带约束**（todo 早有记录）：item 近基座（x≲0.2）时 tool-down 不可达，
   逼近卡死带残差进闭爪。spawn 域 x∈[0.15,0.35] 的低端不可救。
7. **跨进程不可复现**：IK 的 BLAS 多线程浮点非确定性 → 同种子跨进程轨迹混沌发散，
   同 episode 结果翻转。**小样本（<12）对比不可靠**；调参必须大样本或同进程 A/B。
8. **测量陷阱**（详见 todo）：
   - `m.mesh_vert` 是全局扁平数组，必须 `mesh_vertadr/vertnum` 切片；
   - 方块传送到 site 本身是几何错误（site=全闭合收敛点，方块中心放那必与爪重叠）；
   - 渲染目视（`mujoco.Renderer` + PNG）优于一切手工坐标换算。

## 已尝试且无效/更差的方案（避免重试）

| 方案 | 结果 |
|---|---|
| 预压紧 8/12mm（TCP 向固定指侧偏移） | 抓取率不变 |
| 逼近途中半收爪 (1.2 rad) / 全收爪 | 更差或不变；全收爪改为指尖平推方块 |
| 到位后先张爪再闭爪（closed approach + open at target） | 抓取 2/10 |
| 手动状态机抬升保持抓紧角（比 0.0 更紧） | 抓住后全部抬升掉落（过挤压弹出） |
| 抓取失败后同位形重试 | 二次尝试同样失败（失败是位形依赖，非混沌） |
| 镜像手腕（y 轴取反） | 无改善 |
| 常值世界系手腕 ±90°（不随物体方位旋转） | 0/10（IK 不可达；必须用物体相对帧） |

## 未验证的候选方案（下一步优先）

1. **手腕 ±90°（物体相对帧）**：把动爪扫掠平面从"水平切向"变为"竖直径向"——
   切向推力消失，竖向推力被重力/桌面回收。实验代码已写好但被中断未跑：
   `R_des = tool_down_R(from,to) @ rot_x(±90°)`（rot_x 为绕 site x 轴旋转），
   需逐阶段（approach/close/lift）统一用同一 R。
2. **生成器版 A/B 重测**（修正 episode 计数后）：确认逐字转录版与 uv 版同率。
   若仍显著更低，逐帧对比两版动作序列（同进程同 seed 前若干帧应 bit 级一致）。
3. **抬升段爪压扫描**：失败模式 B 的正交实验——抓住后以不同 a[5] 常值（-1/0/0.3/当前角）
   抬升，找保持成功率最高的爪压。
4. **相机反馈伺服**（todo 早有建议）：根治对准残差的方向。

## 复现命令

```bash
# uv 版（无头验收 / viewer 观看）
uv run python scripts/envs_run_expert.py --env SO101Lift-v1 --episodes 10
uv run python scripts/envs_run_expert.py --env SO101Lift-v1 --view

# ros2 链路（用户实际使用场景）
ros2 launch so101_teleop teleop.launch.py teleop:=expert record:=false env:=lift rviz:=true mujoco:=true

# teleop 状态机无头仿真（record_loop 同款帧循环; 注意 episode 计数逻辑要自己写对）
# 关键: SimSO101RobotConfig(env_id="SO101LiftCamera-v1", render=False) 必须用相机版 id

# 单测
uv run pytest tests/test_envs_tasks.py tests/test_il_sim_smoke.py -q
```

## 改动文件清单（全部未提交）

| 文件 | 内容 |
|---|---|
| `libs/so101_envs/assets/so101_env.xml` + 3 个 obj | 抓取物理修复（squint 凸分解网格） |
| `libs/so101_envs/assets/scene_*_can.xml` ×4 | can 场景 |
| `libs/so101_envs/{reach,lift,place,stack,__init__}.py` | 尺寸常量类属性化 + Can 子类 |
| `scripts/envs_run_expert.py` | 抓取管线改造 + `--view` + can 接入 |
| `src/so101_teleop/.../teleoperators/expert.py` | lift 抓取生成器（第三版，未有效验证） |
| `src/so101_teleop/.../cli_config.py`、`ros_viz.py`；`src/so101_envs_viz/.../viz_node.py`；`src/so101_rl_infer/.../rl_infer_node.py`；`src/so101_il_sim_infer/.../il_sim_infer_node.py` | can 短名映射 |
| `tests/test_envs_tasks.py` | can 测试 +6 |
| `CLAUDE.md`、`workflow/todo/envs_grasp_tuning_todo.md` | 文档更新 |

## 给下一会话的建议

1. 先跑生成器版 A/B（修正计数）确定转换是否等价——这是用户的核心质疑；
2. 然后按"未验证候选方案"顺序攻失败模式 A（手腕 ±90°）与 B（爪压扫描）；
3. reach 回归（2~3/5 vs 历史 100%）**疑似 dffb7fd 求解器参数引入**，独立 bisect，勿与 lift 混在一起；
4. 与用户沟通时避免"已完成"式表述——所有成功率必须带样本量与波动范围。
