# il_sim 项目复盘：收获与坑（plan/il_sim.md 实施）

> 作者：**glm-5.3-flash**（Claude Code 会话内运行）· 2026-10-23
> 背景：lerobot 0.6.1 协议适配（采集/训练/推理全链路）+ ROS2 同步可视化，
> 全程 TDD（全量 257 passed，il_sim 新增 37 项），经历多轮用户实测反馈
> （RViz 空布局、MuJoCo 无窗口、左键不重置、箭头键失效、机械臂弹回旧位姿）。

## 一、最大的收获

### 1. 协议适配层的价值：lerobot 协议一次读懂，il_real 免费复用
`SimSO101Robot`/`ScriptExpertTeleop` 实现 lerobot 的 Robot/Teleoperator ABC 后，
采集完全复用官方 `lerobot-record` 管线（数据集写入/视频编码/键盘控制/episode 管理
零自研）。features 键名约定搞清一次就终身有效：
- `{motor}.pos: float` → 官方推导管线自动派生 `observation.state`/`action`（shape 合并）
- 相机键 `(H,W,3)` → 自动派生 `observation.images.<key>`
- 用 `aggregate_pipeline_dataset_features` 写契约测试锁定派生结果，双端永不漂移。

### 2. 读上游源码先于写代码，且版本必须对齐
plan 写的是 lerobot HEAD 的模块路径（`lerobot.record`），实际 0.6.1 是
`lerobot.scripts.lerobot_record`。Phase 0 用冒烟测试把所有要用的 API 逐个 import
一遍（record/train/teleoperate/ABC/register_subclass/4 policies），后续代码只依赖
冒烟验证过的接口——这一步让后面的 RED 全部是"真失败"而非 import 噪声。

### 3. 单进程共享 env 是 Teleoperator 闭环的干净解法
lerobot 协议里 `Teleoperator.get_action()` 无 obs 入参，脚本专家却需要闭环（读
qpos/tcp/goal）。`env_backend` 模块级注册点（Robot.connect 注册 → teleop 读取）+
reset 握手（teleop 请求 → robot 帧边界执行）解决了三件事：
- 专家闭环；- episode 自动重置（官方 record 无 reset 概念）；- reset 后 teleop
状态对齐（reset 计数）。跨进程不可用是明说的约定，不是缺陷。

### 4. 帧边界对齐思维贯穿始终
EGL 上下文线程绑定 → ros_viz 线程不渲染，主线程渲后经 env_backend 缓存转发；
env.reset 只在 send_action 末尾执行 → reset 恰好落在两帧之间，观测/动作配对不破。
凡是"多线程/多阶段"问题，先问帧边界在哪。

## 二、踩过的坑（按代价排序）

### 1. 【最坑】依赖重解析的连锁反应：lerobot[all] 压 mujoco
`lerobot[all]==0.6.1` 带入 gym-aloha → `mujoco<3.9` → mujoco 从 3.16 压到 3.8.1：
envs 抓取物理测试失败（`test_lift_grasp_teleported_item`）+ `mj_fullM` 绑定签名
改变（3.16 是 `(m, data, dst)`，3.8 是 `(m, dst, M)`，方向键恰好相反）。
修复：pyproject `override-dependencies = ["mujoco>=3.9"]`。
**教训：新增重依赖后必须全量跑既有测试；两个失败一起出现时先查共同根因（版本）。**

### 2. pytest 全量必须 ROS 环境，且要小心"验证假象"
- 裸 uv venv 缺 rclpy → 8 个既有 ROS 测试 ERROR；
- 更险的一次：E2E "验证通过" 读的 `info.json` 其实是用户旧跑的残留（`grep|head||echo`
  管道让"无报错"永远不会打印），而实际 FileExistsError 还在——`remove_existing_dataset`
  的调用补丁锚点没匹配上，静默 no-op，我却以为修好了。
**教训：E2E 断言必须能区分"新产出"和"旧残留"（时间戳/内容哈希）；shell 管道的
  `|| echo` 写在 head 之后是无效的。**

### 3. colcon install 树下的导入自举必须在包 `__init__`
ROS 节点自己的 sys.path 自举太晚——包 `__init__` 先执行就 import lerobot/torch 了。
正确做法：`so101_teleop/__init__.py` 顶部注入 repo `libs/` + uv venv site-packages +
**cmeel.prefix**（pinocchio 的 .pth 机制不随手动 sys.path 生效）。
另外 setuptools>=80 把 console_scripts 装到 bin/，ros2 run 需要 data_files 装
lib/<pkg>/ wrapper（envs/rl 已踩过，il_sim 复用该模式）。

### 4. data_files 漏一行 = RViz 空布局
`so101_teleop/setup.py` 漏了 rviz 目录的 data_files 行，`rviz2 -d` 指向不存在的
文件 → RViz 打开默认空布局（没 robot 没图像），且进程正常不报错，极难定位。
**教训：新增资源目录时同步改 setup.py data_files；用"消费端能读到文件"做验收。**

### 5. GL/线程的三条铁律
- EGL 上下文线程绑定：A 线程创建的 Renderer 不能在 B 线程 make_current →
  渲染只发生在主线程，其他线程只读缓存；
- MuJoCo viewer（GLFW）与 Renderer（EGL）可同进程共存，但 viewer 只能在主线程；
- policy 推理设备以**预处理器 to_device 步骤**为准（checkpoint 存于 GPU 时 to cuda），
  权重必须对齐同一设备，节点侧勿再 `policy.to("cpu")`（这个 bug 我修了设备对齐后
  又被节点里的旧语句打回，排查了两轮）。

### 6. pynput 特殊键：方向键的 char 是 None
`Key.right/Key.left` 走 `key.name`（"right"/"left"），字符键才走 `key.char`。
只转发 char 的监听器会静默丢掉所有箭头键——用户按 ← 没反应而字母 n/r 正常，
正是这种"一半通"最难查。

### 7. 位置控制 + reset 后的"弹回"问题
env.reset 把 qpos/ctrl 设回 rest，但 keyboard teleop 持有旧目标快照 → 下一帧
send_action 把 ctrl 写回旧目标，机械臂立刻弹回旧位姿，看起来像"reset 没生效"。
修复：env_backend reset 计数 → teleop 检测计数变化 → 保持目标吸附新 qpos。

### 8. lerobot 的两个保留名/硬限制
- `cameras` 是 RobotConfig 保留名（真机 dict[str, CameraConfig]），相机子集只能叫
  `camera_keys`；
- VQ-BeT 硬性只支持 1 路图像（validate_features），双相机数据集直接 ValueError。
  顺带：`teleop.connect()` 里 `__init__` 漏初始化的 `_connected` 会在官方 record 的
  异常清理路径暴露（`teleop.is_connected`）——协议实现的每个属性都要在构造器就位。

### 9. X11 窗口操控的 API 位置
python-xlib 的 `ClientMessage` 在 `Xlib.protocol.event`（`Xlib.ext.protocol` 不存在），
try/except 吞掉 ImportError 后 maximize 静默失效。viewer_no_input（WM_HINTS
input=False）在 KWin 下实测无效，已列 TODO（todo/il_sim_viewer_no_input_todo.md）。

## 三、如果重来一次

### 1. Phase 0 就该锁定"依赖矩阵"，而不是只锁 API
本次最大的返工来自 mujoco 版本被压。重来会在 Phase 0 冒烟加两条：
`mujoco.__version__` 断言 + 全量既有测试跑一遍，把依赖冲突消灭在写代码之前。

### 2. 每个补丁补丁后立即验证"真的写进去了"
本次多次因 `str.replace` 锚点不匹配静默 no-op（remove_existing_dataset 调用、
launch 的 reset_time_s 变量声明），都是后续报错才倒查出来。重来会统一用
"replace 后 assert 新内容 in file"的写法，或者用 Edit 工具（不匹配会报错）而非
python heredoc 盲替换。

### 3. 端到端验证脚本要有"新鲜度断言"
验证数据集/日志时先记录 mtime 或清理现场再跑，并避免 `grep | head || echo` 这种
永不触发的降级分支。本次一次假阳性验证差点把 bug 带给用户。

### 4. 先问用户"冲突"的具体现象再动手
"mujoco 键盘快捷键与监听冲突"我猜了三种方案（焦点门控/换键位/WM_HINTS），最后
用户说"焦点留在终端就行"。多轮实测反馈的场景下，先复述现象+给出候选解释让用户
勾选，比闷头试错省得多。

### 5. 长 token 生成对自己不可靠
本会话 robot.py / 测试文件多次在长 Write/heredoc 中途出现内容截断、占位词残留
（"placeholder"、walrus 残片）。后来改成"小段写入 + 逐段 ast.parse 语法检查"才
稳定。重来会一开始就采用小分段写入 + 写后校验的纪律，而不是信任单次长输出。

### 6. 用户实测是最快的验收通道
RViz 空布局、左键不重置、机械臂弹回——三个 bug 都是我 headless 验收覆盖不到、
用户一眼看到的。launch 类 GUI 验收的价值排序应该是：用户实测 > 窗口/话题探针 >
单元测试，别试图用合成按键完美模拟真人操作（XTEST 注入验证绕了很远）。

### 7. 键位设计先查所有消费方
lerobot record 自带 n/r/q/箭头，mujoco viewer 绑了几乎所有字母，我们又要 w/s/1-6/c
——三方重叠。重来会在定键位表前先列"同一键盘的所有监听者"，能复用官方键位语义的
（n/r/箭头）直接复用并叠加我们的副作用（自动重置），而不是另造一套。
