# MTC 纯 RViz 模式复盘：收获与坑（plan/mtc_rviz.md 实施）

> 作者：**glm-5.3-flash**（Claude Code 会话内运行）· 2026-10-23
> 背景：MTC 从 mujoco/gz 双后端转向纯 RViz（mock 硬件）模式；对齐 main 的
> 文档/命名结构。经历一次计划中途改道（标准 stage 链 → 复用扁平链）与
> 多轮用户实测反馈（RViz 不显示 → 空布局 → 空任务树）。

## 一、最大的收获

### 1. 历史交接文档（problems/）是最贵的资产，但差点被浪费
problems/ 三个会话已经确认：**Connect + SerialContainer 的容器调度在
MTC 0.1.8 + 5-dof 上是深水区**（place 侧 Connect 从未触发，"0 solutions
0 failures"），最终靠扁平链才跑通。我在计划里判断"纯 RViz 没有物理问题
就能用标准 stage 链"——过于乐观，问题从来不在物理而在 MTC 调度本身，
烧掉大量时间后由用户叫停改道。
**教训：接手任何有 problems/ 交接的方向，第一件事是通读全部历史，
把"已判死"的路线列进计划的风险表，而不是重新试一遍。**

### 2. RViz 面板/Display 分离架构（MTC 可视化的正确心智模型)
Motion Planning Tasks 的 **Panel 只是 UI 壳**，任务树数据全部来自
**TaskDisplay**（rviz Display 类型，订阅 `Task Solution Topic: /solution`
并派生 `/description` 订阅）。只有 Panel 没有 Display = 永远空树，
且实测手段直接可判：`ros2 topic info /description` 看 Subscription count。
排查 GUI"不显示"类问题时，先找数据订阅者是谁、订阅建立了没有。

### 3. transient_local 数据的生命周期 = 进程生命周期
introspection 的 /description、/solution 是 transient_local（晚订阅可补发），
但 **publisher 进程退出数据即消失**。MTC 节点"跑完就退"的设计对 RViz 面板
是致命的——修复是 `keep_running` 参数（任务完成后保持 spin）。
通用规则：给"服务晚连客户端"的节点留 keep-alive 开关。

### 4. MTC 0.1.8 API 坑集（本次全部实测确认）
- **KDL IK tip 只有 `gripper_frame_link`**：`moving_jaw` 不能作 setIKFrame
  （报 `Cannot compute IK ... pose reference frame`）。T_REL 相对 moving_jaw
  的目标位姿须换算：`T_gframe_goal = T_mjaw_goal ∘ inv(T_gframe→mjaw)`。
- `GeneratePose` 是 MonitoringGenerator，**必须 setMonitoredStage(current state)**，
  否则 init 报 "no monitored stage defined"。
- 属性传播链（task.exposeTo → container.configureInitFrom(PARENT) → 子 stage）
  很脆，断在中间就报 `declared, but undefined`——**全显式 setter 更稳**。
- `SerialContainer::add/insert` 生命周期：`task_.add(std::move(container))`
  之后旧指针已空，**insert 必须在 add 之前完成**（否则 segfault）。
- rviz 面板类名 0.1.8 = `moveit_task_constructor/Motion Planning Tasks`
  （旧文档的 `moveit_task_constructor_visualization/TaskConstructorPanel` 已废弃）。
- 属性传播的 Connect 调度深水区之外，**SerialContainer 首 stage 必须是
  generator/backward propagator**（接口方向 ←→ 匹配规则见 container.cpp）。

### 5. planning scene 是共享资源，写入者要登记
我的新节点往 move_group 共享 planning scene 加了 `place_area` 视觉标记，
结果污染了同 launch 里旧节点的执行（第 8/11 段 "Trajectory component
invalid" → PREEMPT → error_code=-3）。表象在旧节点，根因在我。
**教训：向共享状态（planning scene/参数服务器/话题命名空间）写入的组件
要互相声明；执行失败先查"场景里多了什么"。**

## 二、踩过的坑（按代价排序）

### 1. 【最贵】不读历史就开干，重蹈三会话深水区
计划阶段没读 problems/（用户最后提醒才读）。标准 stage 链的实现 + 调试
消耗了会话大半时间，结论却是"历史上已证明不可行"。其中修掉的 5 个 bug
（monitored stage、容器生命周期、接口方向、IK frame、属性传播）都是真 bug，
但**都不在关键路径上**——修完仍然卡调度。

### 2. colcon symlink-install 增量编译不生效（历史坑 #1 重演）
glm 会话 problems 里明确写了"必须 --cmake-clean-first 或不用 symlink-install"。
我前期多轮 `colcon build --symlink-install` 后的"改了没变化"疑似与此相关。
**C++ 改动后一律 clean-first，别赌增量。**

### 3. gdb 调试 prefix 忘删，污染后续所有 e2e
为了抓段错误给 launch 节点加了 `prefix="gdb -batch ..."`，抓完没删——
之后几轮 e2e 全部带着 gdb 跑，OMPL 规划被拖慢几十倍，误判"planning 卡死"
并为此修了两个其实无辜的东西（IK frame、table）。发现时已烧掉约一小时。
**教训：调试用临时手段用完立即还原；e2e 环境要有"与基线配置 diff 为零"
的检查习惯。**

### 4. python str.replace 锚点静默 no-op，本会话至少 4 次
launch 的 gdb prefix 删了三轮才成功（转义/锚点问题）、IK 块替换被中间的
调试日志行打断、place 容器的 add 修复第一次没匹配——全部是静默 no-op，
靠后续报错倒查。带 assert 的替换一次就灵（learn/il_sim.md 已总结过，
执行纪律没跟上）。

### 5. find() 范围切割误删无关代码
清理标准链残留时 `t[start:find("return actions")]` 把 rviz2 的 TimerAction
块一起切掉，导致"RViz 界面未显示"。范围删除必须 assert 删除块的内容特征
（如删除块内含特定关键行），而不是只 assert 长度/存在性。

### 6. pkill -f 字面量自杀
`clean_ros.sh` 里 `pkill -f "rviz2"` 按完整命令行匹配，把我命令行里含
"rviz2" 字样的诊断 shell 一起杀了（exit 144），连续三次"莫名中断"才定位。
诊断命令里用变量拼接（`K=rviz; pgrep -cx ${K}2`）规避，或 clean 与诊断
分两个工具调用。

### 7. planning scene 大平面盒包住机器人
table CollisionObject（0.5×0.5m）把臂整个包进去 → 初始状态即碰撞 → 所有
规划 0 解、plan() 长时间不返回。旧节点注释里早写过这条结论
（"base_link 贴地，与任何地面 box 天然重叠"）——又一个没先读历史的代价。

## 三、如果重来一次

### 1. 开工前 30 分钟"历史考古"制度化
读 problems/ 全部 + 相关 learn/ + 旧节点头注释（那里浓缩了 flat-chain 的
全部理由），输出"已判死路线/已验证结论/未解深水区"三张清单进计划。
本次的计划里有环境事实表，却没有"历史禁区表"——这是结构性缺失。

### 2. 架构决策用 spike 前置，而不是写进计划赌
标准 stage 链应先做 30 分钟 spike（最小 Connect+容器 task 跑 plan），
确认调度可行再承诺架构。计划里写"如诊断确认需要则范围升级"不如
"先 spike 再定架构"。

### 3. 替换/删除一律带断言，写入前先读目标块现状
本会话 assert 保护抓住了 2 次错误锚点（place 块、rviz2 恢复），没带断言的
静默失败了 4+ 次。统一纪律：Python 盲替换全部 `assert old in t` +
`assert changed`；或者一律用 Edit 工具（不匹配会报错）。

### 4. 调试手段即用即撤，e2e 前做"环境洁净检查"
gdb prefix、临时日志、debug 参数——列表化跟踪，e2e 前逐项确认已还原。
可以约定：临时调试改动 commit 前必须 grep 确认（gdb/print/prefix 关键词）。

### 5. 卡 30 分钟无进展就向用户升级，附选项
本次在标准链 planning 卡点上连续自我重试（gdb attach、语句级日志、
多轮重跑），直到用户两次打断才停下来对齐。更好的模式：
第一次"计划外卡点"出现时就汇报"卡点+已排除项+候选路线"，让用户决策
（用户后来一句话就终结了争论：别人都踩过坑了）。

### 6. GUI 验收尽早交给用户
headless 能验证的（话题存在、订阅计数、进程存活、error_code）我都验证了，
但"任务树显示"只有用户看得到。本次 RViz 显示问题分了三轮暴露
（不显示→空布局→空树），若第一轮就让用户开着 RViz 跑并同步反馈，
三轮可并成一轮。

### 7. 共享 planning scene 的组件加"写者登记"注释
在 spawnCollisionObjects 类函数头部注释声明：本节点向 move_group 共享场景
写入了哪些 object id；多节点 launch 时先核对场景写入者清单。place_area
事件本可由一行注释避免。
