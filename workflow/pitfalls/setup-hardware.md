# setup-hardware — 实机控制方向坑与关键约定

来源会话: workflow/plan/setup_hardware.md (2026-10-29, setup 分支)。

| # | 坑 | 根因 | 解法 |
|---|---|---|---|
| H1 | **xacro property 把 "true" 自动转成 bool** | xacro 对 property value 做 Python eval，"true" 是字面量 → bool；"none" 不是字面量保持 str | 条件分支用 `${real}` 布尔判断；`${real == 'true'}` 恒 False（bool 与 str 比较恒 False）且**不报错**，分支静默失效 |
| H2 | **ElementTree 往返重排 SRDF** | ET.tostring 改自闭合标签空格、默认丢注释 | SRDF 更新用文本级插入/删除（`</robot>` 前插入 + 同名块 regex 删除），其余字节不变 |
| H3 | **so101_bringup 安装不存在的 config 目录** | CMakeLists `install(DIRECTORY launch config urdf worlds)` 而 src 无 config/，旧 install 符号链接掩盖 | 干净构建必现 `ament_cmake_symlink_install_directory can't find`；install 清单与实际目录保持一致 |
| H4 | **install(PROGRAMS) 缺执行位** | 源文件 644 时 symlink-install 保留源权限，`ros2 run` 报 "No executable found" | 源脚本 `chmod +x` |
| H5 | **twin 节点 DEFAULT_MODEL 相对脚本目录** | `parents[1]` 拼出 `src/mujoco/...`；symlink-install 下 `__file__` 指向源文件 | `Path(__file__).parent / "mujoco/scene.xml"`（同级 mujoco 目录） |
| H6 | **插件 6 关节标定校验** | motor_calibration.yaml 关节键数 ≠ URDF 关节数时静默用默认值 | on_configure 校验 `joint_calibration_.size() != info_.joints.size()` → ERROR |
| H7 | **实机联合验收清单** | — | 真机验收由用户执行（低速、避开人员）：①标定转换 → ②`real:=true sim:=none` 确认从当前位置起步 → ③twin 镜像 → ④MoveIt 小步规划 → ⑤方向反了用 `--invert <关节>` 转换标定后重启 |
| H8 | **lerobot 0.6.1 没有 MotorNormMode.RANGE_0_1** | 枚举只有 RANGE_0_100 / RANGE_M100_100 / DEGREES | 采集脚本用 RANGE_0_100 即可——读的是 `normalize=False` 的 raw 值，norm_mode 不参与换算（换算走本模块 raw_to_rad） |
| H9 | **退出归位时回程半路被拽停** | on_deactivate 的 park 低速插值与主循环 write()（旧命令 speed 2400）并发写同一批电机 | park 期间用 `parking_` 原子标志让 read/write 直接返回（见 so101_system.cpp） |
| H10 | **park 命令发完臂却松在半路** | 低速下伺服物理运动滞后于插值命令，"命令发完"≠"电机到位"，立即失能=半途松力 | 插值后轮询实际位置（0.06 rad 容差，最多 5s）确认到位再失能；复盘见 learn/setup_hardware.md 收获 3 |
| H11 | **第二次 Ctrl+C 会强杀 launch** | ros2 launch 对第二个 SIGINT 立即升级 SIGKILL，on_deactivate 的 park 被腰斩（力矩都发不出） | 回程上限压 2s 内；操作上按一次 Ctrl+C 等它走完 |

补充: 首次真机联调时若某关节方向反了，重新运行
`uv run python scripts/setup_convert_lerobot_calibration.py --invert <关节名>` 并重启即可，无需改 C++。
