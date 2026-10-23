# TODO: il_sim viewer 键盘焦点抑制（viewer_no_input）

现状：`SO101*Env(viewer_no_input=True)` 经 python-xlib 设 WM_HINTS.input=False，
实测在 KDE/KWin 下无效（GLFW 窗口焦点行为疑似被 WM/重建覆盖）。

影响：mujoco viewer 窗口获得焦点时，字母键（w/s/c/r/n/q…）会同时触发
simulate 内建渲染开关（C++ 层绑定，Python 侧无法禁用）。当前 workaround：
**焦点保持在终端/其他窗口**——pynput 全局监听不受焦点影响，遥操/录制键照常工作，
viewer 仅鼠标交互（拖视角不需要焦点）。

排查方向：
- [ ] 在 launch_passive 返回后轮询窗口出现时机（WM_HINTS 可能在映射前被 GLFW 重写）
- [ ] 改用 _NET_WM_USER_TIME / XSetInputFocus 规避，或 XWayland 场景检查
- [ ] 考虑 GLFW 独立线程版 viewer（mujoco.viewer.launch 阻塞式）+ key_callback 吞键
