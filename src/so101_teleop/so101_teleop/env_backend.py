"""env_backend：SimSO101Robot 与 Teleoperator 之间的共享 env 访问点。

lerobot 协议里 Teleoperator.get_action() 无 obs 入参，脚本专家需要闭环（读 qpos/tcp/goal），
故由 Robot.connect() 注册 env，teleop 经本模块访问；跨进程不可用（单进程适配层约定）。
"""

from __future__ import annotations

_env = None
_reset_requested = False
_reset_count = 0
_latest_images: dict | None = None  # 主线程渲染缓存（EGL 上下文线程绑定, viz 线程只读）


def set_active_env(env) -> None:
    global _env
    _env = env


def get_active_env():
    if _env is None:
        raise RuntimeError(
            "no active sim env: SimSO101Robot.connect() must run before teleoperator.connect()"
        )
    return _env


def get_active_env_or_none():
    return _env


def clear_active_env() -> None:
    global _env
    _env = None


def request_reset() -> None:
    """teleop 侧发起：在下一帧 send_action 末尾执行 env.reset（帧边界对齐）。"""
    global _reset_requested
    _reset_requested = True


def reset_pending() -> bool:
    return _reset_requested


def take_reset_request() -> bool:
    """robot 侧消费：返回是否有待执行 reset 并清零标志。"""
    global _reset_requested
    flag = _reset_requested
    _reset_requested = False
    return flag


def bump_reset_count() -> None:
    """robot 侧 reset 执行后调用；teleop 据此吸附保持目标到新 qpos。"""
    global _reset_count
    _reset_count += 1


def reset_count() -> int:
    return _reset_count


def set_latest_images(images: dict) -> None:
    global _latest_images
    _latest_images = images


def latest_images() -> dict | None:
    return _latest_images
