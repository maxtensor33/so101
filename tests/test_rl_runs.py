"""run 目录管理测试（时间戳层次 + latest 指针, 防重复训练覆盖）。"""

from __future__ import annotations

from pathlib import Path

from so101_rl.runs import new_run_dir, resolve_ckpt_dir


def test_new_run_dir_timestamp_layers(tmp_path):
    """同一 tag 两次训练 → 不同时间戳目录, 不互相覆盖。"""
    d1 = new_run_dir(tmp_path, "sb3-sac-reach")
    d2 = new_run_dir(tmp_path, "sb3-sac-reach")
    assert d1 != d2
    assert d1.parent == tmp_path / "sb3-sac-reach"
    assert d2.parent == d1.parent
    assert d1.exists() and d2.exists()


def test_latest_pointer(tmp_path):
    """latest 指针始终指向最新 run。"""
    d1 = new_run_dir(tmp_path, "sb3-sac-reach")
    p1 = Path((tmp_path / "sb3-sac-reach" / "latest").read_text())
    assert p1 == d1
    d2 = new_run_dir(tmp_path, "sb3-sac-reach")
    p2 = Path((tmp_path / "sb3-sac-reach" / "latest").read_text())
    assert p2 == d2 and p2 != p1


def test_resolve_ckpt_dir_latest(tmp_path):
    """resolve_ckpt_dir: latest 指针 → 实际 run 目录; 普通目录原样返回。"""
    d = new_run_dir(tmp_path, "sb3-sac-reach")
    latest = tmp_path / "sb3-sac-reach" / "latest"
    assert resolve_ckpt_dir(latest) == d
    assert resolve_ckpt_dir(d) == d


def test_collision_suffix(tmp_path):
    """同秒碰撞时自动加后缀不覆盖。"""
    a = new_run_dir(tmp_path, "tag")
    b = new_run_dir(tmp_path, "tag")
    assert a != b
