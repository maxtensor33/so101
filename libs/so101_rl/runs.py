"""run 目录管理：时间戳层次 + latest 指针（防重复训练覆盖）。"""

from __future__ import annotations

import time
from pathlib import Path

LATEST = "latest"


def new_run_dir(root, tag: str) -> Path:
    """创建 outputs/<root>/<tag>/<时间戳>/ 目录（同秒冲突自动加 _n 后缀），
    并写 <root>/<tag>/latest 指针指向最新 run。"""
    tag_dir = Path(root) / tag
    tag_dir.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d-%H%M%S")
    d = tag_dir / ts
    i = 1
    while d.exists():
        d = tag_dir / f"{ts}_{i}"
        i += 1
    d.mkdir(parents=True)
    (tag_dir / LATEST).write_text(str(d))
    return d


def resolve_ckpt_dir(path) -> Path:
    """latest 指针文件 → 实际 run 目录；普通目录原样返回。"""
    p = Path(path)
    if p.name == LATEST and p.is_file():
        return Path(p.read_text().strip())
    return p
