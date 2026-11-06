# Copyright 2026 Dmitri Manajev
# Copyright 2026 maxtensor (adaptation: shared module + outputs/calibration 默认输出)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
"""ChArUco A4 标定板生成（迁移自 so101-ros-physical-ai scripts/gen_charuco_*.py）。

两种板: intrinsic(8x6@25mm/18mm, DICT_5X5_250, 内参) / handeye(4x5@15mm/11mm, DICT_4X4_50, 手眼)。
打印要求: 实际尺寸(100%), 亚光纸, 打印后卡尺量方格边长校验。
"""

from __future__ import annotations

from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

DPI = 300
A4_W_MM, A4_H_MM = 210.0, 297.0
MARGIN_MM = 15.0

BOARD_CONFIGS = {
    "intrinsic": dict(squares=(8, 6), square_mm=25.0, marker_mm=18.0, dict_id=aruco.DICT_5X5_250),
    "handeye": dict(squares=(4, 5), square_mm=15.0, marker_mm=11.0, dict_id=aruco.DICT_4X4_50),
}


def make_board(kind: str):
    """构建 cv2 CharucoBoard（检测与生成共用同一 board 定义）。"""
    cfg = BOARD_CONFIGS[kind]
    aruco_dict = aruco.getPredefinedDictionary(cfg["dict_id"])
    sx, sy = cfg["squares"]
    return aruco.CharucoBoard(
        (sx, sy), cfg["square_mm"] / 1000.0, cfg["marker_mm"] / 1000.0, aruco_dict
    )


def generate_board(kind: str, out_dir: str | Path) -> Path:
    """生成 A4 300DPI PNG + PDF（PDF 页面为 A4 实际尺寸, 打印对话框可选实际大小）。"""
    cfg = BOARD_CONFIGS[kind]
    sx, sy = cfg["squares"]
    mm2px = DPI / 25.4
    bw = int(sx * cfg["square_mm"] * mm2px)
    bh = int(sy * cfg["square_mm"] * mm2px)

    board = make_board(kind)
    board_img = board.generateImage((bw, bh), marginSize=0, borderBits=1)

    pw, ph = int(A4_W_MM * mm2px), int(A4_H_MM * mm2px)
    if bw > bh:
        pw, ph = ph, pw
    if bw > pw or bh > ph:
        raise RuntimeError(f"board {bw}x{bh}px 不适配 A4 页 {pw}x{ph}px")

    page = np.full((ph, pw), 255, dtype=np.uint8)
    ox, oy = (pw - bw) // 2, (ph - bh) // 2
    page[oy : oy + bh, ox : ox + bw] = board_img

    m = int(MARGIN_MM * mm2px / 3)
    for (x, y) in [(ox, oy), (ox + bw, oy), (ox, oy + bh), (ox + bw, oy + bh)]:
        cv2.line(page, (x - m, y), (x + m, y), 0, 2)
        cv2.line(page, (x, y - m), (x, y + m), 0, 2)

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_png = out_dir / f"charuco_{kind}_A4.png"
    cv2.imwrite(str(out_png), page)

    from PIL import Image

    out_pdf = out_dir / f"charuco_{kind}_A4.pdf"
    Image.fromarray(page).save(str(out_pdf), "PDF", resolution=float(DPI))
    return out_png
