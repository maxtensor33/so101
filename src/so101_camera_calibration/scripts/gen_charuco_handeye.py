#!/usr/bin/env python3
# Copyright 2026 Dmitri Manajev
# Copyright 2026 maxtensor (adaptation: 输出默认 outputs/calibration, 共享模块化)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
"""生成手眼标定板 ChArUco A4 PDF/PNG（4x5 方格 15mm/11mm, DICT_4X4_50, 300DPI）。

打印: 实际尺寸(100%, 不缩放), 亚光纸; 打印后卡尺量任一方格边长 = 15.0mm。
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from so101_camera_calibration.board_gen import generate_board  # noqa: E402

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", default="outputs/calibration")
    args = ap.parse_args()
    png = generate_board("handeye", args.out_dir)
    print(f"saved: {png} + .pdf  | 打印: 实际尺寸/亚光纸, 卡尺校验 15.0mm")
