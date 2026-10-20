"""so101 模块路径注入：让测试可以 import scripts/verify_model_alignment.py。"""

import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent.parent / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

MODELS_DIR = Path(__file__).resolve().parent.parent / "models"
URDF_PATH = MODELS_DIR / "so101_new_calib.urdf"
XML_PATH = MODELS_DIR / "so101_new_calib.xml"
