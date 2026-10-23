# Sphinx 配置（参考 stable-baselines3 docs/conf.py）
import datetime

project = "so101"
copyright = f"2026, maxtensor"
author = "maxtensor"
release = "0.1.0"

extensions = [
    "myst_parser",
]
myst_heading_anchors = 3

templates_path = ["_templates"]
exclude_patterns = ["_build", "img"]

html_theme = "sphinx_rtd_theme"
html_static_path = []
