"""文档防漂移检查（workflow/README.md「文档与实际状态不符视为未完成」的机器约束）。

覆盖：
1. md 中 `scripts/<name>.py` 引用必须落盘（防旧脚本名残留）
2. md 中 `workflow/...md` 引用必须落盘（防 .claude/ 迁移断链类问题）
3. 禁词表：已知会漂移的旧写法

豁免：
- `workflow/reviews/`（时点快照, 文件头已注明旧名）
- 行内示例模板（如 `img/<file>.png` 占位）
- URL / 行内代码中的占位尖括号路径
"""

from __future__ import annotations

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

# 扫描范围（排除构建产物与虚拟环境）
SCAN_DIRS = ["docs", "workflow", "scripts", "tests", "libs", "src"]
SCAN_FILES = ["README.md", "CLAUDE.md", "pyproject.toml"]

# 已知漂移旧写法 → 应已不存在（出现即 FAIL）
FORBIDDEN = [
    r"robotics_robotics_",
    r"\.claude/tdd/",
    r"scripts/verify_model_alignment\.py",
    r"scripts/run_expert\.py",
    r"scripts/check_env\.py",
    r"scripts/draw_cube_viewer\.py",
    r"workflow/todo/to_real_joint_calibration\.md",
    r"workflow/todo/il_sim_expert_lift\.md",
    r"workflow/todo/doc_img_list\.md",
    r"workflow/pitfalls/setup-hardware\.md",
]

# 豁免文件（相对路径前缀）
EXEMPT_PREFIXES = ("workflow/reviews/",)

# 已知从未产出的工件（历史叙述引用保留; 若未来产出或引用变更需同步本表）
KNOWN_MISSING = {
    "workflow/tdd/mtc.tdd.md": "mujoco/gz 双后端方向中止, TDD 证据从未产出",
}

SCRIPT_REF = re.compile(r"(?<![\w/])scripts/([a-z0-9_]+)\.py")
WORKFLOW_MD_REF = re.compile(r"workflow/[A-Za-z0-9_/.-]+\.md")


def _iter_md_files():
    for name in SCAN_FILES:
        p = REPO_ROOT / name
        if p.is_file():
            yield p
    for d in SCAN_DIRS:
        base = REPO_ROOT / d
        if not base.is_dir():
            continue
        for p in base.rglob("*.md"):
            rel = p.relative_to(REPO_ROOT).as_posix()
            if "_build" in rel or "__pycache__" in rel or rel.startswith(EXEMPT_PREFIXES):
                continue
            yield p


def test_no_forbidden_stale_references():
    hits = []
    for p in _iter_md_files():
        text = p.read_text(encoding="utf-8", errors="ignore")
        for pat in FORBIDDEN:
            for m in re.finditer(pat, text):
                hits.append(f"{p.relative_to(REPO_ROOT)}: {m.group(0)}")
    assert not hits, "发现过期引用（文档与实际状态不符）:\n" + "\n".join(hits)


def test_script_references_exist():
    missing = []
    for p in _iter_md_files():
        text = p.read_text(encoding="utf-8", errors="ignore")
        for m in SCRIPT_REF.finditer(text):
            script = REPO_ROOT / "scripts" / f"{m.group(1)}.py"
            if not script.is_file():
                missing.append(f"{p.relative_to(REPO_ROOT)}: scripts/{m.group(1)}.py")
    assert not missing, "md 引用的脚本不存在:\n" + "\n".join(sorted(set(missing)))


def test_workflow_md_references_exist():
    missing = []
    for p in _iter_md_files():
        text = p.read_text(encoding="utf-8", errors="ignore")
        for m in WORKFLOW_MD_REF.finditer(text):
            ref = m.group(0)
            if "<" in ref or ">" in ref or ref in KNOWN_MISSING:
                continue  # 占位符 / 已知未产出
            if not (REPO_ROOT / ref).is_file():
                missing.append(f"{p.relative_to(REPO_ROOT)}: {ref}")
    assert not missing, "md 引用的 workflow 文档不存在:\n" + "\n".join(sorted(set(missing)))
