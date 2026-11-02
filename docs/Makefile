# Minimal makefile for Sphinx documentation (参考 stable-baselines3)
# sphinx 在 pyproject 的 docs dependency-group; 经 uv run 确保已 sync（免手动 uv sync --group docs）
SPHINXOPTS    ?=
SPHINXBUILD   = uv run --group docs sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build

help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
