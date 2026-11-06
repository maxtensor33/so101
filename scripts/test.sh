#!/usr/bin/env bash
# pytest 统一入口（Token 节约纪律）: 全文存 /tmp/so101_test/pytest.log, 只回流汇总行。
# 用法: bash scripts/test.sh [pytest 参数...]   （无参数 = 全量）
LOG=/tmp/so101_test/pytest.log
mkdir -p /tmp/so101_test
uv run pytest "$@" > "$LOG" 2>&1
st=$?
grep -E "passed|failed|error" "$LOG" | tail -1
exit $st
