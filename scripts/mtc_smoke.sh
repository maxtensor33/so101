#!/usr/bin/env bash
# 自终止 MTC 冒烟: 后台起 launch, 轮询结果关键词, 一出结果就 kill 整个 launch
# 进程组并立即返回 — 不等 timeout/不残留。用法:
#   bash scripts/mtc_smoke.sh sim:=mujoco [rviz:=false] [--log /tmp/mtc_smoke.log]
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source install/setup.bash >/dev/null 2>&1
export MUJOCO_GL=egl

LOG=/tmp/mtc_smoke.log
SIM=""
EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --log) LOG="$2"; shift 2;;
    sim:=*) SIM="$1"; shift;;
    *) EXTRA+=("$1"); shift;;
  esac
done
[ -n "$SIM" ] || SIM="sim:=mujoco"

bash scripts/clean_ros.sh >/dev/null 2>&1
: > "$LOG"

setsid ros2 launch so101_mtc_pick_place mtc_pick_place.launch.py "$SIM" "${EXTRA[@]}" > "$LOG" 2>&1 &
PGID=$!
echo "[smoke] launch pgid=$PGID  log=$LOG  (auto-kill on result)"

RESULT=""
for _ in $(seq 1 240); do     # 最坏 240s 兜底; 通常几~几十秒出结果
  if grep -q "mtc_pick_place_node exit (SUCCESS)" "$LOG"; then RESULT=SUCCESS; break; fi
  if grep -q "mtc_pick_place_node exit (FAILED)"  "$LOG"; then RESULT=FAILED;  break; fi
  if ! kill -0 "$PGID" 2>/dev/null; then sleep 2; break; fi
  sleep 1
done
echo "[smoke] result=$RESULT"

# 主动杀整个进程组（不残留）
kill -TERM -- -"$PGID" 2>/dev/null; sleep 1
kill -KILL -- -"$PGID" 2>/dev/null
sleep 1

echo "=== MTC stage 诊断 ==="
grep -E "stage '|planning succeeded|execution error" "$LOG" || true

[ "$RESULT" = SUCCESS ]