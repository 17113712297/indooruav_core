#!/bin/bash
# ============================================================================
# bringup_localize.sh
# 启动 FAST-LIO2 localize 并将全部日志保存到文件
#
# 日志位置: ../log/bringup_localize/localize_<timestamp>.log
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SRC="$(cd "$SCRIPT_DIR/../.." && pwd)"

LOG_DIR="$SCRIPT_DIR/../log/bringup_localize"
mkdir -p "$LOG_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/localize_${TIMESTAMP}.log"

source "$WS_SRC/../devel/setup.bash"

echo "=============================================="
echo "  FAST-LIO2 Localize 启动"
echo "  日志文件: $LOG_FILE"
echo "=============================================="

script -q -c "roslaunch indooruav_localize localize.launch" "$LOG_FILE"

sed -i -E \
    -e 's/\x1b\[[0-9;]*[a-zA-Z]//g' \
    -e 's/\x1b\][^\x07]*\x07//g' \
    "$LOG_FILE"

echo "日志已保存: $LOG_FILE"
