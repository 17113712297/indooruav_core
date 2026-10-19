#!/bin/bash
# ============================================================================
# bringup_mode_manager.sh
# 启动 mode_manager_node 并将全部日志保存到文件
#
# 日志位置: ../log/bringup_mode_manager/mode_manager_<timestamp>.log
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SRC="$(cd "$SCRIPT_DIR/../.." && pwd)"

LOG_DIR="$SCRIPT_DIR/../log/bringup_mode_manager"
mkdir -p "$LOG_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/mode_manager_${TIMESTAMP}.log"

source "$WS_SRC/../devel/setup.bash"

echo "=============================================="
echo "  mode_manager_node 启动"
echo "  日志文件: $LOG_FILE"
echo "=============================================="

script -q -c "roslaunch indooruav_core bringup_mode_manager.launch" "$LOG_FILE"

sed -i -E \
    -e 's/\x1b\[[0-9;]*[a-zA-Z]//g' \
    -e 's/\x1b\][^\x07]*\x07//g' \
    "$LOG_FILE"

echo "日志已保存: $LOG_FILE"