#!/bin/bash
# ============================================================================
# bringup_waypoint_recorder.sh [mode]
# 启动 waypoint_recorder_node 并将全部日志保存到文件
#
# 参数:
#   mode  direct     直接订阅里程计话题（默认）
#         path_relay 订阅 /global_path 转发为 Odometry 后记录
#
# 日志位置: ../log/bringup_waypoint_recorder/waypoint_recorder_node_<timestamp>.log
# ============================================================================

MODE="${1:-direct}"

if [[ "$MODE" != "direct" && "$MODE" != "path_relay" ]]; then
    echo "ERROR: invalid mode '$MODE', must be 'direct' or 'path_relay'"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SRC="$(cd "$SCRIPT_DIR/../.." && pwd)"

LOG_DIR="$SCRIPT_DIR/../log/bringup_waypoint_recorder"
mkdir -p "$LOG_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/waypoint_recorder_node_${MODE}_${TIMESTAMP}.log"

source "$WS_SRC/../devel/setup.bash"

echo "=============================================="
echo "  waypoint_recorder_node 启动 (mode=$MODE)"
echo "  日志文件: $LOG_FILE"
echo "=============================================="

# 用 script 创建伪终端，roslaunch 以为连的是真终端，日志完整输出
# -q  静默，不打印 "Script started/done"
# -c  执行命令
script -q -c "roslaunch indooruav_waypoint bringup_waypoint_recorder.launch mode:=$MODE" "$LOG_FILE"

# 清除 ANSI 转义码（颜色、加粗、窗口标题等）
sed -i -E \
    -e 's/\x1b\[[0-9;]*[a-zA-Z]//g' \
    -e 's/\x1b\][^\x07]*\x07//g' \
    "$LOG_FILE"

echo "日志已保存: $LOG_FILE"
