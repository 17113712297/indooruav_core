#!/bin/bash
# ============================================================================
# odometry_recorder.sh
# 启动 odometry_recorder.py，记录 /Odometry 和 /Odometry_global 话题数据
#
# 输出目录: ../indooruav_localize/log/
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SRC="$(cd "$SCRIPT_DIR/../.." && pwd)"

LOG_DIR="$WS_SRC/indooruav_localize/log"
mkdir -p "$LOG_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/odometry_recorder_${TIMESTAMP}.log"

source "$WS_SRC/../devel/setup.bash"

echo "=============================================="
echo "  odometry_recorder 启动"
echo "  YAML 输出目录: $LOG_DIR"
echo "  日志文件: $LOG_FILE"
echo "=============================================="

script -q -c "rosrun indooruav_core odometry_recorder.py" "$LOG_FILE"

sed -i -E \
    -e 's/\x1b\[[0-9;]*[a-zA-Z]//g' \
    -e 's/\x1b\][^\x07]*\x07//g' \
    "$LOG_FILE"

echo "YAML 保存至: $LOG_DIR"
echo "日志已保存: $LOG_FILE"
