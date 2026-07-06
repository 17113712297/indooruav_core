#!/bin/bash
# ============================================================================
# bringup_all.sh
# 开机自启脚本 — 顺序启动所有无人机 ROS 节点组件
#
# 启动顺序（考虑依赖关系）：
#   1. roscore（自动检测，未启动则拉起）
#   2. MID360 激光雷达驱动（定位依赖）
#   3. 定位（状态机/航线跟踪依赖）
#   4. 状态机（核心）
#   5. 实物控制（I2C/串口权限提升）
#   6. HTTP 服务
#   7. 降落任务
#   8. 航线跟踪
#   9. 像素坐标发送
#
# 用法:
#   bash bringup_all.sh            # 启动全部
#   bash bringup_all.sh --stop     # 停止全部
#   bash bringup_all.sh --status   # 查看运行状态
#   bash bringup_all.sh --restart  # 重启全部
#
# 日志位置: ../log/bringup_all/<component>_<timestamp>.log
# PID 文件: /tmp/indooruav_boot_<component>.pid
# ============================================================================

set -eo pipefail

# ========== 路径配置 ==========
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SRC="$(cd "$SCRIPT_DIR/.." && pwd)"                     # indooruav_core
WORKSPACE="$(cd "$WS_SRC/../.." && pwd)"                    # catkin_ws
SHELL_DIR="$WORKSPACE/src/shell"
THREE_D_WORKSPACE="$(cd "$WORKSPACE/../3D/catkin_ws" 2>/dev/null && pwd || true)"

# fallback：如果 3D 工作空间不在标准位置，尝试用户目录
if [ -z "$THREE_D_WORKSPACE" ] || [ ! -d "$THREE_D_WORKSPACE" ]; then
    THREE_D_WORKSPACE="$HOME/Project/3D/catkin_ws"
fi

LOG_DIR="$WS_SRC/log/bringup_all"
mkdir -p "$LOG_DIR"

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
PID_DIR="/tmp/indooruav_boot"
mkdir -p "$PID_DIR"

# ========== 组件定义 ==========
# 格式: 标签@@@命令@@@依赖检查（留空即永真）
# 注意：sudo 部分仅首次启动时执行 chmod
# 警告：不要使用 | 做分隔符！命令中可能包含管道符号
COMPONENTS=(
    "MID360@@@roslaunch --screen ${THREE_D_WORKSPACE}/src/livox_ros_driver2/launch_ROS1/msg_MID360.launch@@@"
    "定位@@@bash ${SHELL_DIR}/bringup_localize.sh@@@"
    "状态机@@@bash ${SHELL_DIR}/bringup_indooruav_core.sh@@@"
    "实物控制@@@bash -c 'echo \"888888\" | sudo -S chmod 666 /dev/i2c-7 2>/dev/null; echo \"888888\" | sudo -S chmod 777 /dev/ttyTHS0 2>/dev/null; bash ${SHELL_DIR}/bringup_controller_hardware.sh'@@@"
    "HTTP服务@@@roslaunch --screen indooruav_http bringup_indooruav_http.launch@@@"
    "降落@@@roslaunch --screen indooruav_mission bringup_mission.launch@@@"
    "航线跟踪@@@bash ${SHELL_DIR}/bringup_waypoint_tracker.sh@@@"
    "像素坐标发送@@@python3 -u ${WORKSPACE}/src/FASTLIO2_SAM_LC/scripts/odometry_to_pixel.py@@@"
)

# ========== 源码 ROS 环境 ==========
source_ros_env() {
    local main_setup="$WORKSPACE/devel/setup.bash"
    local three_d_setup="$THREE_D_WORKSPACE/devel/setup.bash"

    if [ -f "$three_d_setup" ]; then
        echo "[INFO] 源码 3D 工作空间: $three_d_setup"
        # shellcheck source=/dev/null
        source "$three_d_setup"
    else
        echo "[WARN] 3D 工作空间未找到: $three_d_setup"
    fi

    if [ -f "$main_setup" ]; then
        echo "[INFO] 源码主工作空间: $main_setup"
        # shellcheck source=/dev/null
        source "$main_setup" --extend
    else
        echo "[ERR] 主工作空间未找到: $main_setup"
        exit 1
    fi

    export ROSCONSOLE_STDOUT_LINE_BUFFERED=1
    export PYTHONUNBUFFERED=1
}

# ========== 启动 roscore（如未运行）==========
start_roscore() {
    # 快速检查 roscore 是否已运行
    if rostopic list &>/dev/null; then
        echo "[INFO] roscore 已在运行，跳过"
        return 0
    fi

    echo "[INFO] 启动 roscore ..."
    roscore &
    local roscore_pid=$!
    echo "[INFO] roscore PID: $roscore_pid"

    local timeout=30
    local elapsed=0
    while ! rostopic list &>/dev/null; do
        sleep 1
        elapsed=$((elapsed + 1))
        if [ "$elapsed" -ge "$timeout" ]; then
            echo "[ERR] roscore 启动超时（${timeout}s），请手动检查"
            return 1
        fi
        echo "[INFO] 等待 roscore ... ${elapsed}s"
    done
    echo "[OK] roscore 已就绪"
}

# ========== 启动单个组件 ==========
start_component() {
    local label="$1"
    local cmd="$2"
    local log_file="$LOG_DIR/${label}_${TIMESTAMP}.log"
    local pid_file="$PID_DIR/${label}.pid"

    # 检查是否已经在运行
    if [ -f "$pid_file" ]; then
        local old_pid
        old_pid=$(cat "$pid_file")
        if kill -0 "$old_pid" 2>/dev/null; then
            echo "[SKIP] ${label} 已在运行 (PID $old_pid)"
            return 0
        fi
        rm -f "$pid_file"
    fi

    echo "[START] ${label} ..."
    echo "        命令: ${cmd}"
    echo "        日志: ${log_file}"

    # 后台执行
    nohup bash -c "${cmd}" > "$log_file" 2>&1 &
    local pid=$!
    echo "$pid" > "$pid_file"
    echo "        PID: $pid"

    # 等待一小段检查是否立即崩溃
    sleep 2
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "[WARN] ${label} 可能启动失败，检查日志: ${log_file}"
        # 不退出，继续启动其他组件
    fi
}

# ========== 停止单个组件 ==========
stop_component() {
    local label="$1"
    local pid_file="$PID_DIR/${label}.pid"

    if [ -f "$pid_file" ]; then
        local old_pid
        old_pid=$(cat "$pid_file")
        if kill -0 "$old_pid" 2>/dev/null; then
            echo "[STOP] ${label} (PID $old_pid)"
            kill "$old_pid" 2>/dev/null || true
            # 等 3 秒，然后强制
            sleep 3
            if kill -0 "$old_pid" 2>/dev/null; then
                echo "[KILL] ${label} 强制终止"
                kill -9 "$old_pid" 2>/dev/null || true
            fi
        else
            echo "[INFO] ${label} 未在运行"
        fi
        rm -f "$pid_file"
    fi
}

# ========== 状态检查 ==========
show_status() {
    echo ""
    echo "=============================================="
    echo "  系统组件运行状态  $(date '+%Y-%m-%d %H:%M:%S')"
    echo "=============================================="
    local any_running=false
    for entry in "${COMPONENTS[@]}"; do
        local label="${entry%%@@@*}"
        local pid_file="$PID_DIR/${label}.pid"
        if [ -f "$pid_file" ]; then
            local pid
            pid=$(cat "$pid_file")
            if kill -0 "$pid" 2>/dev/null; then
                echo "  ● ${label}  (PID $pid)"
                any_running=true
            else
                echo "  ○ ${label}  (已停止, PID 残留)"
                rm -f "$pid_file"
            fi
        else
            echo "  ○ ${label}  (未启动)"
        fi
    done
    echo ""

    # 检查 roscore
    if rostopic list &>/dev/null; then
        echo "  ● roscore  (运行中)"
    else
        echo "  ○ roscore  (未运行)"
    fi
    echo "=============================================="
}

# ========== 主流程 ==========
main() {
    local action="${1:-start}"

    case "$action" in
        start)
            echo "=============================================="
            echo "  indooruav 全部组件启动  ${TIMESTAMP}"
            echo "=============================================="

            source_ros_env
            start_roscore

            for entry in "${COMPONENTS[@]}"; do
                local label="${entry%%@@@*}"
                local rest="${entry#*@@@}"
                local cmd="${rest%%@@@*}"
                start_component "$label" "$cmd"
                # 组件间等待，确保依赖关系
                sleep 3
            done

            echo ""
            echo "[DONE] 全部组件已启动"
            show_status
            ;;

        stop)
            echo "[INFO] 停止全部组件 ..."
            for entry in "${COMPONENTS[@]}"; do
                local label="${entry%%@@@*}"
                stop_component "$label"
            done
            echo "[DONE] 全部组件已停止"
            ;;

        restart)
            echo "[INFO] 重启全部组件 ..."
            main stop
            echo "[INFO] 等待 3 秒后启动 ..."
            sleep 3
            main start
            ;;

        status)
            show_status
            ;;

        *)
            echo "用法: $0 [start|stop|restart|status]"
            echo ""
            echo "  start   - 启动全部组件（默认）"
            echo "  stop    - 停止全部组件"
            echo "  restart - 重启全部组件"
            echo "  status  - 查看运行状态"
            exit 1
            ;;
    esac
}

main "$@"