#!/usr/bin/env bash
# ============================================================
# 一键启动"视觉·力觉监控台"
#
#   起 5 个东西：
#     1) ros_web_bridge.py    ROS话题 → 浏览器 WebSocket (ws://:9090) + 网页(:8080)
#     2) ati_netft_node.py    ATI 六维力 → /ft_sensor/wrench
#     3) ft_gravity_compensator.py → /ft_sensor/wrench_compensated
#     4) mecheye 相机节点      (SDK 已装则起；没装自动跳过并在网页里显示"相机离线")
#     5) 打开浏览器 http://127.0.0.1:8080/
#
# 用法:
#    bash scripts/start_dashboard.sh              # 完整起
#    bash scripts/start_dashboard.sh --no-browser # 不起浏览器（手动开）
#    bash scripts/start_dashboard.sh --no-camera  # 不起相机
#    bash scripts/start_dashboard.sh --stop       # 停掉本脚本起的进程
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="$SCRIPT_DIR/../web_dashboard"
LOG_DIR="$HOME/ur_learn/generated"
mkdir -p "$LOG_DIR"

PIDS=()
cleanup() {
    for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done
}
trap cleanup EXIT

# ROS setup files may read optional variables before defining them.
set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/colcon_ws/install/setup.bash" ] && source "$HOME/colcon_ws/install/setup.bash"
set -u

# ---------- 停止模式 ----------
if [ "${1:-}" = "--stop" ]; then
    pkill -f ros_web_bridge.py 2>/dev/null || true
    pkill -f ati_netft_node.py 2>/dev/null || true
    pkill -f ft_gravity_compensator.py 2>/dev/null || true
    pkill -f fake_mecheye_publisher.py 2>/dev/null || true
    pkill -f "mecheye_ros_interface.*start" 2>/dev/null || true
    echo "已停止所有监控台相关进程"
    exit 0
fi

NO_BROWSER=0; NO_CAMERA=0
for a in "$@"; do
    [ "$a" = "--no-browser" ] && NO_BROWSER=1
    [ "$a" = "--no-camera" ] && NO_CAMERA=1
done

# ---------- ① 桥 ----------
if ! pgrep -f ros_web_bridge.py >/dev/null; then
    echo "==> 启动 ROS→Web 桥 (ws://:9090, http://:8080)"
    python3 "$SCRIPT_DIR/ros_web_bridge.py" > "$LOG_DIR/web_bridge.log" 2>&1 &
    PIDS+=($!)
    sleep 2
else
    echo "==> 桥已在运行"
fi

# ---------- ② 力传感器 ----------
if ! pgrep -f ati_netft_node.py >/dev/null; then
    echo "==> 启动 ATI 力传感器节点 → /ft_sensor/wrench"
    python3 "$SCRIPT_DIR/ati_netft_node.py" > "$LOG_DIR/ft_node.log" 2>&1 &
    PIDS+=($!)
else
    echo "==> 力传感器节点已在运行"
fi

# ---------- ③ 重力补偿 ----------
if ! pgrep -f ft_gravity_compensator.py >/dev/null; then
    echo "==> 启动力/力矩重力补偿 → /ft_sensor/wrench_compensated"
    (cd "$SCRIPT_DIR/.." && python3 "$SCRIPT_DIR/ft_gravity_compensator.py" \
        > "$LOG_DIR/ft_gravity_compensator.log" 2>&1) &
    PIDS+=($!)
    sleep 1
else
    echo "==> 重力补偿节点已在运行"
fi

# ---------- ④ 相机 ----------
if [ "$NO_CAMERA" = "1" ]; then
    echo "==> 跳过相机"
elif [ -d /opt/mech-mind/mech-eye-sdk ] && [ -f "$HOME/colcon_ws/install/setup.bash" ]; then
    # Never leave the simulator publishing the same topics as the real camera.
    pkill -f fake_mecheye_publisher.py 2>/dev/null || true
    if ! pgrep -f "mecheye_ros_interface" >/dev/null; then
        echo "==> 启动 Mech-Eye 相机节点（SDK 已装）"
        ( set +u
          source "$HOME/colcon_ws/install/setup.bash"
          set -u
          timeout 0 ros2 launch \
            "$HOME/colcon_ws/src/mecheye_ros2_interface/launch/start_camera_ur10.py" \
            > "$LOG_DIR/mecheye.log" 2>&1 ) &
        PIDS+=($!)
        sleep 3
    else
        echo "==> 相机节点已在运行"
    fi
else
    echo "==> 相机：SDK 未装，跳过（网页会显示“相机离线”，力数据不受影响）"
fi

# ---------- ⑤ 浏览器 ----------
URL="http://127.0.0.1:8080/"
echo
echo "==========================================================="
echo "  监控台地址: $URL"
echo "  原始力: /ft_sensor/wrench_raw（实时 200Hz，不受 Tare 影响）"
echo "  补偿力: /ft_sensor/wrench_compensated（原始/补偿可切换）"
echo "  相机  : Mech-Eye（需 SDK，服务触发式采集）"
echo "  停止  : bash scripts/start_dashboard.sh --stop"
echo "==========================================================="
if [ "$NO_BROWSER" = "1" ]; then
    echo "（未自动打开浏览器，请手动访问 $URL）"
else
    (sleep 1; xdg-open "$URL" >/dev/null 2>&1 || true) &
fi

# 前台挂着，Ctrl-C 退出并清理
wait
