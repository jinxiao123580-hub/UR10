#!/usr/bin/env bash
# ============================================================
# 一键启动"视觉·力觉监控台"
#
#   起 5 个东西：
#     1) ros_web_bridge.py    ROS话题 → 浏览器 WebSocket (ws://:9090) + 网页(:8080)
#     2) ati_netft_node.py    ATI 六维力 → /ft_sensor/wrench
#     3) mecheye 相机节点      (SDK 已装则起；没装自动跳过并在网页里显示"相机离线")
#     4) 打开浏览器 http://127.0.0.1:8080/
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
LOG_DIR="$SCRIPT_DIR/../outputs/dashboard"
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

# ---------- 通用：进程与端口工具 ----------
HTTP_URL="http://127.0.0.1:8080/"
WS_PORT=9090
HTTP_PORT=8080

# 本脚本自身及其所有祖先进程（$(...) 包装、bwrap、timeout 等）的 PID 列表，用 | 连接。
# 作用：pkill/pgrep -f 的模式会匹配到"命令行里恰好含该字符串"的调用者自己
# （实测把自己的 shell 一起杀掉），所以清理时必须排除自身进程链。
self_chain() {
    local pid=$$ out=""
    while [ -n "$pid" ] && [ "$pid" != "0" ] && [ "$pid" != "1" ]; do
        out="$out$pid|"
        pid=$(sed 's/.*) //' "/proc/$pid/stat" 2>/dev/null | awk '{print $2}')
    done
    printf '%s' "${out}1"
}

match_pids() {
    local pattern="$1" chain
    chain=$(self_chain)
    pgrep -f "$pattern" 2>/dev/null | grep -vE "^($chain)$" || true
}

# 桥是否**真的能应答**（只看 pgrep 会把僵死/残留实例当成"已在运行"，
# 结果打印了成功地址却打不开页面 —— 见 experiments/2026-09-18-入口可用性审计.md §3.1）
bridge_answering() {
    command -v curl >/dev/null 2>&1 || return 1
    curl -sf -m 2 -o /dev/null "$HTTP_URL"
}

# 优雅停止：SIGTERM → 最多等 3 s → SIGKILL；返回后保证进程消失
stop_pattern() {
    local pattern="$1" name="$2" pids
    pids=$(match_pids "$pattern")
    [ -z "$pids" ] && return 0
    kill $pids 2>/dev/null || true
    for _ in 1 2 3; do
        sleep 1
        pids=$(match_pids "$pattern")
        [ -z "$pids" ] && return 0
    done
    echo "    $name 未响应 SIGTERM，改用 SIGKILL"
    kill -9 $pids 2>/dev/null || true
    sleep 1
    pids=$(match_pids "$pattern")
    [ -z "$pids" ] && return 0
    echo "    ✘ $name 仍存活: $pids"
    return 1
}

port_in_use() {
    (exec 3<>"/dev/tcp/127.0.0.1/$1") 2>/dev/null && { exec 3>&- 3<&-; return 0; }
    return 1
}

# ---------- 停止模式 ----------
if [ "${1:-}" = "--stop" ]; then
    echo "==> 停止监控台相关进程"
    failed=0
    stop_pattern "ros_web_bridge.py" "ROS→Web 桥" || failed=1
    stop_pattern "ati_netft_node.py" "ATI 力传感器节点" || failed=1
    stop_pattern "fake_mecheye_publisher.py" "相机模拟器" || failed=1
    stop_pattern "mecheye_ros_interface" "Mech-Eye 相机节点" || failed=1
    stop_pattern "start_camera_ur10" "相机 launch 包装" || failed=1
    if port_in_use "$HTTP_PORT" || port_in_use "$WS_PORT"; then
        echo "✘ 端口仍被占用:"
        ss -ltn 2>/dev/null | grep -E ":$HTTP_PORT|:$WS_PORT" || true
        failed=1
    fi
    if [ "$failed" = "0" ]; then
        echo "已停止所有监控台相关进程（$HTTP_PORT/$WS_PORT 已释放）"
        exit 0
    fi
    echo "部分进程/端口未能清理，见上面输出；可手动: pkill -9 -f ros_web_bridge.py"
    exit 1
fi

NO_BROWSER=0; NO_CAMERA=0
for a in "$@"; do
    [ "$a" = "--no-browser" ] && NO_BROWSER=1
    [ "$a" = "--no-camera" ] && NO_CAMERA=1
done

# ---------- ① 桥 ----------
ALREADY_RUNNING=0
if bridge_answering; then
    echo "==> 桥已在运行且 $HTTP_URL 正常应答"
    ALREADY_RUNNING=1
elif [ -n "$(match_pids ros_web_bridge.py)" ]; then
    echo "==> 发现僵死的桥进程（端口不应答），先清理再启动"
    stop_pattern "ros_web_bridge.py" "僵尸桥进程" || true
    echo "==> 启动 ROS→Web 桥 (ws://:$WS_PORT, http://:$HTTP_PORT)"
    python3 -u "$SCRIPT_DIR/ros_web_bridge.py" > "$LOG_DIR/web_bridge.log" 2>&1 &
    PIDS+=($!)
    sleep 2
else
    echo "==> 启动 ROS→Web 桥 (ws://:$WS_PORT, http://:$HTTP_PORT)"
    python3 -u "$SCRIPT_DIR/ros_web_bridge.py" > "$LOG_DIR/web_bridge.log" 2>&1 &
    PIDS+=($!)
    sleep 2
fi

# ---------- ② 力传感器 ----------
if [ -z "$(match_pids ati_netft_node.py)" ]; then
    echo "==> 启动 ATI 力传感器节点 → /ft_sensor/wrench_raw"
    python3 -u "$SCRIPT_DIR/ati_netft_node.py" > "$LOG_DIR/ft_node.log" 2>&1 &
    PIDS+=($!)
else
    echo "==> 力传感器节点已在运行"
fi

# ---------- ③ 相机 ----------
if [ "$NO_CAMERA" = "1" ]; then
    echo "==> 跳过相机"
elif [ -d /opt/mech-mind/mech-eye-sdk ] && [ -f "$HOME/colcon_ws/install/setup.bash" ]; then
    # Never leave the simulator publishing the same topics as the real camera.
    pkill -f fake_mecheye_publisher.py 2>/dev/null || true
    if [ -z "$(match_pids mecheye_ros_interface)" ]; then
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

# ---------- ④ 启动自检（不通过就报错退出，不再打印假的成功地址）----------
if ! bridge_answering; then
    for _ in 1 2 3 4 5; do
        sleep 1
        bridge_answering && break
    done
fi
if ! bridge_answering; then
    echo
    echo "✘ 桥没有在 $HTTP_URL 应答，页面无法打开。诊断信息：" >&2
    echo "  - 端口占用: $(ss -ltn 2>/dev/null | grep -E ":$HTTP_PORT|:$WS_PORT" | tr -s ' ' || echo 无)"
    echo "  - 桥日志尾部:" >&2
    tail -20 "$LOG_DIR/web_bridge.log" >&2 2>/dev/null || true
    echo "  - 如遇残留进程: bash scripts/start_dashboard.sh --stop" >&2
    exit 1
fi

# ---------- ⑤ 浏览器 ----------
URL="$HTTP_URL"
echo
echo "==========================================================="
echo "  监控台地址: $URL   （已自检可访问）"
echo "  原始力: /ft_sensor/wrench_raw（实时 200Hz，不受 Tare 影响）"
echo "  重力补偿: 未启动（当前模型未独立验证）"
echo "  相机  : Mech-Eye（需 SDK，服务触发式采集）"
echo "  停止  : bash scripts/start_dashboard.sh --stop"
echo "==========================================================="
if [ "$NO_BROWSER" = "1" ]; then
    echo "（未自动打开浏览器，请手动访问 $URL）"
else
    (sleep 1; xdg-open "$URL" >/dev/null 2>&1 || true) &
fi

# 全部组件本来就在跑：没有子进程可挂，直接说清楚再退出（否则 wait 立刻返回会让人以为失败）
if [ "$ALREADY_RUNNING" = "1" ] && [ "${#PIDS[@]}" = "0" ]; then
    echo "（监控台此前已在运行，本次没有新起进程；本脚本退出，已在运行的实例不受影响）"
    exit 0
fi

# 前台挂着，Ctrl-C 退出并清理
wait
