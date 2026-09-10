#!/usr/bin/env bash
# Start the read-only UR10 RViz digital twin.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-real}"
RVIZ="true"

if [ "$MODE" != "real" ] && [ "$MODE" != "offline" ]; then
    echo "用法: $0 [real|offline] [--no-rviz]"
    exit 2
fi
for arg in "$@"; do
    [ "$arg" = "--no-rviz" ] && RVIZ="false"
done

set +u
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
if [ ! -f "$ROOT_DIR/ros2_ws/install/setup.bash" ]; then
    echo "✘ 数字孪生尚未构建，请先执行："
    echo "  cd $ROOT_DIR/ros2_ws"
    echo "  colcon build --symlink-install --packages-select ur10_digital_twin"
    exit 1
fi
source "$ROOT_DIR/ros2_ws/install/setup.bash"
set -u

START_STATE="false"
if [ "$MODE" = "real" ]; then
    NODES="$(ros2 node list 2>/dev/null || true)"
    if ! grep -Eq '/(ur_state_node|digital_twin_ur_state_node)$' <<< "$NODES"; then
        START_STATE="true"
        echo "==> 未发现 ur_state_node，数字孪生将启动只读状态节点"
    else
        echo "==> 复用现有 /joint_states（不会再连接机器人状态端口）"
    fi
fi

echo "==> 启动 UR10 数字孪生: mode=$MODE rviz=$RVIZ"
echo "    数据方向: /joint_states -> /digital_twin/joint_states -> twin_* TF"
exec ros2 launch ur10_digital_twin digital_twin.launch.py \
    "mode:=$MODE" "start_state_node:=$START_STATE" "rviz:=$RVIZ"
