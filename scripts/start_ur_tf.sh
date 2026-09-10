#!/usr/bin/env bash
# ============================================================
# 起 UR10 的 TF 链（视觉抓取的地基）
#
#   把 /joint_states（ur_link 的 ur_state_node 发的）
#   变成完整的 TF 树： base → base_link → ... → tool0
#
# 依赖:
#   ros-humble-robot-state-publisher / xacro / tf2-ros   (apt 已装)
#   Universal_Robots_ROS2_Description 源码在 ~/ros2_ws/src/
#
# 用法:
#   bash scripts/start_ur_tf.sh                 # 前台运行
#   bash scripts/start_ur_tf.sh ur10            # 指定机型(ur3/ur5/ur10/ur16e...)
#
# 另开终端验证:
#   ros2 run tf2_ros tf2_echo base tool0        # 应与 30001 读到的 TCP 一致
#   python3 scripts/check_fk.py                 # 自动比对并检查 base/base_link 陷阱
# ============================================================
set -euo pipefail

UR_TYPE="${1:-ur10}"
WS="${WS:-$HOME/ros2_ws}"
DESC="$WS/src/Universal_Robots_ROS2_Description"
URDF_OUT="$HOME/ur_learn/generated/${UR_TYPE}.urdf"

source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

if [ ! -d "$DESC" ]; then
    echo "✘ 找不到 $DESC"
    echo "  请先克隆："
    echo "  cd $WS/src && git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git"
    exit 1
fi

if ! command -v xacro >/dev/null; then
    echo "✘ 没有 xacro，请先: sudo apt install ros-humble-xacro"
    exit 1
fi

echo "==> 生成 $UR_TYPE 的 URDF"
mkdir -p "$(dirname "$URDF_OUT")"
xacro "$DESC/urdf/ur.urdf.xacro" "ur_type:=$UR_TYPE" name:=ur > "$URDF_OUT"
echo "    $URDF_OUT ($(wc -l < "$URDF_OUT") 行)"

echo "==> 起 robot_state_publisher（把 /joint_states 变成 TF）"
echo "    注意：必须先有 ur_state_node 在发 /joint_states，否则 TF 不会动"
echo "    另开终端: ros2 run ur_link ur_state_node --ros-args -p robot_ip:=192.168.1.3"
exec ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat "$URDF_OUT")"
