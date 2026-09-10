#!/usr/bin/env bash
# ============================================================
# UR 机械臂网络一键配置（需要 sudo，在电脑终端执行）
#   - 主板网口 enp0s31f6 固定静态 IP 192.168.1.10/24（nmcli 固化，重启不丢）
#   - 加策略路由：去 192.168.1.0/24 的流量绕过 sing-box/Clash 代理隧道
#   - 验证：ping 192.168.1.3 + 端口探测
# 用法: sudo bash network_setup.sh
# ============================================================
set -euo pipefail

IFACE="${1:-enp0s31f6}"
PC_IP="${2:-192.168.1.10}"
ROBOT_IP="${3:-192.168.1.3}"
CONN_NAME="Wired connection 1"

echo "==> [1/4] 主板网口链路检查"
if [ "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo 0)" = "1" ]; then
    echo "    链路 OK"
else
    echo "    [警告] carrier=0，确认控制柜开机、网线插好、交换机有电"
fi

echo "==> [2/4] 固化静态 IP 到 NetworkManager"
# 找绑定该网口的连接名（可能是 "Wired connection 1"）
ACTUAL=$(nmcli -t -f NAME,DEVICE con show | awk -F: -v d="$IFACE" '$2==d {print $1; exit}')
CONN_NAME="${ACTUAL:-$CONN_NAME}"
echo "    连接名: $CONN_NAME"
sudo nmcli con modify "$CONN_NAME" ipv4.method manual ipv4.addresses "$PC_IP/24" ipv4.gateway "" ipv4.dns "" || {
    echo "    修改失败，尝试新建连接"
    sudo nmcli con add type ethernet ifname "$IFACE" con-name UR-robot \
        ipv4.method manual ipv4.addresses "$PC_IP/24" ipv4.gateway "" ipv4.dns ""
    CONN_NAME="UR-robot"
}
sudo nmcli con up "$CONN_NAME"

echo "==> [3/4] 绕过代理隧道（sing-box/Clash）"
sudo ip rule add to 192.168.1.0/24 priority 8000 table main 2>/dev/null || echo "    规则已存在"
ip route get "$ROBOT_IP"

echo "==> [4/4] 验证"
ping -c 3 -W 2 "$ROBOT_IP" && echo "✔ 机械臂网络已通！" || {
    echo "✘ ping 不通，检查：机器人开机？交换机有电？代理规则？"
    exit 1
}

echo
echo "端口自检（应看到 29999/30001/30002/30003 OK）："
python3 ~/ur_learn/scripts/ur_port_test.py "$ROBOT_IP" || true

echo
echo "✔ 网络配置完成！接下来："
echo "   ros2 run ur_link ur_state_node --ros-args -p robot_ip:=$ROBOT_IP"
echo "   python3 ~/ur_learn/scripts/ur_circle.py 0.02   # 画圆演示"
