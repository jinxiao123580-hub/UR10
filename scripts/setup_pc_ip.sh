#!/usr/bin/env bash
# 给本机网口配 192.168.1.x 静态地址并测试到机械臂的连通性。
# 用法(需要 sudo，请在电脑自己的终端里执行):
#   sudo bash setup_pc_ip.sh [网口] [本机IP]
# 默认: 网口 enp0s31f6（主板网口），本机 IP 192.168.1.10，机械臂 192.168.1.3
# 说明: 只在这一个网口上加地址，不影响你现在上网用的 USB 网口。
set -euo pipefail

IFACE="${1:-enp0s31f6}"
IP="${2:-192.168.1.10}"
ROBOT="${3:-192.168.1.3}"

echo "==> 检查网口链路 $IFACE"
if [ "$(cat /sys/class/net/$IFACE/carrier 2>/dev/null || echo 0)" = "1" ]; then
    echo "    链路 OK（网线已检测到）"
else
    echo "    [警告] 没有检测到网线链路(carrier=0) —— 请检查：控制柜已开机约1分钟、"
    echo "    网线插在控制柜的 Network 口和电脑 $IFACE 口、两端指示灯。"
fi

echo "==> 启用网口并添加 IP $IP/24"
ip link set "$IFACE" up || true
if ip addr show dev "$IFACE" | grep -q " $IP/24 "; then
    echo "    IP $IP 已存在，跳过"
else
    ip addr add "$IP/24" dev "$IFACE"
fi
ip -br addr show dev "$IFACE"

echo "==> ping 机械臂 $ROBOT"
ping -c 3 -W 2 "$ROBOT" && echo "✔ 机械臂网络已通！" || {
    echo "✘ ping 不通。若上面无链路警告，请在示教器 设置->关于(About) 里核对机械臂 IP，"
    echo "  并确认控制柜已完全开机。"
    exit 1
}
echo "==> 接下来可以跑连通性测试:"
echo "    python3 ~/ur_learn/scripts/ur_port_test.py $ROBOT"
