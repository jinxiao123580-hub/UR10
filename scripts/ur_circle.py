#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 让 UR 末端在水平面(基座XY平面, Z不变)画圆 —— 字面量圆心版。

关键：这台机器人客户端接口禁止 get_actual_* 内省函数，
因此本脚本从 30001 状态流的 Cartesian info 子包读取当前 TCP 位姿，
把圆心/姿态作为字面量写进 URScript，绕开 get_actual_*。

用法（需已 source ROS 环境）:
    python3 ur_circle.py 0.02                 # 半径20mm，一直画(默认)
    python3 ur_circle.py 0.02 --rounds 3      # 画 3 圈后自动停下
    python3 ur_circle.py 0.05                 # 半径50mm

停止: 按急停，或
    ros2 service call /ur_link/dashboard/stop std_srvs/srv/Trigger
"""
import socket
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ROBOT_IP = "192.168.1.3"


def get_tcp_pose(host=ROBOT_IP, timeout=4.0):
    """从 30001 状态流读取 TCP 位姿(Cartesian info 子包 type=4)。"""
    s = socket.create_connection((host, 30001), timeout=3)
    s.settimeout(timeout)
    buf = b""
    t0 = time.time()
    cart = None
    while time.time() - t0 < timeout and cart is None:
        try:
            buf += s.recv(65536)
        except socket.timeout:
            break
        # 逐帧
        off = 0
        while off + 4 <= len(buf):
            size = struct.unpack(">I", buf[off:off+4])[0]
            if not (5 <= size <= 100000) or off + size > len(buf):
                break
            f = buf[off:off+size]
            off += size
            if len(f) < 5 or f[4] != 16:
                continue
            p = 5
            while p + 5 <= len(f):
                sz = struct.unpack(">I", f[p:p+4])[0]
                ty = f[p+4]
                if sz < 5 or p + sz > len(f):
                    break
                if ty == 4 and sz - 5 >= 48:
                    cart = struct.unpack(">dddddd", f[p+5:p+53])
                    break
                p += sz
        if off:
            buf = buf[off:]
    s.close()
    if not cart:
        raise RuntimeError("读取 TCP 位姿失败(检查 robot_ip)")
    return cart


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help"):
        print("用法: python3 ur_circle.py 半径 [--rounds N]  例: ur_circle.py 0.02")
        sys.exit(1)
    r = float(args[0])
    rounds = None
    if "--rounds" in args:
        rounds = int(args[args.index("--rounds") + 1])

    cx, cy, cz, rx, ry, rz = get_tcp_pose()
    print(f"当前 TCP 位姿: xyz=[{cx:.4f},{cy:.4f},{cz:.4f}] rpy=[{rx:.4f},{ry:.4f},{rz:.4f}]")

    def P(x, y, z):
        return f"p[{x!r},{y!r},{z!r},{rx!r},{ry!r},{rz!r}]"

    start = P(cx + r, cy, cz)
    via1 = P(cx, cy + r, cz)
    to1 = P(cx - r, cy, cz)
    via2 = P(cx, cy - r, cz)

    body = (f"  movec({via1}, {to1}, a=0.3, v=0.05, r=0.001)\n"
            f"  movec({via2}, {start}, a=0.3, v=0.05, r=0.001)")
    if rounds is None:
        loop = f"while True:\n{body}\nend"
    else:
        loop = f"n := 0\nwhile n < {rounds}:\n{body}\n  n := n + 1\nend"

    script = "\n".join([
        f"movel({start}, a=0.3, v=0.05)",
        loop,
    ])

    rclpy.init()
    n = Node("ur_circle")
    pub = n.create_publisher(String, "/ur_link/urscript", 10)
    time.sleep(0.5)
    msg = String()
    msg.data = script
    pub.publish(msg)
    print(f"已发送画圆 URScript（半径 {r}m，圆心 [{cx:.4f},{cy:.4f},{cz:.4f}]）:")
    print(script)
    print("\n停止: ros2 service call /ur_link/dashboard/stop std_srvs/srv/Trigger  或按急停")
    rclpy.shutdown()
