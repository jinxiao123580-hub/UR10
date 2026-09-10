#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 让 UR 末端做"相对移动"（沿基座坐标系/世界坐标轴）—— 字面量位姿版。

说明：这台机器人客户端接口禁止 get_actual_* 内省函数，
因此本脚本从 30001 状态流读当前 TCP 位姿(Cartesian info 子包 type=4)，
把目标位姿作为字面量写进 URScript。

用法（需已 source ROS 环境）:
    python3 ur_rel_move.py 0 0 -0.01        # 末端沿世界 Z 往下 10mm
    python3 ur_rel_move.py 0.1 0 0          # 沿世界 X 前进 100mm
    python3 ur_rel_move.py 0 0 0.05 0 0 0   # 位置+姿态偏移(可给 rx ry rz)

原理:
    cur = 电脑读到的当前 TCP 位姿(基座坐标系)
    target = cur + 偏移
    movel(target, a=0.3, v=0.05)

前提: 机器人已上电、刹车已松、没有程序在运行、示教器特征=Base。
安全: 速度 5cm/s 很保守；移动方向确保无障碍，手放急停。
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
        raise RuntimeError("读取 TCP 位姿失败")
    return cart


if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) < 3:
        print("用法: python3 ur_rel_move.py dx dy dz [rx ry rz]\n"
              "例:   python3 ur_rel_move.py 0 0 -0.01   # 往下 10mm")
        sys.exit(1)
    offs = [float(a) for a in args[:6]] + [0.0] * (6 - len(args))

    cx, cy, cz, rx, ry, rz = get_tcp_pose()
    tx = cx + offs[0]
    ty = cy + offs[1]
    tz = cz + offs[2]
    trx = rx + offs[3]
    try_ = ry + offs[4]
    trz = rz + offs[5]
    print(f"当前 TCP: [{cx:.4f},{cy:.4f},{cz:.4f}] 目标: [{tx:.4f},{ty:.4f},{tz:.4f}]")

    script = f"def ros_move():\n  movel(p[{tx!r},{ty!r},{tz!r},{trx!r},{try_!r},{trz!r}], a=0.3, v=0.05)\nend"

    rclpy.init()
    n = Node("ur_rel_move")
    pub = n.create_publisher(String, "/ur_link/urscript", 10)
    time.sleep(0.5)
    msg = String()
    msg.data = script
    pub.publish(msg)
    print("已发送 URScript:\n" + script)
    rclpy.shutdown()
