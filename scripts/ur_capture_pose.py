#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
记录当前 TCP 位姿到 ~/ur_learn/poses.json（供 ur_pick_place.py 使用）。

用法（需已 source ROS 环境或直接用，纯 stdlib）:
    python3 ur_capture_pose.py pick     # 手动把机器人移到抓取点后运行
    python3 ur_capture_pose.py place    # 移到放置点后运行
"""
import json
import os
import socket
import struct
import sys
import time

ROBOT_IP = "192.168.1.3"
POSES_FILE = os.path.expanduser("~/ur_learn/poses.json")


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
    if len(sys.argv) < 2:
        print("用法: python3 ur_capture_pose.py <点位名称>  例: ur_capture_pose.py pick")
        sys.exit(1)
    name = sys.argv[1]
    pose = get_tcp_pose()
    poses = json.load(open(POSES_FILE)) if os.path.exists(POSES_FILE) else {}
    poses[name] = list(pose)
    json.dump(poses, open(POSES_FILE, "w"), indent=2)
    print(f"✔ 已保存点位 '{name}':")
    print(f"   xyz=[{pose[0]:.4f}, {pose[1]:.4f}, {pose[2]:.4f}]"
          f" rpy=[{pose[3]:.4f}, {pose[4]:.4f}, {pose[5]:.4f}]")
    print(f"   现有点位: {list(poses)}")
