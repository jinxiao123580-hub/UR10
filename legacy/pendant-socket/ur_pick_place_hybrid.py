#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
两个位置循环抓取-放置驱动器 —— 混合架构：
  - 机械臂移动：直接走 30002 发 URScript(movel 字面量位姿) —— 和最初 ur_rel_move 一样
  - 夹爪开合：走 30010 socket 发给示教器"通用执行器"(rq_set_pos)
已验证：执行器程序运行时，30002 注入 movel 依然有效。

用法:
    python3 ur_pick_place_hybrid.py 5                 # 循环 5 次(点位读 poses.json)
    python3 ur_pick_place_hybrid.py --forever        # 一直循环
    Ctrl-C 停止

前置:
    1. 示教器已播放"通用执行器"(支持 grip/release)，连到 30010
    2. 30002 可用(ur_command_node 或直接 socket)
"""
import json
import os
import socket
import sys
import time

ROBOT_IP = "192.168.1.3"
SOCKET_PORT = 30010      # 夹爪执行器
URSCRIPT_PORT = 30002    # 机械臂移动
POSES_FILE = os.path.expanduser("~/ur_learn/poses.json")
H = 0.05                 # 安全高度(米)


_ur_sock = None


def urscript_move(pose):
    """30002 注入 movel(字面量位姿)，用长连接(发完别秒关，防丢)。movel 阻塞到走完。"""
    global _ur_sock
    px, py, pz, rx, ry, rz = pose
    script = (f"def t():\n"
              f"  movel(p[{px!r},{py!r},{pz!r},{rx!r},{ry!r},{rz!r}], a=0.3, v=0.05)\n"
              f"end")
    global _ur_sock
    for _ in range(2):
        try:
            if _ur_sock is None:
                _ur_sock = socket.create_connection((ROBOT_IP, URSCRIPT_PORT), timeout=3)
                _ur_sock.settimeout(0.3)
            _ur_sock.sendall((script + "\n").encode())
            try:
                _ur_sock.recv(4096)
            except socket.timeout:
                pass
            return
        except OSError:
            _ur_sock = None
    raise ConnectionError("30002 发送失败")


def wait_ok(conn, timeout=180):
    buf = b""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            c = conn.recv(1)
        except socket.timeout:
            continue
        except OSError:
            return False
        if not c:
            return False
        buf += c
        if c == b"\n":
            line = buf.decode(errors="replace").strip()
            print(f"  夹爪: {line!r}")
            if "ok" in line:
                return True
            buf = b""
    return False


def grip(conn):
    print("夹爪: 闭合")
    conn.sendall(b"grip\n")
    return wait_ok(conn)


def release(conn):
    print("夹爪: 张开")
    conn.sendall(b"release\n")
    return wait_ok(conn)


if __name__ == "__main__":
    args = sys.argv[1:]
    count = None
    if "--forever" in args:
        count = None
    elif args and args[0].lstrip("-").isdigit():
        count = int(args[0])

    poses = json.load(open(POSES_FILE))
    pk = poses["pick"]
    pl = poses["place"]
    pa = [pk[0], pk[1], pk[2] + H, pk[3], pk[4], pk[5]]
    pla = [pl[0], pl[1], pl[2] + H, pk[3], pk[4], pk[5]]
    print(f"抓取点: {[round(v,4) for v in pk]}")
    print(f"放置点: {[round(v,4) for v in pl]}  安全高度 {H}m"
          + ("  一直循环" if count is None else f"  {count} 轮"))

    # 等夹爪执行器连入(30010)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", SOCKET_PORT))
    srv.listen(1)
    print(f"[驱动器] 监听 {SOCKET_PORT} 等夹爪执行器连入...")
    conn, addr = srv.accept()
    conn.settimeout(3)
    print(f"[驱动器] 夹爪执行器已连入: {addr}")

    def send_stop():
        try:
            conn.sendall(b"stop\n")
        except OSError:
            pass

    i = 0
    try:
        while True:
            if count is not None and i >= count:
                print(f"[驱动器] 完成 {count} 轮，发 stop")
                send_stop()
                break
            i += 1
            print(f"\n=== 第 {i} 轮 ===")
            urscript_move(pa)     # 到抓取点上方
            urscript_move(pk)     # 下降到抓取点
            if not grip(conn):    # 夹爪闭合
                break
            urscript_move(pa)     # 抬起
            urscript_move(pla)    # 移到放置点上方
            urscript_move(pl)     # 下降到放置点
            if not release(conn): # 夹爪张开
                break
            urscript_move(pla)    # 抬起离开
    except KeyboardInterrupt:
        print("\n[驱动器] Ctrl-C，发 stop")
        send_stop()
    finally:
        try:
            conn.close()
        except OSError:
            pass
        srv.close()
        print("[驱动器] 已退出")
