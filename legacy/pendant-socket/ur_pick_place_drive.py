#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
两个位置循环抓取-放置驱动器（配合示教器"通用执行器"用）。

协议(执行器的命令): move+6浮点 / grip / release / stop
流程(每轮): 抓取点上方→抓取点→夹爪闭合→抬起→放置点上方→放置点→夹爪张开→抬起

用法:
    python3 ur_pick_place_drive.py 5                 # 循环 5 次(点位读 poses.json)
    python3 ur_pick_place_drive.py --forever        # 一直循环
    Ctrl-C 停止(自动发 stop)

前置:
    1. 示教器已播放"通用执行器"程序(见 pendant_command_executor.urscript)
    2. 本脚本监听 30010，执行器连入后开始
"""
import json
import os
import socket
import sys
import time

PORT = 30010
POSES_FILE = os.path.expanduser("~/ur_learn/poses.json")
H = 0.05  # 安全高度(米)


def wait_ok(conn, timeout=180):
    """等执行器回执，直到出现 ok。
    超时给 180s：执行器里夹爪 sleep(5)、移动 sleep(10)，每步都可能很久。"""
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
            print(f"  执行器: {line!r}")
            if line == "ok" or "ok" in line:
                return True
            buf = b""
    return False


def cmd(conn, text):
    conn.sendall((text + "\n").encode())


def send_pose(conn, pose):
    conn.sendall((",".join(f"{v:.6f}" for v in pose) + "\n").encode())


def move(conn, pose):
    cmd(conn, "move")
    time.sleep(0.2)
    send_pose(conn, pose)
    return wait_ok(conn)


def grip(conn):
    cmd(conn, "grip")
    return wait_ok(conn)


def release(conn):
    cmd(conn, "release")
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

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT))
    srv.listen(1)
    print(f"[驱动器] 监听 {PORT}，等示教器执行器连入...")
    conn, addr = srv.accept()
    conn.settimeout(3)
    print(f"[驱动器] 执行器已连入: {addr}")

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
            ok = move(conn, pa)          # 到抓取点上方
            if not ok:
                break
            ok = move(conn, pk)          # 下降到抓取点
            if not ok:
                break
            ok = grip(conn)              # 夹爪闭合
            if not ok:
                break
            ok = move(conn, pa)          # 抬起
            if not ok:
                break
            ok = move(conn, pla)         # 移到放置点上方
            if not ok:
                break
            ok = move(conn, pl)          # 下降到放置点
            if not ok:
                break
            ok = release(conn)           # 夹爪张开
            if not ok:
                break
            ok = move(conn, pla)         # 抬起离开
            if not ok:
                break
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
