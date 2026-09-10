#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电脑端"通用循环抓取-放置"驱动器。
配合示教器通用执行器（点位由电脑下发）使用。

协议:
    1. 连接后先发点位:
       "pick\n" + "x,y,z,rx,ry,rz\n"
       "place\n" + "x,y,z,rx,ry,rz\n"
    2. 然后循环: 发 "cycle" → 等机器人回 "done" → 下一轮
       发 "stop" 停止

点位来源: ~/ur_learn/poses.json (ur_capture_pose.py 录制)
         或命令行 --pick x,y,z,rx,ry,rz --place ...

用法:
    python3 ur_pick_place_cycle.py 5               # 循环 5 次(用 poses.json 点位)
    python3 ur_pick_place_cycle.py --forever
    python3 ur_pick_place_cycle.py 5 --pick 0.6,-0.2,0.3,1.9,-2.3,0 --place 0.7,0.0,0.3,1.9,-2.3,0
"""
import json
import os
import socket
import sys
import time

PORT = 30010
POSES_FILE = os.path.expanduser("~/ur_learn/poses.json")


def parse_pose(s):
    vals = [float(x) for x in s.split(",")]
    if len(vals) != 6:
        raise ValueError(f"位姿需 6 个数: {s}")
    return vals


if __name__ == "__main__":
    args = sys.argv[1:]
    count = None
    pick = place = None
    if "--forever" in args:
        count = None
    elif args and args[0].lstrip("-").isdigit():
        count = int(args[0])
    if "--pick" in args:
        pick = parse_pose(args[args.index("--pick") + 1])
    if "--place" in args:
        place = parse_pose(args[args.index("--place") + 1])
    if pick is None or place is None:
        poses = json.load(open(POSES_FILE))
        if pick is None:
            pick = poses["pick"]
        if place is None:
            place = poses["place"]
    print(f"抓取点: {[round(v,4) for v in pick]}")
    print(f"放置点: {[round(v,4) for v in place]}")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", PORT))
    srv.listen(1)
    print(f"[电脑端] 监听 {PORT}，等待示教器执行器连入...")
    conn, addr = srv.accept()
    conn.settimeout(10)
    print(f"[电脑端] 机器人执行器已连入: {addr}")

    # 下发点位
    def send_pose(tag, pose):
        conn.sendall((tag + "\n").encode())
        time.sleep(0.3)
        conn.sendall((",".join(f"{v:.6f}" for v in pose) + "\n").encode())
        time.sleep(0.3)

    send_pose("pick", pick)
    send_pose("place", place)
    print("[电脑端] 点位已下发")

    def send_stop():
        try:
            conn.sendall(b"stop\n")
        except OSError:
            pass

    i = 0
    try:
        while True:
            if count is not None and i >= count:
                print(f"[电脑端] 已完成 {count} 轮，发送 stop")
                send_stop()
                break
            i += 1
            print(f"[电脑端] 第 {i} 轮: 发送 cycle ...")
            conn.sendall(b"cycle\n")
            buf = b""
            got = False
            t0 = time.time()
            while time.time() - t0 < 30:
                try:
                    c = conn.recv(1)
                except socket.timeout:
                    continue
                except OSError:
                    print("[电脑端] 连接断开")
                    raise SystemExit
                if not c:
                    print("[电脑端] 机器人关闭连接")
                    raise SystemExit
                buf += c
                if c == b"\n":
                    msg = buf.decode(errors="replace").strip()
                    print(f"[电脑端] 机器人: {msg!r}")
                    if "done" in msg or "stopped" in msg:
                        got = True
                    buf = b""
                    if got:
                        break
            if not got:
                print("[电脑端] 30秒没等到 done，超时")
    except KeyboardInterrupt:
        print("\n[电脑端] Ctrl-C，发送 stop")
        send_stop()
    finally:
        try:
            conn.close()
        except OSError:
            pass
        srv.close()
        print("[电脑端] 已退出")
