#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robotiq 夹爪远程控制（电脑端服务器）。

原理：示教器跑 demo/pendant_gripper_server.urscript 程序(内含 rq_*，
URCap 函数只能在示教器环境用)，它主动 socket 连到本服务器的 30010 端口，
然后本程序把你敲的命令转发过去执行。

用法:
    1. 先运行本程序（等待机器人连入）:
       python3 ur_gripper_server.py [端口]
    2. 示教器上运行 pendant_gripper_server.urscript（机器人会连上来）
    3. 在本终端输入命令:
       open      -> 夹爪张开  rq_move_to(0)
       close     -> 夹爪闭合  rq_move_to(255)
       quit      -> 断开
"""
import select
import socket
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 30010

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", PORT))
srv.listen(1)
print(f"[服务器] 监听 0.0.0.0:{PORT}，等待机器人连入...")
print("[提示]   示教器运行 demo/pendant_gripper_server.urscript 后这里会显示连接")
conn, addr = srv.accept()
conn.settimeout(0.2)  # 短超时轮询，保证终端输入能被及时处理
print(f"[连接] 机器人已连入: {addr}")
print("[控制] 输入命令: open / close / quit")


def readline(sock):
    buf = b""
    while True:
        try:
            c = sock.recv(1)
        except socket.timeout:
            return None
        if not c:
            return b""
        buf += c
        if c == b"\n":
            return buf


try:
    while True:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            line = sys.stdin.readline().strip()
            if not line:
                continue
            if line == "quit":
                print("[退出]")
                break
            if line in ("open", "close"):
                conn.sendall((line + "\n").encode())
            else:
                print("未知命令: open / close / quit")
        msg = readline(conn)
        if msg is None:
            continue
        if msg == b"":
            print("[机器人断开]")
            break
        print(f"[机器人] {msg.decode().strip()}")
except KeyboardInterrupt:
    pass
finally:
    try:
        conn.close()
    except OSError:
        pass
    srv.close()
    print("\n[服务器] 已退出")
