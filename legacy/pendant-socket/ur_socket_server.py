#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电脑端 TCP 服务器 —— 对应《如何与上位机数据交互》第 3 步：
电脑开 TCP Server，机器人程序里 socket_open("电脑IP", 端口, "socket_1") 连进来。

用法:
    python3 ur_socket_server.py [端口]       # 默认 30000

连上后:
    - 机器人发来的内容会实时打印(机器人侧用 socket_send_line 等)
    - 你在终端输入一行并按回车 = 发给机器人
      (配合机器人端 socket_read_ascii_float(6) 时，发: 1.5,2.2,3.3,4,5,6)
"""
import socket
import sys
import threading

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 30000

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", PORT))
srv.listen(1)

print(f"[服务器] 监听 0.0.0.0:{PORT}，等待机器人连接 ...")
print("[提示]   机器人侧: eth_status := socket_open(\"192.168.1.10\", %d, \"socket_1\")"
      % PORT)
print("         终端里输入一行并回车 = 发送给机器人；Ctrl-C 退出\n")

conn, addr = srv.accept()
print(f"[连接] 机器人已连入: {addr}")

stop = threading.Event()


def reader():
    try:
        while not stop.is_set():
            data = conn.recv(4096)
            if not data:
                print("[连接断开] 机器人关闭了连接")
                stop.set()
                break
            print(f"[机器人→电脑] {data!r}")
    except OSError:
        pass


threading.Thread(target=reader, daemon=True).start()

try:
    while not stop.is_set():
        line = input("")  # 输入内容发送给机器人
        conn.sendall(line.encode() + b"\n")
except (EOFError, KeyboardInterrupt):
    pass
finally:
    stop.set()
    try:
        conn.close()
    except OSError:
        pass
    srv.close()
    print("\n[服务器] 已退出")
