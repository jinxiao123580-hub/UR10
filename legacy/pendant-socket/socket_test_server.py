#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""持久 socket 测试器：监听 30010，接受多次连接，发命令变体，记录回执。"""
import socket
import threading
import time

LOG = "/home/jx/ur_learn/socket_test.log"


def log(msg):
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    print(line, flush=True)
    with open(LOG, "a") as f:
        f.write(line + "\n")


open(LOG, "w").close()
log("持久测试器启动，监听 30010 ...")

srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", 30010))
srv.listen(3)
srv.settimeout(1200)

conn_id = 0
while True:
    try:
        conn, addr = srv.accept()
    except socket.timeout:
        log("20分钟无连接，退出")
        break
    conn_id += 1
    cid = conn_id
    conn.settimeout(1.0)
    log(f"#连接{cid} 机器人连入: {addr}")

    def reader():
        buf = b""
        while True:
            try:
                c = conn.recv(1)
            except socket.timeout:
                continue
            except OSError:
                log(f"#连接{cid} 断开")
                return
            if not c:
                log(f"#连接{cid} 机器人关闭")
                return
            buf += c
            if c == b"\n":
                log(f"#连接{cid} 机器人发来: {buf.decode(errors='replace').strip()!r}")
                buf = b""

    threading.Thread(target=reader, daemon=True).start()

    # 发命令变体：带\n / 不带\n / 回车换行
    time.sleep(1.5)
    variants = [b"open\n", b"open", b"close\n", b"OPEN\n", b"ping\n"]
    for v in variants:
        log(f"#连接{cid} >>> 发送 {v!r}")
        try:
            conn.sendall(v)
        except OSError:
            log(f"#连接{cid} 发送失败(连接已断)")
            break
        time.sleep(3)
    log(f"#连接{cid} 命令序列结束，保持连接观察")
    # 保持连接不关闭，等机器人后续数据
