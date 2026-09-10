#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""循环抓放观察服务器：监听 30010，记录机器人回执(gripped/released)，
约 30 秒后发 stop 停掉循环。日志: ~/ur_learn/pickplace_test.log"""
import socket
import threading
import time

LOG = "/home/jx/ur_learn/pickplace_test.log"
open(LOG, "w").close()


def log(msg):
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    print(line, flush=True)
    with open(LOG, "a") as f:
        f.write(line + "\n")


log("循环抓放观察服务器启动，监听 30010 ...")
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("0.0.0.0", 30010))
srv.listen(2)
srv.settimeout(600)
conn, addr = srv.accept()
conn.settimeout(1.0)
log(f"机器人连入: {addr}")

buf = b""
t0 = time.time()
gripped = released = 0
stop_sent = False
while time.time() - t0 < 45:
    try:
        c = conn.recv(1)
    except socket.timeout:
        # 观察满 30 秒后发 stop
        if time.time() - t0 > 30 and not stop_sent:
            log(">>> 发送 stop")
            conn.sendall(b"stop\n")
            stop_sent = True
        continue
    except OSError:
        log("连接断开")
        break
    if not c:
        log("机器人关闭连接")
        break
    buf += c
    if c == b"\n":
        msg = buf.decode(errors="replace").strip()
        log(f"机器人发来: {msg!r}")
        if "gripped" in msg:
            gripped += 1
        if "released" in msg:
            released += 1
        buf = b""

log(f"观察结束: gripped x{gripped}, released x{released}, stop已发={stop_sent}")
conn.close()
srv.close()
