#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robotiq 2F 夹爪 —— 电脑直控（不需要示教器程序！）

原理：UR 机器人上跑着 Robotiq URCap 的守护进程 drivergripper，
监听 TCP 63352（IPv6 通配，所以电脑能直连），说的是标准 Robotiq ASCII 协议。

用法:
    python3 rq_gripper.py status          # 读状态
    python3 rq_gripper.py open            # 张开
    python3 rq_gripper.py close           # 闭合
    python3 rq_gripper.py test            # 开关一轮自检
"""
import re
import socket
import sys
import time

ROBOT_IP = "192.168.1.3"
GRIPPER_PORT = 63352

# Robotiq ASCII 协议里 POS 的取值
POS_OPEN = 0      # 全开
POS_CLOSE = 255   # 全闭


class RobotiqGripper:
    def __init__(self, ip=ROBOT_IP, port=GRIPPER_PORT, timeout=3.0):
        self.ip, self.port, self.timeout = ip, port, timeout
        self.sock = None
        self._buf = b""

    # ---------- 连接 ----------
    def connect(self):
        if self.sock:
            return self
        s = socket.create_connection((self.ip, self.port), timeout=self.timeout)
        s.settimeout(self.timeout)
        self.sock = s
        self._buf = b""
        return self

    def close_socket(self):
        if self.sock:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def __enter__(self):
        return self.connect()

    def __exit__(self, *a):
        self.close_socket()

    # ---------- 底层收发 ----------
    def _send(self, cmd):
        self.connect()
        self.sock.sendall((cmd + "\n").encode())

    def _read_line(self):
        while b"\n" not in self._buf:
            chunk = self.sock.recv(256)
            if not chunk:
                raise ConnectionError("夹爪守护进程断开连接")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode(errors="replace").strip()

    def query(self, name):
        """发 GET <name> 并返回数值；读不出数字返回 None

        注意：守护进程的回复是**补零/带括号格式**（实测 FLT 00、PRE 000、SID [9]），
        所以不能直接 int(parts[1])，要用正则抠出数字。
        """
        self._send("GET %s" % name)
        line = self._read_line()
        parts = line.split()
        if len(parts) >= 2 and parts[0].upper() == name.upper():
            m = re.search(r"-?\d+", parts[1])
            if m:
                return int(m.group())
        return None

    def set_var(self, name, value):
        self._send("SET %s %d" % (name, int(value)))

    # ---------- 状态 ----------
    def status(self):
        self.connect()
        st = {}
        for k in ("SID", "ACT", "GTO", "STA", "FLT", "POS", "PRE", "OBJ", "SPE", "FOR"):
            try:
                st[k] = self.query(k)
            except Exception as e:
                st[k] = "ERR:%s" % type(e).__name__
        return st

    def is_connected(self):
        sid = self.status().get("SID")
        return bool(sid) and sid > 0

    def is_activated(self):
        return self.query("ACT") == 1

    # ---------- 动作 ----------
    def activate(self, wait=5.0):
        """激活夹爪（机器人断电重启后必须做一次）"""
        self.connect()
        if self.query("ACT") == 1:
            return True
        self.set_var("ACT", 1)
        t0 = time.time()
        while time.time() - t0 < wait:
            if self.query("ACT") == 1:
                return True
            time.sleep(0.2)
        return False

    def move(self, pos, speed=255, force=255, wait=True, timeout=4.0):
        """pos: 0=全开, 255=全闭"""
        self.connect()
        pos = max(0, min(255, int(pos)))
        self.set_var("SPE", speed)
        self.set_var("FOR", force)
        self.set_var("POS", pos)
        self.set_var("GTO", 1)
        if not wait:
            return None
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = self.query("POS")
            if p is not None and abs(p - pos) <= 3:
                return p
            time.sleep(0.1)
        return self.query("POS")

    def open(self, **kw):
        return self.move(POS_OPEN, **kw)

    def close(self, **kw):
        return self.move(POS_CLOSE, **kw)

    def object_detected(self):
        """OBJ: 0=运动中, 1=张开时检测到物体, 2=闭合时检测到物体, 3=到达指定位置未检测到物体"""
        return self.query("OBJ")


def _fmt(st):
    return "  ".join("%s=%s" % (k, st[k]) for k in
                     ("SID", "ACT", "GTO", "STA", "FLT", "POS", "PRE", "OBJ"))


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    g = RobotiqGripper()
    try:
        g.connect()
    except Exception as e:
        print("✘ 无法连接夹爪守护进程 %s:%d —— %s" % (ROBOT_IP, GRIPPER_PORT, e))
        return 1

    if cmd == "status":
        print("夹爪状态:", _fmt(g.status()))

    elif cmd == "open":
        g.activate()
        print("张开 ->POS", g.open(), "|", _fmt(g.status()))

    elif cmd == "close":
        g.activate()
        print("闭合 ->POS", g.close(), "|", _fmt(g.status()))

    elif cmd == "test":
        print("① 初始:", _fmt(g.status()))
        print("② 激活:", "OK" if g.activate() else "失败", "|", _fmt(g.status()))
        print("③ 闭合 -> POS", g.close())
        print("   ", _fmt(g.status()), " OBJ=", g.object_detected())
        time.sleep(1)
        print("④ 张开 -> POS", g.open())
        print("   ", _fmt(g.status()), " OBJ=", g.object_detected())
        print("✔ 自检完成（夹爪如果实际开合过，就说明电脑直控成功）")
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
