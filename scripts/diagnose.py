#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR10 一键体检 —— 出问题先跑这个。

检查项：
  1) 网络连通（ping）
  2) 关键端口（29999 / 30001 / 30002 / 63352）
  3) 机器人模式、安全状态、程序状态、机型
  4) 当前 TCP 位姿（读 30001 状态流）
  5) 夹爪状态（直连 63352 守护进程）

用法:
    python3 diagnose.py
    python3 diagnose.py 192.168.1.3      # 指定机器人 IP
"""
import re
import socket
import struct
import subprocess
import sys
import time

ROBOT_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.3"

PORTS = [
    (29999, "Dashboard (状态/上电/播放)"),
    (30001, "状态流（读关节角/位姿）"),
    (30002, "URScript（机械臂运动）"),
    (63352, "夹爪守护进程（Robotiq ASCII）"),
]

ok_all = True


def line(tag, msg, ok=None):
    global ok_all
    mark = "" if ok is None else ("  OK" if ok else "  ✘")
    if ok is False:
        ok_all = False
    print("[%-6s] %-34s%s" % (tag, msg, mark))


def check_ping(ip, timeout=2):
    try:
        r = subprocess.run(["ping", "-c", "1", "-W", str(timeout), ip],
                           capture_output=True, timeout=timeout + 2)
        if r.returncode == 0:
            out = r.stdout.decode(errors="replace")
            ms = ""
            for tok in out.split():
                if tok.startswith("time="):
                    ms = "  " + tok
                    break
            return True, ms
        return False, ""
    except Exception as e:
        return False, "  %s" % e


def check_port(ip, port, timeout=2):
    try:
        s = socket.create_connection((ip, port), timeout=timeout)
        s.close()
        return True
    except Exception:
        return False


def dashboard(cmd, ip=ROBOT_IP, timeout=3):
    try:
        s = socket.create_connection((ip, 29999), timeout=timeout)
        s.recv(4096)
        s.sendall((cmd + "\n").encode())
        time.sleep(0.35)
        r = s.recv(4096).decode(errors="replace").strip()
        s.close()
        return r
    except Exception as e:
        return "ERR %s" % e


def read_tcp_pose(ip=ROBOT_IP, timeout=3.0):
    """从 30001 读 Cartesian info 子包（type=4）拿 TCP 位姿"""
    try:
        s = socket.create_connection((ip, 30001), timeout=3)
        s.settimeout(timeout)
    except Exception:
        return None
    buf, t0 = b"", time.time()
    while time.time() - t0 < timeout:
        try:
            buf += s.recv(65536)
        except socket.timeout:
            break
        off = 0
        while off + 4 <= len(buf):
            size = struct.unpack(">I", buf[off:off + 4])[0]
            if not (5 <= size <= 100000) or off + size > len(buf):
                break
            f = buf[off:off + size]
            off += size
            if len(f) < 5 or f[4] != 16:
                continue
            p = 5
            while p + 5 <= len(f):
                sz = struct.unpack(">I", f[p:p + 4])[0]
                if sz < 5 or p + sz > len(f):
                    break
                if f[p + 4] == 4 and sz - 5 >= 48:
                    s.close()
                    return struct.unpack(">dddddd", f[p + 5:p + 53])
                p += sz
        if off:
            buf = buf[off:]
    s.close()
    return None


def gripper_status(ip=ROBOT_IP, timeout=3):
    """直连 63352，发 GET 拿状态"""
    try:
        s = socket.create_connection((ip, 63352), timeout=timeout)
        s.settimeout(timeout)
    except Exception as e:
        return None, str(e)
    st = {}
    buf = b""
    for name in ("SID", "ACT", "GTO", "STA", "FLT", "POS", "OBJ"):
        s.sendall(("GET %s\n" % name).encode())
        try:
            while b"\n" not in buf:
                c = s.recv(256)
                if not c:
                    break
                buf += c
        except socket.timeout:
            pass
        if b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            parts = ln.decode(errors="replace").strip().split()
            if len(parts) > 1:
                # 守护进程回复是补零/带括号格式：FLT 00 / PRE 000 / SID [9]
                m = re.search(r"-?\d+", parts[1])
                st[name] = int(m.group()) if m else parts[1]
            else:
                st[name] = "?"
        else:
            st[name] = "无响应"
    s.close()
    return st, None


def main():
    print("=== UR10 一键体检 (%s) ===" % ROBOT_IP)

    print("\n--- 1. 网络 ---")
    pok, extra = check_ping(ROBOT_IP)
    line("网络", "ping %s%s" % (ROBOT_IP, extra), pok)
    if not pok:
        print("\n✘ 网络不通。检查：网线插在控制柜 Network 口 / 控制柜已开机 / "
              "电脑 IP 是 192.168.1.x / 代理是否抢了路由（跑 network_setup.sh）")
        return 1

    print("\n--- 2. 端口 ---")
    ports_ok = {}
    for port, desc in PORTS:
        ok = check_port(ROBOT_IP, port)
        ports_ok[port] = ok
        line("端口", "%d %s" % (port, desc), ok)

    print("\n--- 3. 机器人状态 ---")
    for cmd in ("PolyscopeVersion", "get robot model", "robotmode",
                "safetystatus", "programState"):
        if ports_ok.get(29999):
            line("状态", "%-12s -> %s" % (cmd, dashboard(cmd)))
        else:
            line("状态", "%s  (29999 不通，跳过)" % cmd, None)

    print("\n--- 4. 机械臂 ---")
    pose = read_tcp_pose(ROBOT_IP) if ports_ok.get(30001) else None
    if pose:
        line("机械臂", "TCP 位姿 [%s]" % ", ".join("%.4f" % x for x in pose[:3]), True)
    else:
        line("机械臂", "读不到 TCP 位姿（30001 / 未上电）", False)

    print("\n--- 5. 夹爪 ---")
    if ports_ok.get(63352):
        st, err = gripper_status(ROBOT_IP)
        if st:
            line("夹爪", "ACT=%s STA=%s FLT=%s POS=%s OBJ=%s"
                 % (st.get("ACT"), st.get("STA"), st.get("FLT"),
                    st.get("POS"), st.get("OBJ")), True)
            if st.get("FLT") not in (0, "0", None):
                line("夹爪", "⚠ 有故障码 FLT=%s，发 SET ACT 0 再 SET ACT 1 复位"
                     % st.get("FLT"), None)
            if st.get("ACT") != 1:
                line("夹爪", "未激活，发 SET ACT 1 激活", None)
        else:
            line("夹爪", "连接成功但无响应: %s" % err, False)
    else:
        line("夹爪", "63352 不通（drivergripper 守护进程没起？）", False)

    print("\n" + "=" * 46)
    if ok_all:
        print("诊断结论: 全部正常，可以干活")
        print("下一步: python3 ur_pick_place_full.py 1")
    else:
        print("诊断结论: 有项目异常，见上方 ✘ / ⚠")
        print("排查指引: docs/06-排查手册.md")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
