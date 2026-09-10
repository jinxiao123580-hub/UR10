#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fake UR 控制器（离线开发/自测用，勿用于真实控制）。
在本机回环地址上模拟:
  30001 Primary 客户端接口  -> 10Hz 推送 Robot State(含 6 关节运动的正弦关节角)
  30002 Secondary           -> 只收不发
  29999 Dashboard           -> 任意命令回 OK
用法:
  python3 fake_ur_server.py [port_offset]     # 默认偏移 0，即 30001/30002/29999
然后可让 ROS 节点连 127.0.0.1 测试。
"""
import math
import socket
import struct
import sys
import threading
import time

OFF = int(sys.argv[1]) if len(sys.argv) > 1 else 0
P_STATE = 30001 + OFF
P_SCRIPT = 30002 + OFF
P_DASH = 29999 + OFF

MESSAGE_TYPE_ROBOT_STATE = 16
SUB_ROBOT_MODE = 0
SUB_JOINT_DATA = 1


def robot_mode_pkg(t_ns: int) -> bytes:
    body = struct.pack(">Q", t_ns)
    body += bytes((1, 1, 1, 0, 0, 1, 0))          # conn/enabled/powerOn/...
    body += bytes((7, 0))                          # robotMode=7(RUNNING), controlMode
    body += struct.pack(">ddd", 1.0, 1.0, 1.0)     # speedFraction, speedScaling, limit
    body += b"\x00"                                # reserved
    return struct.pack(">IB", 5 + len(body), SUB_ROBOT_MODE) + body


def joint_data_pkg(qs) -> bytes:
    body = b"".join(
        struct.pack(">dddffffB", q, q, 0.0,      # q_actual q_target qd_actual
                    0.0, 0.0, 0.0, 0.0,          # I V T_motor T_micro (f32)
                    1)                           # jointMode
        for q in qs
    )
    return struct.pack(">IB", 5 + len(body), SUB_JOINT_DATA) + body


def state_frame(t: float) -> bytes:
    qs = [math.sin(t + 0.3 * i) * 1.2 for i in range(6)]
    content = bytes([MESSAGE_TYPE_ROBOT_STATE]) + robot_mode_pkg(int(t * 1e9)) + joint_data_pkg(qs)
    return struct.pack(">I", 4 + len(content)) + content


def dash_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", P_DASH))
    srv.listen(2)
    print(f"[fake] dashboard 127.0.0.1:{P_DASH}")
    while True:
        c, _ = srv.accept()
        threading.Thread(target=_dash_client, args=(c,), daemon=True).start()


def _dash_client(c):
    try:
        c.settimeout(1.0)
        while True:
            if not c.recv(1024):
                break
            c.sendall(b"OK\n")
    except OSError:
        pass
    finally:
        c.close()


def stream_server(port, echo=False):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(4)
    print(f"[fake] state stream 127.0.0.1:{port} (10Hz)")
    while True:
        c, _ = srv.accept()
        threading.Thread(target=_stream_client, args=(c, port, echo), daemon=True).start()


def _stream_client(c, port, echo):
    t0 = time.time()
    try:
        c.settimeout(0.5)
        while True:
            t = time.time() - t0
            try:
                c.sendall(state_frame(t))
            except OSError:
                break
            try:
                c.recv(4096)  # 读掉客户端发来的 URScript(模拟执行)
            except socket.timeout:
                pass
            except OSError:
                break
            time.sleep(0.1)
    except OSError:
        pass
    finally:
        c.close()


if __name__ == "__main__":
    print("[fake] starting ... (Ctrl-C to stop)")
    threading.Thread(target=dash_server, daemon=True).start()
    threading.Thread(target=stream_server, args=(P_STATE,), daemon=True).start()
    threading.Thread(target=stream_server, args=(P_SCRIPT,), daemon=True).start()
    while True:
        time.sleep(1)
