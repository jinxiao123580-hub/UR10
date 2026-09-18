#!/usr/bin/env python3
"""UR10 状态读取（课程第 2 节）—— 只读，绝不发送任何运动或夹爪命令。

读两路数据：
  1) UR 30003 Real-time Client（约 125 Hz）→ 关节角、TCP 位姿、TCP 速度
  2) Dashboard 29999 → robotmode / safetystatus / programState / 版本 / 机型/序列号

用途：示教器点动机器人后，用本脚本读数并与示教器显示对照，验证"示教器看到的"
就是"程序读到的"。这是第 3 节用程序控制运动的前提。

用法:
    python3 scripts/read_ur_state.py                     # 单次快照
    python3 scripts/read_ur_state.py --watch             # 连续显示（默认 5 Hz）
    python3 scripts/read_ur_state.py --watch --hz 2      # 2 Hz
    python3 scripts/read_ur_state.py --json              # JSON（便于落盘存档）
    python3 scripts/read_ur_state.py --json --output outputs/state/snapshot.json
"""
import argparse
import json
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from record_ur_trajectory import RealtimeReader  # noqa: E402

DEFAULT_IP = "192.168.1.3"
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
POSE_NAMES = ["X", "Y", "Z", "RX", "RY", "RZ"]
# Dashboard 命令 → 展示标签。取不到的（CB3 版本差异）会被跳过而不是报错。
DASHBOARD_QUERIES = [
    ("robotmode", "robotmode"),
    ("safetystatus", "safetystatus"),
    ("programState", "programState"),
    ("get robot model", "机型"),
    ("get serial number", "序列号"),
    ("PolyscopeVersion", "PolyScope 版本"),
    ("get loaded program", "已加载程序"),
]


def dashboard(cmd, ip=DEFAULT_IP, timeout=3.0):
    """往 29999 发一行文本，返回应答行；失败返回 None。"""
    try:
        with socket.create_connection((ip, 29999), timeout=timeout) as sock:
            sock.recv(4096)                      # 欢迎语
            sock.sendall((cmd + "\n").encode("ascii"))
            reply = sock.recv(4096).decode("utf-8", "replace").strip()
        return reply or None
    except OSError:
        return None


def snapshot(reader, ip):
    q, tcp, velocity = reader.read()
    state = {
        "read_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "robot_ip": ip,
        "joint_rad": [float(x) for x in q],
        "joint_deg": [math.degrees(float(x)) for x in q],
        "tcp_m_rad": [float(x) for x in tcp],
        "tcp_velocity": [float(x) for x in velocity],
        "dashboard": {},
    }
    for cmd, label in DASHBOARD_QUERIES:
        reply = dashboard(cmd, ip)
        if reply is not None:
            state["dashboard"][label] = reply
    return state


def render(state):
    lines = ["=== UR10 状态快照 ===  %s" % state["read_at"], ""]
    dash = state["dashboard"]
    for label in ("机型", "序列号", "PolyScope 版本", "robotmode", "safetystatus",
                  "programState", "已加载程序"):
        if label in dash:
            lines.append("%-16s: %s" % (label, dash[label]))
    lines.append("")
    lines.append("--- 关节角 q_actual ---")
    for name, deg, rad in zip(JOINT_NAMES, state["joint_deg"], state["joint_rad"]):
        lines.append("  %-3s %9.3f°   %12.6f rad" % (name, deg, rad))
    lines.append("")
    lines.append("--- 末端位姿 TCP（控制器当前 TCP 定义下的 tool_vector_actual）---")
    for name, value in zip(POSE_NAMES[:3], state["tcp_m_rad"][:3]):
        lines.append("  %-3s %12.6f m" % (name, value))
    for name, value in zip(POSE_NAMES[3:], state["tcp_m_rad"][3:]):
        lines.append("  %-3s %12.6f rad  (%8.3f°)" % (name, value, math.degrees(value)))
    lines.append("")
    lines.append("--- TCP 速度 ---")
    for name, value in zip(("vx", "vy", "vz"), state["tcp_velocity"][:3]):
        lines.append("  %-3s %12.6f m/s" % (name, value))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ip", default=DEFAULT_IP)
    parser.add_argument("--watch", action="store_true", help="连续显示")
    parser.add_argument("--hz", type=float, default=5.0, help="连续显示的刷新率")
    parser.add_argument("--duration", type=float, default=0.0, help="连续显示时长（秒），0=直到 Ctrl-C")
    parser.add_argument("--json", action="store_true", help="输出 JSON 而不是表格")
    parser.add_argument("--output", default="", help="把 JSON 落盘到该路径")
    args = parser.parse_args()

    if args.hz <= 0:
        parser.error("--hz 必须为正数")

    try:
        reader = RealtimeReader(args.ip)
    except OSError as error:
        print("连不上 UR 30003 (%s): %s" % (args.ip, error), file=sys.stderr)
        return 1

    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    period = 1.0 / args.hz
    records = []
    try:
        while True:
            state = snapshot(reader, args.ip)
            records.append(state)
            if not args.json:
                if args.watch:
                    sys.stdout.write("\033[H\033[J")     # 回到左上角并清屏
                print(render(state), flush=True)
            if not args.watch:
                break
            if deadline is not None and time.monotonic() >= deadline:
                break
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()

    if args.json:
        payload = records[0] if len(records) == 1 else records
        text = json.dumps(payload, ensure_ascii=False, indent=2)
        if args.output:
            path = args.output if os.path.isabs(args.output) else \
                os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), args.output)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w", encoding="utf-8") as stream:
                stream.write(text + "\n")
            print("OUTPUT %s" % path)
        else:
            print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
