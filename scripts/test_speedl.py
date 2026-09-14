#!/usr/bin/env python3
"""Conservative native TCP velocity test; dry-run unless --execute is set."""
import argparse
import csv
import datetime
import os
import socket
import struct
import threading
import time
import numpy as np


def build_script(axis, speed, duration, acceleration):
    twist = [0.0] * 6
    twist[axis] = speed
    return """def tcp_speed_test():
  speedl([%s], %.6f, %.6f)
  stopl(%.6f)
end
tcp_speed_test()
""" % (", ".join("%.9f" % value for value in twist),
       acceleration, duration, acceleration)


def monitor(host, stop, rows, errors):
    try:
        with socket.create_connection((host, 30003), timeout=2.0) as sock:
            sock.settimeout(0.5)
            buffer = bytearray()
            while not stop.is_set():
                try:
                    chunk = sock.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    raise ConnectionError("30003 closed")
                buffer.extend(chunk)
                while len(buffer) >= 4:
                    size = struct.unpack_from(">I", buffer, 0)[0]
                    if size < 540 or size > 4096:
                        raise ValueError("unexpected frame size %d" % size)
                    if len(buffer) < size:
                        break
                    frame = bytes(buffer[:size]); del buffer[:size]
                    rows.append((time.monotonic(),
                                 struct.unpack_from(">6d", frame, 444),
                                 struct.unpack_from(">6d", frame, 492)))
    except Exception as exc:
        errors.append(str(exc))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.3")
    ap.add_argument("--axis", type=int, choices=range(6), default=0,
                    help="0..2=base XYZ, 3..5=angular XYZ")
    ap.add_argument("--speed", type=float, default=0.005)
    ap.add_argument("--duration", type=float, default=0.5)
    ap.add_argument("--acceleration", type=float, default=0.05)
    ap.add_argument("--execute", action="store_true")
    args = ap.parse_args()
    limit = 0.01 if args.axis < 3 else 0.05
    if not 0 < abs(args.speed) <= limit:
        ap.error("speed magnitude must be in (0, %.3f]" % limit)
    if not 0 < args.duration <= 1.0:
        ap.error("duration must be in (0, 1.0] s")
    script = build_script(args.axis, args.speed, args.duration, args.acceleration)
    print(script)
    print("位移上界: %.6f %s" % (abs(args.speed * args.duration),
                                    "m" if args.axis < 3 else "rad"))
    if not args.execute:
        print("DRY RUN: 未发送。")
        return
    if input("确认周围无障碍、人在机械臂旁且手放急停？输入 YES: ") != "YES":
        print("已取消")
        return
    stop = threading.Event(); rows = []; errors = []
    thread = threading.Thread(target=monitor, args=(args.host, stop, rows, errors), daemon=True)
    thread.start()
    deadline = time.monotonic() + 2.0
    while len(rows) < 20 and not errors and time.monotonic() < deadline:
        time.sleep(0.05)
    if errors or len(rows) < 20:
        stop.set(); thread.join(1.0)
        raise RuntimeError("30003 运动前门禁失败: %s, frames=%d" % (errors, len(rows)))
    before = np.mean([row[1] for row in rows[-20:]], axis=0)
    sent = time.monotonic()
    with socket.create_connection((args.host, 30002), timeout=2.0) as sock:
        sock.sendall(script.encode("ascii"))
    time.sleep(args.duration + 1.0); stop.set(); thread.join(2.0)
    if errors:
        raise RuntimeError("30003 监测失败: " + "; ".join(errors))
    active = [row for row in rows if row[0] >= sent]
    out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "outputs", "speedl")
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    csv_path = os.path.join(out_dir, "speedl-%s.csv" % stamp)
    with open(csv_path, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "x", "y", "z", "rx", "ry", "rz",
                         "vx", "vy", "vz", "wx", "wy", "wz"])
        for timestamp, pose, velocity in rows:
            writer.writerow([timestamp - sent, *pose, *velocity])
    print("CSV: %s" % csv_path)
    after = np.mean([row[1] for row in active[-20:]], axis=0)
    peak = max(abs(row[2][args.axis]) for row in active)
    delta = after[args.axis] - before[args.axis]
    times = np.asarray([row[0] for row in active])
    positions = np.asarray([row[1][args.axis] for row in active])
    window = min(11, len(active) if len(active) % 2 else len(active) - 1)
    smooth = np.convolve(positions, np.ones(window) / window, mode="valid")
    smooth_times = np.convolve(times, np.ones(window) / window, mode="valid")
    smooth_speed = np.gradient(smooth, smooth_times)
    speed_p95 = float(np.percentile(np.abs(smooth_speed), 95))
    print("frames=%d delta_axis=%.6f raw_peak_speed=%.6f filtered_p95=%.6f" %
          (len(active), delta, peak, speed_p95))
    expected = args.speed * args.duration
    tolerance = max(abs(expected) * 0.6, 0.001 if args.axis < 3 else 0.01)
    if abs(delta - expected) > tolerance:
        raise RuntimeError("实际位移与指令不符: expected=%.6f actual=%.6f" % (expected, delta))
    if speed_p95 > abs(args.speed) * 1.2 + 1e-4:
        raise RuntimeError("滤波速度超限: command=%.6f p95=%.6f" %
                           (abs(args.speed), speed_p95))
    print("PASS: speedl 实际响应在门限内")


if __name__ == "__main__":
    main()
