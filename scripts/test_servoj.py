#!/usr/bin/env python3
"""Conservative CB3 servoj acceptance test with 125Hz response measurement."""
import argparse
import socket
import struct
import threading
import time

import numpy as np


def build_script(joint, amplitude, period, cycles, control_period, lookahead, gain):
    duration = period * cycles
    target = ["q0[%d]" % index for index in range(6)]
    target[joint] = ("q0[%d] + %.12f*sin(2.0*3.141592653589793*elapsed/%.9f)" %
                     (joint, amplitude, period))
    return """def servoj_acceptance_test():
  q0 = get_actual_joint_positions()
  elapsed = 0.0
  while (elapsed < %.9f):
    q = [%s]
    servoj(q, t=%.9f, lookahead_time=%.6f, gain=%.3f)
    elapsed = elapsed + %.9f
  end
  servoj(q0, t=%.9f, lookahead_time=%.6f, gain=%.3f)
  stopj(1.0)
end
servoj_acceptance_test()
""" % (duration, ", ".join(target), control_period, lookahead, gain,
         control_period, control_period, lookahead, gain)


def monitor_state(host, stop, rows, error):
    try:
        sock = socket.create_connection((host, 30003), timeout=2.0)
        sock.settimeout(0.5)
        buffer = bytearray()
        while not stop.is_set():
            try:
                data = sock.recv(65536)
            except socket.timeout:
                continue
            if not data:
                raise ConnectionError("30003 peer closed")
            buffer.extend(data)
            while len(buffer) >= 4:
                size = struct.unpack_from(">I", buffer)[0]
                if size < 540 or size > 10000:
                    raise ValueError("unexpected frame size %d" % size)
                if len(buffer) < size:
                    break
                frame = bytes(buffer[:size])
                del buffer[:size]
                q = struct.unpack_from(">6d", frame, 252)
                qd = struct.unpack_from(">6d", frame, 300)
                rows.append((time.monotonic(), q, qd, size))
    except Exception as exc:
        error.append(str(exc))
    finally:
        try:
            sock.close()
        except UnboundLocalError:
            pass


def send_script(host, script):
    with socket.create_connection((host, 30002), timeout=2.0) as sock:
        sock.sendall(script.encode("ascii"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--joint", type=int, default=5, choices=range(6))
    parser.add_argument("--amplitude", type=float, default=0.005,
                        help="joint amplitude in rad; maximum allowed is 0.01")
    parser.add_argument("--period", type=float, default=2.0)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--control-period", type=float, default=0.008)
    parser.add_argument("--lookahead", type=float, default=0.1)
    parser.add_argument("--gain", type=float, default=100.0)
    args = parser.parse_args()
    if not 0 < args.amplitude <= 0.01:
        parser.error("--amplitude must be in (0, 0.01] rad")
    if not 0.004 <= args.control_period <= 0.02:
        parser.error("--control-period must be in [0.004, 0.02] s")
    script = build_script(args.joint, args.amplitude, args.period, args.cycles,
                          args.control_period, args.lookahead, args.gain)
    print(script)
    if not args.execute:
        print("DRY RUN: 未发送。加 --execute 后仍需输入 YES。")
        return 0
    answer = input("确认人员在机械臂旁、手放急停、周围无障碍？输入 YES: ")
    if answer != "YES":
        print("已取消")
        return 1
    stop = threading.Event()
    rows, errors = [], []
    thread = threading.Thread(target=monitor_state,
                              args=(args.host, stop, rows, errors), daemon=True)
    thread.start()
    time.sleep(1.0)
    send_time = time.monotonic()
    send_script(args.host, script)
    time.sleep(args.period * args.cycles + 2.0)
    stop.set()
    thread.join(timeout=2.0)
    if errors:
        raise RuntimeError("30003 monitor failed: " + "; ".join(errors))
    active = [row for row in rows if row[0] >= send_time]
    if len(active) < 100:
        raise RuntimeError("insufficient 30003 frames: %d" % len(active))
    times = np.array([row[0] for row in active])
    q = np.array([row[1] for row in active])[:, args.joint]
    qd = np.array([row[2] for row in active])[:, args.joint]
    rate = (len(times) - 1) / (times[-1] - times[0])
    movement = float(q.max() - q.min())
    max_speed = float(np.max(np.abs(qd)))
    max_step = float(np.max(np.abs(np.diff(q))))
    print("frames=%d rate=%.2fHz joint=%d range=%.6frad "
          "max_speed=%.6frad/s max_step=%.6frad" %
          (len(active), rate, args.joint, movement, max_speed, max_step))
    expected = 2.0 * args.amplitude
    if movement < expected * 0.4:
        raise RuntimeError("servoj response too small; command may not have executed")
    if movement > expected * 1.8:
        raise RuntimeError("servoj response exceeded safety bound")
    print("PASS: servoj 被接受且产生受限响应；仍需检查轨迹平滑度后用于闭环。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
