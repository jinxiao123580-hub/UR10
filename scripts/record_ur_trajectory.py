#!/usr/bin/env python3
"""Read-only recorder for a UR teach-pendant trajectory.

Records actual joint angles plus actual TCP pose/velocity from UR port 30003.
It never opens port 30002 and never sends URScript, gripper, dashboard, or
force-sensor commands.  The output is evidence for offline review only; it is
not an approved trajectory executor.
"""
import argparse
import csv
import datetime as dt
import json
import os
import socket
import struct
import time

import numpy as np


FRAME_MIN_SIZE = 540
FRAME_MAX_SIZE = 4096
Q_ACTUAL_OFFSET = 252
TCP_FORCE_OFFSET = 396
TCP_POSE_OFFSET = 444
TCP_VELOCITY_OFFSET = 492


class RealtimeReader:
    def __init__(self, host, port=30003):
        self.host = host
        self.port = port
        self.sock = None
        self.buffer = bytearray()
        self.reconnects = 0
        self.last_warning = 0.0
        self.last_frame = None
        self.connect()

    def connect(self):
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=3.0)
        self.sock.settimeout(1.0)
        self.buffer.clear()
        self.reconnects += 1

    def read(self, max_wait_seconds=None):
        """Return one decoded state frame.

        ``max_wait_seconds`` is optional so existing continuous readers keep
        their reconnect behaviour.  Interactive calibration callers can set a
        finite limit instead of appearing to hang when a controller accepts a
        TCP connection but temporarily stops publishing realtime frames.
        """
        deadline = (time.monotonic() + max_wait_seconds
                    if max_wait_seconds is not None else None)
        while True:
            try:
                while len(self.buffer) < 4:
                    chunk = self.sock.recv(65536)
                    if not chunk:
                        raise ConnectionError("UR 30003 closed")
                    self.buffer.extend(chunk)
                size = struct.unpack_from(">I", self.buffer)[0]
                if not FRAME_MIN_SIZE <= size <= FRAME_MAX_SIZE:
                    raise RuntimeError("unexpected UR 30003 frame size %d" % size)
                while len(self.buffer) < size:
                    chunk = self.sock.recv(65536)
                    if not chunk:
                        raise ConnectionError("UR 30003 closed")
                    self.buffer.extend(chunk)
                frame = bytes(self.buffer[:size])
                del self.buffer[:size]
                self.last_frame = frame
                return decode_frame(frame)
            except (socket.timeout, OSError, ConnectionError) as exc:
                if deadline is not None and time.monotonic() >= deadline:
                    raise TimeoutError(
                        "UR %d did not provide a state frame within %.1f s" %
                        (self.port, max_wait_seconds)) from exc
                now = time.monotonic()
                if now - self.last_warning >= 5.0:
                    print("UR %d 暂无状态，正在自动重连：%s" % (self.port, exc), flush=True)
                    self.last_warning = now
                while True:
                    try:
                        self.connect()
                        break
                    except OSError as connect_error:
                        if deadline is not None and time.monotonic() >= deadline:
                            raise TimeoutError(
                                "UR %d reconnect timed out after %.1f s" %
                                (self.port, max_wait_seconds)) from connect_error
                        if time.monotonic() - self.last_warning >= 5.0:
                            print("UR %d 重连失败，继续等待：%s" % (self.port, connect_error), flush=True)
                            self.last_warning = time.monotonic()
                        time.sleep(0.5)

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def read_extended(self):
        """``(q, tcp_pose, tcp_velocity, tcp_force)``; force is None if absent.

        ``tcp_force`` is the controller's own estimate at the TCP, in the TCP
        frame.  It depends on the configured payload, and this repository has
        never independently validated that payload, so treat the absolute value
        as biased.  Magnitude *changes* against a resting baseline are far more
        trustworthy (gravity keeps ``|F|`` roughly constant as the tool rotates).
        """
        q, tcp, velocity = self.read()
        return q, tcp, velocity, decode_tcp_force(self.last_frame)


def decode_frame(frame):
    if len(frame) < TCP_VELOCITY_OFFSET + 48:
        raise RuntimeError("UR 30003 frame too short: %d" % len(frame))
    return (struct.unpack_from(">6d", frame, Q_ACTUAL_OFFSET),
            struct.unpack_from(">6d", frame, TCP_POSE_OFFSET),
            struct.unpack_from(">6d", frame, TCP_VELOCITY_OFFSET))


def decode_tcp_force(frame):
    """``actual_TCP_force`` (Fx Fy Fz Mx My Mz, TCP frame) or None if too short."""
    if frame is None or len(frame) < TCP_FORCE_OFFSET + 48:
        return None
    return struct.unpack_from(">6d", frame, TCP_FORCE_OFFSET)


def write_json(path, document):
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2)
        stream.write("\n")
    os.replace(temporary, path)


def record(args):
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    csv_path = os.path.abspath(args.csv_output or os.path.join(
        root, "outputs", "ur_trajectory", "teach-%s.csv" % stamp))
    json_path = os.path.abspath(args.json_output or os.path.join(
        root, "outputs", "ur_trajectory", "teach-%s.json" % stamp))
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    reader = RealtimeReader(args.host)
    started_wall = time.time()
    started_mono = time.monotonic()
    count = 0
    first = last = None
    print("只读轨迹记录已开始：请在示教器运行轨迹；Ctrl-C 停止并保存。")
    print("CSV:", csv_path)
    print("JSON:", json_path)
    try:
        with open(csv_path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["elapsed_s", "wall_time_s",
                             "q1_rad", "q2_rad", "q3_rad", "q4_rad", "q5_rad", "q6_rad",
                             "tcp_x_m", "tcp_y_m", "tcp_z_m", "tcp_rx_rad", "tcp_ry_rad", "tcp_rz_rad",
                             "tcp_vx_m_s", "tcp_vy_m_s", "tcp_vz_m_s",
                             "tcp_wx_rad_s", "tcp_wy_rad_s", "tcp_wz_rad_s"])
            while args.duration <= 0 or time.monotonic() - started_mono < args.duration:
                q, tcp, velocity = reader.read()
                elapsed = time.monotonic() - started_mono
                row = (elapsed, time.time(), *q, *tcp, *velocity)
                writer.writerow(["%.9f" % value for value in row])
                count += 1
                first = first or {"q_rad": list(q), "tcp_pose": list(tcp)}
                last = {"q_rad": list(q), "tcp_pose": list(tcp)}
                if count % 125 == 0:
                    stream.flush()
    except KeyboardInterrupt:
        print("停止请求已收到，正在保存。")
    finally:
        reader.close()
    elapsed = time.monotonic() - started_mono
    document = {
        "schema_version": 1,
        "recorded_at": dt.datetime.fromtimestamp(started_wall, dt.timezone.utc).isoformat(),
        "source": "UR 30003 actual state, read-only",
        "host": args.host,
        "sample_count": count,
        "duration_s": elapsed,
        "average_rate_hz": count / elapsed if elapsed else 0.0,
        "connection_attempts": reader.reconnects,
        "first": first,
        "last": last,
        "csv": csv_path,
        "safety_note": "No robot/gripper command was sent. Recording is not trajectory validation or execution authorization.",
    }
    write_json(json_path, document)
    print("已保存：%d 帧，%.2fs，%.1fHz" %
          (count, elapsed, document["average_rate_hz"]))
    print("JSON:", json_path)


def self_test():
    frame = bytearray(1108)
    struct.pack_into(">I", frame, 0, len(frame))
    expected_q = np.arange(6, dtype=float) + 0.1
    expected_tcp = np.arange(6, dtype=float) + 10.1
    expected_velocity = np.arange(6, dtype=float) + 20.1
    struct.pack_into(">6d", frame, Q_ACTUAL_OFFSET, *expected_q)
    struct.pack_into(">6d", frame, TCP_POSE_OFFSET, *expected_tcp)
    struct.pack_into(">6d", frame, TCP_VELOCITY_OFFSET, *expected_velocity)
    actual = decode_frame(frame)
    assert all(np.allclose(got, want) for got, want in zip(
        actual, (expected_q, expected_tcp, expected_velocity)))
    print("self-test passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="记录秒数；0 表示直到 Ctrl-C")
    parser.add_argument("--csv-output")
    parser.add_argument("--json-output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.duration < 0:
        parser.error("duration 必须 ≥0")
    record(args)


if __name__ == "__main__":
    main()
