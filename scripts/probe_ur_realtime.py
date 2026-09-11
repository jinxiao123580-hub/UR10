#!/usr/bin/env python3
"""Measure framed state traffic on UR client ports without sending commands."""
import argparse
from collections import Counter
import socket
import struct
import time


def probe(host, port, duration, timeout):
    started = time.monotonic()
    frames = []
    buffer = bytearray()
    error = None
    try:
        sock = socket.create_connection((host, port), timeout=timeout)
        sock.settimeout(min(timeout, 0.5))
    except OSError as exc:
        return {"port": port, "connect": False, "error": str(exc)}
    connected = time.monotonic()
    deadline = connected + duration
    try:
        while time.monotonic() < deadline:
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                continue
            if not chunk:
                error = "peer closed connection"
                break
            buffer.extend(chunk)
            while len(buffer) >= 4:
                size = struct.unpack_from(">I", buffer)[0]
                if size < 5 or size > 1_000_000:
                    error = "invalid frame size %d" % size
                    buffer.clear()
                    break
                if len(buffer) < size:
                    break
                frames.append((time.monotonic(), size))
                del buffer[:size]
            if error:
                break
    finally:
        sock.close()
    ended = time.monotonic()
    times = [row[0] for row in frames]
    gaps = [b - a for a, b in zip(times, times[1:])]
    sample_span = (times[-1] - times[0]) if len(times) > 1 else 0.0
    return {
        "port": port,
        "connect": True,
        "elapsed": ended - connected,
        "frames": len(frames),
        "hz": ((len(times) - 1) / sample_span) if sample_span > 0 else 0.0,
        "sizes": Counter(size for _, size in frames),
        "max_gap_ms": max(gaps, default=0.0) * 1000.0,
        "buffered_bytes": len(buffer),
        "error": error,
        "connect_ms": (connected - started) * 1000.0,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--ports", nargs="+", type=int,
                        default=[30003, 30011, 30012, 30013])
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()
    failed = False
    for port in args.ports:
        result = probe(args.host, port, args.duration, args.timeout)
        if not result["connect"]:
            failed = True
            print("%d: CONNECT FAIL: %s" % (port, result["error"]))
            continue
        sizes = ", ".join("%dB x%d" % item for item in sorted(result["sizes"].items()))
        print("%d: frames=%d, %.2fHz, max_gap=%.2fms, sizes=[%s], "
              "tail=%dB, connect=%.1fms%s" % (
                  port, result["frames"], result["hz"], result["max_gap_ms"],
                  sizes, result["buffered_bytes"], result["connect_ms"],
                  ", error=" + result["error"] if result["error"] else ""))
        if result["frames"] < 2 or result["error"]:
            failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
