#!/usr/bin/env python3
"""Read-only UR Dashboard safety-state recorder."""
import argparse
import csv
import datetime
import os
import socket
import time


def recv_line(sock, buffer):
    while b"\n" not in buffer:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("Dashboard closed the connection")
        buffer += chunk
    line, buffer = buffer.split(b"\n", 1)
    return line.decode("utf-8", "replace").strip(), buffer


def query(sock, buffer, command):
    sock.sendall((command + "\n").encode("ascii"))
    return recv_line(sock, buffer)


def normalized_value(response):
    return response.split(":", 1)[-1].strip().split()[0].upper()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--period", type=float, default=0.05)
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.duration <= 0 or args.period < 0.02:
        parser.error("duration must be positive and period must be >= 0.02 s")

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    output = args.output or os.path.join(
        root, "outputs", "safety", "dashboard-%s.csv" %
        datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)

    rows = []
    deadline = time.monotonic() + args.duration
    with socket.create_connection((args.host, 29999), timeout=3.0) as sock:
        sock.settimeout(2.0)
        _, buffer = recv_line(sock, b"")
        while time.monotonic() < deadline:
            started = time.monotonic()
            robot, buffer = query(sock, buffer, "robotmode")
            safety, buffer = query(sock, buffer, "safetystatus")
            program, buffer = query(sock, buffer, "programState")
            rows.append((time.time(), robot, safety, program))
            delay = args.period - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)

    with open(output, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["unix_time_s", "robotmode", "safetystatus", "program_state"])
        writer.writerows(rows)

    bad = [row for row in rows if normalized_value(row[1]) != "RUNNING" or
           normalized_value(row[2]) != "NORMAL"]
    elapsed = rows[-1][0] - rows[0][0] if len(rows) > 1 else 0.0
    rate = (len(rows) - 1) / elapsed if elapsed > 0 else 0.0
    print("samples=%d rate=%.2fHz abnormal=%d" % (len(rows), rate, len(bad)))
    print("last:", rows[-1][1], "|", rows[-1][2], "|", rows[-1][3])
    print("CSV:", output)
    if bad:
        raise SystemExit("检测到非 RUNNING/NORMAL 状态")


if __name__ == "__main__":
    main()
