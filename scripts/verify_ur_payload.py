#!/usr/bin/env python3
"""Read or set-and-verify the UR payload without commanding robot motion.

The read-only path is machine-validated. The setting/automatic-restore path is
guarded but remains unvalidated; retain the JSON backup for manual restoration.
"""
import argparse
import datetime
import json
import os
import socket
import struct
import time


FRAME_SIZE = 1220
PAYLOAD_OFFSET = 1140
COG_OFFSET = 1148


def read_frame(sock, buffer):
    while len(buffer) < 4:
        buffer.extend(sock.recv(65536))
    size = struct.unpack_from(">I", buffer)[0]
    if size != FRAME_SIZE:
        raise RuntimeError("expected %d-byte frame, got %d" % (FRAME_SIZE, size))
    while len(buffer) < size:
        buffer.extend(sock.recv(65536))
    frame = bytes(buffer[:size])
    del buffer[:size]
    return frame


def sample_payload(host, count=25):
    values = []
    with socket.create_connection((host, 30003), timeout=3.0) as sock:
        sock.settimeout(2.0)
        buffer = bytearray()
        for _ in range(count):
            frame = read_frame(sock, buffer)
            mass = struct.unpack_from(">d", frame, PAYLOAD_OFFSET)[0]
            cog = struct.unpack_from(">3d", frame, COG_OFFSET)
            values.append((mass, *cog))
    return values


def send_payload(host, mass, cog):
    script = ("def verify_payload():\n"
              "  set_payload(%.9f, [%.9f, %.9f, %.9f])\n"
              "  sync()\n"
              "end\n"
              "verify_payload()\n") % (mass, cog[0], cog[1], cog[2])
    with socket.create_connection((host, 30002), timeout=3.0) as sock:
        sock.sendall(script.encode("ascii"))


def stable_value(values, tolerance=1e-9):
    reference = values[0]
    stable = all(max(abs(a - b) for a, b in zip(reference, row)) <= tolerance
                 for row in values[1:])
    return reference, stable


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--set", dest="mass", type=float,
                        help="set mass in kg, then verify it from 30003")
    parser.add_argument("--cog", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--yes", action="store_true",
                        help="confirm a non-moving payload configuration command")
    parser.add_argument("--restore-from", metavar="JSON",
                        help="restore mass and CoG from a backup made by this tool")
    args = parser.parse_args()

    if args.restore_from:
        if args.mass is not None or args.cog is not None:
            parser.error("--restore-from cannot be combined with --set/--cog")
        with open(args.restore_from) as stream:
            backup = json.load(stream)
        args.mass = float(backup["mass_kg"])
        args.cog = [float(value) for value in backup["cog_m"]]

    before, stable = stable_value(sample_payload(args.host))
    print("before: mass=%.6f kg cog=[%.6f, %.6f, %.6f] m stable=%s" %
          (*before, stable))
    if args.mass is None:
        if args.cog is not None or args.yes:
            parser.error("--cog/--yes require --set")
        return 0
    if args.cog is None:
        parser.error("--set requires --cog X Y Z")
    if not 0.0 <= args.mass <= 10.0:
        parser.error("mass must be between 0 and 10 kg for this UR10")
    if max(abs(value) for value in args.cog) > 0.5:
        parser.error("each CoG component must be within +/-0.5 m")
    if not args.yes:
        parser.error("setting is disabled without --yes")

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    backup_dir = os.path.join(root, "outputs", "payload")
    os.makedirs(backup_dir, exist_ok=True)
    backup_path = os.path.join(
        backup_dir, "payload-before-%s.json" %
        datetime.datetime.now().strftime("%Y%m%d-%H%M%S"))
    with open(backup_path, "w") as stream:
        json.dump({"mass_kg": before[0], "cog_m": list(before[1:])},
                  stream, indent=2)
        stream.write("\n")
    print("backup:", backup_path)
    print("restore: python3 scripts/verify_ur_payload.py --restore-from %s --yes" %
          backup_path)

    try:
        send_payload(args.host, args.mass, args.cog)
        time.sleep(0.3)
        after, stable = stable_value(sample_payload(args.host))
    except Exception:
        print("写入/回读异常，尝试恢复写入前参数...")
        send_payload(args.host, before[0], before[1:])
        raise
    print("after:  mass=%.6f kg cog=[%.6f, %.6f, %.6f] m stable=%s" %
          (*after, stable))
    expected = (args.mass, *args.cog)
    error = max(abs(a - b) for a, b in zip(after, expected))
    if not stable or error > 1e-6:
        print("回读不匹配，恢复写入前参数...")
        send_payload(args.host, before[0], before[1:])
        time.sleep(0.3)
        restored, restored_stable = stable_value(sample_payload(args.host))
        print("restored: mass=%.6f kg cog=[%.6f, %.6f, %.6f] m stable=%s" %
              (*restored, restored_stable))
        raise RuntimeError("payload readback mismatch; max error %.9g" % error)
    print("PASS: 30003 payload readback matches the requested values")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
