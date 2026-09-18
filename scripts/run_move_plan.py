#!/usr/bin/env python3
"""Replay an offline-gated Cartesian move plan with a live reactive watchdog.

This is the real-motion layer.  It refuses to move anything unless a matching
``check_move_plan.py`` report exists and passed, and then it replays **that
report's own segment list, in that report's own visit order**.  That fidelity is
the whole point: reachability here is a property of the *chain*, not of the
individual poses - one slot in this plan passes when evaluated on its own and
fails when approached straight after its neighbour - so the gate verified a
specific ordered trajectory and the executor must not improvise a different one.
Per-slot clearance and roll corrections made by the gate are replayed for the
same reason.

What gets executed is deliberately boring: ``lift -> reorient -> translate ->
descend`` with every waypoint at or above a clearance height, using
controller-native ``movel`` (the controller does its own IK).  That structure is
the stand-in for a collision model, because this repository has none.

The watchdog is the part that makes it more than "trust the plan":

* **straight-line invariant** - ``movel`` is a straight Cartesian line, so the
  controller-reported TCP must stay within a few millimetres of the segment.
  Any excursion means something external moved the arm, or the controller is
  off-path.  For a pure reorientation the segment is a single point, so the
  invariant becomes "do not move at all".
* **force transient** - the controller's own ``actual_TCP_force`` magnitude
  should stay near its resting value (gravity keeps ``|F|`` roughly constant
  while the tool rotates).  A contact adds a vector and shows up as a magnitude
  change.  Absolute values are biased by the never-independently-validated
  payload, which is exactly why the check is on the change.

On violation the watchdog sends ``stopj`` on its own 30002 connection.

Honest limits, stated up front: this is **reactive**, not avoidance.  It cannot
see an obstacle on the path and only reacts after the controller reports
something.  The real backstops remain the teach-pendant safety configuration
(force limits) and a human next to the emergency stop.

Never opens 30002 unless ``--execute`` is given.
"""
import argparse
import datetime as dt
import json
import os
import socket
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from check_move_plan import (interpolate, pose_from_tcp_target, read_state,
                             tcp_values)  # noqa: E402
from record_ur_trajectory import RealtimeReader  # noqa: E402

DEVICE_HOST = "192.168.1.3"
SEGMENT_TIMEOUT_FACTOR = 6.0
SETTLE_SPEED = 0.003
SETTLE_ANGULAR_SPEED = 0.02      # rad/s
POSITION_TOLERANCE_M = 0.0015
ROTATION_TOLERANCE_RAD = 0.006   # 0.34 deg
STALL_TIMEOUT_S = 3.0
START_MATCH_TOLERANCE_M = 0.015
GATE_DRIFT_TOLERANCE_RAD = 0.02


def query_dashboard(command, timeout=3.0):
    with socket.create_connection((DEVICE_HOST, 29999), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.recv(4096)
        sock.sendall((command + "\n").encode("ascii"))
        return sock.recv(4096).decode("utf-8", "replace").strip()


def send_urscript(program, timeout=3.0):
    with socket.create_connection((DEVICE_HOST, 30002), timeout=timeout) as sock:
        sock.sendall(program.encode("ascii"))


def collect_sample(root, dataset, attempts=3):
    """Collect one calibration sample with the arm parked at the current pose.

    The collector re-checks stationarity and full checkerboard detection, so a
    rejection is a real rejection.  Retries only cover transient failures; a pose
    that simply cannot see the board fails every attempt and is reported.
    """
    script = os.path.join(root, "scripts", "collect_handeye_sample.py")
    history = []
    for attempt in range(1, attempts + 1):
        try:
            completed = subprocess.run(
                [sys.executable, script, "--save-cloud", "--dataset", dataset],
                cwd=root, capture_output=True, text=True, timeout=300)
            code = completed.returncode
            tail = [line for line in completed.stdout.strip().splitlines() if line][-4:]
            stderr = completed.stderr.strip()[-400:]
        except subprocess.TimeoutExpired:
            code, tail, stderr = -1, [], "collector timeout"
        history.append({"attempt": attempt, "exit_code": code,
                        "output_tail": tail, "stderr_tail": stderr})
        if code == 0:
            return True, history
    return False, history


class Watchdog(threading.Thread):
    """Reactive guard: straight-line deviation and TCP force transient."""

    def __init__(self, max_deviation_m, max_force_delta_n):
        super().__init__(daemon=True)
        self.reader = RealtimeReader(DEVICE_HOST)
        self.max_deviation_m = max_deviation_m
        self.max_force_delta_n = max_force_delta_n
        self.stop_event = threading.Event()
        self.segment = None
        self.force_baseline = None
        self.tripped = None
        self.trace = []
        self.max_deviation_seen = 0.0
        self.max_force_delta_seen = 0.0
        self.samples = 0
        self.error = None
        self.j6_start_deg = None
        self.j6_last_deg = None
        self.j6_prev_deg = None
        self.j6_travel_deg = 0.0
        self.j6_span_deg = -1e9
        self.j6_span_min_deg = 1e9

    def set_segment(self, start_tcp, end_tcp):
        self.segment = (np.asarray(start_tcp[:3], dtype=float),
                        np.asarray(end_tcp[:3], dtype=float))
        self.max_deviation_seen = 0.0
        self.max_force_delta_seen = 0.0
        self.trace = []
        self.samples = 0

    def measure_force_baseline(self, count=40):
        magnitudes = []
        for _ in range(count):
            _q, _tcp, _v, force = self.reader.read_extended()
            if force is not None:
                magnitudes.append(float(np.linalg.norm(force[:3])))
        if magnitudes:
            self.force_baseline = float(np.median(magnitudes))
        return self.force_baseline

    def deviation(self, point):
        p0, p1 = self.segment
        delta = p1 - p0
        length = float(np.linalg.norm(delta))
        if length < 1e-9:
            return float(np.linalg.norm(point - p0))
        along = float(np.clip(np.dot(point - p0, delta) / (length * length),
                              0.0, 1.0))
        return float(np.linalg.norm(point - (p0 + along * delta)))

    def run(self):
        try:
            while not self.stop_event.is_set():
                q, tcp, velocity, force = self.reader.read_extended()
                if self.segment is None:
                    continue
                if self.j6_start_deg is None:
                    self.j6_start_deg = float(np.degrees(q[5]))
                self.j6_last_deg = float(np.degrees(q[5]))
                self.j6_travel_deg += abs(self.j6_last_deg - self.j6_prev_deg) \
                    if self.j6_prev_deg is not None else 0.0
                self.j6_prev_deg = self.j6_last_deg
                self.j6_span_deg = max(self.j6_span_deg, self.j6_last_deg)
                self.j6_span_min_deg = min(self.j6_span_min_deg, self.j6_last_deg)
                self.samples += 1
                point = np.asarray(tcp[:3], dtype=float)
                deviation = self.deviation(point)
                self.max_deviation_seen = max(self.max_deviation_seen, deviation)
                force_delta = None
                if force is not None and self.force_baseline is not None:
                    force_delta = abs(float(np.linalg.norm(force[:3])) -
                                      self.force_baseline)
                    self.max_force_delta_seen = max(self.max_force_delta_seen,
                                                    force_delta)
                if self.samples % 2 == 0:
                    self.trace.append({
                        "tcp": [round(x, 6) for x in tcp[:3]],
                        "speed": round(float(np.linalg.norm(velocity[:3])), 5),
                        "deviation_mm": round(deviation * 1000.0, 4),
                        "force_delta_n": (None if force_delta is None
                                          else round(force_delta, 4)),
                    })
                if deviation > self.max_deviation_m:
                    self.trip("path_deviation", deviation * 1000.0)
                elif (force_delta is not None and
                      force_delta > self.max_force_delta_n):
                    self.trip("force_delta", force_delta)
        except Exception as exc:  # keep the guard thread from dying silently
            self.error = repr(exc)

    def trip(self, reason, value):
        if self.tripped:
            return
        self.tripped = {"reason": reason, "value": value,
                        "at": dt.datetime.now().astimezone().isoformat()}
        print("*** 看门狗触发：%s = %.4f —— 发送 stopj ***" % (reason, value),
              flush=True)
        try:
            send_urscript("stopj(5.0)\n")
        except OSError as exc:
            print("看门狗 stopj 发送失败：%r" % exc, flush=True)

    def stop(self):
        self.stop_event.set()
        self.join(timeout=2.0)
        self.reader.close()


def pose_error(actual, target):
    """(position error in metres, rotation error in radians)."""
    position = float(np.linalg.norm(np.asarray(actual[:3]) -
                                    np.asarray(target[:3])))
    rotation_a, _ = cv2.Rodrigues(np.asarray(actual[3:], dtype=float))
    rotation_b, _ = cv2.Rodrigues(np.asarray(target[3:], dtype=float))
    cosine = np.clip((np.trace(rotation_a.T @ rotation_b) - 1.0) / 2.0, -1.0, 1.0)
    return position, float(np.arccos(cosine))


def wait_for_arrival(target, speed, watchdog):
    """Wait until the arm is at ``target`` in BOTH position and orientation.

    Checking position alone is not enough, and that mistake is worth spelling out:
    a pure reorientation has a TCP *linear* speed of about zero, so a
    position-only test declares the segment finished the moment it starts, the
    next ``movel`` is sent while the rotation is still running, and the two
    motions collide.  That is exactly how slot 17's 1.7 rad reorientation ended
    up crammed into its translation segment.
    """
    reader = RealtimeReader(DEVICE_HOST)
    try:
        _q, tcp, _v = reader.read()
        distance, rotation = pose_error(tcp, target)
        deadline = time.monotonic() + max(
            10.0, distance / max(speed, 1e-6) * SEGMENT_TIMEOUT_FACTOR +
            rotation / 0.3 * 3.0 + 5.0)
        final = tcp
        last_progress = time.monotonic()
        best = distance + rotation
        while time.monotonic() < deadline:
            _q, tcp, velocity = reader.read()
            final = tcp
            if watchdog.tripped:
                return final, False, "watchdog"
            position, rotation = pose_error(tcp, target)
            if position <= POSITION_TOLERANCE_M and rotation <= ROTATION_TOLERANCE_RAD \
                    and float(np.linalg.norm(velocity[:3])) <= SETTLE_SPEED \
                    and float(np.linalg.norm(velocity[3:])) <= SETTLE_ANGULAR_SPEED:
                return final, True, "arrived"
            combined = position + rotation
            if combined < best - 1e-4:
                best = combined
                last_progress = time.monotonic()
            elif time.monotonic() - last_progress > STALL_TIMEOUT_S:
                return final, False, "stalled"
        return final, False, "timeout"
    finally:
        reader.close()


def build_segments(gate, slots=None, first_slots=None):
    segments = [value for value in gate["segments"]
                if value.get("segment") != "slot_summary"
                and "start_tcp_m_rad" in value and "end_tcp_m_rad" in value]
    if first_slots:
        # A prefix of the visit order.  Chunking must take prefixes, not an
        # arbitrary subset: the gate verified ONE ordered chain, and a subset
        # starting mid-chain begins from a pose the chain never visits first.
        wanted = set(gate.get("visit_order") or [])
        keep = [slot for slot in (gate.get("visit_order") or [])
                if slot in {value["slot"] for value in segments}]
        chosen = set(keep[:first_slots])
        segments = [value for value in segments if value["slot"] in chosen]
        del wanted
    elif slots:
        wanted = {int(x) for x in slots.split(",") if x.strip()}
        segments = [value for value in segments if value["slot"] in wanted]
    return segments


def split_segment(start_tcp, end_tcp, max_rot_deg, max_trans_m):
    """Sub-waypoints with bounded rotation and translation per step.

    A single ``movel`` that reorients the tool by 82 deg was stopped 44 deg short
    because the camera cable resisted it.  Small steps keep each command modest,
    so a snag is confined to one short step instead of eating a whole segment,
    and the stall detector reacts sooner.
    """
    start = pose_from_tcp_target(start_tcp)
    end = pose_from_tcp_target(end_tcp)
    _, rotation = pose_error(start_tcp, end_tcp)
    translation = float(np.linalg.norm(np.asarray(end_tcp[:3]) -
                                       np.asarray(start_tcp[:3])))
    steps = 1
    if max_rot_deg > 0:
        steps = max(steps, int(np.ceil(np.degrees(rotation) / max_rot_deg)))
    if max_trans_m > 0:
        steps = max(steps, int(np.ceil(translation / max_trans_m)))
    if steps <= 1:
        return [list(end_tcp)]
    return [tcp_values(pose) for pose in interpolate(start, end, steps)][1:]


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gate", default="outputs/vision/scripted-move-offline-gate-26.json")
    parser.add_argument("--slots", default=None,
                        help="comma-separated slot numbers; only safe when they form "
                             "the start of the gate's visit order")
    parser.add_argument("--first-slots", type=int, default=None,
                        help="run only the first N slots of the gate's visit order "
                             "(a proper prefix, so the chain stays faithful)")
    parser.add_argument("--speed", type=float, default=0.05, help="movel speed (m/s)")
    parser.add_argument("--acceleration", type=float, default=0.10)
    parser.add_argument("--relative-dz", type=float, default=None,
                        help="L2 test mode: one pure +Z movel of this many metres "
                             "instead of the plan; ignores --gate")
    parser.add_argument("--max-deviation-mm", type=float, default=8.0,
                        help="watchdog straight-line invariant; the nominal-URDF vs "
                             "factory FK disagreement is ~2.5 mm, so leave headroom")
    parser.add_argument("--max-force-delta-n", type=float, default=25.0,
                        help="watchdog on the CHANGE of |actual_TCP_force| against the "
                             "resting baseline; the absolute value is payload-biased. "
                             "NOTE: this could NOT see the observed cable snag "
                             "(measured 1.69 N), so it is a coarse guard only - the "
                             "stall detector is the real defence")
    parser.add_argument("--max-step-rot-deg", type=float, default=20.0,
                        help="split a segment so no single movel changes the tool "
                             "orientation by more than this; one 82 deg reorientation "
                             "was stopped 44 deg short by the camera cable")
    parser.add_argument("--max-step-trans-m", type=float, default=0.12,
                        help="split a segment so no single movel translates more than "
                             "this")
    parser.add_argument("--collect", action="store_true",
                        help="after each slot's descent, run the checkerboard sample "
                             "collector (image + full robot frame log + raw cloud)")
    parser.add_argument("--collect-dataset",
                        default="outputs/handeye/eye-in-hand-20260918.json")
    parser.add_argument("--collect-attempts", type=int, default=3)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--out-dir", default="outputs/vision/motion")
    args = parser.parse_args()

    if not (0.005 <= args.speed <= 0.08):
        parser.error("speed must be within 0.005..0.08 m/s")
    if not (0.01 <= args.acceleration <= 0.50):
        parser.error("acceleration must be within 0.01..0.50 m/s^2")

    q_now, tcp_now = read_state(DEVICE_HOST)
    print("当前 TCP: %s" % [round(x, 4) for x in tcp_now[:3]])

    gate_document = None
    if args.relative_dz is not None:
        if not 0.0 < args.relative_dz <= 0.10:
            parser.error("--relative-dz must be within (0, 0.10] m")
        target = list(tcp_now)
        target[2] += args.relative_dz
        segments = [{"slot": 0, "segment": "relative_lift",
                     "start_tcp_m_rad": list(tcp_now), "end_tcp_m_rad": target}]
    else:
        gate_path = args.gate if os.path.isabs(args.gate) \
            else os.path.join(root, args.gate)
        with open(gate_path, encoding="utf-8") as stream:
            gate_document = json.load(stream)
        if not gate_document.get("passed"):
            print("拒绝执行：离线门禁未通过（%s）" % os.path.relpath(gate_path, root))
            return 2
        gate_q = np.asarray(gate_document["current"]["q_rad"], dtype=float)
        drift = float(np.max(np.abs(gate_q - np.asarray(q_now, dtype=float))))
        if drift > GATE_DRIFT_TOLERANCE_RAD:
            print("拒绝执行：机械臂自门禁运行后已移动 %.4f rad，门禁已过期。"
                  "重放必须从门禁假设的起始位姿开始，请先重跑 check_move_plan.py"
                  % drift)
            return 2
        segments = build_segments(gate_document, args.slots, args.first_slots)
        if not segments:
            print("没有要执行的段")
            return 2
        start_error = float(np.linalg.norm(
            np.asarray(segments[0]["start_tcp_m_rad"][:3]) -
            np.asarray(tcp_now[:3])))
        if start_error > START_MATCH_TOLERANCE_M:
            print("拒绝执行：门禁首段起点与当前 TCP 相差 %.1f mm，无法忠实重放"
                  % (start_error * 1000.0))
            return 2
        print("门禁校验：通过（漂移 %.5f rad，首段起点差 %.2f mm，共 %d 段 / %d 槽）" %
              (drift, start_error * 1000.0, len(segments),
               len({value["slot"] for value in segments})))
        print("重放门禁访问顺序: %s" % gate_document.get("visit_order"))

    mode = query_dashboard("robotmode")
    safety = query_dashboard("safetystatus")
    print("robotmode %s / safetystatus %s" % (mode, safety))
    if mode != "Robotmode: RUNNING" or safety != "Safetystatus: NORMAL":
        print("拒绝执行：控制器不在 RUNNING/NORMAL")
        return 2

    if not args.execute:
        print("\n--- DRY RUN（未发送任何指令）---")
        print("速度 %.3f m/s，加速度 %.3f m/s²" % (args.speed, args.acceleration))
        print("看门狗：路径偏离 > %.1f mm 或 |F| 变化 > %.1f N 即 stopj" %
              (args.max_deviation_mm, args.max_force_delta_n))
        for value in segments:
            print("  slot %s %-12s -> %s" %
                  (value["slot"], value["segment"],
                   [round(x, 4) for x in value["end_tcp_m_rad"][:3]]))
        print("加 --execute 才会真正运动。")
        return 0

    watchdog = Watchdog(args.max_deviation_mm / 1000.0, args.max_force_delta_n)
    watchdog.start()
    baseline = watchdog.measure_force_baseline()
    print("静止受力基线 |F| = %.3f N（用于变化量判据）" % (baseline or float("nan")))

    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    document = {
        "schema_version": 1,
        "kind": "gated_trajectory_replay",
        "started_at": dt.datetime.now(dt.timezone.utc).astimezone().isoformat(),
        "gate": (None if args.relative_dz is not None
                 else os.path.relpath(args.gate, root)),
        "visit_order_replayed": (None if gate_document is None
                                 else gate_document.get("visit_order")),
        "replay_note": "segments and visit order are taken verbatim from the gate "
                       "report; the executor does not recompute waypoints",
        "parameters": {"speed_m_s": args.speed, "acceleration_m_s2": args.acceleration,
                       "max_deviation_mm": args.max_deviation_mm,
                       "max_force_delta_n": args.max_force_delta_n,
                       "relative_dz_m": args.relative_dz},
        "force_baseline_n": baseline,
        "start_tcp_m_rad": list(tcp_now),
        "watchdog": {"meaning": "reactive only; not collision avoidance"},
        "segments": [],
    }

    overall_ok = True
    try:
        for value in segments:
            if watchdog.tripped:
                break
            end_tcp = value["end_tcp_m_rad"]
            watchdog.set_segment(value["start_tcp_m_rad"], end_tcp)
            # The call line is mandatory: URScript sent on 30002 only runs when it
            # is invoked.  position_pick_place.py (which completed a real
            # pick-place cycle) uses exactly this def-plus-call shape.
            started = time.monotonic()
            final, arrived, verdict = None, True, "arrived"
            sub_waypoints = split_segment(value["start_tcp_m_rad"], end_tcp,
                                          args.max_step_rot_deg,
                                          args.max_step_trans_m)
            for step_index, step_tcp in enumerate(sub_waypoints, start=1):
                program = ("def gated_move():\n"
                           "  movel(p[%s], a=%.6f, v=%.6f)\n"
                           "  sleep(0.4)\n"
                           "end\n"
                           "gated_move()\n") % (
                    ", ".join("%.9f" % item for item in step_tcp),
                    args.acceleration, args.speed)
                send_urscript(program)
                final, arrived, verdict = wait_for_arrival(step_tcp, args.speed,
                                                           watchdog)
                if not arrived:
                    verdict = "%s@step%d/%d" % (verdict, step_index,
                                                len(sub_waypoints))
                    break
            duration = time.monotonic() - started
            record = {
                "slot": value["slot"], "segment": value["segment"],
                "gate_order": value.get("order"),
                "gate_clearance_z_m": value.get("clearance_z_m"),
                "gate_roll_rescue_offset_deg": value.get("roll_rescue_offset_deg"),
                "commanded_tcp_m_rad": end_tcp,
                "start_tcp_m_rad": value["start_tcp_m_rad"],
                "final_tcp_m_rad": list(final),
                "sub_waypoint_count": len(sub_waypoints),
                "final_error_mm": pose_error(final, end_tcp)[0] * 1000.0,
                "final_rotation_error_deg": float(
                    np.degrees(pose_error(final, end_tcp)[1])),
                "duration_s": duration, "verdict": verdict, "arrived": bool(arrived),
                "watchdog_samples": watchdog.samples,
                "max_deviation_mm": watchdog.max_deviation_seen * 1000.0,
                "max_force_delta_n": watchdog.max_force_delta_seen,
                "trace": watchdog.trace[::3],
                "actual_j6_start_deg": watchdog.j6_start_deg,
                "actual_j6_end_deg": watchdog.j6_last_deg,
                "actual_j6_travel_deg_total": watchdog.j6_travel_deg,
                "actual_j6_span_deg": watchdog.j6_span_deg - watchdog.j6_span_min_deg,
            }
            document["segments"].append(record)
            print("  slot %-3s %-10s %-8s 位置 %6.3f mm  姿态 %6.3f°  %5.1fs  "
                  "最大偏离 %6.3f mm  |F|变化 %6.2f N" %
                  (value["slot"], value["segment"], verdict,
                   record["final_error_mm"], record["final_rotation_error_deg"],
                   duration, record["max_deviation_mm"],
                   record["max_force_delta_n"]))
            if not arrived:
                overall_ok = False
                print("  -> 该段未正常到位（%s），停止后续运动" % verdict)
                break
            if (args.collect and value["segment"] == "descend"
                    and args.relative_dz is None):
                ok, history = collect_sample(root, args.collect_dataset,
                                             args.collect_attempts)
                record["collection"] = history
                record["sample_accepted"] = bool(ok)
                print("    采集：%s（%d 次尝试）" % ("接受" if ok else "拒绝",
                                                    len(history)))
                for line in history[-1]["output_tail"]:
                    print("      " + line)
                if not ok:
                    overall_ok = False
                    print("    -> 采集被拒，停止后续运动")
                    break
    finally:
        watchdog.stop()

    document["finished_at"] = dt.datetime.now(dt.timezone.utc).astimezone().isoformat()
    document["actual_wrist"] = {
        "note": "measured from the controller's own q_actual on 30003, not from IK - "
                "this is the authoritative cable-strain evidence",
        "j6_start_deg": watchdog.j6_start_deg,
        "j6_end_deg": watchdog.j6_last_deg,
        "j6_net_deg": (None if watchdog.j6_start_deg is None
                       or watchdog.j6_last_deg is None
                       else watchdog.j6_last_deg - watchdog.j6_start_deg),
        "j6_travel_deg": watchdog.j6_travel_deg,
        "j6_span_deg": watchdog.j6_span_deg - watchdog.j6_span_min_deg,
    }
    document["watchdog_trip"] = watchdog.tripped
    document["watchdog_error"] = watchdog.error
    document["passed"] = bool(overall_ok and not watchdog.tripped)
    out_dir = args.out_dir if os.path.isabs(args.out_dir) \
        else os.path.join(root, args.out_dir)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "run-%s.json" % stamp)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print()
    if watchdog.tripped:
        print("结果：看门狗触发 —— %s" % watchdog.tripped)
    print("结果：%s" % ("PASS" if document["passed"] else "FAIL"))
    print("OUTPUT:", path)
    return 0 if document["passed"] else 3


if __name__ == "__main__":
    sys.exit(main())
