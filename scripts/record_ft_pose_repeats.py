#!/usr/bin/env python3
"""Record repeated ATI raw-wrench windows at static robot poses.

This tool never sends URScript or gripper commands.  The operator chooses each
pose with the teach pendant, then presses Enter.  A group consists of several
short windows at that unchanged pose so pose-to-pose repeatability can be
separated from within-window sensor noise.

``--watch-playback`` is intended for a *teach-pendant-recorded* trajectory:
this program only watches TCP state, waits for each new stable dwell, and then
collects repeated raw-wrench windows.  It never commands the robot.
"""
import argparse
import csv
import json
import math
import os
import socket
import struct
import subprocess
import threading
import time
from collections import deque

import numpy as np


class TcpPoseReader:
    """Persistent UR 30003 reader; reconnect-per-poll is too slow on CB3."""
    def __init__(self, host):
        self.host = host
        self.sock = None
        self.buffer = bytearray()
        self._connect()

    def _connect(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = socket.create_connection((self.host, 30003), timeout=3.0)
        self.sock.settimeout(3.0)
        self.buffer.clear()

    def read_state(self):
        # CB3 can silently stop an idle 30003 connection.  A single reconnect
        # keeps passive collection alive without issuing any control command.
        for attempt in range(2):
            try:
                while len(self.buffer) < 4:
                    self.buffer.extend(self.sock.recv(4096))
                size = struct.unpack_from(">I", self.buffer)[0]
                if size < 492 or size > 4096:
                    raise RuntimeError("unexpected UR 30003 frame size %d" % size)
                while len(self.buffer) < size:
                    self.buffer.extend(self.sock.recv(4096))
                frame = self.buffer[:size]
                del self.buffer[:size]
                q = np.asarray(struct.unpack_from(">6d", frame, 252), dtype=float)
                tcp = np.asarray(struct.unpack_from(">6d", frame, 444), dtype=float)
                return q, tcp
            except (socket.timeout, OSError):
                if attempt:
                    raise
                self._connect()

    def read(self):
        return self.read_state()[1]

    def close(self):
        self.sock.close()


def read_tcp_pose(host):
    """One-shot compatibility helper; repeated sampling should use TcpPoseReader."""
    reader = TcpPoseReader(host)
    try:
        return reader.read()
    finally:
        reader.close()


def skew(v):
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rotvec_to_matrix(v):
    angle = np.linalg.norm(v)
    if angle < 1e-12:
        return np.eye(3)
    axis = v / angle
    cross = skew(axis)
    return np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * (cross @ cross)


def rotation_error_deg(before, after):
    relative = rotvec_to_matrix(before).T @ rotvec_to_matrix(after)
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(float(cosine)))


class Collector:
    def __init__(self, topic):
        import rclpy
        from geometry_msgs.msg import WrenchStamped
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self.node = Node("record_ft_pose_repeats")
        self.rows = deque(maxlen=30000)
        self.lock = threading.Lock()

        def callback(msg):
            w = msg.wrench
            wrench = [w.force.x, w.force.y, w.force.z,
                      w.torque.x, w.torque.y, w.torque.z]
            with self.lock:
                self.rows.append((time.monotonic(), time.time(), wrench))

        self.node.create_subscription(WrenchStamped, topic, callback, 500)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def capture(self, duration, min_messages):
        started = time.monotonic()
        time.sleep(duration)
        with self.lock:
            records = [(wall, wrench) for stamp, wall, wrench in self.rows if stamp >= started]
        values = np.asarray([wrench for _, wrench in records], dtype=float)
        if len(values) < min_messages:
            raise RuntimeError("本窗口仅收到 %d 条消息（要求 %d）" %
                               (len(values), min_messages))
        return records, values

    def close(self):
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()
        self.thread.join(timeout=2.0)


def json_safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.floating):
        return float(value)
    raise TypeError(type(value).__name__)


def write_document(path, document):
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, default=json_safe)
        stream.write("\n")
    os.replace(temporary, path)


def announce(message, bell=False, terminal_broadcast=False):
    """Operator-visible notification; optional broadcast reaches local terminals."""
    prefix = time.strftime("[%H:%M:%S]")
    print("\n%s ===== %s =====%s" % (prefix, message, "\a" if bell else ""), flush=True)
    if terminal_broadcast:
        try:
            subprocess.run(["wall", "UR10 ATI: " + message], check=False,
                           timeout=2.0, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
        except (OSError, subprocess.TimeoutExpired):
            print("%s 终端广播失败；采集仍继续。" % prefix, flush=True)


def pose_change(before, after):
    """Return Cartesian translation (mm) and rotation-vector separation (deg)."""
    return (float(np.linalg.norm(after[:3] - before[:3]) * 1000.0),
            rotation_error_deg(before[3:], after[3:]))


def capture_group(collector, writer, stream, args, pose_reader, group_index, document, label):
    """Capture one repeated static group and durably append it to CSV/JSON data."""
    group = {"index": group_index, "label": label, "windows": []}
    print("开始第 %d 组：请保持姿态、线缆和外部接触状态不变。" % group_index)
    for repeat_index in range(1, args.repeats + 1):
        before = pose_reader.read()
        records, values = collector.capture(args.duration, args.min_messages)
        after = pose_reader.read()
        position_motion_mm, orientation_motion_deg = pose_change(before, after)
        accepted = (position_motion_mm <= args.max_position_motion_mm and
                    orientation_motion_deg <= args.max_orientation_motion_deg)
        window = {
            "repeat": repeat_index, "accepted": accepted,
            "message_count": int(len(values)),
            "tcp_pose_start": before, "tcp_pose_end": after,
            "position_motion_mm": position_motion_mm,
            "orientation_motion_deg": orientation_motion_deg,
            "wrench_mean": values.mean(axis=0),
            "wrench_std": values.std(axis=0, ddof=1),
        }
        group["windows"].append(window)
        for wall, wrench in records:
            writer.writerow([group_index, repeat_index, "%.9f" % wall,
                             *["%.12g" % value for value in wrench]])
        stream.flush()
        status = "接受" if accepted else "拒绝（采样期未停稳）"
        print("  %d/%d %s: %d 点, motion=%.3fmm/%.3fdeg, F=[% .3f % .3f % .3f]N" %
              (repeat_index, args.repeats, status, len(values),
               position_motion_mm, orientation_motion_deg, *values.mean(axis=0)[:3]))
        if repeat_index < args.repeats:
            time.sleep(args.gap)
    accepted = [window for window in group["windows"] if window["accepted"]]
    group["accepted_windows"] = len(accepted)
    if accepted:
        means = np.asarray([window["wrench_mean"] for window in accepted])
        group["between_window_mean_std"] = (
            means.std(axis=0, ddof=1) if len(means) > 1 else np.zeros(6))
    else:
        group["between_window_mean_std"] = np.full(6, np.nan)
    document["groups"].append(group)
    return group


def run(args):
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = os.path.abspath(args.csv_output or os.path.join(
        root, "outputs", "ft_repeats", "pose-repeats-%s.csv" % stamp))
    json_path = os.path.abspath(args.json_output or os.path.join(
        root, "outputs", "ft_repeats", "pose-repeats-%s.json" % stamp))
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    document = {
        "schema_version": 1,
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "topic": args.topic,
        "duration_s": args.duration,
        "repeats_per_group": args.repeats,
        "static_limits": {
            "position_motion_mm": args.max_position_motion_mm,
            "orientation_motion_deg": args.max_orientation_motion_deg,
        },
        "groups": [],
        "note": "No robot or gripper commands were sent. Use wrench_raw only.",
    }
    collector = Collector(args.topic)
    pose_reader = TcpPoseReader(args.host)
    try:
        with open(csv_path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["group", "repeat", "wall_time_s", "fx_n", "fy_n", "fz_n",
                             "tx_nm", "ty_nm", "tz_nm"])
            print("只读重复采样：每组 %d 个 %.1fs 窗口；不发送任何运动命令。" %
                  (args.repeats, args.duration))
            print("CSV:", csv_path)
            print("JSON:", json_path)
            while True:
                command = input("[%d 组] Enter=自动采本组, q=退出: " %
                                len(document["groups"])).strip().lower()
                if command == "q":
                    break
                group_index = len(document["groups"]) + 1
                group = capture_group(collector, writer, stream, args, pose_reader, group_index,
                                      document, "manual-%d" % group_index)
                write_document(json_path, document)
                print("第 %d 组已落盘：accepted=%d/%d" %
                      (group_index, group["accepted_windows"], args.repeats))
    finally:
        write_document(json_path, document)
        pose_reader.close()
        collector.close()


def run_watch_playback(args):
    """Passively collect every stable dwell while a pendant program is playing."""
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = os.path.abspath(args.csv_output or os.path.join(
        root, "outputs", "ft_repeats", "playback-repeats-%s.csv" % stamp))
    json_path = os.path.abspath(args.json_output or os.path.join(
        root, "outputs", "ft_repeats", "playback-repeats-%s.json" % stamp))
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    waypoint_plan = None
    if args.waypoint_plan:
        with open(os.path.abspath(os.path.expanduser(args.waypoint_plan)), encoding="utf-8") as stream:
            waypoint_document = json.load(stream)
        waypoint_plan = waypoint_document.get("outbound_waypoints")
        if not waypoint_plan:
            raise RuntimeError("waypoint plan 未包含 outbound_waypoints")
        if args.max_groups > 0 and args.max_groups != len(waypoint_plan):
            raise RuntimeError("--max-groups=%d 与计划点数 %d 不一致" %
                               (args.max_groups, len(waypoint_plan)))
    document = {
        "schema_version": 1, "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "mode": "passive_teach_pendant_playback", "topic": args.topic,
        "duration_s": args.duration, "repeats_per_group": args.repeats,
        "dwell_detection": {"settle_s": args.settle,
                            "position_motion_mm": args.max_position_motion_mm,
                            "orientation_motion_deg": args.max_orientation_motion_deg,
                            "new_pose_separation_mm": args.new_pose_distance_mm,
                            "new_pose_separation_deg": args.new_pose_angle_deg},
        "groups": [],
        "waypoint_plan": (os.path.abspath(os.path.expanduser(args.waypoint_plan))
                          if args.waypoint_plan else None),
        "joint_match_tolerance_deg": args.joint_match_tolerance_deg,
        "note": "Passive observer only: no URScript, robot, or gripper command was sent.",
    }
    file_mode = "w"
    if args.resume:
        if not (os.path.isfile(csv_path) and os.path.isfile(json_path)):
            raise RuntimeError("--resume 需要已有 CSV 和 JSON：%s / %s" % (csv_path, json_path))
        with open(json_path, encoding="utf-8") as stream:
            document = json.load(stream)
        if document.get("mode") != "passive_teach_pendant_playback":
            raise RuntimeError("--resume 的 JSON 不是被动示教播放采集记录")
        file_mode = "a"
    collector = Collector(args.topic)
    pose_reader = TcpPoseReader(args.host)
    history = deque()
    last_group_pose = None
    armed_for_new_pose = True
    if args.resume and document["groups"]:
        last_windows = document["groups"][-1].get("windows", [])
        if last_windows:
            last_group_pose = np.asarray(last_windows[-1]["tcp_pose_end"], dtype=float)
            armed_for_new_pose = False
    last_status = 0.0
    try:
        with open(csv_path, file_mode, newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            if file_mode == "w":
                writer.writerow(["group", "repeat", "wall_time_s", "fx_n", "fy_n", "fz_n",
                                 "tx_nm", "ty_nm", "tz_nm"])
            print("被动播放采集已就绪：请在示教器运行你录制的程序。")
            print("每个新姿态停稳至少 %.1fs；脚本将自动采 %d 个 %.1fs 窗口。" %
                  (args.settle, args.repeats, args.duration))
            print("CSV:", csv_path)
            print("JSON:", json_path)
            while args.max_groups <= 0 or len(document["groups"]) < args.max_groups:
                q_actual, pose = pose_reader.read_state()
                now = time.monotonic()
                history.append((now, pose))
                # Keep one polling interval beyond the requested dwell.  If
                # we trim exactly at ``settle`` first, floating-point/timer
                # jitter can make the oldest sample disappear just before the
                # subsequent duration test, so a stable pose never triggers.
                while history and now - history[0][0] > args.settle + args.poll:
                    history.popleft()
                if len(history) < 2 or now - history[0][0] < args.settle:
                    if args.status_interval and now - last_status >= args.status_interval:
                        span = now - history[0][0] if history else 0.0
                        print("[%s] 等待停稳：已连续观察 %.2fs / 需 %.2fs" %
                              (time.strftime("%H:%M:%S"), span, args.settle), flush=True)
                        last_status = now
                    time.sleep(args.poll)
                    continue
                reference = history[0][1]
                changes = [pose_change(reference, candidate) for _, candidate in history]
                stable = (max(change[0] for change in changes) <= args.max_position_motion_mm and
                          max(change[1] for change in changes) <= args.max_orientation_motion_deg)
                if (not stable and args.status_interval and
                        now - last_status >= args.status_interval):
                    print("[%s] 等待停稳：%.3fmm / %.3fdeg（门限 %.3fmm / %.3fdeg）" %
                          (time.strftime("%H:%M:%S"),
                           max(change[0] for change in changes),
                           max(change[1] for change in changes),
                           args.max_position_motion_mm, args.max_orientation_motion_deg),
                          flush=True)
                    last_status = now
                if last_group_pose is not None:
                    separation = pose_change(last_group_pose, pose)
                    if (separation[0] >= args.new_pose_distance_mm or
                            separation[1] >= args.new_pose_angle_deg):
                        armed_for_new_pose = True
                plan_match = True
                joint_error_deg = 0.0
                if waypoint_plan:
                    plan_index = len(document["groups"])
                    if plan_index >= len(waypoint_plan):
                        break
                    target_q = np.asarray(waypoint_plan[plan_index]["q_rad"], dtype=float)
                    joint_error_deg = float(np.degrees(np.max(np.abs(q_actual - target_q))))
                    plan_match = joint_error_deg <= args.joint_match_tolerance_deg
                    if (stable and not plan_match and args.status_interval and
                            now - last_status >= args.status_interval):
                        print("[%s] 等待计划点 %d：最大关节误差 %.3fdeg / 门限 %.3fdeg" %
                              (time.strftime("%H:%M:%S"), plan_index + 1,
                               joint_error_deg, args.joint_match_tolerance_deg), flush=True)
                        last_status = now
                if stable and armed_for_new_pose and plan_match:
                    group_index = len(document["groups"]) + 1
                    detail = ("计划点 %d 匹配（关节误差 %.3fdeg），开始采集" %
                              (group_index, joint_error_deg) if waypoint_plan else
                              "检测到第 %d 个新停靠姿态，开始采集" % group_index)
                    announce(detail,
                             args.bell, args.terminal_broadcast)
                    group = capture_group(collector, writer, stream, args, pose_reader, group_index,
                                          document, "playback-%d" % group_index)
                    write_document(json_path, document)
                    last_group_pose = pose_reader.read()
                    armed_for_new_pose = False
                    history.clear()
                    announce("第 %d 组已落盘：accepted=%d/%d" %
                             (group_index, group["accepted_windows"], args.repeats),
                             args.bell, args.terminal_broadcast)
                time.sleep(args.poll)
    finally:
        write_document(json_path, document)
        pose_reader.close()
        collector.close()


def self_test():
    assert rotation_error_deg(np.zeros(3), np.zeros(3)) == 0.0
    assert abs(rotation_error_deg(np.zeros(3), np.array([0.0, 0.0, math.pi / 2])) - 90.0) < 1e-9
    print("self-test passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.3")
    parser.add_argument("--topic", default="/ft_sensor/wrench_raw")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--min-messages", type=int, default=300)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--gap", type=float, default=0.3)
    parser.add_argument("--watch-playback", action="store_true",
                        help="被动监测示教器录制程序的停靠点；不发送运动命令")
    parser.add_argument("--bell", action="store_true",
                        help="播放模式检测到或完成节点时触发终端响铃")
    parser.add_argument("--terminal-broadcast", action="store_true",
                        help="播放模式在每个节点开始/完成时向本机终端广播提示")
    parser.add_argument("--settle", type=float, default=3.0,
                        help="播放模式判定停稳所需的连续静止时间（秒）")
    parser.add_argument("--poll", type=float, default=0.1,
                        help="播放模式 TCP 轮询周期（秒）")
    parser.add_argument("--new-pose-distance-mm", type=float, default=5.0,
                        help="播放模式两个采集姿态的最小平移间隔（毫米）")
    parser.add_argument("--new-pose-angle-deg", type=float, default=2.0,
                        help="播放模式两个采集姿态的最小转角间隔（度）")
    parser.add_argument("--max-groups", type=int, default=12,
                        help="播放模式最多自动采集的姿态数；0 表示直到 Ctrl-C")
    parser.add_argument("--status-interval", type=float, default=5.0,
                        help="播放模式等待节点时的终端状态输出周期（秒；0=关闭）")
    parser.add_argument("--resume", action="store_true",
                        help="追加到已有的播放采集 CSV/JSON；不覆盖已完成节点")
    parser.add_argument("--waypoint-plan",
                        help="只在依次匹配计划 JSON 的 outbound_waypoints 时采集")
    parser.add_argument("--joint-match-tolerance-deg", type=float, default=1.0,
                        help="计划点匹配的最大单关节误差（度）")
    parser.add_argument("--max-position-motion-mm", type=float, default=0.5)
    parser.add_argument("--max-orientation-motion-deg", type=float, default=0.1)
    parser.add_argument("--csv-output")
    parser.add_argument("--json-output")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if (args.duration <= 0 or args.min_messages < 1 or args.repeats < 1 or args.gap < 0 or
            args.settle <= 0 or args.poll <= 0 or args.new_pose_distance_mm <= 0 or
            args.new_pose_angle_deg <= 0 or args.max_groups < 0 or args.status_interval < 0 or
            args.joint_match_tolerance_deg <= 0):
        parser.error("采样、停稳和姿态分隔参数必须为有效正数（gap/max-groups 可为 0）")
    if args.watch_playback:
        run_watch_playback(args)
    else:
        run(args)


if __name__ == "__main__":
    main()
