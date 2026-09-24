#!/usr/bin/env python3
"""位置式抓放示例：控制器原生 movel + Robotiq ASCII，不使用力控。"""

import argparse
import json
import math
import os
import signal
import socket
import subprocess
import sys
import time

import cv2
import numpy as np
from record_ur_trajectory import RealtimeReader
from rq_gripper import RobotiqGripper


ROBOT_IP = "192.168.1.3"
COMMAND_PORT = 30002


class PositionPickPlace:
    def __init__(self, args):
        self.args = args
        self.reader = None
        self.stopped = False

    def stop_robot(self, *_):
        if self.stopped or self.reader is None:
            return
        self.stopped = True
        try:
            with socket.create_connection((ROBOT_IP, COMMAND_PORT), timeout=1.0) as sock:
                sock.sendall(b"stopj(2.0)\n")
            print("已发送 stopj；夹爪保持当前状态。", file=sys.stderr)
        except OSError as exc:
            print("发送 stopj 失败：%s；请立即使用示教器急停。" % exc, file=sys.stderr)

    @staticmethod
    def load_poses(path):
        with open(path, encoding="utf-8") as stream:
            poses = json.load(stream)
        for name in ("pick", "place"):
            pose = poses.get(name)
            if not isinstance(pose, list) or len(pose) != 6:
                raise ValueError("%s 必须是 6 个数的 UR 位姿 [x,y,z,rx,ry,rz]" % name)
            if not all(math.isfinite(float(value)) for value in pose):
                raise ValueError("%s 含非有限数值" % name)
        return [[float(value) for value in poses[name]] for name in ("pick", "place")]

    def validate(self, pick, place):
        if not 0.005 <= self.args.height <= 0.20:
            raise ValueError("height 必须在 0.005~0.20 m")
        if not 0.005 <= self.args.speed <= 0.08:
            raise ValueError("speed 必须在 0.005~0.08 m/s")
        if not 0.005 <= self.args.acceleration <= 0.30:
            raise ValueError("acceleration 必须在 0.005~0.30 m/s²")
        for name, pose in (("pick", pick), ("place", place)):
            if pose[2] < 0.05:
                raise ValueError("%s 的 Z=%.3f m 低于 0.05 m 门限" % (name, pose[2]))
        if math.dist(pick[:3], place[:3]) > self.args.max_span:
            raise ValueError("pick/place 距离超过 %.3f m 门限" % self.args.max_span)

    def move_and_verify(self, pose, label):
        print(label, flush=True)
        program = (
            "def position_pick_place_move():\n"
            "  movel(p[%s], a=%.6f, v=%.6f)\n"
            "  sleep(0.4)\n"
            "end\n"
            "position_pick_place_move()\n"
        ) % (", ".join("%.9f" % value for value in pose), self.args.acceleration, self.args.speed)
        with socket.create_connection((ROBOT_IP, COMMAND_PORT), timeout=3.0) as sock:
            sock.sendall(program.encode("ascii"))

        def rotation_error(actual, target):
            left, _ = cv2.Rodrigues(np.asarray(actual[3:], dtype=float))
            right, _ = cv2.Rodrigues(np.asarray(target[3:], dtype=float))
            return float(np.linalg.norm(cv2.Rodrigues(left.T @ right)[0]))
        expected = max(math.dist(pose[:3], self.current[:3]) / self.args.speed,
                       rotation_error(self.current, pose) / self.args.speed)
        deadline = time.monotonic() + max(15.0, expected * 2.5 + 8.0)
        last = self.current
        while time.monotonic() < deadline:
            if self.stopped:
                raise RuntimeError("收到停止信号")
            _, last, velocity = self.reader.read()
            position_error = math.dist(last[:3], pose[:3])
            linear_speed = math.sqrt(sum(value * value for value in velocity[:3]))
            if (position_error <= self.args.position_tolerance and
                    rotation_error(last, pose) <= 0.02 and linear_speed <= 0.003):
                self.current = list(last)
                return
        self.stop_robot()
        raise RuntimeError("未到位且已发送 stopj：%s，位置误差 %.1f mm" %
                           (label, math.dist(last[:3], pose[:3]) * 1000.0))

    def refresh_auto_plan(self):
        """Re-observe from the pick hover and rebuild a board-aligned plan.

        This is intentionally fixed-command rather than a user-provided shell
        hook: a plan may gain a new target only through the read-only camera
        locator and the bounded 50 mm-cube planner.  Any failed observation
        aborts before the descent and leaves the gripper open.
        """
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        env = os.environ.copy()
        env.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")
        observation = self.args.refresh_observation
        refreshed = self.args.refresh_poses
        initial_path = (self.args.poses if os.path.isabs(self.args.poses)
                        else os.path.join(root, self.args.poses))
        print("① 悬停位复拍：只识别物块点云（不看棋盘；失败则不下降）", flush=True)
        tracked = subprocess.run(
            [sys.executable, os.path.join(root, "scripts", "track_cube_without_checkerboard.py"),
             "--anchor", initial_path, "--min-coverage", "0.45", "--output", observation],
            cwd=root, env=env).returncode == 0
        if not tracked:
            # Near the jaws a complete top face is often occluded.  Fall back
            # to a bounded analytic-cube registration; it can correct only a
            # small translation and never changes the frozen board-aligned yaw.
            fitted = subprocess.run(
                [sys.executable, os.path.join(root, "scripts", "fit_cube_partial_cloud.py"),
                 "--template", initial_path, "--partial", observation,
                 "--max-shift-mm", "8", "--max-p80-mm", "9",
                 "--output", observation + ".partial-fit.json"],
                cwd=root, env=env).returncode == 0
            if fitted:
                observation += ".partial-fit.json"
            elif self.args.active_view_plan:
                print("② 普通复拍不稳定：开始门禁后的低速多视角环视", flush=True)
                active_dir = self.args.active_view_output_dir
                subprocess.run(
                    [sys.executable, os.path.join(root, "scripts", "run_active_cube_views.py"),
                     "--plan", self.args.active_view_plan, "--anchor", initial_path,
                     "--output-dir", active_dir, "--execute"],
                    cwd=root, env=env, check=True)
                merged = os.path.join(active_dir, "merged.json")
                subprocess.run(
                    [sys.executable, os.path.join(root, "scripts", "merge_active_cube_views.py"),
                     "--manifest", os.path.join(active_dir, "manifest.json"), "--output", merged],
                    cwd=root, env=env, check=True)
                subprocess.run(
                    [sys.executable, os.path.join(root, "scripts", "fit_cube_partial_cloud.py"),
                     "--template", initial_path, "--partial", merged,
                     "--max-shift-mm", "8", "--max-p80-mm", "2.5",
                     "--output", merged + ".partial-fit.json"],
                    cwd=root, env=env, check=True)
                observation = merged + ".partial-fit.json"
            else:
                raise RuntimeError("悬停点云不稳定，且未提供主动多视角计划；停止下降")
        replan_command = [sys.executable, os.path.join(root, "scripts", "replan_pick_from_cube_track.py"),
                          "--plan", initial_path, "--track", observation, "--output", refreshed]
        replan = subprocess.run(replan_command, cwd=root, env=env).returncode == 0
        if not replan:
            # A stable cloud that differs greatly from the first board-based
            # estimate is evidence, not permission.  It may be a real first
            # pass bias or a different cluster.  Use it solely to centre a
            # high hover multi-view scan; the final correction is then bounded
            # tightly against this provisional candidate.
            if not tracked or not self.args.active_view_plan:
                raise RuntimeError("复拍修正超限，未获得可执行的多视角复核；停止下降")
            provisional = refreshed + ".provisional.json"
            print("② 首次/复拍差异过大：仅上方环视复核，不允许下降", flush=True)
            subprocess.run(
                [sys.executable, os.path.join(root, "scripts", "replan_pick_from_cube_track.py"),
                 "--plan", initial_path, "--track", observation, "--max-correction-mm", "50",
                 "--output", provisional], cwd=root, env=env, check=True)
            subprocess.run(
                [sys.executable, os.path.join(root, "scripts", "plan_active_cube_views.py"),
                 "--plan", provisional, "--output", self.args.active_view_plan],
                cwd=root, env=env, check=True)
            active_dir = self.args.active_view_output_dir
            subprocess.run(
                [sys.executable, os.path.join(root, "scripts", "run_active_cube_views.py"),
                 "--plan", self.args.active_view_plan, "--anchor", provisional,
                 "--output-dir", active_dir, "--execute"], cwd=root, env=env, check=True)
            merged = os.path.join(active_dir, "merged.json")
            subprocess.run(
                [sys.executable, os.path.join(root, "scripts", "merge_active_cube_views.py"),
                 "--manifest", os.path.join(active_dir, "manifest.json"), "--output", merged],
                cwd=root, env=env, check=True)
            fitted = merged + ".partial-fit.json"
            subprocess.run(
                [sys.executable, os.path.join(root, "scripts", "fit_cube_partial_cloud.py"),
                 "--template", provisional, "--partial", merged,
                 "--max-shift-mm", "8", "--max-p80-mm", "9", "--output", fitted],
                cwd=root, env=env, check=True)
            subprocess.run(
                [sys.executable, os.path.join(root, "scripts", "replan_pick_from_cube_track.py"),
                 "--plan", provisional, "--track", fitted, "--max-correction-mm", "8",
                 "--output", refreshed], cwd=root, env=env, check=True)
        pick, place = self.load_poses(refreshed)
        self.validate(pick, place)
        return pick, place

    def run(self):
        pick, place = self.load_poses(self.args.poses)
        self.validate(pick, place)
        print("位置抓放示例：不使用 ATI、重力补偿或碰撞判定。")
        print("pick = %s\nplace = %s\ncycles = %d%s" %
              (pick, place, self.args.cycles,
               " pingpong" if self.args.pingpong else ""))
        if self.args.dry_run:
            print("DRY-RUN：点位和参数有效，未连接机器人。")
            return 0
        if not self.args.execute:
            raise RuntimeError("默认拒绝真机运动；先用 --dry-run，现场确认后才可加 --execute")

        self.reader = RealtimeReader(ROBOT_IP, port=self.args.state_port)
        _, self.current, _ = self.reader.read()
        gripper = RobotiqGripper()
        try:
            gripper.connect()
            status = gripper.status()
            print("夹爪：ACT=%s STA=%s FLT=%s POS=%s" %
                  tuple(status[key] for key in ("ACT", "STA", "FLT", "POS")))
            if status["FLT"] not in (0, None):
                raise RuntimeError("夹爪故障码 FLT=%s" % status["FLT"])
            if not gripper.activate():
                raise RuntimeError("夹爪激活未确认")
            for cycle in range(1, self.args.cycles + 1):
                reverse = self.args.swap or (self.args.pingpong and cycle % 2 == 0)
                src, dst = (place, pick) if reverse else (pick, place)
                src_up = list(src); src_up[2] += self.args.height
                dst_up = list(dst); dst_up[2] += self.args.height
                print("\n=== 第 %d/%d 轮：%s -> %s ===" %
                      (cycle, self.args.cycles, "终点" if reverse else "起点",
                       "起点" if reverse else "终点"), flush=True)
                gripper.open()
                self.move_and_verify(src_up, "① 到取物点上方")
                if self.args.reobserve_at_hover:
                    pick, place = self.refresh_auto_plan()
                    src, dst = (place, pick) if reverse else (pick, place)
                    src_up = list(src); src_up[2] += self.args.height
                    dst_up = list(dst); dst_up[2] += self.args.height
                    self.move_and_verify(src_up, "①b 按复测结果调整到取物点上方")
                self.move_and_verify(src, "② 下降到取物点")
                print("③ 闭合夹爪（仅检查 Robotiq OBJ，不使用力传感器）")
                gripper.close()
                obj = gripper.object_detected()
                pos = gripper.query("POS")
                print("   POS=%s OBJ=%s" % (pos, obj))
                if obj != 2 and not self.args.demo:
                    raise RuntimeError("夹爪未确认夹住物体：OBJ=%s POS=%s，停止搬运" % (obj, pos))
                if obj != 2 and self.args.demo:
                    print("   演示模式：忽略 OBJ=%s，继续位置搬运（不代表夹持成功）" % obj)
                self.move_and_verify(src_up, "④ 抬起")
                if self.args.hold_after_pick:
                    print("✔ 已抓取并抬起：保持夹紧，交由接触式放置流程处理。")
                    return 0
                self.move_and_verify(dst_up, "⑤ 到放置点上方")
                self.move_and_verify(dst, "⑥ 下降到放置点")
                print("⑦ 张开夹爪")
                gripper.open()
                self.move_and_verify(dst_up, "⑧ 抬起离开")
                print("✔ 第 %d 轮完成" % cycle)
            print("位置抓放完成，共 %d 轮。" % self.args.cycles)
            return 0
        finally:
            gripper.close_socket()
            if self.reader:
                self.reader.close()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses", required=True, help="JSON：{pick:[6], place:[6]}，均为 base 系 UR 位姿")
    parser.add_argument("--height", type=float, default=0.05, help="安全抬升高度，单位 m")
    parser.add_argument("--speed", type=float, default=0.02, help="movel 速度，单位 m/s")
    parser.add_argument("--acceleration", type=float, default=0.05, help="movel 加速度，单位 m/s²")
    parser.add_argument("--max-span", type=float, default=0.40, help="两点最大距离，单位 m")
    parser.add_argument("--position-tolerance", type=float, default=0.002, help="到位门限，单位 m")
    parser.add_argument("--demo", action="store_true",
                        help="演示模式：忽略 OBJ，不确认夹持；仅用于固定位置演示")
    parser.add_argument("--swap", action="store_true",
                        help="反向：从 place 点抓回 pick 点")
    parser.add_argument("--pingpong", action="store_true",
                        help="往返：奇数轮 pick→place，偶数轮 place→pick")
    parser.add_argument("--cycles", type=int, default=1, help="执行轮数，默认 1")
    parser.add_argument("--dry-run", action="store_true", help="只检查 JSON 和门限，不连接硬件")
    parser.add_argument("--execute", action="store_true",
                        help="显式授权真机运动；未提供时脚本拒绝打开运动流程")
    parser.add_argument("--state-port", type=int, default=30013,
                        help="UR 只读状态端口；本机为 30013")
    parser.add_argument("--reobserve-at-hover", action="store_true",
                        help="到初始取物悬停位后，复拍物块/棋盘并重算点位；失败则不下降")
    parser.add_argument("--refresh-observation",
                        default="outputs/vision/cube-track-refresh.json")
    parser.add_argument("--refresh-poses",
                        default="outputs/vision/auto-cube-pick-place-plan-refresh.json")
    parser.add_argument("--active-view-plan", default=None,
                        help="optional active_cube_view_plan; used only after ordinary hover tracking fails")
    parser.add_argument("--active-view-output-dir", default="outputs/vision/active-cube-views")
    parser.add_argument("--hold-after-pick", action="store_true",
                        help="after verified grasp/lift, keep the cube clamped and exit without fixed-height placement")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.cycles < 1:
        raise SystemExit("--cycles 必须 >= 1")
    if args.swap and args.pingpong:
        raise SystemExit("--swap 与 --pingpong 不能同时使用")
    task = PositionPickPlace(args)
    signal.signal(signal.SIGINT, task.stop_robot)
    signal.signal(signal.SIGTERM, task.stop_robot)
    try:
        return task.run()
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print("[故障停止] %s" % exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
