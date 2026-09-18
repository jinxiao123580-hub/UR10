#!/usr/bin/env python3
"""位置式抓放示例：控制器原生 movel + Robotiq ASCII，不使用力控。"""

import argparse
import json
import math
import os
import signal
import socket
import sys
import time

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

        deadline = time.monotonic() + max(8.0, math.dist(pose[:3], self.current[:3]) / self.args.speed * 3.0)
        last = self.current
        while time.monotonic() < deadline:
            if self.stopped:
                raise RuntimeError("收到停止信号")
            _, last, velocity = self.reader.read()
            position_error = math.dist(last[:3], pose[:3])
            linear_speed = math.sqrt(sum(value * value for value in velocity[:3]))
            if position_error <= self.args.position_tolerance and linear_speed <= 0.003:
                self.current = list(last)
                return
        raise RuntimeError("未到位：%s，位置误差 %.1f mm" % (label, math.dist(last[:3], pose[:3]) * 1000.0))

    def run(self):
        pick, place = self.load_poses(self.args.poses)
        self.validate(pick, place)
        print("位置抓放示例：不使用 ATI、重力补偿或碰撞判定。")
        print("pick = %s\nplace = %s" % (pick, place))
        if self.args.dry_run:
            print("DRY-RUN：点位和参数有效，未连接机器人。")
            return 0

        self.reader = RealtimeReader(ROBOT_IP)
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
            gripper.open()
            pick_up = list(pick); pick_up[2] += self.args.height
            place_up = list(place); place_up[2] += self.args.height
            self.move_and_verify(pick_up, "① 到取物点上方")
            self.move_and_verify(pick, "② 下降到取物点")
            print("③ 闭合夹爪（仅检查 Robotiq OBJ，不使用力传感器）")
            gripper.close()
            obj = gripper.object_detected()
            pos = gripper.query("POS")
            print("   POS=%s OBJ=%s" % (pos, obj))
            if obj != 2:
                raise RuntimeError("夹爪未确认夹住物体：OBJ=%s POS=%s，停止搬运" % (obj, pos))
            self.move_and_verify(pick_up, "④ 抬起")
            self.move_and_verify(place_up, "⑤ 到放置点上方")
            self.move_and_verify(place, "⑥ 下降到放置点")
            print("⑦ 张开夹爪")
            gripper.open()
            self.move_and_verify(place_up, "⑧ 抬起离开")
            print("位置抓放完成。")
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
    parser.add_argument("--dry-run", action="store_true", help="只检查 JSON 和门限，不连接硬件")
    return parser.parse_args()


def main():
    args = parse_args()
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
