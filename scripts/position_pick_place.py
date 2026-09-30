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


def rotation_error_between(actual, target):
    left, _ = cv2.Rodrigues(np.asarray(actual[3:], dtype=float))
    right, _ = cv2.Rodrigues(np.asarray(target[3:], dtype=float))
    return float(np.linalg.norm(cv2.Rodrigues(left.T @ right)[0]))


class PositionPickPlace:
    def __init__(self, args):
        self.args = args
        self.reader = None
        self.stopped = False

    def preflight_initial_hover(self, target):
        """Reject the staging move before opening the UR motion port."""
        import pinocchio as pin
        from check_move_plan import (BASE_FROM_URDF_ROOT, check_segment,
                                     pose_from_tcp_target, robot_pose)
        from self_collision import SelfCollisionModel
        from ur_pose_ik import UR10IK
        q, tcp, _ = self.reader.read()
        q = np.asarray(q, dtype=float)
        tcp = np.asarray(tcp, dtype=float)
        ik = UR10IK()
        error = pin.log6((BASE_FROM_URDF_ROOT * ik.pose(q)).inverse() * robot_pose(tcp)).vector
        if np.linalg.norm(error[:3]) > 0.010 or np.linalg.norm(error[3:]) > 0.020:
            raise RuntimeError("悬停预检失败：控制器 TCP 与 IK 模型 FK 不一致")
        result = check_segment(ik, robot_pose(tcp), pose_from_tcp_target(target),
                               q, 36, SelfCollisionModel(), 0.020)
        worst = result["worst"]
        if (result["reasons"] or worst["joint_margin_rad"] < math.radians(5) or
                worst["min_singular"] < 0.02):
            raise RuntimeError("到初始悬停位的路径未通过门禁：%s" %
                               (result["reasons"][:2] or [worst]))
        self.current = tcp.tolist()

    def execute_oriented_route(self, route):
        """Replay only the dense-gated route, checking the IK branch at each stop."""
        if not route or not route.get("passed") or not route.get("waypoints"):
            raise RuntimeError("没有通过密采样门禁的物块朝向路径")
        waypoints = route["waypoints"]
        if [row["name"] for row in waypoints[:3]] != [
                "pick_hover", "pick_descent", "lift_with_cube"]:
            raise RuntimeError("抓取路径顺序无效")
        if waypoints[-1]["name"] not in ("transfer_above_board", "descend_to_place_hover"):
            raise RuntimeError("抓取路径没有到达放置悬停位")
        for index, waypoint in enumerate(waypoints):
            name = waypoint["name"]
            if index == 2:
                print("③ 闭合夹爪并确认夹住物块", flush=True)
                self.gripper.close()
                obj = self.gripper.object_detected()
                if obj != 2:
                    raise RuntimeError("夹爪未确认夹住物体：OBJ=%s；停止搬运" % obj)
            target = [float(value) for value in waypoint["tcp_m_rad"]]
            self.move_and_verify(target, "%02d %s" % (index + 1, name))
            actual_q, _tcp, _velocity = self.reader.read()
            expected_q = np.asarray(waypoint["end_q_rad"], dtype=float)
            if np.max(np.abs(np.asarray(actual_q) - expected_q)) > 0.10:
                raise RuntimeError("%s 的实际关节解偏离离线 IK 分支超过 0.10 rad" % name)
        print("✔ 已在标定板上方按板方向持物；交由原始 Fz 接触式放置", flush=True)

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

        expected = max(math.dist(pose[:3], self.current[:3]) / self.args.speed,
                       rotation_error_between(self.current, pose) / self.args.speed)
        deadline = time.monotonic() + max(15.0, expected * 2.5 + 8.0)
        last = self.current
        while time.monotonic() < deadline:
            if self.stopped:
                raise RuntimeError("收到停止信号")
            _, last, velocity = self.reader.read()
            position_error = math.dist(last[:3], pose[:3])
            linear_speed = math.sqrt(sum(value * value for value in velocity[:3]))
            if (position_error <= self.args.position_tolerance and
                    rotation_error_between(last, pose) <= 0.02 and linear_speed <= 0.003):
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
        if self.args.require_cube_orientation:
            from plan_oriented_cube_transfer import cube_yaw_from_track

            def orientation_quality(path):
                with open(path if os.path.isabs(path) else os.path.join(root, path),
                          encoding="utf-8") as stream:
                    report = json.load(stream)
                try:
                    _yaw, _spread, _angles = cube_yaw_from_track(report)
                except (ValueError, KeyError, TypeError) as exc:
                    return None, str(exc)
                return min(float(row["geometry"]["top_face"]["footprint"]["coverage"])
                           for row in report["observations"]), None

            quality, reason = orientation_quality(observation)
            if quality is None:
                if not self.args.active_view_plan:
                    raise RuntimeError("复拍不足以确定物块朝向：%s；停止下降" % reason)
                print("② 单视角朝向不足（%s）；仅在门禁通过后多视角复拍" % reason,
                      flush=True)
                active_dir = self.args.active_view_output_dir
                command = [sys.executable, os.path.join(root, "scripts", "run_active_cube_views.py"),
                           "--plan", self.args.active_view_plan, "--anchor", initial_path,
                           "--output-dir", active_dir, "--execute"]
                if subprocess.run(command, cwd=root, env=env).returncode != 0:
                    raise RuntimeError("主动视角运动或采集未通过门禁；停止下降")
                manifest_path = (os.path.join(active_dir, "manifest.json") if os.path.isabs(active_dir)
                                 else os.path.join(root, active_dir, "manifest.json"))
                with open(manifest_path, encoding="utf-8") as stream:
                    manifest = json.load(stream)
                if not manifest.get("complete") or not manifest.get("motion_sent"):
                    raise RuntimeError("主动视角记录不完整；停止下降")
                accepted = []
                for view in manifest.get("views", []):
                    view_path = view["report"]
                    view_quality, view_reason = orientation_quality(view_path)
                    if view_quality is not None:
                        accepted.append((view_quality, view_path))
                    else:
                        print("视角 %s 朝向不足：%s" % (view.get("slot"), view_reason), flush=True)
                if not accepted:
                    fused = observation + ".multiview-yaw.json"
                    command = [sys.executable, os.path.join(root, "scripts", "fuse_cube_yaw_views.py"),
                               "--initial-track", observation, "--manifest", manifest_path,
                               "--output", fused]
                    if subprocess.run(command, cwd=root, env=env).returncode != 0:
                        raise RuntimeError("主动视角未取得一致的物块朝向；禁止下降抓取")
                    observation = fused
                    tracked = True
                    print("选用跨视角一致的物块朝向证据：%s" % observation, flush=True)
                if len(accepted) > 1:
                    centers, yaws = [], []
                    for _view_quality, view_path in accepted:
                        with open(view_path if os.path.isabs(view_path)
                                  else os.path.join(root, view_path), encoding="utf-8") as stream:
                            view_report = json.load(stream)
                        yaw, _spread, _angles = cube_yaw_from_track(view_report)
                        centers.append(np.asarray(view_report["center_base_m"], dtype=float))
                        yaws.append(yaw)
                    center_spread_mm = max(np.linalg.norm(center - centers[0])
                                           for center in centers) * 1000.0
                    yaw_spread_deg = max(abs(((yaw - yaws[0] + 45.0) % 90.0) - 45.0)
                                         for yaw in yaws)
                    if center_spread_mm > 3.0 or yaw_spread_deg > 3.0:
                        raise RuntimeError("主动视角间物块位置/朝向不一致：%.1f mm / %.1f°；停止下降" %
                                           (center_spread_mm, yaw_spread_deg))
                if accepted:
                    quality, observation = max(accepted)
                    tracked = True
                    print("选用完整顶面观测：%s（最低覆盖率 %.1f%%）" %
                          (observation, quality * 100.0), flush=True)
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
        multiview_verified = observation.endswith(".multiview-yaw.json")
        if multiview_verified:
            # Only the independently validated two-view cube consensus may
            # correct a biased first-pass side silhouette by up to 25 mm.
            replan_command += ["--max-correction-mm", "25"]
        replan = subprocess.run(replan_command, cwd=root, env=env).returncode == 0
        if not replan:
            if multiview_verified:
                raise RuntimeError("多视角修正超过 25 mm；首次目标可能不是同一物块，禁止下降")
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
        route = None
        if self.args.require_cube_orientation:
            q, tcp, _ = self.reader.read()
            self.current = list(tcp)
            route_path = refreshed + ".oriented-route.json"
            command = [sys.executable, os.path.join(root, "scripts", "plan_oriented_cube_transfer.py"),
                       "--plan", refreshed, "--track", observation,
                       "--start-q=" + ",".join(str(x) for x in q),
                       "--start-tcp=" + ",".join(str(x) for x in tcp),
                       "--output", route_path]
            if not self.args.allow_unmodelled_detour:
                # Do not spend time planning lateral paths that the executor
                # would later refuse because external obstacles are unmodelled.
                command += ["--via-x-offset-mm", "0"]
            subprocess.run(command, cwd=root, env=env, check=True)
            with open(route_path, encoding="utf-8") as stream:
                oriented = json.load(stream)
            route = oriented.get("selected_route")
            if not route or not route.get("passed") or oriented.get("selected_quarter_turn") is None:
                raise RuntimeError("物块朝向或 20 mm 路径门禁未通过，禁止下降")
            if (abs(float(route.get("via_x_offset_m", 0.0))) > 1e-9 and
                    not self.args.allow_unmodelled_detour):
                raise RuntimeError("选中横向绕行，但工作台外部障碍尚未建模；需现场核对后显式允许绕行")
            with open(refreshed, encoding="utf-8") as stream:
                revised = json.load(stream)
            revised["pick"] = oriented["pick"]
            revised["place"] = oriented["place"]
            revised["tool_orientation_source"] = "cube top-face edge for pick; checkerboard X for place"
            revised["oriented_route"] = route
            revised["oriented_route_start_state"] = oriented["start_state"]
            revised["oriented_route_file"] = route_path
            with open(refreshed + ".tmp", "w", encoding="utf-8") as stream:
                json.dump(revised, stream, ensure_ascii=False, indent=2)
                stream.write("\n")
            os.replace(refreshed + ".tmp", refreshed)
        pick, place = self.load_poses(refreshed)
        self.validate(pick, place)
        return pick, place, route

    def run(self):
        pick, place = self.load_poses(self.args.poses)
        self.validate(pick, place)
        if self.args.require_cube_orientation:
            print("物块朝向抓取：路径做模型自碰撞检查；放置下降使用六轴力突变保护。")
            print("未建模的工作台、线缆和外部障碍仍需现场确认；不使用重力补偿。")
        else:
            print("位置抓放示例：不使用 ATI、重力补偿或碰撞判定。")
        print("pick = %s\nplace = %s\ncycles = %d%s" %
              (pick, place, self.args.cycles,
               " pingpong" if self.args.pingpong else ""))
        if self.args.dry_run:
            print("DRY-RUN：点位和参数有效，未连接机器人。")
            return 0
        if not self.args.execute:
            raise RuntimeError("默认拒绝真机运动；先用 --dry-run，现场确认后才可加 --execute")
        if self.args.require_cube_orientation and (not self.args.reobserve_at_hover or
                not self.args.hold_after_pick or self.args.cycles != 1 or
                self.args.swap or self.args.pingpong or self.args.demo):
            raise RuntimeError("物块朝向自动抓取只允许：复拍、单次、正向、OBJ 确认及接触式放置")

        self.reader = RealtimeReader(ROBOT_IP, port=self.args.state_port)
        _, self.current, _ = self.reader.read()
        gripper = RobotiqGripper()
        self.gripper = gripper
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
                if self.args.require_cube_orientation:
                    self.preflight_initial_hover(src_up)
                self.move_and_verify(src_up, "① 到取物点上方")
                if self.args.reobserve_at_hover:
                    pick, place, route = self.refresh_auto_plan()
                    if self.args.require_cube_orientation:
                        q_now, tcp_now, _ = self.reader.read()
                        with open(self.args.refresh_poses, encoding="utf-8") as stream:
                            state = json.load(stream)["oriented_route_start_state"]
                        if (np.max(np.abs(np.asarray(q_now) - np.asarray(state["q_rad"]))) > 0.03 or
                                math.dist(tcp_now[:3], state["tcp_m_rad"][:3]) > 0.003 or
                                rotation_error_between(tcp_now, state["tcp_m_rad"]) > 0.03):
                            raise RuntimeError("规划后机械臂状态已改变，拒绝执行旧路径")
                        self.execute_oriented_route(route)
                        return 0
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
    parser.add_argument("--require-cube-orientation", action="store_true",
                        help="fail closed unless full cube yaw and the dense IK/collision transfer route pass")
    parser.add_argument("--allow-unmodelled-detour", action="store_true",
                        help="explicitly acknowledge that selected lateral detour has no workcell obstacle map")
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
