#!/usr/bin/env python3
"""Teach two diagonal corners, then force-probe 12 points on an XY-aligned paper.

Two corner points define an axis-aligned rectangle, not arbitrary paper yaw.
Default is teach/plan only. --execute enables guarded raise/traverse/approach
and the existing bounded force contact probe. No gripper/TCP settings change.
"""
import argparse
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from calibrate_pen_tcp import capture, save
import probe_paper_height as probe

ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / 'outputs/paper_calibration'


def rectangle_points(corners):
    xyz = np.asarray(corners, dtype=float)
    if xyz.shape != (2, 3) or not np.all(np.isfinite(xyz)):
        raise ValueError('两角必须是两个有限的 XYZ 坐标')
    xy = xyz[:, :2]
    low, high = xy.min(axis=0), xy.max(axis=0)
    size = high - low
    if np.any(size < 0.08):
        raise ValueError('两角在基座 X/Y 方向的跨度都需至少 80 mm，请选择更大的纸内矩形')
    margin = np.maximum(0.012, size * 0.12)
    inner_low, inner_high = low + margin, high - margin
    # First three points span both XY dimensions. Remaining training points
    # form a 3x3 grid; holdouts are distinct interior interpolation points.
    uv = [(1, 1), (.5, 1), (.5, .5), (0, 1), (0, .5), (0, 0),
          (.5, 0), (1, 0), (1, .5)]
    # Begin near the second taught corner, then visit neighboring grid points.
    # The set of nine training locations is unchanged; only the order changes.
    uv = [(1-u if xy[1, 0] == low[0] else u,
           1-v if xy[1, 1] == low[1] else v) for u, v in uv]
    uv += [(.25, .25), (.75, .35), (.35, .75)]
    points = [{'index': i + 1, 'role': 'training' if i < 9 else 'holdout',
               'xy_base_m': (inner_low + np.asarray(p) * (inner_high - inner_low)).tolist()}
              for i, p in enumerate(uv)]
    return {'axes': 'robot base X/Y, operator-confirmed paper alignment',
            'outer_min_xy_m': low.tolist(), 'outer_max_xy_m': high.tolist(),
            'inner_min_xy_m': inner_low.tolist(), 'inner_max_xy_m': inner_high.tolist(),
            'margin_xy_mm': (margin * 1000).tolist(), 'points': points}


def fit_plane(points):
    xyz = np.asarray(points, dtype=float)
    if xyz.shape != (12, 3) or not np.all(np.isfinite(xyz)):
        raise ValueError('平面验收需要 12 个有效接触点，前三维均须有限')
    train, test = xyz[:9], xyz[9:]
    center = train.mean(axis=0)
    xy = train[:, :2] - center[:2]
    design = np.column_stack((xy, np.ones(9)))
    coef, _, rank, _ = np.linalg.lstsq(design, train[:, 2] - center[2], rcond=None)
    spread = np.linalg.svd(xy, compute_uv=False)
    if rank < 3 or spread[-1] < 0.02:
        return {'passed': False, 'reason': 'XY 点位覆盖不足或近似共线', 'rank': int(rank),
                'xy_spread_singular_m': spread.tolist()}
    a, b, intercept = coef
    c = center[2] + intercept - a * center[0] - b * center[1]
    normal = np.asarray([-a, -b, 1.0]); normal /= np.linalg.norm(normal)
    distance = (xyz[:, 2] - a * xyz[:, 0] - b * xyz[:, 1] - c) * normal[2]
    errors_mm = distance * 1000
    rms = float(np.sqrt(np.mean(errors_mm[:9] ** 2)))
    holdout_max = float(np.abs(errors_mm[9:]).max())
    hull = ConvexHull(train[:, :2])
    inside = bool(np.all(test[:, :2] @ hull.equations[:, :2].T
                         + hull.equations[:, 2] <= 0.001))
    origin = np.array([train[:, 0].min(), train[:, 1].min(), 0.0])
    origin[2] = a * origin[0] + b * origin[1] + c
    x_axis = np.array([1., 0, 0]) - normal[0] * normal
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal, x_axis)
    transform = np.eye(4)
    transform[:3, :3] = np.column_stack((x_axis, y_axis, normal))
    transform[:3, 3] = origin
    reasons = []
    if rms > 0.5:
        reasons.append('训练点到平面 RMS 超过 0.5 mm')
    if holdout_max > 1:
        reasons.append('留出点到平面最大距离超过 1 mm')
    if not inside:
        reasons.append('留出点不全位于训练点凸包内')
    if hull.volume < 0.0025:
        reasons.append('训练点覆盖面积小于 2500 mm²')
    if normal[2] < 0.5:
        reasons.append('纸面倾斜过大，不适用于当前竖直探测流程')
    return {'passed': not reasons, 'validation_failures': reasons,
            'equation': 'z_base_m = a*x_base_m + b*y_base_m + c',
            'a': float(a), 'b': float(b), 'c_m': float(c), 'normal_base': normal.tolist(),
            'tilt_deg': float(np.rad2deg(np.arccos(normal[2]))),
            'training_rms_perpendicular_mm': rms,
            'training_max_perpendicular_mm': float(np.abs(errors_mm[:9]).max()),
            'holdout_errors_perpendicular_mm': errors_mm[9:].tolist(),
            'holdout_max_perpendicular_mm': holdout_max,
            'training_count': 9, 'holdout_count': 3, 'holdouts_inside_training_hull': inside,
            'training_area_mm2': float(hull.volume * 1e6),
            'xy_spread_singular_m': spread.tolist(),
            'base_from_paper_4x4': transform.tolist(),
            'paper_frame_definition': 'origin=lower XY corner of sampled region on fitted plane; X=projected base +X; Z=upward normal',
            'meaning': 'plane of force-triggered loaded pen-tip contact points, including TCP and pen compliance errors',
            'controller_tcp_written': False}


def validate_session(session, require_complete=False):
    """Keep the saved sampling plan and reject duplicate/mislabelled samples."""
    plan = session.get('rectangle')
    records = session['accepted_points']
    if plan is None:
        if records:
            raise ValueError('已有采样却缺少原始布点计划，不能重排续采点')
        return
    points = plan['points']
    if len(points) != 12 or sorted(p['index'] for p in points) != list(range(1, 13)):
        raise ValueError('布点计划必须包含编号 1–12 各一次')
    expected = {p['index']: p for p in points}
    low, high = (np.asarray(plan[key], dtype=float)
                 for key in ('outer_min_xy_m', 'outer_max_xy_m'))
    if low.shape != (2,) or high.shape != (2,) or not np.all(np.isfinite([low, high])) or np.any(low >= high):
        raise ValueError('原始纸面范围无效')
    for p in points:
        xy = np.asarray(p['xy_base_m'], dtype=float)
        if xy.shape != (2,) or not np.all(np.isfinite(xy)) or np.any(xy <= low) or np.any(xy >= high):
            raise ValueError('布点必须在原始纸面矩形内部')
        if p['role'] != ('training' if p['index'] <= 9 else 'holdout'):
            raise ValueError('前 9 点拟合、后 3 点验证的角色不能改变')
    indices = [r['index'] for r in records]
    if len(set(indices)) != len(indices) or any(i not in expected for i in indices):
        raise ValueError('已采点编号重复或不属于原始布点计划')
    for r in records:
        xyz = np.asarray(r['pen_tip_base_m'], dtype=float)
        p = expected[r['index']]
        if (xyz.shape != (3,) or not np.all(np.isfinite(xyz)) or r['role'] != p['role']
                or np.linalg.norm(xyz[:2] - p['xy_base_m']) > .0005):
            raise ValueError('已有点的坐标或角色与原始布点计划不符')
    if require_complete and sorted(indices) != list(range(1, 13)):
        raise ValueError('平面验收需要编号 1–12 的全部有效点')


def tcp_target(point, tcp_orientation, offset, tip):
    flange_rotation = Rotation.from_rotvec(tcp_orientation) * Rotation.from_rotvec(offset[3:]).inv()
    position = np.asarray(point) - flange_rotation.apply(tip) + flange_rotation.apply(offset[:3])
    return np.r_[position, tcp_orientation]


def estimate_height(session, xy):
    records = session['accepted_points']
    if len(records) >= 3:
        p = np.asarray([r['pen_tip_base_m'] for r in records if r['index'] <= 9])
        design = np.column_stack((p[:, :2], np.ones(len(p))))
        if np.linalg.matrix_rank(design, tol=1e-8) == 3:
            coef = np.linalg.lstsq(design, p[:, 2], rcond=None)[0]
            return float(np.r_[xy, 1] @ coef)
    corners = np.asarray([r['pen_tip_base_m'] for r in session['corners']])
    diagonal = corners[1, :2] - corners[0, :2]
    fraction = float(np.dot(np.asarray(xy) - corners[0, :2], diagonal) / np.dot(diagonal, diagonal))
    return float(corners[0, 2] + fraction * (corners[1, 2] - corners[0, 2]))


def state_tip(reader, offset, tip):
    q, tcp, velocity = probe.stationary_state(reader)
    return q, tcp, probe.tip_position(tcp, offset, tip)


def prepare_start(reader, offset, tip, session, interactive=True):
    while True:
        state = state_tip(reader, offset, tip)
        point = state[2]
        estimated = estimate_height(session, point[:2])
        gap_mm = (point[2] - estimated) * 1000
        print('当前笔尖 Z=%.2f mm，估计纸面 Z=%.2f mm，离纸约 %.2f mm。' %
              (point[2] * 1000, estimated * 1000, gap_mm))
        if 1.5 <= gap_mm <= 100:
            return state
        if not interactive:
            raise RuntimeError('非交互起点离纸 %.2f mm，不满足 2–100 mm 范围' % gap_mm)
        print('请仅沿基座 Z 调整，使笔尖离纸 2–100 mm；不必精确停在 3–5 mm。')
        answer = input('调整并停稳后 Enter 重新读取；q 保存退出 > ')
        if answer.strip().lower() == 'q':
            return None


def travel_segment(reader, force, baseline, host, endpoint, orientation, speed, acceleration, entry):
    probe.force_level(force, baseline)
    reader.sock.settimeout(.1)
    script = probe.finite_movel_program('paper_travel_step', endpoint, acceleration, speed)
    probe.command(host, script)
    timeout = probe.finite_move_timeout(speed, acceleration)
    entry['segment_timeout_s'] = timeout
    deadline, contact_since, recent = time.monotonic() + timeout, None, []
    recovery = entry.setdefault('realtime_recoveries', [])
    segment_recovery = []
    while time.monotonic() < deadline:
        level = probe.force_level(force, baseline)
        now = time.monotonic()
        if level >= 1:
            contact_since = now if contact_since is None else contact_since
            if now - contact_since > .08:
                raise RuntimeError('平移时持续异常力 %.3f N，停止' % level)
        else:
            contact_since = None
        before_recoveries = len(segment_recovery)
        q_now, actual, velocity = probe.motion_state(reader, force, baseline, host, endpoint, script, segment_recovery)
        if len(segment_recovery) > before_recoveries:
            recovery.append(segment_recovery[-1])
            deadline = time.monotonic() + timeout
            recent.clear()
            contact_since = None
        entry['last_controller_tcp_m_rad'] = actual.tolist()
        margin = min(np.min(q_now - probe.LOWER), np.min(probe.UPPER - q_now))
        if margin < np.deg2rad(3):
            raise RuntimeError('实时关节限位余量低于 3°')
        recent.append(actual[:3]); recent = recent[-9:]
        error = Rotation.from_rotvec(actual[3:]).inv() * Rotation.from_rotvec(orientation)
        if error.magnitude() > np.deg2rad(.2):
            raise RuntimeError('平移过程中笔姿态改变')
        if (len(recent) == 9 and np.linalg.norm(np.mean(recent, axis=0) - endpoint[:3]) < .0001
                and np.linalg.norm(velocity[:3]) < .0001):
            return
    residual_mm = float(np.linalg.norm(actual[:3] - endpoint[:3]) * 1000)
    entry['timeout_residual_mm'] = residual_mm
    raise RuntimeError('有限 2 mm 平移在 %.1f s 内未到位；剩余误差 %.3f mm' % (timeout, residual_mm))


def recover_high_clearance_spike(reader, force, baseline, host, event):
    # A raw 5 N event ALWAYS stops the robot first. Resume is allowed only for
    # high-clearance XY travel after force returns to the ORIGINAL baseline.
    # No tare, re-zero, or threshold increase is used to hide persistent load.
    probe.stop(host)
    event['status'] = 'stopped_for_check'
    event['force_trace_at_stop'] = probe.force_snapshot(force, baseline)
    deadline, quiet_since = time.monotonic() + 3, None
    while time.monotonic() < deadline:
        try:
            level = probe.force_level(force, baseline)
        except probe.ForceGuardError:
            level = float('inf')
        _, actual, velocity = probe.fresh(reader, max_wait_seconds=.25)
        now = time.monotonic()
        if level < .3 and np.linalg.norm(velocity[:3]) < .0001:
            quiet_since = now if quiet_since is None else quiet_since
            if now - quiet_since >= .3:
                event['status'] = 'returned_to_original_baseline'
                event['settled_delta_n'] = level
                event['stopped_controller_tcp_m_rad'] = actual.tolist()
                return
        else:
            quiet_since = None
        time.sleep(.01)
    raise RuntimeError('停机后力未恢复至原基线附近，禁止继续横移')


def travel(reader, force, host, offset, tip, desired_tip, orientation, log, allow_transient_pause=False, on_progress=None):
    q, start, start_tip = state_tip(reader, offset, tip)
    target = tcp_target(desired_tip, orientation, offset, tip)
    entry = {'status': 'preflight', 'start_pen_tip_base_m': start_tip.tolist(),
             'target_pen_tip_base_m': list(desired_tip), 'travel_speed_m_s': .003,
             'travel_acceleration_m_s2': .003, 'completed_steps': 0}
    log.append(entry)
    baseline, commanded = None, False
    try:
        rotation_error = Rotation.from_rotvec(start[3:]).inv() * Rotation.from_rotvec(orientation)
        if rotation_error.magnitude() > np.deg2rad(0.2):
            raise RuntimeError('笔姿态改变，自动平移停止')
        if math.dist(start[:3], target[:3]) < .0001:
            entry['status'] = 'already_at_target'
            if on_progress:
                on_progress()
            return
        probe.controller_ready(host)
        entry['model_gate'] = probe.model_gate(q, start, target, offset)
        baseline, entry['baseline'] = force.baseline()
        _, unchanged, _ = state_tip(reader, offset, tip)
        drift = math.dist(unchanged[:3], start[:3])
        entry['preflight_rechecked_tcp_m_rad'] = unchanged.tolist()
        entry['preflight_drift_mm'] = drift * 1000
        if drift > 0.0001:
            raise RuntimeError('路径预检期间机器人移动了')
        count = max(1, int(math.ceil(math.dist(start[:3], target[:3]) / 0.002)))
        entry['finite_step_count'] = count
        entry['status'] = 'moving'
        # Vertical approach is close to the paper. Use gentler starts than
        # high-clearance travel without weakening any force watchdog.
        vertical = np.linalg.norm(np.asarray(desired_tip)[:2] - start_tip[:2]) < .0001
        speed, acceleration = (.001, .0005) if vertical else (.003, .003)
        entry['travel_speed_m_s'] = speed
        entry['travel_acceleration_m_s2'] = acceleration
        for index in range(count):
            endpoint = start + (target - start) * ((index + 1) / count)
            entry['pending_step'] = index + 1
            entry['pending_endpoint_tcp_m_rad'] = endpoint.tolist()
            if on_progress:
                on_progress()
            while True:
                try:
                    commanded = True
                    travel_segment(reader, force, baseline, host, endpoint, orientation, speed, acceleration, entry)
                    break
                except probe.ForceGuardError as exc:
                    events = entry.setdefault('transient_stops', [])
                    if not allow_transient_pause or len(events) >= 2:
                        raise
                    event = {'step': index + 1, 'trigger': exc.diagnostics}
                    events.append(event)
                    recover_high_clearance_spike(reader, force, baseline, host, event)
                    speed, acceleration = .001, .001
                    entry['recovery_speed_m_s'] = speed
                    print('高位横移瞬态：已停下并确认力恢复原基线；以 1 mm/s 重试当前小段。', flush=True)
            entry['completed_steps'] = index + 1
            if on_progress:
                on_progress()
            if (index + 1) % 10 == 0:
                print('  平移小段 %d/%d' % (index + 1, count), flush=True)
        entry['status'] = 'completed'
        entry.pop('pending_step', None)
    except BaseException as exc:
        if commanded:
            try:
                probe.stop(host)
            except OSError as stop_error:
                entry['stop_command_error'] = str(stop_error)
        entry['status'] = 'failed'
        entry['error'] = str(exc)
        if isinstance(exc, probe.ForceGuardError):
            entry['force_guard_diagnostics'] = exc.diagnostics
        if isinstance(exc, probe.BaselineNoiseError):
            entry['baseline_noise_diagnostics'] = exc.diagnostics
        entry['force_trace'] = probe.force_snapshot(force, baseline)
        raise


def run_contact(host, calibration, output):
    command = [sys.executable, str(Path(__file__).with_name('probe_paper_height.py')),
               '--host', host, '--tcp-calibration', str(calibration), '--execute', '--output', str(output)]
    child = subprocess.Popen(command)
    try:
        code = child.wait()
    except BaseException:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                probe.stop(host)
                child.terminate()
        raise
    result = json.loads(output.read_text()) if output.exists() else {}
    if code != 0 or result.get('status') != 'contact_recorded_retracted_2mm':
        raise RuntimeError('触纸子流程停止：%s；记录 %s' % (result.get('error', '无完整结果'), output))
    return result


def read_only_check(session, args, tip):
    if len(session['corners']) != 2:
        raise RuntimeError('只读预检需要先记录两个纸角')
    reader, force = None, None
    report = {'kind': 'paper_plane_read_only_preflight', 'motion_sent': False,
              'source_session': str(args.output), 'status': 'checking', 'script_revision': 9}
    path = args.output.parent / ('check-' + args.output.stem + '-' + dt.datetime.now().strftime('%Y%m%d-%H%M%S') + '.json')
    try:
        info = capture(args.host, 30011, 1)
        offset = np.asarray(info['active_tcp_offset_m_rad'])
        reader = probe.RealtimeReader(args.host, 30013)
        _, tcp, point = state_tip(reader, offset, tip)
        report['pen_tip_base_m'] = point.tolist()
        report['estimated_gap_mm'] = (point[2] - estimate_height(session, point[:2])) * 1000
        if not 1.5 <= report['estimated_gap_mm'] <= 100:
            raise RuntimeError('预检起点仍需离纸 2–100 mm')
        orientation = np.asarray(session.get('tcp_orientation_rotvec', tcp[3:]))
        if (Rotation.from_rotvec(tcp[3:]).inv() * Rotation.from_rotvec(orientation)).magnitude() > np.deg2rad(.2):
            raise RuntimeError('当前笔姿态与已记录的自动采集姿态不符')
        if 'active_tcp_offset_m_rad' in session and not np.allclose(offset, session['active_tcp_offset_m_rad'], atol=1e-9, rtol=0):
            raise RuntimeError('控制器 TCP 设置改变')
        report['controller'] = probe.controller_ready(args.host)
        force = probe.RawForce()
        _, report['force_baseline'] = force.baseline()
        # Read again after DDS/force baseline collection to verify stale
        # buffered state has been discarded without opening another socket.
        q, tcp, rechecked = state_tip(reader, offset, tip)
        if np.linalg.norm(rechecked - point) > .0001:
            raise RuntimeError('只读预检期间笔尖发生移动')
        report['realtime_connection_count'] = reader.reconnects
        corners = [c['pen_tip_base_m'] for c in session['corners']]
        known_z = [p[2] for p in corners] + [p['pen_tip_base_m'][2] for p in session['accepted_points']]
        clearance = max(max(known_z) + .020, point[2])
        target_point = [*point[:2], clearance]
        report['next_action'] = 'raise'
        if clearance - point[2] < .00005:
            rectangle = session.get('rectangle') or rectangle_points(corners)
            pending = [p for p in rectangle['points'] if not any(r['index'] == p['index'] for r in session['accepted_points'])]
            if pending:
                target_point = [*pending[0]['xy_base_m'], clearance]
                report['next_action'] = 'horizontal_at_clearance'
        report['next_target_pen_tip_base_m'] = list(target_point)
        report['next_model_gate'] = probe.model_gate(q, tcp, tcp_target(target_point, orientation, offset, tip), offset)
        report['status'] = 'passed'
        print('只读预检通过：离纸 %.2f mm；30013 连接次数 %d；无运动。' %
              (report['estimated_gap_mm'], reader.reconnects))
    except BaseException as exc:
        report['status'] = 'failed'; report['error'] = str(exc)
        raise
    finally:
        if force is not None:
            force.close()
        if reader is not None:
            reader.close()
        save(path, report)
        print('CHECK:', path)


def self_test():
    corners = [[0.4, -0.3, 0.01], [0.6, -0.1, 0.012]]
    rectangle = rectangle_points(corners)
    xy = np.asarray([p['xy_base_m'] for p in rectangle['points']])
    xyz = np.column_stack((xy, .01 * xy[:, 0] - .02 * xy[:, 1] + .015))
    result = fit_plane(xyz)
    assert result['passed'] and abs(result['a'] - .01) < 1e-10
    assert abs(result['b'] + .02) < 1e-10
    assert np.all(xy > np.asarray(rectangle['outer_min_xy_m']))
    assert np.all(xy < np.asarray(rectangle['outer_max_xy_m']))
    bad = xyz.copy(); bad[10, 2] += .003
    assert not fit_plane(bad)['passed']
    collinear = xyz.copy(); collinear[:, 1] = .2 * collinear[:, 0]
    assert not fit_plane(collinear)['passed']
    tip = np.array([.0001, -.0016, .329])
    offset = np.r_[.01, .02, .03, .1, -.1, .2]
    desired = np.array([.5, -.2, .01])
    tcp = tcp_target(desired, [np.pi, 0, 0], offset, tip)
    assert np.allclose(probe.tip_position(tcp, offset, tip), desired)
    print('SELF-TEST PASS: interior 9+3 grid, tilted plane, independent outlier rejection, collinear rejection, TCP target conversion')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.1.3')
    parser.add_argument('--tcp-calibration', type=Path, default=ROOT / 'outputs/tcp_calibration/ballpoint-pen-20261016.json')
    parser.add_argument('--output', type=Path, default=FOLDER / ('paper-plane-' + dt.datetime.now().strftime('%Y%m%d') + '.json'))
    parser.add_argument('--resume-if-exists', action='store_true')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--fit-only', action='store_true')
    parser.add_argument('--check', action='store_true', help='read-only current state/force/next path check; no motion')
    parser.add_argument('--non-interactive', action='store_true', help='execute an existing taught region without prompts; invalid start still aborts')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test(); return
    output = args.output.resolve()
    if args.fit_only:
        session = json.loads(output.read_text())
        validate_session(session, require_complete=True)
        ordered = sorted(session['accepted_points'], key=lambda p: p['index'])
        session['result'] = fit_plane([p['pen_tip_base_m'] for p in ordered])
        session['status'] = 'validated' if session['result']['passed'] else 'validation_failed'
        save(output, session)
        print(json.dumps(session['result'], ensure_ascii=False, indent=2)); return
    calibration_bytes = args.tcp_calibration.read_bytes()
    calibration = json.loads(calibration_bytes)
    if not calibration.get('result', {}).get('passed'):
        raise RuntimeError('圆珠笔 TCP 尚未通过验证')
    tip = np.asarray(calibration['result']['flange_to_pen_tip_m'])
    fingerprint = hashlib.sha256(calibration_bytes).hexdigest()
    if output.exists():
        if not args.resume_if_exists:
            parser.error('文件已存在，请加 --resume-if-exists 或换 --output')
        session = json.loads(output.read_text())
        if session.get('kind') != 'two_corner_xy_rectangle_force_paper_plane':
            raise RuntimeError('所选续采文件不是当前两角示教纸面会话')
        if session['tcp_calibration_sha256'] != fingerprint:
            raise RuntimeError('TCP 标定文件改变了，不能混用旧纸面采样')
        validate_session(session)
        if session.get('result') is not None:
            print(json.dumps(session['result'], ensure_ascii=False, indent=2)); return
    else:
        session = {'kind': 'two_corner_xy_rectangle_force_paper_plane',
                   'created_at': dt.datetime.now().astimezone().isoformat(),
                   'tcp_calibration': str(args.tcp_calibration), 'tcp_calibration_sha256': fingerprint,
                   'flange_to_pen_tip_m': tip.tolist(), 'corners': [], 'accepted_points': [],
                   'travel_log': [], 'attempts': [], 'status': 'teaching',
                   'paper_xy_alignment': 'operator confirmed parallel to base XY',
                   'training_count': 9, 'holdout_count': 3}
        save(output, session)
    if args.check:
        read_only_check(session, args, tip)
        return
    print('先示教两个对角：笔尖轻触纸面两角，纸和笔安装保持不变。q 保存退出。')
    if args.non_interactive and len(session['corners']) != 2:
        raise RuntimeError('非交互模式需要已示教的两个纸角')
    while len(session['corners']) < 2:
        answer = input('对角点 %d/2 就位并停稳后按 Enter；q 退出 > ' % (len(session['corners']) + 1))
        if answer.strip().lower() == 'q':
            return
        sample = capture(args.host, 30011, 2)
        if sample['max_motion_mm'] > .2 or sample['max_motion_deg'] > .1:
            print('未停稳，请重试。'); continue
        pose = np.asarray(sample['flange_pose_m_rad'])
        point = pose[:3] + Rotation.from_rotvec(pose[3:]).apply(tip)
        session['corners'].append({'pen_tip_base_m': point.tolist(), 'measurement': sample})
        save(output, session)
        print('已记录笔尖 XYZ(mm)：', np.round(point * 1000, 3).tolist())
    corners = [c['pen_tip_base_m'] for c in session['corners']]
    if abs(corners[0][2] - corners[1][2]) > .002:
        raise RuntimeError('两对角高度差超过 2 mm；当前初始近水平假设不足，需增加第三角示教')
    if not session.get('rectangle') or not session['accepted_points']:
        session['rectangle'] = rectangle_points(corners)
    validate_session(session)
    session['status'] = 'planned'; save(output, session)
    print('自动采样范围 XY(mm)：', np.round(np.asarray([
        session['rectangle']['inner_min_xy_m'], session['rectangle']['inner_max_xy_m']]) * 1000, 2).tolist())
    for point in session['rectangle']['points']:
        print('%02d %-8s XY(mm)=%s' % (point['index'], point['role'],
                                      np.round(np.asarray(point['xy_base_m']) * 1000, 2).tolist()))
    if not args.execute:
        print('只读规划完成；加 --execute --resume-if-exists 进入自动采集。OUTPUT:', output)
        return
    print('保持原笔姿态，笔尖只需离纸 2–100 mm；程序会自动接近，无需精确手调 3–5 mm。')
    answer = '' if args.non_interactive else input('核对以上范围和点位，准备好后 Enter 开始连续自动采集；q 保存退出 > ')
    if answer.strip().lower() == 'q':
        return
    reader, force = None, None
    motion_possible = False
    try:
        info = capture(args.host, 30011, 1)
        offset = np.asarray(info['active_tcp_offset_m_rad'])
        reader = probe.RealtimeReader(args.host, 30013)
        prepared = prepare_start(reader, offset, tip, session, interactive=not args.non_interactive)
        if prepared is None:
            session['status'] = 'awaiting_start'; save(output, session)
            return
        q, tcp, point = prepared
        if 'tcp_orientation_rotvec' in session:
            orientation = np.asarray(session['tcp_orientation_rotvec'])
            if (Rotation.from_rotvec(tcp[3:]).inv() * Rotation.from_rotvec(orientation)).magnitude() > np.deg2rad(.2):
                raise RuntimeError('续采时笔姿态改变，请恢复原姿态')
            if not np.allclose(offset, session['active_tcp_offset_m_rad'], atol=1e-9, rtol=0):
                raise RuntimeError('续采时控制器 TCP 设置改变')
        else:
            orientation = tcp[3:].copy()
            session['tcp_orientation_rotvec'] = orientation.tolist()
            session['active_tcp_offset_m_rad'] = offset.tolist()
        tilt = math.degrees(math.acos(float(np.clip(-Rotation.from_rotvec(probe.flange_pose(tcp, offset)[3:]).apply([0, 0, 1])[2], -1, 1))))
        if tilt > 20:
            raise RuntimeError('笔倾斜超过 20°，请先调为接近竖直')
        low = np.asarray(session['rectangle']['outer_min_xy_m'])
        high = np.asarray(session['rectangle']['outer_max_xy_m'])
        if np.any(point[:2] < low - .005) or np.any(point[:2] > high + .005):
            raise RuntimeError('起点 XY 不在已示教纸面范围内')
        session['robot_identity'] = probe.controller_ready(args.host)
        force = probe.RawForce()
        session.setdefault('run_history', []).append({
            'started_at': dt.datetime.now().astimezone().isoformat(),
            'previous_error': session.pop('error', None), 'script_revision': 9,
            'start_pen_tip_base_m': point.tolist()})
        session['status'] = 'collecting'; save(output, session)
        for target in session['rectangle']['points']:
            if any(p['index'] == target['index'] for p in session['accepted_points']):
                continue
            print('自动点 %d/12 (%s)' % (target['index'], target['role']), flush=True)
            _, _, current_tip = state_tip(reader, offset, tip)
            known_z = [c[2] for c in corners] + [r['pen_tip_base_m'][2] for r in session['accepted_points']]
            clearance_z = max(max(known_z) + .020, float(current_tip[2]))
            hover_z = estimate_height(session, target['xy_base_m']) + .003
            if hover_z > clearance_z - .005:
                raise RuntimeError('估计纸面与横移高度余量不足')
            motion_possible = True
            travel(reader, force, args.host, offset, tip,
                   [*current_tip[:2], clearance_z], orientation, session['travel_log'],
                   on_progress=lambda: save(output, session))
            travel(reader, force, args.host, offset, tip,
                   [*target['xy_base_m'], clearance_z], orientation, session['travel_log'],
                   allow_transient_pause=True, on_progress=lambda: save(output, session))
            travel(reader, force, args.host, offset, tip,
                   [*target['xy_base_m'], hover_z], orientation, session['travel_log'],
                   on_progress=lambda: save(output, session))
            stamp = dt.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
            contact_path = output.parent / (output.stem + '-points') / ('point-%02d-%s.json' % (target['index'], stamp))
            session['attempts'].append({'index': target['index'], 'measurement': str(contact_path), 'status': 'started'})
            save(output, session)
            # Release the parent's idle realtime connection while the probe
            # owns state acquisition; resume with a clean connection afterward.
            reader.close()
            reader = None
            probe.close_commands()
            contact = run_contact(args.host, args.tcp_calibration, contact_path)
            reader = probe.RealtimeReader(args.host, 30013)
            actual = np.asarray(contact['contact_pen_tip_base_m'])
            if np.linalg.norm(actual[:2] - target['xy_base_m']) > .0005:
                raise RuntimeError('实测接触 XY 偏离计划超过 0.5 mm')
            session['accepted_points'].append({'index': target['index'], 'role': target['role'],
                                              'pen_tip_base_m': actual.tolist(), 'measurement': str(contact_path)})
            session['attempts'][-1]['status'] = 'accepted'
            save(output, session)
            print('已保存点 %d：XYZ(mm)=%s' % (target['index'], np.round(actual * 1000, 3).tolist()), flush=True)
        ordered = sorted(session['accepted_points'], key=lambda p: p['index'])
        validate_session(session, require_complete=True)
        session['result'] = fit_plane([p['pen_tip_base_m'] for p in ordered])
        session['status'] = 'validated' if session['result']['passed'] else 'validation_failed'
        save(output, session)
        print(json.dumps(session['result'], ensure_ascii=False, indent=2))
    except BaseException as exc:
        if motion_possible:
            try:
                probe.stop(args.host)
            except OSError as stop_error:
                session['stop_command_error'] = str(stop_error)
        session['status'] = 'stopped_on_error'
        session['error'] = str(exc)
        if session['attempts'] and session['attempts'][-1]['status'] == 'started':
            session['attempts'][-1]['status'] = 'failed'
            session['attempts'][-1]['error'] = str(exc)
        save(output, session)
        raise
    finally:
        if force is not None:
            force.close()
        if reader is not None:
            reader.close()
        probe.close_commands()
        print('OUTPUT:', output, flush=True)


if __name__ == '__main__':
    main()
