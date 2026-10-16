#!/usr/bin/env python3
"""Bounded, low-speed force-triggered paper height probe at the current XY.

Default mode is read-only. Execute only from a pen tip 2--5 mm above paper.
Descent uses finite <=0.1 mm steps, followed by a finite 2 mm release.
Realtime recovery always stops before checking and retrying the same endpoint.
"""
import argparse
import atexit
from collections import deque
import datetime as dt
import json
import math
from pathlib import Path
import re
import select
import socket
import struct
import sys
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from calibrate_pen_tcp import capture, flange_pose, save
from paper_ur_support import RealtimeReader, ScriptConnection, decode_frame, FRAME_MIN_SIZE, FRAME_MAX_SIZE, dashboard

LOWER = np.deg2rad([-360, -360, -180, -360, -360, -360])
UPPER = -LOWER
ROBOT_SERIAL = '2022300602'
CALIBRATION_HASH = 'calib_15120592593058779304'
VERTICAL_ARRIVAL_TOLERANCE_M = .000075
_command_connections = {}


class RawForce:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import WrenchStamped
        self.rclpy = rclpy
        rclpy.init(args=[])
        self.node = rclpy.create_node('pen_paper_force_probe')
        self.lock = threading.Lock()
        self.rows = deque(maxlen=50000)
        def callback(msg):
            w = msg.wrench
            values = [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]
            source = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with self.lock:
                self.rows.append((time.monotonic(), time.time(), source, values))
        self.sub = self.node.create_subscription(WrenchStamped, '/ft_sensor/wrench_raw', callback, 200)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def recent(self):
        with self.lock:
            rows = list(self.rows)[-10:]
        if not rows or time.monotonic() - rows[-1][0] > 0.15:
            raise RuntimeError('raw force stream stale >150 ms')
        age = rows[-1][1] - rows[-1][2]
        if not -0.05 <= age <= 0.15:
            raise RuntimeError('raw force source timestamp stale or invalid')
        if not np.all(np.isfinite([r[3] for r in rows])):
            raise RuntimeError('nonfinite force measurement')
        return rows

    def baseline(self):
        until = time.monotonic() + 10
        while time.monotonic() < until:
            try:
                self.recent()
                break
            except RuntimeError:
                time.sleep(0.1)
        else:
            raise RuntimeError('no fresh ROS force stream within 10 s')
        attempts = []
        for attempt in range(3):
            try:
                median, stats = self._baseline_window()
                stats['settle_attempts'] = attempts
                return median, stats
            except BaselineNoiseError as exc:
                attempts.append(exc.diagnostics)
                if attempt == 2:
                    raise
                time.sleep(.3)

    def _baseline_window(self):
        start = time.monotonic()
        time.sleep(2)
        self.recent()
        with self.lock:
            rows = [r for r in self.rows if r[0] >= start]
        if len(rows) < 200:
            raise RuntimeError('force rate too low for contact probe')
        values = np.asarray([r[3] for r in rows])
        if not np.all(np.isfinite(values)) or len(np.unique(values, axis=0)) < 20:
            raise RuntimeError('force stream is invalid or appears frozen')
        median = np.median(values, axis=0)
        raw_noise = float(np.quantile(np.linalg.norm(values[:, :3] - median[:3], axis=1), 0.99))
        # Evaluate the same five-sample median used by force_level().
        filtered = np.asarray([np.median(values[i-4:i+1, :3], axis=0)
                               for i in range(4, len(values))])
        noise = float(np.quantile(np.linalg.norm(filtered - median[:3], axis=1), 0.99))
        stats = {'count': len(rows), 'median_n_nm': median.tolist(),
                 'std_n_nm': values.std(axis=0).tolist(), 'filtered_noise_p99_n': noise,
                 'raw_noise_p99_n': raw_noise}
        if noise > 0.3 or np.max(values[:, :3].std(axis=0)) > 0.15:
            raise BaselineNoiseError('baseline force noise too high: filtered p99 %.3f N, max std %.3f N' %
                                     (noise, np.max(values[:, :3].std(axis=0))), stats)
        return median, stats

    def close(self):
        self.rclpy.shutdown()
        self.thread.join(timeout=2)
        self.node.destroy_node()


def command(host, text):
    if host not in _command_connections:
        _command_connections[host] = ScriptConnection(host)
    try:
        _command_connections[host].send(text)
    except BaseException:
        _command_connections.pop(host).close()
        raise


def close_commands():
    while _command_connections:
        _, connection = _command_connections.popitem()
        connection.close()


atexit.register(close_commands)


def stop(host):
    command(host, 'stopl(0.05)\n')


def fresh(reader, reconnect=False, max_wait_seconds=3):
    """Drain a persistent stream to its newest complete frame.

    Do not reconnect on every stationary read: immediate close/reopen can
    delay the first frame. Old buffered frames alone never qualify as fresh.
    """
    if reconnect:
        reader.connect(timeout_seconds=2)
    reader.sock.settimeout(0.1)
    deadline = time.monotonic() + max_wait_seconds
    newest, received_new_bytes, fresh_complete = None, False, False
    fresh_since = None
    while time.monotonic() < deadline:
        while len(reader.buffer) >= 4:
            size = struct.unpack_from('>I', reader.buffer)[0]
            if not FRAME_MIN_SIZE <= size <= FRAME_MAX_SIZE:
                raise RuntimeError('invalid realtime state frame while draining stream')
            if len(reader.buffer) < size:
                break
            newest = bytes(reader.buffer[:size])
            del reader.buffer[:size]
            fresh_complete = received_new_bytes
            if fresh_complete and fresh_since is None:
                fresh_since = time.monotonic()
        # A busy stream need not become empty. Bound draining after obtaining
        # a new complete frame, instead of timing out despite live data.
        if fresh_complete and time.monotonic() - fresh_since >= .002:
            break
        readable, _, _ = select.select([reader.sock], [], [],
                                      0 if fresh_complete else 0.1)
        if not readable:
            if fresh_complete:
                break
            continue
        chunk = reader.sock.recv(65536)
        if not chunk:
            raise ConnectionError('UR realtime stream closed')
        reader.buffer.extend(chunk)
        received_new_bytes = True
    else:
        if not fresh_complete:
            raise TimeoutError('no fresh UR realtime frame within %.2f seconds' % max_wait_seconds)
    reader.last_frame = newest
    q, tcp, speed = decode_frame(newest)
    if not np.all(np.isfinite(np.r_[q, tcp, speed])):
        raise RuntimeError('nonfinite UR state')
    return np.asarray(q), np.asarray(tcp), np.asarray(speed)


def tip_position(tcp, active_offset, tip):
    pose = flange_pose(tcp, active_offset)
    return pose[:3] + Rotation.from_rotvec(pose[3:]).apply(tip)


def stationary_state(reader, timeout=3):
    """Average nine fresh, quiet frames before a stationary path preflight.

    The movement gate still compares positions at 0.1 mm. Averaging both
    observations prevents a single encoder/FK fluctuation from looking like
    movement while retaining rejection of actual drift or continuing motion.
    """
    deadline = time.monotonic() + timeout
    window = deque(maxlen=9)
    last_received = None
    while time.monotonic() < deadline:
        # No motion has been commanded here. A newly opened socket may need
        # longer than the moving watchdog to publish its first complete frame.
        wait = min(3., max(.001, deadline - time.monotonic()))
        q, tcp, velocity = fresh(reader, max_wait_seconds=wait)
        received = time.monotonic()
        if last_received is not None and received - last_received > .15:
            window.clear()
        last_received = received
        if np.linalg.norm(velocity[:3]) > .0001 or np.linalg.norm(velocity[3:]) > .001:
            window.clear()
            continue
        window.append((q.copy(), tcp.copy(), velocity.copy()))
        if len(window) < 9:
            continue
        positions = np.asarray([x[1][:3] for x in window])
        if np.max(np.linalg.norm(positions - positions.mean(axis=0), axis=1)) > .0001:
            continue
        rotations = Rotation.from_rotvec([x[1][3:] for x in window])
        orientation = rotations.mean()
        if np.max((orientation.inv() * rotations).magnitude()) > np.deg2rad(.05):
            continue
        return (np.mean([x[0] for x in window], axis=0),
                np.r_[positions.mean(axis=0), orientation.as_rotvec()],
                np.mean([x[2] for x in window], axis=0))
    raise RuntimeError('robot did not provide a stable stationary window within %.1f s' % timeout)


def controller_ready(host):
    mode, safety, program = (dashboard(query, ip=host)
                             for query in ('robotmode', 'safetystatus', 'programState'))
    if mode != 'Robotmode: RUNNING' or safety != 'Safetystatus: NORMAL' or not isinstance(program, str) or not program.startswith('STOPPED'):
        raise RuntimeError('controller not RUNNING/NORMAL/STOPPED: %s / %s / %s' % (mode, safety, program))
    model = dashboard('get robot model', ip=host)
    serial = dashboard('get serial number', ip=host)
    if model != 'UR10' or serial != ROBOT_SERIAL:
        raise RuntimeError('expected UR10 CB3 serial %s, received %s / %s' % (ROBOT_SERIAL, model, serial))
    factory = ROOT / 'config/ur10_factory_calibration.yaml'
    match = re.search(r'\bhash:\s*(\S+)', factory.read_text())
    if match is None or match[1] != CALIBRATION_HASH:
        raise RuntimeError('factory calibration differs from the local ROS 2 UR10 identity')
    return {'robotmode': mode, 'safetystatus': safety, 'programState': program,
            'host': host, 'model': model, 'serial': serial, 'base_frame': 'base',
            'factory_calibration_hash': CALIBRATION_HASH}


def model_gate(q, tcp, target, offset):
    # Hardware-only dependencies are required for every real path gate, while
    # --help, --self-test and numerical/fault tests remain hardware independent.
    try:
        import pinocchio as pin
    except ModuleNotFoundError as exc:
        if exc.name != 'pinocchio':
            raise
        # jx's interactive ~/.bashrc adds the existing robotpkg installation.
        # SSH and child processes must load it too; no dependency is installed
        # here and the path/collision checks are never bypassed.
        robotpkg = Path('/opt/openrobots/lib/python%d.%d/site-packages' % sys.version_info[:2])
        if not (robotpkg / 'pinocchio').is_dir():
            raise RuntimeError('Pinocchio is required for path checks; source the existing robotpkg Python environment') from exc
        sys.path.append(str(robotpkg))
        import pinocchio as pin
    from check_move_plan import BASE_FROM_URDF_ROOT, check_segment, robot_pose
    from self_collision import SelfCollisionModel
    from ur_pose_ik import UR10IK
    ik = UR10IK()
    flange = flange_pose(tcp, offset)
    end = flange_pose(target, offset)
    error = pin.log6((BASE_FROM_URDF_ROOT * ik.pose(q)).inverse() * robot_pose(flange)).vector
    if np.linalg.norm(error[:3]) > 0.01 or np.linalg.norm(error[3:]) > 0.02:
        raise RuntimeError('controller flange disagrees with model')
    margin = float(np.rad2deg(min(np.min(q - LOWER), np.min(UPPER - q))))
    if margin < 5:
        raise RuntimeError('live joint limit margin below 5 degrees')
    gate = check_segment(ik, robot_pose(flange), robot_pose(end), q, 30, SelfCollisionModel(), 0.02)
    if gate['reasons'] or gate['worst']['joint_margin_rad'] < np.deg2rad(5) or gate['worst']['min_singular'] < 0.02:
        raise RuntimeError('model descent gate failed: %s' % gate['reasons'][:3])
    return gate['worst']


class ForceGuardError(RuntimeError):
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


class BaselineNoiseError(RuntimeError):
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


class MotionGuardError(RuntimeError):
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = diagnostics


def force_snapshot(force, baseline=None, seconds=5):
    with force.lock:
        recent = [row for row in force.rows if row[0] >= time.monotonic() - seconds]
    result = {'frames': [{'received_unix_s': row[1], 'source_unix_s': row[2],
                          'wrench_n_nm': row[3]} for row in recent]}
    if baseline is not None:
        result['baseline_n_nm'] = np.asarray(baseline).tolist()
    return result


def force_level(force, baseline):
    rows = force.recent()
    raw = np.asarray([r[3][:3] for r in rows]) - baseline[:3]
    peak = float(np.linalg.norm(raw, axis=1).max())
    level = float(np.linalg.norm(np.median(raw[-5:], axis=0)))
    if peak >= 5:
        raise ForceGuardError('force delta %.3f N reached 5 N overload gate (filtered %.3f N)' %
                              (peak, level), {'raw_peak_delta_n': peak, 'filtered_delta_n': level,
                                             'baseline_n_nm': np.asarray(baseline).tolist(),
                                             'latest_wrench_n_nm': rows[-1][3],
                                             'latest_delta_xyz_n': raw[-1].tolist()})
    return level


def finite_move_timeout(speed, acceleration, max_distance=.0025):
    """Conservative trapezoidal motion time plus startup/settling allowance."""
    if speed <= 0 or acceleration <= 0 or max_distance <= 0:
        raise ValueError('motion timing inputs must be positive')
    return max(5., max_distance / speed + speed / acceleration + 3.)


def finite_movel_program(name, target, acceleration, speed):
    """One complete URScript program; its outer def is executed by URControl.

    Do not append a second top-level function call after the terminating end.
    """
    target = np.asarray(target, dtype=float)
    if (target.shape != (6,) or not np.all(np.isfinite(target))
            or not math.isfinite(acceleration) or not math.isfinite(speed)
            or acceleration <= 0 or speed <= 0 or not name.isidentifier()):
        raise ValueError('invalid finite movel program')
    return ('def %s():\n  movel(p[%s], a=%.6f, v=%.7f)\nend\n') % (
        name, ','.join('%.9f' % value for value in target), acceleration, speed)


def motion_state(reader, force, baseline, host, target, script, recovery):
    """Stop before reconnecting; only resume the same bounded endpoint."""
    try:
        return fresh(reader, max_wait_seconds=.25)
    except (TimeoutError, ConnectionError, OSError):
        stop(host)
        if len(recovery) >= 2:
            raise RuntimeError('realtime stream interrupted repeatedly in one finite move')
        recovery.append({'stopped_before_reconnect': True})
        reader.connect(timeout_seconds=2)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            force_level(force, baseline)
            # The robot has already been stopped. A newly opened realtime
            # socket can take longer than the moving-state 250 ms watchdog
            # to publish its first frame; this wait never permits blind motion.
            q, actual, velocity = fresh(reader, max_wait_seconds=2)
            if np.linalg.norm(actual[:3] - target[:3]) > .0025:
                raise RuntimeError('reconnect state outside bounded move envelope')
            if (Rotation.from_rotvec(actual[3:]).inv() * Rotation.from_rotvec(target[3:])).magnitude() > np.deg2rad(.2):
                raise RuntimeError('orientation changed during realtime interruption')
            if min(np.min(q - LOWER), np.min(UPPER - q)) < np.deg2rad(3):
                raise RuntimeError('reconnect joint margin below 3 degrees')
            if np.linalg.norm(velocity[:3]) < .00005:
                # Do not continue a descent into an already detected contact.
                if force_level(force, baseline) >= 1:
                    raise RuntimeError('contact force present after realtime interruption; stopped')
                # A completed endpoint does not need a second motion command.
                reissue = np.linalg.norm(actual[:3] - target[:3]) > .00005
                recovery[-1]['command_reissued'] = bool(reissue)
                recovery[-1]['remaining_distance_mm'] = float(np.linalg.norm(actual[:3] - target[:3]) * 1000)
                if reissue:
                    command(host, script)
                print('实时流中断：已停止、重连并核对同一有限小段。', flush=True)
                return q, actual, velocity
        raise RuntimeError('robot did not settle after realtime interruption')


def wait_stopped(reader, force, baseline, diagnostics, timeout=3, settle_seconds=.15):
    """Observe a stopped contact without sending motion or changing its baseline.

    stopl decelerates; returning from its socket send does not prove zero speed.
    Require a continuous quiet window instead of sleeping .1 s and judging one
    frame. A missing frame, overload or expired deadline remains a hard error.
    """
    deadline = time.monotonic() + timeout
    quiet_since = None
    positions = deque(maxlen=9)
    while time.monotonic() < deadline:
        level = force_level(force, baseline)
        q, tcp, velocity = fresh(reader, max_wait_seconds=.25)
        now = time.monotonic()
        diagnostics.append({'force_delta_n': level, 'controller_tcp_m_rad': tcp.tolist(),
                            'velocity_m_rad_s': velocity.tolist(), 'observed_monotonic_s': now})
        if min(np.min(q - LOWER), np.min(UPPER - q)) < np.deg2rad(3):
            raise RuntimeError('joint margin below 3 degrees while waiting for contact stop')
        if np.linalg.norm(velocity[:3]) < .0001 and np.linalg.norm(velocity[3:]) < .001:
            quiet_since = now if quiet_since is None else quiet_since
            positions.append(tcp[:3].copy())
            if len(positions) == 9 and np.max(np.linalg.norm(np.asarray(positions) - tcp[:3], axis=1)) > .0001:
                quiet_since = now
                positions.clear()
            if len(positions) == 9 and now - quiet_since >= settle_seconds:
                return q, tcp, velocity
        else:
            quiet_since = None
            positions.clear()
    raise RuntimeError('robot did not settle within %.1f s after contact stop' % timeout)


def move_step(reader, force, baseline, host, target, speed, contact_allowed):
    script = finite_movel_program('pen_paper_step', target, .002, speed)
    force_level(force, baseline)
    reader.sock.settimeout(0.1)
    command(host, script)
    timeout = finite_move_timeout(speed, .002)
    deadline, contact_since, settled_since = time.monotonic() + timeout, None, None
    position_window = deque(maxlen=9)
    state_trace = deque(maxlen=32)
    position_error, filtered = float('inf'), None
    recovery = []
    while time.monotonic() < deadline:
        level = force_level(force, baseline)
        now = time.monotonic()
        if contact_allowed and level >= 1:
            contact_since = now if contact_since is None else contact_since
            if now - contact_since >= 0.08:
                stop(host)
                return True
        else:
            contact_since = None
        recovery_count = len(recovery)
        _, actual, velocity = motion_state(reader, force, baseline, host, target, script, recovery)
        if len(recovery) > recovery_count:
            deadline = time.monotonic() + timeout
            position_window.clear()
            settled_since, contact_since = None, None
        position_window.append(np.asarray(actual[:3]))
        error = Rotation.from_rotvec(actual[3:]).inv() * Rotation.from_rotvec(target[3:])
        xy_error = math.dist(actual[:2], target[:2])
        rotation_error_deg = float(np.rad2deg(error.magnitude()))
        state_trace.append({'tcp_m_rad': actual.tolist(), 'velocity_m_rad_s': velocity.tolist(),
                            'xy_error_mm': xy_error * 1000, 'rotation_error_deg': rotation_error_deg,
                            'force_delta_n': level})
        if xy_error > 0.0002 or rotation_error_deg > .1:
            raise MotionGuardError('vertical probe drift: XY %.3f mm / orientation %.4f deg' %
                                   (xy_error * 1000, rotation_error_deg),
                                   {'target_tcp_m_rad': np.asarray(target).tolist(), 'state_trace': list(state_trace),
                                    'realtime_recoveries': recovery})
        # A saved stationary 32-frame trace had a -0.0504 mm median residual,
        # 0.0106 mm raw std and zero velocity. Require nine fresh frames and
        # 0.15 s quiet within 0.075 mm, still below the 0.1 mm target increment.
        # This is a vertical move. XY is checked independently at 0.2 mm
        # above; adding its harmless residual to Z makes the vertical
        # arrival gate impossible to meet near its boundary.
        filtered = np.mean(position_window, axis=0)
        position_error = (float(abs(filtered[2] - target[2]))
                          if len(position_window) == 9 else float('inf'))
        if position_error < VERTICAL_ARRIVAL_TOLERANCE_M and np.linalg.norm(velocity[:3]) < 0.00005:
            settled_since = now if settled_since is None else settled_since
            if now - settled_since >= 0.15:
                return False
        else:
            settled_since = None
    raise MotionGuardError('finite vertical move did not settle; filtered Z error %.3f mm' %
                           (position_error * 1000),
                           {'target_tcp_m_rad': np.asarray(target).tolist(),
                            'filtered_position_m': filtered.tolist() if filtered is not None else None,
                            'state_trace': list(state_trace), 'realtime_recoveries': recovery})


def self_test():
    from calibrate_pen_tcp import self_test as tcp_self_test
    tcp_self_test()
    offset = np.r_[0.01, 0.02, 0.03, 0.2, -0.1, 0.1]
    flange = np.r_[0.5, -0.2, 0.33, np.pi, 0, 0]
    r = Rotation.from_rotvec(flange[3:])
    tcp = np.r_[flange[:3] + r.apply(offset[:3]), (r * Rotation.from_rotvec(offset[3:])).as_rotvec()]
    tip = np.array([0, 0, 0.329])
    assert np.allclose(tip_position(tcp, offset, tip), [0.5, -0.2, 0.001])
    class TestForce:
        def __init__(self, vector):
            self.vector = vector
        def recent(self):
            return [(0, 0, 0, self.vector)] * 10
    assert abs(force_level(TestForce([1.1, 0, 0, 0, 0, 0]), np.zeros(6)) - 1.1) < 1e-10
    try:
        force_level(TestForce([0, 0, 5.1, 0, 0, 0]), np.zeros(6))
    except RuntimeError:
        pass
    else:
        raise AssertionError('overload gate must reject >=5 N')
    print('PAPER SELF-TEST PASS: active TCP removal and base-frame pen-tip height')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.1.3')
    parser.add_argument('--tcp-calibration', type=Path, default=ROOT / 'outputs/tcp_calibration/ballpoint-pen-20261016.json')
    parser.add_argument('--max-descent-mm', type=float, default=6)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not 0 < args.max_descent_mm <= 6:
        parser.error('max descent must be >0 and <=6 mm')
    calibration = json.loads(args.tcp_calibration.read_text())
    if not calibration.get('result', {}).get('passed'):
        raise RuntimeError('pen TCP calibration has not passed')
    tip = np.asarray(calibration['result']['flange_to_pen_tip_m'])
    output = args.output or ROOT / 'outputs/paper_calibration' / ('height-' + dt.datetime.now().strftime('%Y%m%d-%H%M%S') + '.json')
    if output.exists():
        raise RuntimeError('output exists; use a new output path')
    report = {'kind': 'force_triggered_pen_paper_height', 'created_at': dt.datetime.now().astimezone().isoformat(),
              'tcp_calibration': str(args.tcp_calibration), 'status': 'preflight', 'motion_sent': False,
              'contact_threshold_n': 1.0, 'overload_delta_n': 5.0, 'force_is_baseline_relative_sensor_vector': True,
              'step_mm': 0.1, 'arrival_z_tolerance_mm': VERTICAL_ARRIVAL_TOLERANCE_M * 1000,
              'speed_m_s': 0.0003, 'max_descent_mm': args.max_descent_mm, 'steps': []}
    reader, force = None, None
    try:
        info = capture(args.host, 30011, 1)
        offset = np.asarray(info['active_tcp_offset_m_rad'])
        reader = RealtimeReader(args.host, 30013)
        q, tcp, velocity = fresh(reader, reconnect=False)
        if np.linalg.norm(velocity[:3]) > 0.0001 or np.linalg.norm(velocity[3:]) > 0.001:
            raise RuntimeError('robot is moving before probe')
        tilt = math.degrees(math.acos(float(np.clip(-Rotation.from_rotvec(flange_pose(tcp, offset)[3:]).apply([0, 0, 1])[2], -1, 1))))
        if tilt > 20:
            raise RuntimeError('pen/flange Z tilt %.1f deg exceeds near-vertical 20 deg gate' % tilt)
        report['controller'] = controller_ready(args.host)
        report['start_controller_tcp_m_rad'] = tcp.tolist()
        report['start_pen_tip_base_m'] = tip_position(tcp, offset, tip).tolist()
        report['active_tcp_offset_m_rad'] = offset.tolist()
        report['pen_tilt_deg'] = tilt
        bottom = tcp.copy(); bottom[2] -= args.max_descent_mm / 1000
        report['path_gate'] = model_gate(q, tcp, bottom, offset)
        force = RawForce()
        baseline, report['baseline'] = force.baseline()
        _, rechecked, speed = fresh(reader)
        recheck_rotation = Rotation.from_rotvec(rechecked[3:]).inv() * Rotation.from_rotvec(tcp[3:])
        if (math.dist(rechecked[:3], tcp[:3]) > 0.0001
                or recheck_rotation.magnitude() > np.deg2rad(0.05)
                or np.linalg.norm(speed[:3]) > 0.0001):
            raise RuntimeError('robot moved during preflight')
        if not args.execute:
            report['status'] = 'dry_run_pass'
            save(output, report)
            print('DRY-RUN PASS; no motion; initial pen tip:', report['start_pen_tip_base_m'])
            print('OUTPUT:', output)
            return
        controller_ready(args.host)
        report['status'] = 'probing'; save(output, report)
        contacted = False
        for index in range(int(math.ceil(args.max_descent_mm / 0.1))):
            target = tcp.copy(); target[2] -= min((index + 1) * 0.0001, args.max_descent_mm / 1000)
            report['motion_sent'] = True
            save(output, report)
            contacted = move_step(reader, force, baseline, args.host, target, 0.0003, True)
            _, actual, velocity = fresh(reader)
            report['steps'].append({'index': index + 1, 'controller_tcp_m_rad': actual.tolist(), 'force_delta_n': force_level(force, baseline)})
            save(output, report)
            if contacted:
                break
            if index % 10 == 0:
                print('descent %.1f mm' % ((index + 1) * 0.1), flush=True)
        if not contacted:
            raise RuntimeError('no contact within bounded descent; probe stopped')
        report['stop_settle_frames'] = []
        contact_q, actual, velocity = wait_stopped(reader, force, baseline, report['stop_settle_frames'])
        point = tip_position(actual, offset, tip)
        report['contact_pen_tip_base_m'] = point.tolist()
        report['paper_height_base_mm'] = float(point[2] * 1000)
        report['contact_controller_tcp_m_rad'] = actual.tolist()
        report['height_meaning'] = 'force-triggered loaded pen-tip height at this XY; pen compliance and TCP uncertainty included'
        report['status'] = 'contact_recorded'; save(output, report)
        # Release the contact in finite 0.1 mm upward steps. Fixed orientation
        # keeps gravity unchanged; force overload/stale-data gates stay active.
        start_retract = actual.copy()
        retract_target = actual.copy(); retract_target[2] += .002
        report['retract_model_gate'] = model_gate(contact_q, actual, retract_target, offset)
        # The release is a separate, finite 2 mm upward move. A single smooth
        # release avoids twenty stop/start impulses; force/state guards remain.
        move_step(reader, force, baseline, args.host, retract_target, 0.002, False)
        report['status'] = 'contact_recorded_retracted_2mm'
        print('CONTACT HEIGHT base Z = %.3f mm; retracted 2 mm' % report['paper_height_base_mm'], flush=True)
    except BaseException as exc:
        if report['motion_sent']:
            try:
                stop(args.host)
            except OSError as stop_error:
                report['stop_command_error'] = str(stop_error)
        report['status'] = 'stopped_on_error'
        report['error'] = str(exc)
        if isinstance(exc, ForceGuardError):
            report['force_guard_diagnostics'] = exc.diagnostics
        if isinstance(exc, BaselineNoiseError):
            report['baseline_noise_diagnostics'] = exc.diagnostics
        if isinstance(exc, MotionGuardError):
            report['motion_guard_diagnostics'] = exc.diagnostics
        raise
    finally:
        if force is not None:
            with force.lock:
                report['raw_force_frames'] = [{'received_unix_s': row[1], 'source_unix_s': row[2], 'wrench_n_nm': row[3]} for row in force.rows]
            force.close()
        if reader is not None:
            reader.close()
        close_commands()
        save(output, report)
        print('OUTPUT:', output, flush=True)


if __name__ == '__main__':
    main()
