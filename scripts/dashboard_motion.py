"""Bounded single-step native movel, with explicit browser lease and path gates.

No servo streaming. Compensation is never used as a motion protection signal.
Hardware execution is only reached by a separate armed jog request.
"""
import hashlib
import json
import math
from pathlib import Path
import re
import secrets
import socket
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from calibrate_pen_tcp import cartesian_info, receive_exact
from read_ur_state import dashboard
from record_ur_trajectory import RealtimeReader
from self_collision import SelfCollisionModel
from ur10_nullspace_ik import UR10NullspaceIK


def matrix(pose):
    p = np.asarray(pose, dtype=float)
    if p.shape != (6,) or not np.isfinite(p).all():
        raise ValueError('无效控制器位姿')
    result = np.eye(4); result[:3, :3] = Rotation.from_rotvec(p[3:]).as_matrix(); result[:3, 3] = p[:3]
    return result


def values(transform):
    return np.r_[transform[:3, 3], Rotation.from_matrix(transform[:3, :3]).as_rotvec()].tolist()


def target_step(start, kind, axis, direction, step, frame, translation_limit=5.):
    if kind not in ('translation', 'rotation') or axis not in ('x', 'y', 'z') or frame not in ('base', 'tcp'):
        raise ValueError('无效点动方向或坐标系')
    if type(direction) is not int or direction not in (-1, 1):
        raise ValueError('方向必须是 +1 或 -1')
    step = float(step)
    limit = translation_limit if kind == 'translation' else 2.
    if not math.isfinite(step) or not .5 <= step <= limit:
        raise ValueError('平移请求0.5–%gmm；旋转单步0.5–2°' % translation_limit)
    result = np.asarray(start).copy(); vector = np.eye(3)['xyz'.index(axis)] * direction
    if kind == 'translation':
        result[:3, 3] += (start[:3, :3] @ vector if frame == 'tcp' else vector) * step / 1000
    else:
        delta = Rotation.from_rotvec(vector * np.deg2rad(step)).as_matrix()
        result[:3, :3] = start[:3, :3] @ delta if frame == 'tcp' else delta @ start[:3, :3]
    return result


def compile_step(target, duration):
    pose = ','.join('%.10f' % x for x in target)
    # Controller-side bounded motion thread. This follows the existing native
    # thread cancellation pattern; it is not a hardware-validated emergency stop.
    return ('def dashboard_single_step():\n'
            '  global dashboard_step_done = False\n'
            '  thread dashboard_step_motion():\n'
            '    movel(p[%s], a=0.030000, v=0.005000, r=0.0)\n'
            '    global dashboard_step_done = True\n'
            '  end\n'
            '  handle = run dashboard_step_motion()\n'
            '  elapsed = 0.0\n'
            '  while (not dashboard_step_done) and (elapsed < %.3f):\n'
            '    sleep(0.02)\n'
            '    elapsed = elapsed + 0.02\n'
            '  end\n'
            '  if not dashboard_step_done:\n'
            '    kill handle\n'
            '    stopj(2.0)\n'
            '  else:\n'
            '    join handle\n'
            '  end\n'
            'end\ndashboard_single_step()\n') % (pose, duration)


class RobotIO:
    def __init__(self):
        self.reader = None

    def ready(self):
        state = [dashboard(command) for command in ('robotmode', 'safetystatus', 'programState')]
        if state[:2] != ['Robotmode: RUNNING', 'Safetystatus: NORMAL'] or not (state[2] or '').startswith('STOPPED'):
            raise ValueError('控制器须为 RUNNING / NORMAL，且当前程序已停止：' + str(state))

    def read(self, fresh=False):
        if fresh:
            self.close()
        if self.reader is None:
            self.reader = RealtimeReader('192.168.1.3', 30013)
        return self.reader.read(max_wait_seconds=.5)

    def offset(self):
        import struct
        deadline = time.monotonic() + 2.
        with socket.create_connection(('192.168.1.3', 30011), timeout=2) as sock:
            sock.settimeout(.5)
            while time.monotonic() < deadline:
                header = receive_exact(sock, 4); size = struct.unpack('>I', header)[0]
                if not 5 <= size <= 200000:
                    raise ValueError('无效Primary状态帧')
                info = cartesian_info(header + receive_exact(sock, size - 4))
                if info is not None:
                    return np.asarray(info[1])
        raise TimeoutError('读取当前TCP偏移超时')

    def send(self, script):
        with socket.create_connection(('192.168.1.3', 30002), timeout=1) as sock:
            sock.sendall(script.encode('ascii'))

    def stop(self):
        self.send('stopj(2.0)\n')

    def close(self):
        if self.reader is not None:
            self.reader.close(); self.reader = None


class MotionController:
    def __init__(self, io=None, model_check=None, audit_dir=None):
        self.io = io or RobotIO()
        self.model_check = model_check or (lambda: None)
        self.audit_dir = Path(audit_dir or Path(__file__).resolve().parents[1] / 'outputs/dashboard/motion-jobs')
        self.lock = threading.RLock(); self.stop_lock = threading.Lock()
        self.token = None; self.expires = 0.; self.last_heartbeat = 0.
        self.jobs = {}; self.current = None; self.latest = None

    def _authorize(self, token, heartbeat=False):
        if not isinstance(token, str) or self.token is None or not secrets.compare_digest(token, self.token) or time.monotonic() > self.expires:
            raise ValueError('点动未启用或已过期，请重新启用')
        if heartbeat and time.monotonic() - self.last_heartbeat > 1.:
            raise ValueError('浏览器心跳中断，已停止本次点动')

    def arm(self):
        with self.lock:
            if self.current:
                raise ValueError('已有点动正在校验或运动，请先停止')
        self.model_check(); self.io.ready()
        with self.lock:
            if self.current:
                raise ValueError('已有点动进行中')
            self.token = secrets.token_urlsafe(24); self.last_heartbeat = time.monotonic(); self.expires = self.last_heartbeat + 30
            return {'ok': True, 'token': self.token, 'lease_seconds': 30, 'hardware_validated': False}

    def heartbeat(self, token):
        with self.lock:
            self._authorize(token); self.last_heartbeat = time.monotonic(); self.expires = self.last_heartbeat + 30
        return {'ok': True}

    def status(self):
        with self.lock:
            job = self.jobs.get(self.current or self.latest, {}).copy()
            return {'ok': True, 'observed_at': time.time(), 'busy': self.current is not None, 'armed': self.token is not None and time.monotonic() < self.expires,
                    'hardware_validated': False, 'job': {k: v for k, v in job.items() if k not in ('cancel', 'token')}}

    def stop(self, token, disarm=False):
        with self.lock:
            # A matching expired lease can still stop its own motion.
            if not isinstance(token, str) or self.token is None or not secrets.compare_digest(token, self.token):
                raise ValueError('无效点动会话')
            job = self.jobs.get(self.current)
            if job:
                job['cancel'].set()
            if disarm:
                self.expires = 0.
        if job and job.get('motion_sent'):
            self._stop_motion(job)
        return self.status()

    def jog(self, request):
        token = request.get('token'); request_id = request.get('request_id')
        if not isinstance(request_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{8,64}', request_id):
            raise ValueError('需要唯一点动请求ID')
        # Validate the allowlisted request before any I/O or queue admission.
        target_step(np.eye(4), request.get('kind'), request.get('axis'), request.get('direction'), request.get('step'), request.get('frame'), translation_limit=50.)
        if request.get('kind') == 'translation' and float(request['step']) > 5 and request.get('confirm_large') is not True:
            raise ValueError('超过5mm的平移须先确认本次分段移动')
        with self.lock:
            self._authorize(token, heartbeat=True)
            if request_id in self.jobs:
                return self.status()  # Duplicate clicks never resubmit a move.
            if self.current:
                raise ValueError('已有点动进行中，不接受排队')
            job = {key: request[key] for key in ('kind', 'axis', 'direction', 'step', 'frame')}
            job.update(id=request_id, state='checking', motion_sent=False, stop_sent=False,
                       token=token, cancel=threading.Event(), started_at=time.time())
            self.jobs[request_id] = job; self.current = self.latest = request_id
            if len(self.jobs) > 32:
                self.jobs.pop(next(iter(self.jobs)))
        threading.Thread(target=self._run, args=(job,), daemon=True).start()
        return self.status()

    def _alive(self, job):
        with self.lock:
            self._authorize(job['token'], heartbeat=True)
            if job['cancel'].is_set():
                raise ValueError('用户停止本次点动')

    def _stop_motion(self, job):
        with self.stop_lock:
            if not job['stop_sent']:
                try:
                    self.io.stop(); job['stop_sent'] = True
                except Exception as exc:
                    job['stop_error'] = str(exc)
                    raise RuntimeError('停止命令发送失败，请使用示教器停止') from exc

    def shutdown(self):
        with self.lock:
            self.expires = 0.
            job = self.jobs.get(self.current)
            if job:
                job['cancel'].set()
        if job and job.get('motion_sent'):
            self._stop_motion(job)

    def _gate(self, q, start, target, offset, progress=None):
        ik = UR10NullspaceIK(); collision = SelfCollisionModel()
        flange = start @ np.linalg.inv(offset)
        actual = ik.fk_jacobian(q)[0]
        if (np.linalg.norm(actual[:3, 3] - flange[:3, 3]) > .001 or
                (Rotation.from_matrix(actual[:3, :3]).inv() * Rotation.from_matrix(flange[:3, :3])).magnitude() > .01):
            raise ValueError('当前TCP偏移还原后，控制器位姿与工厂模型不一致')
        orientation = Slerp([0., 1.], Rotation.from_matrix([start[:3, :3], target[:3, :3]]))
        previous = np.asarray(q); seed = previous.copy(); gap = float('inf'); sigma = float('inf'); margin = float('inf')
        for index, fraction in enumerate(np.linspace(0, 1, 25)):
            tcp = np.eye(4); tcp[:3, :3] = orientation(fraction).as_matrix(); tcp[:3, 3] = (1 - fraction) * start[:3, 3] + fraction * target[:3, 3]
            pose = tcp @ np.linalg.inv(offset)
            if min(tcp[2, 3], pose[2, 3]) < .25:
                raise ValueError('TCP或法兰低于0.25m点动门限')
            solved = ik.solve(pose, previous, mode='pose')
            candidate = np.asarray(solved['q'])
            if not solved['success'] or np.max(np.abs(candidate - previous)) > np.deg2rad(.5) or np.max(np.abs(candidate - seed)) > np.deg2rad(5):
                raise ValueError('路径IK或关节连续性门禁未通过')
            margin = min(margin, solved['minimum_joint_margin_deg'])
            sigma = min(sigma, float(np.linalg.svd(ik.fk_jacobian(candidate)[1], compute_uv=False)[-1]))
            gap = min(gap, float(collision.min_clearance(candidate)[0]))
            previous = candidate
            if progress is not None:
                progress(index + 1)
        if margin < 5 or sigma < .02 or gap < .02:
            raise ValueError('路径限位/奇异性/选定碰撞对门禁未通过：%.2f° / %.4f / %.1fmm' % (margin, sigma, gap * 1000))
        return {'sample_count': 25, 'joint_margin_deg': margin, 'minimum_singular': sigma, 'clearance_mm': gap * 1000,
                'end_q_rad': previous.tolist()}

    def _run(self, job):
        try:
            self.audit_dir.mkdir(parents=True, exist_ok=True)
            initial_audit = {k: v for k, v in job.copy().items() if k not in ('cancel', 'token')}
            (self.audit_dir / (job['id'] + '.json')).write_text(json.dumps(initial_audit, ensure_ascii=False) + '\n')
            job['phase'] = 'controller'
            self._alive(job); self.model_check(); self.io.ready()
            job['phase'] = 'tcp'
            offset_values = self.io.offset(); offset = matrix(offset_values)
            q, tcp, velocity = self.io.read(fresh=True); start = matrix(tcp)
            if np.linalg.norm(velocity[:3]) > .002 or np.linalg.norm(velocity[3:]) > .005:
                raise ValueError('机器人尚未静止')
            target = target_step(start, job['kind'], job['axis'], job['direction'], job['step'], job['frame'], translation_limit=50.)
            job['before_tcp_m_rad'] = list(tcp)
            job['target_tcp_m_rad'] = values(target)
            # Reserve room for settling error; cap every actual command at 5mm.
            count = math.ceil(float(job['step']) / 4.5) if job['kind'] == 'translation' and float(job['step']) > 5 else 1
            job.update(segments_total=count, segments_completed=0, segment_index=0)
            targets = []
            for index in range(count):
                segment = target.copy()
                if count > 1:
                    segment[:3, 3] = start[:3, 3] + (target[:3, 3] - start[:3, 3]) * (index + 1) / count
                targets.append(segment)
            if count > 1:
                job.update(phase='plan', checked_samples=0, total_path_samples=count * 25)
                predicted_q, predicted_tcp = np.asarray(q).copy(), start.copy()
                full_gates = []
                for index, segment in enumerate(targets):
                    def planned(n, index=index):
                        self._alive(job); job['checked_samples'] = index * 25 + n
                    checked = self._gate(predicted_q, predicted_tcp, segment, offset, progress=planned)
                    full_gates.append(checked)
                    predicted_q = np.asarray(checked['end_q_rad']); predicted_tcp = segment
                job['full_path_gate'] = {'sample_count': count * 25,
                    'joint_margin_deg': min(g['joint_margin_deg'] for g in full_gates),
                    'minimum_singular': min(g['minimum_singular'] for g in full_gates),
                    'clearance_mm': min(g['clearance_mm'] for g in full_gates)}
            job['script_sha256_segments'] = []
            for index, segment in enumerate(targets):
                job.update(phase='controller', state='checking', segment_index=index + 1)
                self._alive(job); self.model_check(); self.io.ready()
                if not np.allclose(self.io.offset(), offset_values, atol=1e-9, rtol=0):
                    raise ValueError('当前TCP配置变化，取消剩余分段')
                q, tcp, velocity = self.io.read(fresh=True); segment_start = matrix(tcp)
                if np.linalg.norm(velocity[:3]) > .002 or np.linalg.norm(velocity[3:]) > .005:
                    raise ValueError('机器人尚未静止')
                expected = start if index == 0 else targets[index - 1]
                if (np.linalg.norm(segment_start[:3, 3] - expected[:3, 3]) > .0003 or
                        (Rotation.from_matrix(segment_start[:3, :3]).inv() * Rotation.from_matrix(expected[:3, :3])).magnitude() > .0007):
                    raise ValueError('当前位姿偏离已校验路径，取消剩余分段')
                if job['kind'] == 'translation' and np.linalg.norm(segment[:3, 3] - segment_start[:3, 3]) > .005000001:
                    raise ValueError('单段实际位移超过5mm，取消剩余分段')
                job.update(phase='path', checked_samples=0, segment_target_tcp_m_rad=values(segment))
                def progress(n):
                    self._alive(job); job['checked_samples'] = n
                job['gate'] = self._gate(q, segment_start, segment, offset, progress=progress)
                job['phase'] = 'recheck'
                self._alive(job); self.model_check(); self.io.ready()
                if not np.allclose(self.io.offset(), offset_values, atol=1e-9, rtol=0):
                    raise ValueError('路径校验后当前TCP配置变化')
                q_now, tcp_now, speed = self.io.read(fresh=True)
                if (np.max(np.abs(np.asarray(q_now) - q)) > .005 or np.linalg.norm(np.asarray(tcp_now[:3]) - np.asarray(tcp[:3])) > .0002 or
                        (Rotation.from_rotvec(tcp_now[3:]).inv() * Rotation.from_rotvec(tcp[3:])).magnitude() > .001 or
                        np.linalg.norm(speed[:3]) > .002 or np.linalg.norm(speed[3:]) > .005):
                    raise ValueError('路径校验后机器人状态变化，未发送运动')
                if job['kind'] == 'translation' and np.linalg.norm(segment[:3, 3] - np.asarray(tcp_now[:3])) > .005000001:
                    raise ValueError('发送前实际位移超过5mm')
                duration = 10. if job['kind'] == 'translation' else max(10., np.deg2rad(float(job['step'])) / .005 * 2 + 4)
                script = compile_step(values(segment), duration)
                job['script_sha256'] = hashlib.sha256(script.encode()).hexdigest()
                job['script_sha256_segments'].append(job['script_sha256'])
                with self.lock:
                    self._alive(job)
                    job['motion_sent'] = True; job['state'] = 'moving'; job.setdefault('motion_started_at', time.time())
                    self.io.send(script)
                deadline = time.monotonic() + duration; settled = 0
                while time.monotonic() < deadline:
                    self._alive(job)
                    _, after, speed = self.io.read()
                    job['after_tcp_m_rad'] = list(after)
                    delta = np.asarray(after[:3]) - start[:3, 3]
                    after_rotation = Rotation.from_rotvec(after[3:]).as_matrix()
                    if job['frame'] == 'tcp':
                        delta = start[:3, :3].T @ delta
                        rotation_delta = start[:3, :3].T @ after_rotation
                    else:
                        rotation_delta = after_rotation @ start[:3, :3].T
                    job['actual_delta_mm'] = (delta * 1000).tolist()
                    job['actual_rotation_delta_deg'] = np.rad2deg(Rotation.from_matrix(rotation_delta).as_rotvec()).tolist()
                    pe = float(np.linalg.norm(np.asarray(after[:3]) - segment[:3, 3]))
                    re = float((Rotation.from_rotvec(after[3:]).inv() * Rotation.from_matrix(segment[:3, :3])).magnitude())
                    if pe < .0003 and re < .0007 and np.linalg.norm(speed[:3]) < .001 and np.linalg.norm(speed[3:]) < .002:
                        settled += 1
                    else:
                        settled = 0
                    if settled >= 3:
                        job['segments_completed'] = index + 1
                        break
                else:
                    raise TimeoutError('分段运动未在时限内到位，取消剩余分段')
                if index + 1 < count:
                    time.sleep(.06); self._alive(job)
            job.update(state='complete', final_error_mm=pe * 1000, final_error_deg=float(np.rad2deg(re)))
        except Exception as exc:
            job['error'] = str(exc)
            if job['motion_sent']:
                try:
                    self._stop_motion(job)
                except Exception as stop_error:
                    job['error'] += '；' + str(stop_error)
                job['state'] = 'stopped'
            else:
                job['state'] = 'rejected'
        finally:
            self.io.close(); job['finished_at'] = time.time()
            with self.lock:
                self.current = None
                audit = {k: v for k, v in job.copy().items() if k not in ('cancel', 'token')}
            try:
                self.audit_dir.mkdir(parents=True, exist_ok=True)
                (self.audit_dir / (job['id'] + '.json')).write_text(json.dumps(audit, ensure_ascii=False, indent=2) + '\n')
            except OSError:
                pass
