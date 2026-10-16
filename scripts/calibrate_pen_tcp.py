#!/usr/bin/env python3
"""Read-only fixed-point pen TCP calibration using controller flange poses.

Receives Primary CartesianInfo only; sends no robot/gripper commands.
Protocol: https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/primary-secondary-guide.html
"""
import argparse
import datetime as dt
import json
import socket
import struct
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def receive_exact(sock, size):
    parts = bytearray()
    while len(parts) < size:
        chunk = sock.recv(size - len(parts))
        if not chunk:
            raise ConnectionError('controller closed state connection')
        parts.extend(chunk)
    return bytes(parts)


def cartesian_info(frame):
    if len(frame) < 5 or frame[4] != 16:
        return None
    cursor = 5
    while cursor + 5 <= len(frame):
        size, kind = struct.unpack_from('>IB', frame, cursor)
        if size < 5 or cursor + size > len(frame):
            raise ValueError('invalid primary subpacket')
        if kind == 4:
            if size < 101:
                raise ValueError('CartesianInfo has no complete active TCP offset')
            values = np.asarray(struct.unpack_from('>12d', frame, cursor + 5))
            if not np.all(np.isfinite(values)):
                raise ValueError('nonfinite controller pose/offset')
            return values[:6], values[6:]
        cursor += size
    return None


def flange_pose(tcp, offset):
    rotation = Rotation.from_rotvec(tcp[3:]) * Rotation.from_rotvec(offset[3:]).inv()
    position = tcp[:3] - rotation.apply(offset[:3])
    return np.r_[position, rotation.as_rotvec()]


def capture(host, port, duration):
    # Primary read-only sockets can be closed transiently by the controller.
    # Retry the whole stationary window on a fresh connection; never combine
    # frames from a failed window with the next attempt.
    for attempt in range(3):
        try:
            return _capture_once(host, port, duration)
        except (OSError, ConnectionError):
            if attempt == 2:
                raise
            time.sleep(0.2)


def _capture_once(host, port, duration):
    raw, poses = [], []
    # A new receive-only socket on every sample avoids queued old states.
    with socket.create_connection((host, port), timeout=3) as sock:
        sock.settimeout(2)
        deadline = time.monotonic() + duration + 5
        began = None
        while time.monotonic() < deadline:
            head = receive_exact(sock, 4)
            size = struct.unpack('>I', head)[0]
            if not 5 <= size <= 200000:
                raise ValueError('invalid primary message size')
            info = cartesian_info(head + receive_exact(sock, size - 4))
            if info is None:
                continue
            tcp, offset = info
            stamp = time.monotonic()
            if began is None:
                began = stamp
            if raw and not np.allclose(offset, raw[0]['active_tcp_offset_m_rad'], atol=1e-9, rtol=0):
                raise RuntimeError('active TCP changed during capture')
            pose = flange_pose(tcp, offset)
            poses.append(pose)
            raw.append({'received_unix_s': time.time(), 'controller_tcp_pose_m_rad': tcp.tolist(),
                        'active_tcp_offset_m_rad': offset.tolist(), 'flange_pose_m_rad': pose.tolist()})
            if stamp - began >= duration:
                break
    if len(raw) < 10:
        raise RuntimeError('too few fresh CartesianInfo frames: %d' % len(raw))
    poses = np.asarray(poses)
    position = poses[:, :3].mean(axis=0)
    rotations = Rotation.from_rotvec(poses[:, 3:])
    rotation = rotations.mean()
    return {'frame_count': len(raw), 'flange_pose_m_rad': np.r_[position, rotation.as_rotvec()].tolist(),
            'active_tcp_offset_m_rad': raw[0]['active_tcp_offset_m_rad'],
            'max_motion_mm': float(np.linalg.norm(poses[:, :3] - position, axis=1).max() * 1000),
            'max_motion_deg': float(np.rad2deg((rotation.inv() * rotations).magnitude()).max()),
            'raw_frames': raw}


def fit(samples, holdout):
    train, test = samples[:-holdout], samples[-holdout:]
    poses = np.asarray([s['flange_pose_m_rad'] for s in train])
    rotations = Rotation.from_rotvec(poses[:, 3:]).as_matrix()
    centered_r = rotations - rotations.mean(axis=0)
    a = centered_r.reshape(-1, 3)
    b = -(poses[:, :3] - poses[:, :3].mean(axis=0)).reshape(-1)
    tip, _, rank, singular = np.linalg.lstsq(a, b, rcond=None)
    rank = np.linalg.matrix_rank(a, tol=1e-8)
    if rank < 3:
        return {'passed': False, 'reason': '姿态覆盖不足：笔尖位置不能唯一求解', 'rank': int(rank)}
    fixed = np.einsum('nij,j->ni', rotations, tip) + poses[:, :3]
    reference = fixed.mean(axis=0)
    train_error = np.linalg.norm(fixed - reference, axis=1) * 1000
    test_error = []
    for row in test:
        p = np.asarray(row['flange_pose_m_rad'])
        test_error.append(float(np.linalg.norm(Rotation.from_rotvec(p[3:]).apply(tip)
                                               + p[:3] - reference) * 1000))
    condition = float(singular[0] / singular[-1])
    rms = float(np.sqrt(np.mean(train_error ** 2)))
    passed = condition <= 120 and rms <= 0.5 and max(test_error) <= 1.0
    return {'passed': bool(passed), 'flange_to_pen_tip_m': tip.tolist(),
            'flange_to_pen_tip_mm': (tip * 1000).tolist(), 'fixed_point_base_m': reference.tolist(),
            'condition_number': condition, 'rank': int(rank), 'training_rms_mm': rms,
            'training_max_mm': float(train_error.max()), 'holdout_errors_mm': test_error,
            'holdout_max_mm': max(test_error), 'training_count': len(train), 'holdout_count': len(test),
            'orientation_calibrated': False, 'controller_tcp_written': False,
            'meaning': 'fixed-point consistency, not independently measured absolute accuracy'}


def self_test():
    rng = np.random.default_rng(16)
    tip = np.array([0.01, -0.02, 0.27])
    fixed = np.array([0.5, -0.2, 0.1])
    rows = []
    for _ in range(12):
        r = Rotation.from_rotvec(rng.normal(0, 0.6, 3))
        rows.append({'flange_pose_m_rad': np.r_[fixed - r.apply(tip), r.as_rotvec()].tolist()})
    result = fit(rows, 3)
    assert result['passed'] and np.allclose(result['flange_to_pen_tip_m'], tip, atol=1e-10)
    offset = np.r_[0.02, 0.03, 0.1, 0.2, -0.1, 0.3]
    p = np.asarray(rows[0]['flange_pose_m_rad'])
    r = Rotation.from_rotvec(p[3:])
    tcp = np.r_[p[:3] + r.apply(offset[:3]), (r * Rotation.from_rotvec(offset[3:])).as_rotvec()]
    recovered = flange_pose(tcp, offset)
    assert np.allclose(recovered[:3], p[:3], atol=1e-10)
    assert (Rotation.from_rotvec(recovered[3:]).inv() * r).magnitude() < 1e-10
    sub = struct.pack('>IB12d', 101, 4, *tcp, *offset)
    frame = struct.pack('>IB', len(sub) + 5, 16) + sub
    assert np.allclose(cartesian_info(frame)[1], offset)
    assert not fit([rows[0]] * 12, 3)['passed']
    print('SELF-TEST PASS: known TCP recovery, active-offset conversion, packet parsing, degenerate poses')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.1.3')
    parser.add_argument('--port', type=int, default=30011)
    parser.add_argument('--samples', type=int, default=12)
    parser.add_argument('--holdout', type=int, default=3)
    parser.add_argument('--duration', type=float, default=2.0)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--resume-if-exists', action='store_true',
                        help='resume the chosen output automatically when it already exists')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.holdout < 1 or args.samples < args.holdout + 4 or not 1 <= args.duration <= 10:
        parser.error('require holdout >=1, samples >= holdout+4, duration 1..10 s')
    if args.check:
        sample = capture(args.host, args.port, args.duration)
        print(json.dumps({k: v for k, v in sample.items() if k != 'raw_frames'}, ensure_ascii=False, indent=2))
        return
    path = args.output or Path(__file__).resolve().parents[1] / 'outputs/tcp_calibration' / (
        'ballpoint-pen-' + dt.datetime.now().strftime('%Y%m%d-%H%M%S') + '.json')
    if args.resume or (args.resume_if_exists and path.exists()):
        document = json.loads(path.read_text())
        if document.get('tool') != 'ballpoint_pen_tip':
            parser.error('resume requires an unfinished ballpoint pen calibration')
        if document.get('result') is not None:
            print('此文件已完成，原结果保留：')
            print(json.dumps(document['result'], ensure_ascii=False, indent=2))
            print('重新标定请用新的 --output 文件名。')
            return
        if (document['sample_target'], document['holdout_count']) != (args.samples, args.holdout):
            parser.error('resume sample/holdout counts must match the saved session')
    else:
        if path.exists():
            parser.error('output exists; use --resume or a new output')
        document = {'tool': 'ballpoint_pen_tip', 'method': 'fixed-point pivot',
                    'created_at': dt.datetime.now().astimezone().isoformat(),
                    'sample_target': args.samples, 'holdout_count': args.holdout,
                    'frame': 'controller flange, active TCP offset removed',
                    'controller_tcp_written': False, 'samples': [], 'rejected': []}
        save(path, document)
    print('圆珠笔 TCP 标定：共 %d 组，前 %d 组求解，末尾 %d 组独立验证。只读，不控制运动。' %
          (args.samples, args.samples - args.holdout, args.holdout))
    print('已保存 %d 组：%s' % (len(document['samples']), path))
    print('固定笔的夹持位置/笔尖伸出状态；始终让同一笔尖轻触同一固定尖点，勿压弯笔杆。')
    print('交替向不同方向倾斜笔；姿态差至少 5°，建议有条件时变化 10–20°。仅绕笔轴自转不足。')
    print('合格门禁：训练 RMS ≤0.5 mm、独立验证最大误差 ≤1 mm；不代表绝对精度达到0.1 mm。')
    print('每次用示教器改变姿态再重新对准，停稳后回车。q 保存退出。')
    while len(document['samples']) < args.samples:
        n = len(document['samples'])
        phase = '独立验证' if n >= args.samples - args.holdout else '求解样本'
        answer = input('%s %d/%d：对准并停稳后 Enter；q 退出 > ' % (phase, n + 1, args.samples))
        if answer.strip().lower() == 'q':
            print('已保存；使用同一 --output 加 --resume 继续：', path)
            return
        try:
            row = capture(args.host, args.port, args.duration)
            if row['max_motion_mm'] > 0.2 or row['max_motion_deg'] > 0.1:
                raise ValueError('采样未停稳：%.3f mm / %.3f°' % (row['max_motion_mm'], row['max_motion_deg']))
            if document['samples']:
                if not np.allclose(row['active_tcp_offset_m_rad'], document['samples'][0]['active_tcp_offset_m_rad'], atol=1e-9, rtol=0):
                    raise ValueError('本次会话期间控制器 TCP 配置发生变化')
                r = Rotation.from_rotvec(row['flange_pose_m_rad'][3:])
                distances = [(r.inv() * Rotation.from_rotvec(s['flange_pose_m_rad'][3:])).magnitude()
                             for s in document['samples']]
                if min(distances) < np.deg2rad(5):
                    raise ValueError('与已有姿态差异不足 5°，请换姿态')
            row['sample_index'] = n + 1
            document['samples'].append(row)
            print('接受：%d 帧，运动 %.3f mm / %.3f°' %
                  (row['frame_count'], row['max_motion_mm'], row['max_motion_deg']))
        except (OSError, ValueError, RuntimeError) as exc:
            document['rejected'].append({'at': time.time(), 'reason': str(exc)})
            print('未接受：', exc)
        save(path, document)
    document['result'] = fit(document['samples'], args.holdout)
    save(path, document)
    print(json.dumps(document['result'], ensure_ascii=False, indent=2))
    print('OUTPUT:', path)


if __name__ == '__main__':
    main()
