"""Read-only dashboard model, offline IK and verified-cache preview service."""
import hashlib
import json
from pathlib import Path
import threading

import numpy as np
import pinocchio as pin
from scipy.spatial.transform import Rotation
import yaml

from robot_attachments import load
from ur10_nullspace_ik import UR10NullspaceIK
from self_collision import SelfCollisionModel

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'config/dashboard_kinematics_20261027.json'
CACHE_DIR = ROOT / 'outputs/ik/dashboard_cache'
LINK_NAMES = ('base_link', 'shoulder_link', 'upper_arm_link', 'forearm_link',
              'wrist_1_link', 'wrist_2_link', 'wrist_3_link', 'tool0')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def joints(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (6,) or not np.isfinite(result).all():
        raise ValueError('需要六个有限关节角，单位 rad')
    return result


class DashboardKinematics:
    def __init__(self):
        self.report = json.loads(MANIFEST.read_text())
        self.calibration_path = ROOT / 'config/handeye_eye_in_hand_20261027.yaml'
        self.calibration = yaml.safe_load(self.calibration_path.read_text())
        self.geometry = load()
        self.X = self.geometry['tool0_from_camera'].copy()
        self.ik = UR10NullspaceIK()
        self.urdf = ROOT / 'outputs/vision/ur10_with_camera.urdf'
        self.model = pin.buildModelFromUrdf(str(self.urdf))
        self.frames = {name: self.model.getFrameId(name) for name in LINK_NAMES}
        self.lock = threading.Lock()
        self.collision = None
        self.loaded = {str(path): digest(path) for path in (
            self.calibration_path, self.urdf, self.ik.urdf_path, ROOT / 'config/robot_attachments.yaml', MANIFEST)}

    def link_matrices(self, q):
        data = self.model.createData()
        pin.forwardKinematics(self.model, data, joints(q))
        pin.updateFramePlacements(self.model, data)
        return {name: data.oMf[index].homogeneous.reshape(-1).tolist() for name, index in self.frames.items()}

    def status(self):
        report = self.report
        same = all(digest(path) == value for path, value in self.loaded.items())
        same = same and digest(self.ik.urdf_path) == report['factory_urdf_sha256']
        same = same and digest(self.calibration_path) == report['calibration_sha256']
        same = same and digest(self.urdf) == report['final_generated_model_sha256'][str(self.urdf.relative_to(ROOT))]
        same = same and report.get('passed') is True and self.calibration.get('valid') is True
        return {'ok': True, 'execution_allowed': False, 'revision': '2026-10-27',
                'integrity_ok': bool(same), 'calibration_valid': bool(self.calibration['valid']),
                'tool0_from_camera': self.X.tolist(), 'translation_mm': (self.X[:3, 3] * 1000).tolist(),
                'optical_axis_angle_deg': self.calibration['optical_z_to_tool0_positive_z_deg'],
                'independent': self.calibration['postfit_independent_samples'],
                'camera_housing_size_m': self.geometry['camera_housing']['size_m'],
                'validation': {key: report[key] for key in ('passed', 'initial_ik_passed', 'tracking_ik_passed',
                    'camera_target_ik_passed', 'mujoco_trajectory_points', 'watchdog_passed')},
                'caches': [{key: c[key] for key in ('mode', 'sample_count', 'duration_s', 'max_position_error_mm',
                            'minimum_checked_clearance_mm')} for c in report['cache_cases']],
                'example_seed_q_rad': report['cache_cases'][0]['start_q_rad']}

    def require_current(self):
        if not self.status()['integrity_ok']:
            raise ValueError('配置或模型已变化，请重新验收并重载控制台')

    def fk(self, request):
        self.require_current()
        q = joints(request.get('seed_q_rad'))
        frame = request.get('frame', 'tool0')
        if frame not in ('tool0', 'camera'):
            raise ValueError('坐标系须为 tool0 或 camera')
        T = self.ik.fk_jacobian(q)[0]
        if frame == 'camera':
            T = T @ self.X
        return {'ok': True, 'execution_allowed': False, 'frame': frame,
                'pose_m_rad': np.r_[T[:3, 3], Rotation.from_matrix(T[:3, :3]).as_rotvec()].tolist(),
                'link_world_matrices': self.link_matrices(q)}

    def solve(self, request):
        self.require_current()
        seed = joints(request.get('seed_q_rad'))
        target = joints(request.get('target_base_pose_m_rad'))
        frame, mode = request.get('frame', 'tool0'), request.get('mode', 'pose')
        if frame not in ('tool0', 'camera') or mode not in ('pose', 'axis'):
            raise ValueError('不支持的坐标系或任务模式')
        if frame == 'camera' and mode != 'pose':
            raise ValueError('相机目标须使用完整位姿模式')
        T = np.eye(4); T[:3, :3] = Rotation.from_rotvec(target[3:]).as_matrix(); T[:3, 3] = target[:3]
        if frame == 'camera':
            T = T @ np.linalg.inv(self.X)
        with self.lock:
            result = self.ik.solve(T, seed, mode=mode)
            q = np.asarray(result['q'])
            if self.collision is None:
                self.collision = SelfCollisionModel()
            steps = max(1, int(np.ceil(np.max(np.abs(q - seed)) / np.deg2rad(.5))))
            clearance, pair = min((self.collision.min_clearance(seed + a * (q - seed))
                                   for a in np.linspace(0, 1, steps + 1)), key=lambda x: x[0])
        result.update(collision_checked=True, minimum_checked_clearance_mm=float(clearance * 1000),
                      closest_pair=pair, passes_offline_gates=bool(result['success'] and clearance >= .02),
                      collision_scope='选定附件与机械臂碰撞对；未覆盖完整环境', execution_allowed=False)
        return {'ok': True, 'execution_allowed': False, 'result': result,
                'link_world_matrices': self.link_matrices(q)}

    def cache(self, mode):
        self.require_current()
        if mode not in ('pose', 'axis'):
            raise ValueError('轨迹模式须为 pose 或 axis')
        path = CACHE_DIR / ('cache-' + mode + '.json')
        if digest(path) != self.report['cache_sha256'][mode]:
            raise ValueError('缓存文件哈希已变化，请重新验收')
        plan = json.loads(path.read_text())
        from buffered_joint_replay import validate_cache
        times, q = validate_cache(plan)
        if plan.get('model_urdf_sha256') != digest(self.ik.urdf_path):
            raise ValueError('缓存与当前机器人模型不一致')
        ids = np.unique(np.linspace(0, len(times) - 1, min(320, len(times))).astype(int))
        return {'ok': True, 'execution_allowed': False, 'mode': mode,
                'sample_count': len(times), 'duration_s': plan['duration_s'],
                'frames': [{'t_s': float(times[i]), 'link_world_matrices': self.link_matrices(q[i])} for i in ids]}
