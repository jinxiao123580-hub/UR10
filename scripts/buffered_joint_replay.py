#!/usr/bin/env python3
"""Offline bounded-buffer/watchdog model. No robot, ROS or socket transport.

Time and observations are supplied by the simulator. Returned targets are local
data only; this module is not a controller or a hardware execution gate.
"""
import argparse
from collections import deque
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np


def validate_cache(document):
    """Check schema and numerical consistency, without FK, IK or scene checks."""
    if (document.get('schema_version') != 1 or
            document.get('status') != 'offline_simulation_candidate' or
            document.get('execution_allowed') is not False):
        raise ValueError('expected an offline simulation candidate cache')
    if document.get('units') != {'q': 'rad', 'qd': 'rad/s', 'qdd': 'rad/s^2', 't': 's'}:
        raise ValueError('invalid cache units')
    names = document.get('joint_names')
    if (not isinstance(names, list) or len(names) != 6 or
            not all(isinstance(n, str) and n for n in names) or len(set(names)) != 6):
        raise ValueError('invalid joint names')
    if document.get('mode') not in ('pose', 'axis'):
        raise ValueError('invalid task mode')
    t = np.asarray(document['t_s'], dtype=float).copy()
    if t.ndim != 1 or not 2 <= len(t) <= 1000000 or not np.isfinite(t).all():
        raise ValueError('invalid sample times')
    arrays = [np.asarray(document[key], dtype=float).copy()
              for key in ('q_rad', 'qd_rad_s', 'qdd_rad_s2')]
    if any(a.shape != (len(t), 6) or not np.isfinite(a).all() for a in arrays):
        raise ValueError('invalid joint samples')
    period = float(document['period_s'])
    duration = float(document['duration_s'])
    if (not np.isfinite([period, duration]).all() or period <= 0 or duration <= 0 or
            t[0] != 0 or np.any(np.diff(t) <= 0) or
            not np.allclose(np.diff(t), period, rtol=1e-7, atol=1e-10) or
            not np.isclose(t[-1], duration, rtol=1e-9, atol=1e-10)):
        raise ValueError('cache timing mismatch')
    q, qd, qdd = arrays
    seed = np.asarray(document['start_q_rad'], dtype=float)
    if seed.shape != (6,) or not np.isfinite(seed).all() or not np.allclose(seed, q[0], rtol=0, atol=1e-10):
        raise ValueError('cache start mismatch')
    for key, bound_key, samples in (
            ('velocity_limit_rad_s', 'analytic_max_velocity_rad_s', qd),
            ('acceleration_limit_rad_s2', 'analytic_max_acceleration_rad_s2', qdd)):
        limit = float(document[key])
        bound = np.asarray(document[bound_key], dtype=float)
        if (not np.isfinite(limit) or limit <= 0 or bound.shape != (6,) or
                not np.isfinite(bound).all() or np.any(bound < 0) or
                np.any(bound > limit + 1e-10) or
                np.any(np.max(np.abs(samples), axis=0) > bound + 1e-9)):
            raise ValueError('cache derivative bounds mismatch')
    if (np.max(np.abs(np.diff(q, axis=0))) / period > document['velocity_limit_rad_s'] + 1e-8 or
            np.max(np.abs(np.diff(qd, axis=0))) / period > document['acceleration_limit_rad_s2'] + 1e-8 or
            np.max(np.abs(qd[[0, -1]])) > 1e-8):
        raise ValueError('cache motion continuity mismatch')
    t.setflags(write=False)
    q.setflags(write=False)
    return t, q


@dataclass(frozen=True)
class Observation:
    timestamp_s: float
    q_rad: tuple
    ready: bool = True


class PlaybackStopped(RuntimeError):
    pass


class BufferedReplay:
    """Producer fills a bounded queue; simulated consumer checks each due tick.

    STOPPED is latched. A stop clears pending targets and never emits a hold,
    deceleration or hardware stop command. A real controller must implement those.
    """
    def __init__(self, document, capacity=64, prefill=16, max_lateness_s=None,
                 state_timeout_s=.024, max_tracking_error_rad=.05):
        self.t, self.q = validate_cache(document)
        self.period = float(document['period_s'])
        if (type(capacity) is not int or type(prefill) is not int or
                not 1 <= prefill <= capacity):
            raise ValueError('invalid buffer capacity/prefill')
        self.max_lateness = self.period / 2 if max_lateness_s is None else float(max_lateness_s)
        self.state_timeout = float(state_timeout_s)
        self.tracking_limit = float(max_tracking_error_rad)
        if (not np.isfinite([self.max_lateness, self.state_timeout, self.tracking_limit]).all() or
                not 0 < self.max_lateness < self.period or self.state_timeout <= 0 or self.tracking_limit <= 0):
            raise ValueError('invalid watchdog settings')
        canonical = json.dumps(document, sort_keys=True, separators=(',', ':'), allow_nan=False)
        self.cache_sha256 = hashlib.sha256(canonical.encode()).hexdigest()
        self.capacity, self.prefill = capacity, min(prefill, len(self.t))
        self.queue = deque()
        self.next_fill = self.emitted = self.peak_occupancy = 0
        self.state, self.reason = 'NEW', None
        self.last_now = self.last_observation = None
        self.start_time = None

    def stop(self, reason):
        if self.state not in ('STOPPED', 'COMPLETE'):
            self.state, self.reason = 'STOPPED', reason
            self.queue.clear()

    def _fail(self, reason):
        self.stop(reason)
        raise PlaybackStopped(self.reason or reason)

    def fill(self, count):
        if self.state in ('STOPPED', 'COMPLETE'):
            return 0
        if type(count) is not int or count < 0:
            raise ValueError('invalid fill count')
        count = min(count, self.capacity - len(self.queue), len(self.t) - self.next_fill)
        self.queue.extend(range(self.next_fill, self.next_fill + count))
        self.next_fill += count
        self.peak_occupancy = max(self.peak_occupancy, len(self.queue))
        if self.state == 'NEW' and len(self.queue) >= self.prefill:
            self.state = 'PRIMED'
        return count

    def _observe(self, now, observation):
        try:
            now = float(now)
        except (TypeError, ValueError):
            self._fail('clock_regression_or_invalid_time')
        if (not np.isfinite(now) or
                (self.last_now is not None and now < self.last_now)):
            self._fail('clock_regression_or_invalid_time')
        self.last_now = now
        try:
            stamp = float(observation.timestamp_s)
            actual = np.asarray(observation.q_rad, dtype=float)
        except (TypeError, ValueError):
            self._fail('invalid_observation')
        if actual.shape != (6,) or not np.isfinite(actual).all() or not np.isfinite(stamp):
            self._fail('invalid_observation')
        if observation.ready is not True:
            self._fail('controller_not_ready')
        if stamp > now + 1e-9:
            self._fail('observation_from_future')
        if self.last_observation is not None and stamp < self.last_observation:
            self._fail('observation_out_of_order')
        if now - stamp > self.state_timeout + 1e-9:
            self._fail('stale_observation')
        self.last_observation = stamp
        # This simulator expects feedback for the previously consumed target.
        # It does not model servo lag, interpolation or a real controller clock.
        reference = self.q[max(0, self.emitted - 1)]
        if np.max(np.abs(actual - reference)) > self.tracking_limit:
            self._fail('joint_tracking_error')
        return now

    def start(self, now, observation):
        if self.state != 'PRIMED':
            self._fail('buffer_not_primed')
        now = self._observe(now, observation)
        self.start_time = now
        self.state = 'RUNNING'

    def tick(self, now, observation):
        if self.state == 'STOPPED':
            raise PlaybackStopped(self.reason)
        if self.state == 'COMPLETE':
            return None
        if self.state != 'RUNNING':
            self._fail('playback_not_started')
        now = self._observe(now, observation)
        deadline = self.start_time + float(self.t[self.emitted])
        if now < deadline - 1e-9:
            return None
        if now - deadline > self.max_lateness + 1e-9:
            self._fail('consumer_deadline_missed')
        if not self.queue:
            self._fail('buffer_underflow')
        index = self.queue.popleft()
        if index != self.emitted:
            self._fail('buffer_sequence_error')
        self.emitted += 1
        if self.emitted == len(self.t):
            self.state = 'COMPLETE'
        return float(self.t[index]), self.q[index].tolist()


def simulate(document):
    """Deterministic ideal feedback; no wall-clock timing or physical dynamics."""
    player = BufferedReplay(document)
    player.fill(player.capacity)
    player.start(0., Observation(0., tuple(player.q[0])))
    for stamp in player.t:
        now = float(stamp)
        if len(player.queue) <= player.prefill:
            player.fill(player.capacity)
        observation = Observation(now, tuple(player.q[max(0, player.emitted - 1)]))
        player.tick(now, observation)
    return {'state': player.state, 'emitted_samples': player.emitted,
            'peak_buffer_samples': player.peak_occupancy, 'capacity': player.capacity,
            'cache_sha256': player.cache_sha256, 'hardware_connected': False,
            'execution_allowed': False, 'clock': 'deterministic_simulation'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(simulate(json.loads(args.input.read_text())), indent=2))
