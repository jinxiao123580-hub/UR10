#!/usr/bin/env python3
"""Hardware-free fault/resume acceptance test for the paper collector."""
from contextlib import ExitStack, redirect_stdout
import io
import json
from pathlib import Path
import struct
import tempfile
from unittest.mock import patch

import numpy as np

import calibrate_paper_plane_auto as app


def regression_tests():
    assert app.probe.finite_move_timeout(.001, .0005) >= 7.5
    assert app.probe.finite_move_timeout(.003, .003) >= 5
    try:
        app.probe.finite_move_timeout(0, .001)
    except ValueError:
        pass
    else:
        raise AssertionError('invalid motion timing must fail')
    import calibrate_pen_tcp as tcp_capture
    with patch.object(tcp_capture, '_capture_once', side_effect=[ConnectionError('temporary close'), {'frame_count': 21}]) as read, patch.object(tcp_capture.time, 'sleep'):
        assert tcp_capture.capture('test', 30011, 2)['frame_count'] == 21
        assert read.call_count == 2
    # A noisy stationary window must be retried at rest, never accepted by
    # raising the threshold. Missing/stale data remains a hard failure.
    sensor = object.__new__(app.probe.RawForce)
    stats = {'std_n_nm': [.01] * 6}
    with patch.object(sensor, 'recent', return_value=[1]), patch.object(sensor, '_baseline_window',
            side_effect=[app.probe.BaselineNoiseError('noise', {'noise': .4}),
                         (np.zeros(6), stats.copy())]) as windows, patch.object(app.probe.time, 'sleep'):
        _, passed = sensor.baseline()
        assert windows.call_count == 2 and len(passed['settle_attempts']) == 1
    class FakeSocket:
        def __init__(self, chunks):
            self.chunks = list(chunks)
        def settimeout(self, value):
            pass
        def recv(self, size):
            return self.chunks.pop(0)
    class FakeReader:
        def __init__(self, buffered, chunks):
            self.buffer = bytearray(buffered)
            self.sock = FakeSocket(chunks)
            self.last_frame = None
        def connect(self, **kwargs):
            raise AssertionError('fresh reads must not reopen the socket')
    def frame(tag):
        data = bytearray(1220)
        struct.pack_into('>I', data, 0, len(data))
        struct.pack_into('>6d', data, 444, tag, 0, .4, np.pi, 0, 0)
        return bytes(data)
    def ready(read, write, error, timeout):
        return (read if read[0].chunks else [], [], [])
    with patch.object(app.probe.select, 'select', ready):
        reader = FakeReader(frame(1), [frame(2)[:100], frame(2)[100:] + frame(3)])
        _, tcp, _ = app.probe.fresh(reader)
        assert tcp[0] == 3, 'must drain stale and partial packets to latest frame'
        reader = FakeReader(frame(1), [frame(2)[:100]])
        with patch.object(app.probe.time, 'monotonic', side_effect=[0, 0, .1, 4]):
            try:
                app.probe.fresh(reader)
            except TimeoutError:
                pass
            else:
                raise AssertionError('old complete frame plus new partial bytes must not qualify as fresh')
    class SpikeForce:
        def recent(self):
            return [(0, 0, 0, [0, 0, 0, 0, 0, 0])] * 9 + [(0, 0, 0, [0, 0, 5.1, 0, 0, 0])]
    try:
        app.probe.force_level(SpikeForce(), np.zeros(6))
    except app.probe.ForceGuardError as exc:
        assert exc.diagnostics['raw_peak_delta_n'] == 5.1
        assert exc.diagnostics['filtered_delta_n'] == 0
    else:
        raise AssertionError('raw 5 N gate must remain active even for isolated spikes')
    session = {'accepted_points': [], 'corners': [{'pen_tip_base_m': [.4, -.3, .02]},
                                                 {'pen_tip_base_m': [.6, -.1, .02]}]}
    for gap in (.01587, .06473):
        zero = (np.zeros(6), np.zeros(6), np.array([.5, -.2, .01997]))
        raised = (np.zeros(6), np.zeros(6), np.array([.5, -.2, .02 + gap]))
        with patch.object(app, 'state_tip', side_effect=[zero, raised]), patch('builtins.input', return_value=''), redirect_stdout(io.StringIO()):
            result = app.prepare_start(None, None, None, session)
        assert abs(result[2][2] - raised[2][2]) < 1e-12
    # Exercise the real travel error path: one bounded command, then stop and
    # retain the triggering force evidence. Never dispatch a second segment.
    logs, commands, stops = [], [], []
    initial = (np.zeros(6), np.r_[.5, -.2, .4, np.pi, 0, 0], np.array([.5, -.2, .071]))
    overload = app.probe.ForceGuardError('test overload', {'raw_peak_delta_n': 2.1})
    class BaselineForce:
        def baseline(self):
            return np.zeros(6), {'count': 400}
    reader = FakeReader(b'', [])
    with ExitStack() as stack:
        stack.enter_context(patch.object(app, 'state_tip', return_value=initial))
        stack.enter_context(patch.object(app.probe, 'controller_ready', return_value={}))
        stack.enter_context(patch.object(app.probe, 'model_gate', return_value={}))
        stack.enter_context(patch.object(app.probe, 'force_level', side_effect=[0, overload]))
        stack.enter_context(patch.object(app.probe, 'command', side_effect=lambda *args: commands.append(args)))
        stack.enter_context(patch.object(app.probe, 'stop', side_effect=lambda *args: stops.append(args)))
        stack.enter_context(patch.object(app.probe, 'force_snapshot', return_value={'frames': ['evidence']}))
        try:
            app.travel(reader, BaselineForce(), 'test', np.zeros(6), np.array([0, 0, .329]),
                       [.504, -.2, .071], np.array([np.pi, 0, 0]), logs)
        except app.probe.ForceGuardError:
            pass
        else:
            raise AssertionError('overload must stop real travel implementation')
    assert len(commands) == 1 and len(stops) == 1
    assert logs[-1]['status'] == 'failed'
    assert logs[-1]['force_guard_diagnostics']['raw_peak_delta_n'] == 2.1
    assert logs[-1]['force_trace']['frames'] == ['evidence']
    event, halted = {}, []
    with patch.object(app.probe, 'stop', side_effect=lambda *a: halted.append(1)), \
            patch.object(app.probe, 'force_snapshot', return_value={}), \
            patch.object(app.probe, 'force_level', return_value=.05), \
            patch.object(app.probe, 'fresh', return_value=(np.zeros(6), np.zeros(6), np.zeros(6))):
        app.recover_high_clearance_spike(None, None, np.zeros(6), 'test', event)
    assert halted == [1] and event['status'] == 'returned_to_original_baseline'
    ticks = iter(np.arange(0, 20, .25))
    event = {}
    with patch.object(app.probe, 'stop'), patch.object(app.probe, 'force_snapshot', return_value={}), \
            patch.object(app.probe, 'force_level', side_effect=app.probe.ForceGuardError('persistent load', {})), \
            patch.object(app.probe, 'fresh', return_value=(np.zeros(6), np.zeros(6), np.zeros(6))), \
            patch.object(app.time, 'monotonic', side_effect=lambda: next(ticks)), patch.object(app.time, 'sleep'):
        try:
            app.recover_high_clearance_spike(None, None, np.zeros(6), 'test', event)
        except RuntimeError as exc:
            assert '禁止继续' in str(exc)
        else:
            raise AssertionError('persistent 2 N load must never resume')
    print('REGRESSION PASS: persistent latest-frame reads, partial-packet rejection, high-start recovery, 5 N overload gate, failed-travel evidence')


def main():
    app.self_test()
    regression_tests()
    with tempfile.TemporaryDirectory(prefix='paper-plane-test-') as temporary:
        folder = Path(temporary)
        calibration = folder / 'tcp.json'
        output = folder / 'plane.json'
        tip = np.array([0.0001, -0.0016, 0.329])
        app.save(calibration, {'result': {'passed': True, 'flange_to_pen_tip_m': tip.tolist()}})
        def height(xy):
            return .012 + .004 * xy[0] - .002 * xy[1]
        corners = [np.array([.4, -.3, height([.4, -.3])]),
                   np.array([.6, -.1, height([.6, -.1])])]
        state = {'point': corners[1] + [0, 0, .004], 'capture_count': 0,
                 'fail_once': True, 'accepted': [], 'stops': 0, 'traverses': 0, 'first_ready': True}
        def capture(*args):
            n = state['capture_count']; state['capture_count'] += 1
            point = corners[n] if n < 2 else state['point']
            tcp = app.tcp_target(point, [np.pi, 0, 0], np.zeros(6), tip)
            return {'flange_pose_m_rad': tcp.tolist(), 'active_tcp_offset_m_rad': [0] * 6,
                    'max_motion_mm': .02, 'max_motion_deg': .003}
        def input_ready(prompt):
            if '开始连续' in prompt:
                state['point'][2] = height(state['point'][:2]) + (.06473 if state['first_ready'] else .004)
                state['first_ready'] = False
            return ''
        def read_state(*args):
            point = state['point']
            return np.zeros(6), app.tcp_target(point, [np.pi, 0, 0], np.zeros(6), tip), point.copy()
        def travel(reader, force, host, offset, tip_arg, desired, orientation, log, allow_transient_pause=False, on_progress=None):
            previous, target = state['point'].copy(), np.asarray(desired)
            if np.linalg.norm(previous[:2] - target[:2]) > 1e-9:
                assert abs(previous[2] - target[2]) < 1e-9, 'XY travel must be level'
                assert target[2] >= max(c[2] for c in corners) + .0199, 'XY travel must be raised'
                state['traverses'] += 1
            state['point'] = target.copy()
            log.append({'mock_target': target.tolist()})
        def contact(host, calibration_path, path):
            index = int(path.name.split('-')[1])
            if index == 4 and state['fail_once']:
                state['fail_once'] = False
                app.save(path, {'status': 'stopped_on_error', 'error': 'synthetic force fault'})
                raise RuntimeError('synthetic force fault')
            point = state['point'].copy(); point[2] = height(point[:2]) - .0002
            result = {'status': 'contact_recorded_retracted_2mm', 'contact_pen_tip_base_m': point.tolist()}
            app.save(path, result)
            state['point'] = point + [0, 0, .002]
            state['accepted'].append(index)
            return result
        class FakeResource:
            def __init__(self, *args, **kwargs):
                pass
            def close(self):
                pass
        def stop(*args):
            state['stops'] += 1
        argv = ['paper-test', '--execute', '--resume-if-exists',
                '--tcp-calibration', str(calibration), '--output', str(output)]
        with ExitStack() as stack:
            for name, value in [('capture', capture), ('state_tip', read_state),
                                ('travel', travel), ('run_contact', contact)]:
                stack.enter_context(patch.object(app, name, value))
            stack.enter_context(patch.object(app.probe, 'RealtimeReader', FakeResource))
            stack.enter_context(patch.object(app.probe, 'RawForce', FakeResource))
            stack.enter_context(patch.object(app.probe, 'controller_ready', lambda *args: {}))
            stack.enter_context(patch.object(app.probe, 'stop', stop))
            stack.enter_context(patch('builtins.input', input_ready))
            stack.enter_context(patch('sys.argv', argv))
            stack.enter_context(redirect_stdout(io.StringIO()))
            try:
                app.main()
            except RuntimeError as exc:
                assert 'synthetic force fault' in str(exc)
            else:
                raise AssertionError('fault must stop the session')
            failed = json.loads(output.read_text())
            assert failed['status'] == 'stopped_on_error'
            assert len(failed['accepted_points']) == 3
            assert failed['attempts'][-1]['status'] == 'failed'
            assert state['stops'] == 1
            app.main()
        complete = json.loads(output.read_text())
        assert complete['status'] == 'validated'
        assert state['accepted'] == list(range(1, 13)), 'resume must not repeat accepted points'
        assert len(complete['accepted_points']) == 12
        assert complete['result']['training_count'] == 9
        assert complete['result']['holdout_count'] == 3
        assert state['traverses'] > 0
        for row in complete['accepted_points']:
            assert Path(row['measurement']).is_file()
    print('INTEGRATION PASS: fault stops, three points persist, resume skips them, all 12 fit/validate, XY traverses remain raised')


if __name__ == '__main__':
    main()
