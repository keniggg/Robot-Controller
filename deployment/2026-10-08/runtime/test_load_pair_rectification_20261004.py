"""Offline loader contracts. No ROS initialization, process signals or devices."""
from collections import OrderedDict
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import threading
import time

import pytest


PATH = Path(__file__).with_name('load_pair_rectification_20261004.py')
spec = importlib.util.spec_from_file_location('pair_loader_under_test', PATH)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def stage_args():
    return ['/usr/bin/python3', str(m.STAGE_WRAPPER), '--camera-script', str(m.WORK / m.RELAY['camera']),
            '--output', str(m.EXT / 'ros_log/old/stages'), '--seconds', '30', '--samples', '12',
            '--interval', '2', '--warmup', '3', '--', '/perception/object:=/perception/carton/object',
            '__name:=supervisor_camera', '__log:=/old.log']


def test_import_and_default_main_never_initialize_ros_or_spawn(monkeypatch):
    monkeypatch.setattr(m.subprocess, 'run', lambda *a, **k: pytest.fail('spawn'))
    monkeypatch.setattr(m.os, 'kill', lambda *a, **k: pytest.fail('signal'))
    monkeypatch.setattr(m, 'Runtime', lambda *a: pytest.fail('ROS initialization'))
    with pytest.raises(SystemExit):
        m.main([])


def test_strict_unwrap_discards_sampling_flags_preserves_complete_ros_tail():
    before = stage_args()
    args = m.direct_argv('camera', before)
    assert args == [before[0], before[3]] + before[before.index('--') + 1:]
    assert str(m.STAGE_WRAPPER) not in args
    assert not any(a.startswith('--') for a in args)
    assert before == stage_args()


@pytest.mark.parametrize('case', ['unknown', 'repeated', 'missing', 'extra', 'nested', 'foreign',
    'delimiter', 'no_delimiter', 'nan', 'duration', 'samples', 'interval', 'warmup', 'relative_output',
    'empty', 'name', 'duplicate_remap', 'non_ros', 'interpreter'])
def test_malformed_wrapper_or_ros_tail_rejected(case):
    args = stage_args()
    if case == 'unknown': args[4] = '--query-temperature'
    elif case == 'repeated': args[4] = '--camera-script'
    elif case == 'missing': del args[4:6]
    elif case == 'extra': args[14:14] = ['--new', '1']
    elif case == 'nested': args[3] = str(m.STAGE_WRAPPER)
    elif case == 'foreign': args[3] = '/different/camera_node.py'
    elif case == 'delimiter': args += ['--']
    elif case == 'no_delimiter': args.remove('--')
    elif case == 'nan': args[7] = 'nan'
    elif case == 'duration': args[7] = '31'
    elif case == 'samples': args[9] = '13'
    elif case == 'interval': args[11] = '0'
    elif case == 'warmup': args[13] = '30'
    elif case == 'relative_output': args[5] = 'relative'
    elif case == 'empty': args[5] = ''
    elif case == 'name': args[-2] = '__name:=different'
    elif case == 'duplicate_remap': args.append('/perception/object:=other')
    elif case == 'non_ros': args.append('--execute')
    elif case == 'interpreter': args[0] = '/unverified/python'
    with pytest.raises(RuntimeError):
        m.direct_argv('camera', args)


@pytest.mark.parametrize('role', ['camera', 'unknown', 'remote', 'gateway'])
def test_real_catkin_entry_matches_tested_main(role):
    m.validate_relay(role)
    args = ['/usr/bin/python3', str(m.WORK / m.RELAY[role]), '__name:=' + m.NODES[role][1:]]
    assert m.direct_argv(role, args) == args


def test_wrong_catkin_relay_cannot_load_unhashed_main(tmp_path):
    path = tmp_path / m.RELAY['remote']
    path.parent.mkdir(parents=True)
    path.write_text("python_script = '/foreign/mode_aware_remote_grasp6d.py'\n")
    with pytest.raises(RuntimeError, match='source mismatch'):
        m.validate_relay('remote', tmp_path)


def test_remote_environment_cwd_remaps_and_external_logs_preserved(tmp_path):
    old = dict(cwd='/original/cwd', argv=['/usr/bin/python3', str(m.WORK / m.RELAY['remote']),
               '__name:=remote_grasp6d_node', 'image:=/custom/image', '__log:=/old.log'])
    env = dict(PYTHONPATH='/original/python', GRASP6D_URL='http://actual:8000',
               ALICIA_UNKNOWN_LOG_DIR=str(m.EXT / 'ros_log/unknown'), SECRET='do-not-copy')
    args = m.service_argv('remote', old, env, tmp_path, 'one-owner', ('PYTHONPATH',) + m.EXTRA_ENV)
    assert '--property=WorkingDirectory=/original/cwd' in args
    assert '--setenv=GRASP6D_URL=http://actual:8000' in args
    assert '--setenv=PYTHONPATH=/original/python' in args
    assert '--property=Restart=no' in args
    assert 'image:=/custom/image' in args
    assert '__log:=' + str(tmp_path / 'remote.ros.log') in args
    assert not any('SECRET' in a or 'do-not-copy' in a or '/old.log' in a for a in args)
    with pytest.raises(RuntimeError, match='original roslaunch'):
        m.service_argv('gateway', old, env, tmp_path, 'duplicate', ())


def manifest_fixture(tmp_path):
    files = ['main.py', m.TEST, 'tests/test_main.py']
    for name in files:
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('# tested ' + name)
    return dict(test_exit_code=0, test_result='1 passed', source_unchanged_during_test=True,
        changed_during_test=[], command=['/usr/bin/python3', '-m', 'pytest', 'tests/test_main.py'],
        source_sha256={'main.py': m.sha(tmp_path / 'main.py')},
        test_sha256={name: m.sha(tmp_path / name) for name in files[1:]},
        additional_test_runs=[dict(test_exit_code=0, test_result='1 passed',
                                  command=['/usr/bin/python3', '-m', 'pytest', m.TEST])])


def test_manifest_includes_main_and_additional_test_commands(tmp_path):
    manifest = manifest_fixture(tmp_path)
    assert len(m.verify_manifest(manifest, tmp_path, {'main.py'})) == 3


@pytest.mark.parametrize('case', ['missing_source', 'missing_test', 'changed_source', 'changed_test',
    'failed', 'additional_failed', 'unstable', 'missing_command', 'missing_extra_command',
    'unhashed_extra_test', 'escape', 'missing_remote_main'])
def test_manifest_failure_happens_before_any_reload(case, tmp_path):
    manifest = manifest_fixture(tmp_path)
    required = {'main.py'}
    if case == 'missing_source': manifest['source_sha256'] = {}
    elif case == 'missing_test': manifest['test_sha256'].pop(m.TEST)
    elif case == 'changed_source': (tmp_path / 'main.py').write_text('changed')
    elif case == 'changed_test': (tmp_path / m.TEST).write_text('changed')
    elif case == 'failed': manifest['test_exit_code'] = 1
    elif case == 'additional_failed': manifest['additional_test_runs'][0]['test_exit_code'] = 1
    elif case == 'unstable': manifest['source_unchanged_during_test'] = False
    elif case == 'missing_command': manifest['command'] = []
    elif case == 'missing_extra_command': manifest['additional_test_runs'] = []
    elif case == 'unhashed_extra_test': manifest['command'].append('tests/test_extra.py')
    elif case == 'escape': manifest['source_sha256']['../escape.py'] = '0' * 64
    elif case == 'missing_remote_main': required.add(m.MAIN['remote'])
    with pytest.raises(RuntimeError):
        m.verify_manifest(manifest, tmp_path, required)


def original_camera():
    intrinsic = dict(width=640, height=480, fx=400., fy=400., cx=320., cy=240.,
                     model='none', coeffs=[0.] * 5)
    return dict(width=640, height=480, fx=400., fy=400., cx=320., cy=240., depth_scale=.0001,
        fps=30, runtime_profile=dict(source='realsense_active_profile', color_intrinsics=intrinsic,
                                    geometry_intrinsics=deepcopy(intrinsic)))


def test_stream_dimensions_separate_from_published_canvas_and_rollback_boundary():
    original = original_camera()
    new = m.enabled_configuration(original)
    assert (new['stream_width'], new['stream_height']) == (640, 480)
    assert new['pair_rectification'] == {'enabled': True}
    assert 'pair_rectification' not in original
    new.update(width=653, height=489, cx=327., cy=246.)
    m.unchanged_camera_settings(original, new)
    restored = m.rollback_configuration(original, 1791116000000000000)
    assert restored['width'] == 640
    assert restored['runtime_profile']['geometry_activation_stamp_ns'] == '1791116000000000000'
    assert 'geometry_activation_stamp_ns' not in original['runtime_profile']
    new['fps'] = 15
    with pytest.raises(RuntimeError): m.unchanged_camera_settings(original, new)


@pytest.mark.parametrize('case', ['simulation', 'unaligned', 'legacy_projection', 'wrong_stream', 'already_rectified'])
def test_incompatible_camera_start_rejected(case):
    original = original_camera()
    if case == 'simulation': original['simulate'] = True
    elif case == 'unaligned': original['align_depth_to_color'] = False
    elif case == 'legacy_projection': original['color_projection_correction'] = {'enabled': True}
    elif case == 'wrong_stream': original['runtime_profile']['color_intrinsics']['width'] = 653
    elif case == 'already_rectified': original['runtime_profile']['rectification'] = {}
    with pytest.raises(RuntimeError): m.enabled_configuration(original)


def test_existing_gen13_expired_ledger_is_read_only_not_reset():
    ledger = dict(phase='awaiting_measured_target', deadline_ns='1791114290248470783',
                  attempts=3, max_observations=3, selection={'generation': 13})
    before = deepcopy(ledger)
    assert m.terminal_ledger(ledger, 1791114290248470784) == 'original_deadline_expired_nonexecuting'
    assert ledger == before
    with pytest.raises(RuntimeError): m.terminal_ledger(ledger, 1791114290248470783)


@pytest.mark.parametrize('phase', ['executing', 'reserved', 'preparing', 'unknown'])
def test_old_deadline_does_not_authorize_shutdown_of_active_or_unknown_phase(phase):
    with pytest.raises(RuntimeError): m.terminal_ledger(dict(phase=phase, deadline_ns='1'), 100)


class Orchestration:
    def __init__(self, fail=None): self.events, self.fail = [], fail
    def event(self, event):
        self.events.append(event)
        if event == self.fail: raise RuntimeError(event)
    def verify(self): self.event('verify')
    def stop(self, role): self.event('stop:' + role)
    def configure(self, enable): self.event('config:' + str(enable))
    def start(self, role): self.event('start:' + role)
    def validate_camera(self, enable): self.event('camera_ready:' + str(enable))
    def restart_gateway(self): self.event('respawn_gateway')
    def final_validate(self): self.event('final')
    def rollback(self): self.event('rollback')


def test_consumers_stop_before_geometry_change_and_wait_for_camera_before_start():
    io = Orchestration()
    m.perform_reload(io)
    assert io.events == ['verify', 'stop:unknown', 'stop:remote', 'stop:camera', 'config:True',
                         'start:camera', 'camera_ready:True', 'start:unknown', 'start:remote',
                         'respawn_gateway', 'final']
    assert not any('driver' in e or 'enable' in e or 'selection' in e for e in io.events)


@pytest.mark.parametrize('failure', ['verify', 'stop:remote', 'start:camera', 'camera_ready:True', 'start:remote', 'final'])
def test_each_failure_stops_forward_progress_and_requests_only_visual_rollback(failure):
    io = Orchestration(failure)
    with pytest.raises(RuntimeError): m.perform_reload(io)
    if failure == 'verify': assert io.events == ['verify']
    else: assert io.events[-2:] == [failure, 'rollback']
    if failure in ('start:camera', 'camera_ready:True'):
        assert 'start:unknown' not in io.events and 'respawn_gateway' not in io.events


def callback_runtime():
    runtime = m.Runtime.__new__(m.Runtime)
    runtime.lock = threading.RLock()
    runtime.faults, runtime.latest, runtime.frozen = [], {}, {}
    return runtime


@pytest.mark.parametrize('key,original,changed', [('sdk', {'Joint1': 0.}, {'Joint1': .001}),
                                                 ('epoch', 10, 11), ('task', False, True)])
def test_transient_hardware_or_task_change_cannot_be_erased(key, original, changed):
    runtime = callback_runtime()
    runtime.frozen[key] = original
    runtime.latch(key, changed)
    runtime.latch(key, original)
    assert runtime.latest[key] == original
    assert runtime.faults == [key + '_changed']


def test_sdk_callback_checks_all_named_joints_including_gripper():
    runtime = callback_runtime()
    names = ['Joint' + str(i) for i in range(1, 7)] + ['right_finger']
    runtime.sdk_cb(SimpleNamespace(name=names, position=[0.] * 7))
    runtime.frozen['sdk'] = deepcopy(runtime.latest['sdk'])
    runtime.sdk_cb(SimpleNamespace(name=names, position=[0.] * 6 + [.01]))
    assert runtime.faults == ['sdk_changed']


def test_runtime_rollback_only_restores_visual_config_and_restarts_three_independent_nodes():
    runtime = m.Runtime.__new__(m.Runtime)
    runtime.report, runtime.units, runtime.current = {}, {}, dict(camera=None, unknown=None, remote=None)
    events = []
    runtime.verify = lambda: events.append('verify')
    runtime.stop = lambda role: events.append('stop:' + role)
    runtime.configure = lambda enable: events.append('config:' + str(enable))
    runtime.start = lambda role: events.append('start:' + role)
    runtime.validate_camera = lambda enable: events.append('ready:' + str(enable))
    runtime.record = lambda event: events.append(event)
    runtime.rollback()
    assert events == ['verify', 'stop:unknown', 'stop:remote', 'stop:camera', 'config:False',
                      'start:camera', 'ready:False', 'start:unknown', 'start:remote', 'rollback_finished']
    assert runtime.report['rollback']['restored'] is True
    assert runtime.report['loaded'] is False


@pytest.mark.parametrize('change', ['pid_reused', 'registered_owner'])
def test_visual_signal_rechecks_identity_after_final_disk_record(change, monkeypatch):
    runtime = m.Runtime.__new__(m.Runtime)
    old = {'pid': 123, 'start_ticks': 111}
    runtime.current = {'unknown': old}
    state = {'alive': True, 'registered': 123}
    runtime.helper = SimpleNamespace(alive=lambda _: state['alive'])
    runtime.registered = lambda _: state['registered']
    runtime.verify = lambda: None
    def record(*a, **k):
        state['alive' if change == 'pid_reused' else 'registered'] = False if change == 'pid_reused' else 124
    runtime.record = record
    monkeypatch.setattr(m.os, 'kill', lambda *a: pytest.fail('signaled stale owner'))
    with pytest.raises(RuntimeError, match='immediately before signal'):
        runtime.stop('unknown')


def test_gateway_shutdown_resolves_exact_owner_again_after_record(monkeypatch):
    runtime = m.Runtime.__new__(m.Runtime)
    runtime.current = {'gateway': {'pid': 123}}
    runtime.helper = SimpleNamespace(alive=lambda _: True, Transport=lambda: None)
    runtime.registered = lambda _: 123
    runtime.verify = lambda: None
    state = {'pid': 123}
    runtime.record = lambda *a, **k: state.update(pid=124)
    runtime.ros = SimpleNamespace(get_name=lambda: '/loader',
        get_master=lambda: SimpleNamespace(lookupNode=lambda _: (1, '', 'http://current-gateway')))
    proxy = SimpleNamespace(getPid=lambda _: (1, '', state['pid']),
                            shutdown=lambda *a: pytest.fail('shut down replacement owner'))
    monkeypatch.setattr(m.xmlrpc.client, 'ServerProxy', lambda *a, **k: proxy)
    with pytest.raises(RuntimeError, match='immediately before shutdown'):
        runtime.restart_gateway()


@pytest.fixture(scope='module')
def valid_camera():
    from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification
    camera = original_camera()
    mapper = PairedPinholeRectification(camera['runtime_profile']['color_intrinsics'])
    camera['runtime_profile'].update(rectification=mapper.contract, geometry_activation_stamp_ns='1000')
    camera['pair_rectification'] = {'enabled': True}
    return camera


def camera_runtime(camera):
    runtime = callback_runtime()
    runtime.configuration = original_camera()
    runtime.new_configuration = m.enabled_configuration(runtime.configuration)
    runtime.camera_started_ns = 500
    runtime.report = {}
    runtime.ros = SimpleNamespace(get_param=lambda _: camera,
        Time=SimpleNamespace(now=lambda: SimpleNamespace(to_nsec=lambda: 2000)))
    runtime.guard = lambda: None
    runtime.record = lambda *a, **k: None
    def wait(check, seconds, label, guard):
        guard()
        if not check(): raise RuntimeError('not yet ready')
    runtime.helper = SimpleNamespace(wait=wait)
    runtime.pairs = OrderedDict()
    for stamp in (1100, 1200, 1300):
        runtime.pairs[stamp] = {kind: dict(width=640, height=480, encoding=encoding,
            step=640 * size, bytes=640 * 480 * size, frame_id='camera_link', caller='/supervisor_camera')
            for kind, size, encoding in (('rgb', 3, 'rgb8'), ('depth', 2, '16UC1'))}
    return runtime


def test_camera_gate_uses_new_activation_same_stamp_shape_and_original_units(valid_camera):
    runtime = camera_runtime(deepcopy(valid_camera))
    runtime.validate_camera(True)
    assert runtime.report['verified_pair_stamps_ns'] == [1100, 1200, 1300]


@pytest.mark.parametrize('change', ['selection', 'handeye', 'ledger', 'driver', 'sdk_stale',
                                   'camera_fx', 'camera_activation', 'camera_setting', 'camera_stream'])
def test_restart_guard_keeps_original_owners_and_validated_geometry(change, valid_camera, monkeypatch):
    camera = deepcopy(valid_camera)
    runtime = camera_runtime(camera)
    runtime.validate_camera(True)
    params = {'/camera': camera, '/grasp_mode/selection': {'generation': 13},
              '/handeye': {'matrix': [1., 0.]}, '/rgb_acquisition/ledger': {'phase': 'expired'}}
    runtime.selection, runtime.handeye, runtime.ledger = [deepcopy(params[k]) for k in
        ('/grasp_mode/selection', '/handeye', '/rgb_acquisition/ledger')]
    runtime.ros.get_param = lambda name: params[name]
    runtime.ros.is_shutdown = lambda: False
    runtime.before = {name: {'pid': pid} for name, pid in m.PRESERVE.items()}
    runtime.launch = {'pid': m.LAUNCH_PID}
    runtime.latest['sdk_receipt'] = time.monotonic()
    runtime.helper.alive = lambda original: True
    runtime.helper.node_pid = lambda ros, name: m.PRESERVE[name]
    monkeypatch.setattr(Path, 'is_mount', lambda _: True)
    if change == 'selection': params['/grasp_mode/selection']['generation'] = 14
    elif change == 'handeye': params['/handeye']['matrix'][0] = .9
    elif change == 'ledger': params['/rgb_acquisition/ledger']['phase'] = 'reserved'
    elif change == 'driver': runtime.helper.alive = lambda original: original['pid'] != 133511
    elif change == 'sdk_stale': runtime.latest['sdk_receipt'] -= 2.
    elif change == 'camera_fx': camera['fx'] += 1.
    elif change == 'camera_activation': camera['runtime_profile']['geometry_activation_stamp_ns'] = '2000'
    elif change == 'camera_setting': camera['pair_rectification']['enabled'] = False
    elif change == 'camera_stream': camera['stream_width'] = 653
    with pytest.raises(RuntimeError): m.Runtime.guard(runtime)


@pytest.mark.parametrize('case', ['old_activation', 'before_activation', 'old_shape', 'unpaired',
    'other_publisher', 'encoding', 'truncated', 'frame', 'both_wrong_frame', 'stale', 'units', 'stream', 'bad_contract', 'disabled'])
def test_camera_gate_rejects_old_queue_and_inconsistent_pairs(case, valid_camera):
    camera = deepcopy(valid_camera)
    runtime = camera_runtime(camera)
    if case == 'old_activation': camera['runtime_profile']['geometry_activation_stamp_ns'] = '400'
    elif case == 'before_activation': camera['runtime_profile']['geometry_activation_stamp_ns'] = '1500'
    elif case == 'old_shape': runtime.pairs[1100]['rgb']['width'] = 639
    elif case == 'unpaired': del runtime.pairs[1100]['depth']
    elif case == 'other_publisher': runtime.pairs[1100]['rgb']['caller'] = '/stale_camera'
    elif case == 'encoding': runtime.pairs[1100]['depth']['encoding'] = '32FC1'
    elif case == 'truncated': runtime.pairs[1100]['depth']['bytes'] -= 1
    elif case == 'frame': runtime.pairs[1100]['depth']['frame_id'] = 'wrong'
    elif case == 'both_wrong_frame':
        for image in runtime.pairs[1100].values(): image['frame_id'] = 'wrong'
    elif case == 'stale': runtime.ros.Time.now = lambda: SimpleNamespace(to_nsec=lambda: 900000000)
    elif case == 'units': camera['depth_scale'] = .001
    elif case == 'stream': camera['stream_width'] = 653
    elif case == 'bad_contract': camera['runtime_profile']['rectification']['digest'] = '0' * 64
    elif case == 'disabled': camera['pair_rectification']['enabled'] = False
    with pytest.raises((RuntimeError, ValueError)):
        runtime.validate_camera(True)
