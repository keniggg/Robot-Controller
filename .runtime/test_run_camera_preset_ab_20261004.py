import copy
import importlib.util
from pathlib import Path
import pytest
import json
import threading
from types import SimpleNamespace as NS

spec = importlib.util.spec_from_file_location('ab_run', Path(__file__).with_name('run_camera_preset_ab_20261004.py'))
M = importlib.util.module_from_spec(spec); spec.loader.exec_module(M)


def inputs():
    camera = dict(width=653, height=489, frame_id='camera_link',
                  runtime_profile=dict(geometry_activation_stamp_ns='1000000000'))
    pair = {kind:dict(width=653, height=489, frame_id='camera_link', caller='/supervisor_camera',
        encoding=encoding, step=653*size, bytes=653*489*size)
        for kind,size,encoding in [('rgb',3,'bgr8'),('depth',2,'16UC1')]}
    return pair, camera


def test_valid_pair_after_restore_and_activation():
    pair, camera = inputs()
    assert M.complete_pair_is_valid(1500000000, pair, camera, 1400000000, 1600000000)


@pytest.mark.parametrize('kind', ['rgb','depth'])
@pytest.mark.parametrize('field,value', [('width',640),('height',480),('frame_id','other'),
    ('caller','/other'),('encoding','mono8'),('step',1),('bytes',0)])
def test_corrupted_or_other_publisher_pairs_never_count(kind, field, value):
    pair,camera=inputs();pair[kind][field]=value
    assert not M.complete_pair_is_valid(1500000000, pair, camera, 1400000000, 1600000000)


@pytest.mark.parametrize('stamp,cutoff,now',[(150,150,160),(150,140,149),(1500000000,1400000000,2400000000)])
def test_old_future_and_stale_pairs_never_count(stamp,cutoff,now):
    pair,camera=inputs()
    assert not M.complete_pair_is_valid(stamp,pair,camera,cutoff,now)


def test_frame_before_geometry_is_rejected_even_after_restore_cutoff():
    pair,camera=inputs()
    assert not M.complete_pair_is_valid(1000000000,pair,camera,900000000,1100000000)


def test_incomplete_pair_is_not_a_complete_read():
    pair,camera=inputs(); del pair['rgb']
    assert not M.complete_pair_is_valid(1500000000,pair,camera,1400000000,1600000000)


@pytest.fixture
def main_rig(monkeypatch, tmp_path):
    """Run the real orchestrator and Runtime.stop with fake external boundaries.

    No ROS constructor, process inspection, signal, systemctl or SDK is called.
    Only synthetic receipts are written beneath pytest's temporary directory.
    """
    loader = M.module('ab_main_test_loader', M.WORK / '.runtime/load_pair_rectification_20261004.py')
    state = NS(scenario=None, runtime=None, clock=100., processes={}, registry={},
               units={}, trace=[], camera_starts=0, max_live_cameras=0, stop_failed=False)
    pair, camera = inputs()
    camera['runtime_profile']['mode_depth_preset'] = {'preset': 'high_accuracy'}
    original = {}
    for index, role in enumerate(loader.NODES):
        identity = dict(pid=100+index, start_ticks=1000+index,
                        argv=['/usr/bin/python3', '/fake/'+role+'.py'],
                        exe_sha256='f'*64, cwd='/fake')
        original[role] = identity
        state.processes[identity['pid']] = dict(identity=identity, alive=True, role=role)
        state.registry[role] = identity['pid']

    def alive(identity):
        process = state.processes.get(identity['pid'])
        return bool(process and process['alive'] and process['identity'] == identity)

    def tick():
        state.clock += .1
        runtime = state.runtime
        stamp = int(state.clock*10**9)
        runtime.pairs[stamp] = copy.deepcopy(pair)
        while len(runtime.pairs) > 12:
            del runtime.pairs[min(runtime.pairs)]

    def wait(check, seconds, label, guard=None):
        for _ in range(int(seconds*10)+2):
            if guard: guard()
            value = check()
            if value: return value
            tick()
        raise TimeoutError(label)

    def end_process(pid):
        process = state.processes[pid]
        process['alive'] = False
        if state.registry.get(process['role']) == pid:
            state.registry[process['role']] = None

    def foreign_camera():
        identity = dict(original['camera'], pid=999, start_ticks=9999)
        state.processes[999] = dict(identity=identity, alive=True, role='camera')
        state.registry['camera'] = 999

    def kill(pid, signal):
        role = state.processes[pid]['role']
        state.trace.append(('signal', role, pid))
        if state.scenario == 'partial_stop' and role == 'remote' and not state.stop_failed:
            state.stop_failed = True
            raise OSError('remote stop failed before signal')
        end_process(pid)

    class FakeRuntime(loader.Runtime):
        def __init__(self, manifest, out):
            self.before = copy.deepcopy(original)
            self.current = copy.deepcopy(original)
            self.report = dict(events=[], manifest_sha256='fake')
            self.out, self.units, self.environments = out, {}, {role:{} for role in original}
            self.configuration = copy.deepcopy(camera)
            self.new_configuration = copy.deepcopy(camera)
            self.verified_camera = self.verified_camera_settings = None
            self.lock, self.pairs, self.subscribers = threading.RLock(), {}, []
            self.allowed_env = []
            self.ros = NS(Time=NS(now=lambda:NS(to_nsec=lambda:int(state.clock*10**9))),
                          get_param=lambda key:copy.deepcopy(camera))
            self.helper = NS(alive=alive, identity=lambda pid:state.processes[pid]['identity'], wait=wait)
            state.runtime = self

        def verify(self):
            pass  # The subclass still checks the actual frozen wrapper hashes.

        def guard(self):
            pass

        def record(self, event, **fields):
            super().record(event, **fields)
            state.trace.append(('record', event))
            if event == 'known_camera_owner_exited' and state.scenario == 'dead_owner_race':
                foreign_camera()

        def start(self, role):
            # External launch boundary only; the orchestration under test must
            # decide whether this call is allowed and serialize its recovery.
            self.verify()
            if self.current.get(role) is not None or self.registered(role) is not None:
                raise RuntimeError('duplicate visual process would be created: '+role)
            unit = 'fake-'+role+'-'+str(len(state.units))
            command = loader.service_argv(role, self.before[role], {}, self.out, unit, [])
            self.record('start_requested', role=role, command=command)
            self.units[role] = unit
            pid = 1000+len(state.units)
            identity = dict(self.before[role], pid=pid, start_ticks=10000+pid,
                            argv=command[command.index('/usr/bin/python3'):])
            state.units[unit] = pid
            state.processes[pid] = dict(identity=identity, alive=True, role=role)
            state.trace.append(('start', role, pid))
            if role == 'camera':
                state.camera_starts += 1
                live = sum(p['alive'] and p['role']=='camera' for p in state.processes.values())
                state.max_live_cameras = max(state.max_live_cameras, live)
                if state.camera_starts == 1 and state.scenario.startswith('ambiguous'):
                    if state.scenario == 'ambiguous_foreign':
                        identity['argv'] = ['/usr/bin/python3', '/foreign.py']
                    # The systemd unit exists, but start has not learned its PID.
                    raise TimeoutError('ambiguous systemd launch')
            self.current[role] = identity
            state.registry[role] = pid
            if role == 'camera' and state.camera_starts == 1:
                stages = self.out / 'stages'; stages.mkdir()
                summary = dict(restoration={'succeeded':True}, original_read_restored=True,
                               original_sync_restored=True, files=[], experiment_valid=True)
                (stages/'summary.json').write_text(json.dumps(summary))

        def validate_camera(self, enabled):
            state.trace.append(('validate', state.camera_starts))
            if state.scenario == 'validation_failure':
                raise RuntimeError('camera validation failed')
            if state.camera_starts == 1 and state.scenario in ('dead_owner', 'dead_replacement', 'dead_owner_race'):
                end_process(self.current['camera']['pid'])
                if state.scenario == 'dead_replacement': foreign_camera()
                raise RuntimeError('known camera exited')
            self.verified_camera = copy.deepcopy(camera)
            if 'post_restore_warmup' in self.report:
                state.trace.append(('warmup_already_recorded',))

    def service(role, old, environment, directory, unit, allowed):
        return ['systemd-run', '--unit='+unit] + old['argv']

    def check_output(command, **kwargs):
        assert command[:3] == ['systemctl', '--user', 'show']
        pid = state.units[command[3]]
        return str(pid if state.processes[pid]['alive'] else 0)+'\n'

    def run(command, **kwargs):
        assert command[:3] == ['systemctl', '--user', 'stop']
        state.trace.append(('stop_unit', command[3]))
        if state.scenario == 'ambiguous_stop_failure':
            raise TimeoutError('unit exit unknown')
        end_process(state.units[command[3]])

    previous = tmp_path/'previous.json'
    previous.write_text(json.dumps(dict(loaded=True, after=original)))
    manifest = tmp_path/'manifest.json'; manifest.write_text('{}')
    (tmp_path/'ros_log').mkdir()
    loader.Runtime, loader.EXT, loader.MANIFEST = FakeRuntime, tmp_path, manifest
    loader.MOUNT, loader.verify_manifest = NS(is_mount=lambda:True), lambda value:None
    loader.service_argv = service
    stale = NS(registered_or_pending=lambda ros,name,old,helper:
               state.registry[next(role for role,node in loader.NODES.items() if node==name)])
    monkeypatch.setattr(M, 'module', lambda name,path:loader if name=='visual_loader' else stale)
    monkeypatch.setattr(M, 'PREVIOUS', previous)
    monkeypatch.setattr(M.time, 'monotonic', lambda:state.clock)
    monkeypatch.setattr(loader.os, 'kill', kill)
    monkeypatch.setattr(M.subprocess, 'run', run)
    monkeypatch.setattr(M.subprocess, 'check_output', check_output)
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    return state


def run_failure(rig, scenario):
    rig.scenario = scenario
    with pytest.raises((RuntimeError, TimeoutError, OSError, AssertionError)):
        M.main()
    return rig.runtime.report


def test_main_camera_validation_failure_keeps_consumers_stopped(main_rig):
    report = run_failure(main_rig, 'validation_failure')
    assert 'normal_camera_recovery_error' in report
    assert 'consumers remain quiesced' in report['consumer_recovery_error']
    assert not any(e[:2] in [('start','unknown'),('start','remote')] for e in main_rig.trace)
    assert main_rig.runtime.current['unknown'] is main_rig.runtime.current['remote'] is None


def test_main_ambiguous_launch_stops_owned_unit_before_second_camera(main_rig):
    report = run_failure(main_rig, 'ambiguous_owned')
    assert report['normal_camera_recovered_after_error'] and report['visual_nodes_restored']
    events = main_rig.trace
    stop = next(i for i,e in enumerate(events) if e[0]=='stop_unit')
    second_start = [i for i,e in enumerate(events) if e[:2]==('start','camera')][1]
    assert stop < second_start and main_rig.max_live_cameras == 1
    assert report['post_restore_warmup']['duration_sec'] >= 3
    assert report['post_restore_warmup']['distinct_complete_published_pairs'] >= 10


@pytest.mark.parametrize('scenario', ['ambiguous_foreign','ambiguous_stop_failure'])
def test_main_unproved_ambiguous_owner_never_starts_another_camera_or_consumers(main_rig, scenario):
    report = run_failure(main_rig, scenario)
    assert main_rig.camera_starts == 1 and 'normal_camera_recovery_error' in report
    assert not any(e[:2] in [('start','unknown'),('start','remote')] for e in main_rig.trace)


def test_main_known_dead_camera_is_cleared_and_recovered(main_rig):
    report = run_failure(main_rig, 'dead_owner')
    assert report['normal_camera_recovered_after_error'] and report['visual_nodes_restored']
    assert main_rig.camera_starts == 2 and main_rig.max_live_cameras == 1
    assert ('record','known_camera_owner_exited') in main_rig.trace
    assert not any(e[0]=='stop_unit' for e in main_rig.trace)
    assert report['post_restore_warmup']['distinct_complete_published_pairs'] >= 10


@pytest.mark.parametrize('scenario', ['dead_replacement','dead_owner_race'])
def test_main_dead_owner_does_not_erase_foreign_or_late_replacement(main_rig, scenario):
    report = run_failure(main_rig, scenario)
    assert main_rig.camera_starts == 1 and main_rig.registry['camera'] == 999
    assert main_rig.processes[999]['alive']
    assert 'normal_camera_recovery_error' in report
    assert not any(e[:2] in [('start','unknown'),('start','remote')] for e in main_rig.trace)


def test_main_partial_consumer_stop_does_not_duplicate_live_consumer(main_rig):
    report = run_failure(main_rig, 'partial_stop')
    assert report['visual_nodes_restored']
    assert sum(e[:2]==('start','unknown') for e in main_rig.trace) == 1
    assert not any(e[:2]==('start','remote') for e in main_rig.trace)


def test_main_success_warms_camera_before_serial_consumer_restore(main_rig):
    main_rig.scenario = 'success'
    M.main()
    report = main_rig.runtime.report
    assert report['experiment_valid'] and report['visual_nodes_restored']
    assert main_rig.camera_starts == 1
    assert report['post_restore_warmup']['duration_sec'] >= 3
    events = main_rig.trace
    final_validation = max(i for i,e in enumerate(events) if e[0]=='validate')
    unknown = next(i for i,e in enumerate(events) if e[:2]==('start','unknown'))
    remote = next(i for i,e in enumerate(events) if e[:2]==('start','remote'))
    assert final_validation < unknown < remote
