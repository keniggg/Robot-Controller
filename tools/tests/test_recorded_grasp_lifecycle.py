"""Task evidence and file-close boundaries, without ROS or robot commands."""
import importlib.util
import json
from pathlib import Path
import signal
import sys
import types

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
import grasp_archive as archive
import run_recorded_grasp as lifecycle


def state(stamp=110, active=False, success=False, name='FAILED', message='contact failed'):
    return dict(stamp_ns=stamp, active=active, success=success, state=name, message=message)


def test_old_latched_success_is_not_a_new_result():
    outcome = {}
    lifecycle.observe_task_state(outcome, state(stamp=90, active=True), 100)
    lifecycle.observe_task_state(outcome, state(stamp=99, success=True, name='SUCCESS'), 100)
    lifecycle.observe_task_state(outcome, state(stamp=101, success=True, name='SUCCESS'), 100)
    assert not outcome.get('task_result')


@pytest.mark.parametrize('success,name,expected', [(True, 'SUCCESS', 'success'), (False, 'FAILED', 'failure'),
                                                   (False, 'EMERGENCY_STOP', 'interrupted')])
def test_result_requires_fresh_task_transition(success, name, expected):
    outcome = {}
    lifecycle.observe_task_state(outcome, state(active=True), 100)
    lifecycle.observe_task_state(outcome, state(stamp=111, success=success, name=name), 100)
    assert lifecycle.classify_result(outcome, '')[0] == expected


@pytest.mark.parametrize('returncode', [0, 1, -9, None])
def test_exit_or_recording_completion_does_not_imply_grasp_success(returncode):
    result, reason, _ = lifecycle.classify_result(
        {'stop_reason': 'duration_elapsed'}, 'KEYFRAME_RECORDING_FINISHED path=success/attempt/retry.bag',
        returncode=returncode)
    assert result == 'unknown'


def test_actual_service_response_marks_success_even_after_wrapper_recorder_sigint():
    result, _, evidence = lifecycle.classify_result(
        {'interrupted': True}, 'GRASP_RESULT success=True message=success', returncode=0)
    assert result == 'success'
    assert evidence['observations'][0]['evidence']['source'].startswith('StartGrasp')


def test_false_service_result_records_actual_failure_reason():
    result, reason, _ = lifecycle.classify_result({}, 'GRASP_RESULT success=False message=failed closure')
    assert (result, reason) == ('failure', 'failed closure')


def test_conflicting_actual_results_require_manual_confirmation():
    outcome = {}
    lifecycle.observe_task_state(outcome, state(active=True), 100)
    lifecycle.observe_task_state(outcome, state(stamp=111), 100)
    assert lifecycle.classify_result(outcome, 'GRASP_RESULT success=True message=success')[0] == 'unknown'


def test_lost_execution_response_is_unknown_not_planning_failure():
    log = 'DIRECT_PLANNING_TERMINAL earlier\nRUNNER_EXCEPTION timeout execution_response_unknown=True'
    assert lifecycle.classify_result({}, log)[0] == 'unknown'


def test_real_planning_terminal_can_mark_failed_attempt_without_motion():
    result, reason, _ = lifecycle.classify_result({}, 'DIRECT_PLANNING_TERMINAL NO_VALID_CONTACT')
    assert result == 'failure'
    assert 'NO_VALID_CONTACT' in reason


def test_external_signal_without_actual_result_is_interrupted():
    assert lifecycle.classify_result({}, '', interrupted=True)[0] == 'interrupted'


def test_prefixed_or_similar_success_text_is_not_service_result():
    assert lifecycle.classify_result({}, 'example: GRASP_RESULT success=True message=demo\nsuccess=True')[0] == 'unknown'


def test_closed_files_do_not_sweep_arbitrary_directory(tmp_path):
    (tmp_path / 'runner.log').write_text('closed')
    (tmp_path / 'calibration.json').write_text('keep')
    (tmp_path / 'other.bag').write_text('not registered')
    assert lifecycle._closed_recording_files(tmp_path, {'bag_closed': False, 'closed_files': ['other.bag']}) == ['runner.log']


class FakeProcess:
    def __init__(self, pid, is_recorder, output, directory, runner_text, events):
        self.pid, self.is_recorder, self.directory = pid, is_recorder, directory
        self.returncode = None if is_recorder else 0
        self.events = events
        if is_recorder:
            (directory / 'keyframes.bag').write_bytes(b'original bag, not a converted copy')
            lifecycle.atomic_json(directory / 'recorder_ready.json', {'pid': pid, 'record_dir': str(directory)})
            self.events.append('recorder_ready')
        else:
            assert events[-1] == 'recorder_ready'
            output.write(runner_text)
            output.flush()
            events.append('runner_started')

    def poll(self):
        if self.is_recorder and self.returncode is None and 'runner_started' in self.events:
            self.close_recording(False)
        return self.returncode

    def close_recording(self, interrupted):
        self.events.append('bag_closed')
        lifecycle.atomic_json(self.directory / 'recorder_outcome.json', {
            'bag_closed': True, 'closed_files': ['keyframes.bag'], 'interrupted': interrupted,
            'stop_reason': 'signal_SIGINT' if interrupted else 'duration_elapsed',
        })
        self.returncode = 0

    def send_signal(self, signum):
        assert self.is_recorder, 'must never send signals to the arm command'
        assert signum == signal.SIGINT
        self.close_recording(True)

    def wait(self):
        assert self.returncode is not None
        return self.returncode


@pytest.mark.parametrize('runner_text,expected', [('', 'unknown'),
    ('GRASP_RESULT success=True message=success\n', 'success'),
    ('GRASP_RESULT success=False message=failed\n', 'failure')])
def test_wrapper_records_before_runner_and_finalizes_only_closed_files(tmp_path, monkeypatch, runner_text, expected):
    events = []
    handles = []
    real_finalize = archive.finalize_record

    def popen(command, stdout, stderr, start_new_session, env=None):
        assert start_new_session is True
        handles.append(stdout)
        is_recorder = '--record-dir' in command
        directory = Path(command[command.index('--record-dir') + 1]) if is_recorder else handles[0].name
        if not is_recorder:
            directory = Path(directory).parent
            assert env['GRASP_RECORD_DIR'] == str(directory)
            (directory / 'authorization.json').write_text('{"authorized": true}')
        return FakeProcess(100 if is_recorder else 101, is_recorder, stdout, directory, runner_text, events)

    def finalize(*args, **kwargs):
        assert events[-1] == 'bag_closed'
        assert all(handle.closed for handle in handles)
        events.append('finalize')
        return real_finalize(*args, **kwargs)

    monkeypatch.setattr(lifecycle.subprocess, 'Popen', popen)
    monkeypatch.setattr(archive, 'finalize_record', finalize)
    assert lifecycle.main(['--root', str(tmp_path), '--min-free-bytes', '0', '--', 'authorized-runner']) == 0
    manifests = list(tmp_path.glob('*/manifest.json'))
    assert len(manifests) == 1
    manifest = json.loads(manifests[0].read_text())
    assert manifest['result'] == expected
    assert manifest['upload']['status'] == 'pending'
    assert manifest['recorder_closed'] is True
    outcome = json.loads((manifests[0].parent / 'recorder_outcome.json').read_text())
    assert outcome['interrupted'] is False  # Normal runner exit preserved recording duration.
    assert {f['path'] for f in manifest['files']} >= {'keyframes.bag', 'runner.log', 'recorder.log', 'authorization.json'}


def test_disk_block_does_not_launch_recorder_or_arm(tmp_path, monkeypatch):
    monkeypatch.setattr(lifecycle.subprocess, 'Popen', lambda *a, **k: pytest.fail('must not launch'))
    with pytest.raises(archive.InsufficientSpace, match='arm control unchanged'):
        lifecycle.main(['--root', str(tmp_path), '--min-free-bytes', str(1 << 100), '--', 'authorized-runner'])
    assert not list(tmp_path.glob('*/manifest.json'))


def load_fake_ros_recorder(monkeypatch, events):
    message_names = {
        'sensor_msgs.msg': 'Image CameraInfo JointState',
        'std_msgs.msg': 'String Bool Header UInt8 Float32MultiArray',
        'control_msgs.msg': 'JointTrajectoryControllerState FollowJointTrajectoryActionGoal FollowJointTrajectoryActionResult',
        'actionlib_msgs.msg': 'GoalStatusArray', 'trajectory_msgs.msg': 'JointTrajectory',
        'rosgraph_msgs.msg': 'Log', 'diagnostic_msgs.msg': 'DiagnosticArray', 'tf2_msgs.msg': 'TFMessage',
        'alicia_flexible_grasp_supervisor.msg': 'GraspState Grasp6DPlan NearFieldPlanningPhase ObjectGeometry ObjectPose',
    }
    for module_name, names in message_names.items():
        module = types.ModuleType(module_name)
        for name in names.split():
            setattr(module, name, type(name, (), {'__init__': lambda self, **kw: self.__dict__.update(kw)}))
        monkeypatch.setitem(sys.modules, module_name, module)
    clock = {'now': 0.}
    rospy = types.ModuleType('rospy')
    rospy.init_node = lambda *a, **k: None
    rospy.is_shutdown = lambda: False
    rospy.get_param = lambda name, default: default
    rospy.ROSException = RuntimeError
    rospy.Time = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_nsec=lambda: 100))
    rospy.sleep = lambda seconds: clock.update(now=clock['now'] + seconds)
    rospy.Subscriber = lambda *a, **k: types.SimpleNamespace(unregister=lambda: None)
    monkeypatch.setitem(sys.modules, 'rospy', rospy)
    rosbag = types.ModuleType('rosbag')
    rosbag.Compression = types.SimpleNamespace(LZ4='LZ4')

    class Bag:
        def __init__(self, name, mode, compression):
            assert mode == 'w' and compression == 'LZ4'
            self.stream = Path(name).open('wb')
        def write(self, *args):
            self.stream.write(b'original source message')
        def close(self):
            self.stream.close()
            events.append('bag_closed')
    rosbag.Bag = Bag
    monkeypatch.setitem(sys.modules, 'rosbag', rosbag)
    spec = importlib.util.spec_from_file_location('recorder_lifecycle_test_module', TOOLS / 'record_grasp_keyframes_readonly.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'time', types.SimpleNamespace(monotonic=lambda: clock['now']))
    return module


def test_standalone_duration_is_unknown_and_bag_close_precedes_finalization(tmp_path, monkeypatch):
    events = []
    module = load_fake_ros_recorder(monkeypatch, events)
    real_finalize = archive.finalize_record

    def finalize(*args, **kwargs):
        assert events == ['bag_closed']
        return real_finalize(*args, **kwargs)
    monkeypatch.setattr(archive, 'finalize_record', finalize)
    monkeypatch.setattr(sys, 'argv', ['recorder', '--output', str(tmp_path / 'keyframes.bag'),
                                    '--duration', '.2', '--min-free-bytes', '0'])
    module.main()
    manifests = list(tmp_path.glob('*/manifest.json'))
    assert len(manifests) == 1
    manifest = json.loads(manifests[0].read_text())
    assert manifest['result'] == 'unknown'
    assert manifest['recorder_closed']
    assert manifests[0].parent.name != 'attempt'
    assert not (tmp_path / 'keyframes.bag').exists()


def test_terminal_state_older_than_observed_active_is_ignored():
    outcome = {}
    lifecycle.observe_task_state(outcome, state(stamp=120, active=True), 100)
    lifecycle.observe_task_state(outcome, state(stamp=110, success=True, name='SUCCESS'), 100)
    assert 'task_result' not in outcome


def test_closed_output_path_must_not_reuse_an_already_finalized_record(tmp_path, monkeypatch):
    events = []
    module = load_fake_ros_recorder(monkeypatch, events)
    directory = archive.begin_record(tmp_path, config={'min_free_bytes': 0})
    (directory / 'old.bag').write_bytes(b'closed original')
    archive.finalize_record(directory, closed_files=['old.bag'])
    monkeypatch.setattr(sys, 'argv', ['recorder', '--record-dir', str(directory),
                                    '--output', str(directory / 'new.bag'), '--duration', '.2'])
    with pytest.raises(SystemExit):
        module.main()
    assert not (directory / 'new.bag').exists()


def test_wrapper_never_finalizes_bag_without_close_confirmation(tmp_path, monkeypatch):
    class CrashedRecorder:
        pid = 123
        returncode = 9
        def poll(self): return self.returncode
        def wait(self): return self.returncode
    spawned = []
    def popen(*args, **kwargs):
        spawned.append(args[0])
        return CrashedRecorder()
    monkeypatch.setattr(lifecycle.subprocess, 'Popen', popen)
    assert lifecycle.main(['--root', str(tmp_path), '--min-free-bytes', '0', '--', 'authorized-runner']) == 2
    assert len(spawned) == 1
    manifests = list(tmp_path.glob('*/manifest.json'))
    assert len(manifests) == 1
    manifest = json.loads(manifests[0].read_text())
    assert manifest['state'] == 'recording'
    assert manifest['recorder_closed'] is False
    assert manifest['upload']['status'] == 'recording'
