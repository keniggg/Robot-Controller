"""Offline-only contract tests: module import creates no ROS node/publisher."""
import ast
from collections import deque
import importlib.util
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

PATH = Path(__file__).with_name('bounded_joint3_response_diagnostic.py')
spec = importlib.util.spec_from_file_location('joint3_diagnostic', PATH)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def vectors(direction=1):
    base = np.array([0., -.6, 1.1, 0., -.8, 0., .04975])
    target = base.copy()
    target[2] += direction*22*mod.QUANTUM
    return base, target


@pytest.mark.parametrize('direction', [-1, 1])
def test_quantized_outbound_and_exact_return(direction):
    base, target = vectors(direction)
    mod.validate_joint3_command(target, base, base, 'outbound', 0)
    mod.validate_joint3_command(base, target, base, 'return_original_sdk', 1)
    assert 0 < abs(target[2]-base[2]) < math.radians(2.)


@pytest.mark.parametrize('mutation', ['larger', 'zero', 'j1', 'j2', 'j4', 'j5', 'j6', 'gripper', 'nan', 'wrong_label', 'third', 'changed_baseline', 'false_return'])
def test_reject_invalid_command(mutation):
    base, target = vectors()
    reference = base.copy()
    label, count = 'outbound', 0
    if mutation == 'larger': target[2] = base[2]+23*mod.QUANTUM
    elif mutation == 'zero': target[2] = base[2]
    elif mutation.startswith('j'): target[int(mutation[1:])-1] += mod.QUANTUM
    elif mutation == 'gripper': target[6] -= .001
    elif mutation == 'nan': target[2] = float('nan')
    elif mutation == 'wrong_label': label = 'return_original_sdk'
    elif mutation == 'third': count = 2
    elif mutation == 'changed_baseline': reference[2] += mod.QUANTUM
    elif mutation == 'false_return':
        reference = target.copy(); target = base.copy(); target[2] += mod.QUANTUM
        label, count = 'return_original_sdk', 1
    with pytest.raises(RuntimeError):
        mod.validate_joint3_command(target, reference, base, label, count)


def message(q, source_ns, frame):
    m = mod.JointState()
    m.name = mod.NAMES
    m.position = q.tolist()
    m.header.stamp = mod.rospy.Time(source_ns//10**9, source_ns%10**9)
    m.header.frame_id = frame
    return m


def guarded(monkeypatch):
    base, _ = vectors()
    now_ns = 10_000_000_000
    monkeypatch.setattr(mod.rospy.Time, 'now', lambda: mod.rospy.Time(10, 0))
    monkeypatch.setattr(mod.rospy, 'is_shutdown', lambda: False)
    obj = mod.Diagnostic.__new__(mod.Diagnostic)
    obj.lock = threading.RLock()
    obj.deadline = time.monotonic()+10
    obj.fault = None
    obj.bootstrap_phase = None
    obj.joint3_response_bounds = (base[2]-math.radians(4.8), base[2]+math.radians(4.8))
    obj.epoch = 8_000_000_000
    obj.frozen_epoch = obj.epoch
    obj.state = {'status': 'CONFIRMED:MEASURED_DIRECTIONAL_RESPONSE',
                 'status_received': time.monotonic(),
                 '/alicia_d/feedback_ready': True,
                 '/alicia_d/protection_latched': False,
                 '/gui/joint_direct_mode': True,
                 'task_active': False,
                 'accepted_received': time.monotonic()}
    obj.accepted = deque([message(base, now_ns-10_000_000, 'sdk_measured')])
    obj.reference = deque([message(base, now_ns-10_000_000, 'sdk_manual')])
    return obj


def test_guard_accepts_confirmed_fresh_same_epoch(monkeypatch):
    guarded(monkeypatch).guard()


@pytest.mark.parametrize('mutation', ['pending', 'stale_sdk', 'future_sdk', 'pre_epoch_sdk', 'sdk_tracking', 'not_measured', 'stale_accepted', 'manual_false', 'task_active', 'expired_deadline'])
def test_guard_rejects_invalid_state(monkeypatch, mutation):
    obj = guarded(monkeypatch)
    if mutation == 'pending': obj.state['status'] = 'PENDING:POSITIVE_ENABLE_REQUESTED'
    elif mutation == 'stale_sdk': obj.reference[-1].header.stamp = mod.rospy.Time(9, 600_000_000)
    elif mutation == 'future_sdk': obj.reference[-1].header.stamp = mod.rospy.Time(10, 1)
    elif mutation == 'pre_epoch_sdk': obj.epoch = 9_995_000_000
    elif mutation == 'sdk_tracking': obj.reference[-1].header.frame_id = 'sdk_tracking'
    elif mutation == 'not_measured': obj.accepted[-1].header.frame_id = 'bad'
    elif mutation == 'stale_accepted': obj.accepted[-1].header.stamp = mod.rospy.Time(9, 600_000_000)
    elif mutation == 'manual_false': obj.state['/gui/joint_direct_mode'] = False
    elif mutation == 'task_active': obj.state['task_active'] = True
    elif mutation == 'expired_deadline': obj.deadline = time.monotonic()-1
    with pytest.raises(RuntimeError): obj.guard()


def test_prepare_only_cannot_call_execute_step():
    obj = mod.Diagnostic.__new__(mod.Diagnostic)
    obj.args = SimpleNamespace(execute=False)
    obj.publisher = None
    with pytest.raises(RuntimeError, match='prepare-only'):
        obj.execute_step('outbound', vectors()[1])


def test_single_publish_is_j3_one_hot_and_counted(monkeypatch):
    obj = guarded(monkeypatch)
    base, target = vectors()
    obj.args = SimpleNamespace(execute=True, joint=3)
    obj.started_ns = 9_000_000_000
    obj.guard = lambda: None
    obj.stationary = lambda stamp: {}
    obj.snapshot = lambda label: {'Y': np.eye(4), 'X': np.eye(4), 'source_stamp_ns': 9_990_000_000}
    obj.path = lambda *args: {}
    obj.driver_connection = lambda: None
    obj.baseline_wire_q = base
    obj.expected_words = mod.wire_words(base)
    obj.expected_gripper = base[6]
    obj.publish_count = 0
    messages = []
    class Published(Exception): pass
    obj.publisher = SimpleNamespace(get_num_connections=lambda: 1, publish=messages.append)
    def event(name, **values):
        if name == 'command': raise Published()
    obj.event = event
    with pytest.raises(Published): obj.execute_step('outbound', target)
    assert obj.publish_count == 1 and len(messages) == 1
    m = messages[0]
    assert m.header.frame_id == 'gui_direct'
    assert m.effort == [0.,0.,1.,0.,0.,0.,0.]
    assert m.name == mod.NAMES and m.position == target.tolist()


def test_no_bootstrap_sync_enable_or_more_publish_sites():
    text = PATH.read_text()
    tree = ast.parse(text)
    assert not any(isinstance(n, ast.FunctionDef) and n.name == 'bootstrap_once' for n in ast.walk(tree))
    assert 'gui_direct_sync' not in text and "'--bootstrap-pending'" not in text
    assert text.count('self.publisher.publish(msg)') == 1
    assert 'signal.setitimer(signal.ITIMER_REAL,57.)' in text
    assert 'self.deadline = time.monotonic()+56' in text


def offline_path_fixture():
    """Current archived URDF, synthetic board pose; never live geometry approval."""
    import json
    root = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
    urdf = (root/'bounded_joint_response_20260928T104443Z_12672/robot_description.urdf').read_text()
    context = json.loads((root/'grasp_two_stage_diagnostic_20260928T093247Z/initial_context.json').read_text())
    obj = mod.Diagnostic.__new__(mod.Diagnostic)
    obj.args = SimpleNamespace(joint=3)
    obj.fk = mod.SerialUrdfFk(urdf, mod.NAMES[:6])
    obj.gripper = mod.GripperGeometry(.05, .002, np.array([.0434,.0286,.06]), np.array([.1175,.155,.0774]), .003)
    obj.lo = np.array([-2.16,-1.57,-.5,-3.14,-1.57,-3.14])
    obj.hi = np.array([2.16,1.57,2.35619,3.14,1.5,3.14])
    checked = []
    def validity(request):
        checked.append(np.asarray(request.robot_state.joint_state.position))
        return SimpleNamespace(valid=True)
    obj.validity = validity
    q = np.asarray(context['joint_positions'])
    obj.joint3_response_bounds = (q[2]-math.radians(4.8), q[2]+math.radians(4.8))
    he = context['handeye']
    X = mod.quaternion_matrix(he['rotation_xyzw']); X[:3,3] = he['translation_xyz']
    model = mod.CAMERA.RuntimeColorModel(mod.CAMERA.runtime_color_model_evidence(context['camera'], 'camera_link', mod.RGB))
    # Explicit synthetic fixture: a 180x135 mm board centered 300 mm forward.
    board_in_camera = np.eye(4)
    board_in_camera[:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]
    board_in_camera[:3,3] = [.3,.09,.0675]
    geometry = dict(X=X, Y=obj.fk(q[:6])@X@board_in_camera, model=model)
    return obj, q, geometry, checked


@pytest.mark.parametrize('direction', [-1, 1])
def test_j3_both_direction_reservation_uses_actual_urdf(direction):
    obj, q, geometry, checked = offline_path_fixture()
    target = q.copy(); target[2] += direction*22*mod.QUANTUM
    details = obj.path(q, target, geometry)
    values = np.asarray(checked)
    assert len(checked) == 51
    assert details['minimum_plane_clearance_m'] >= .04
    assert details['minimum_full_board_edge_px'] >= 20
    assert np.allclose(np.min(values[:,2])-q[2], -math.radians(4.8))
    assert np.allclose(np.max(values[:,2])-q[2], math.radians(4.8))
    assert np.allclose(values[:,[0,1,3,4,5,6]], q[[0,1,3,4,5,6]], rtol=0, atol=1e-15)


@pytest.mark.parametrize('failure', ['target_far', 'joint_limit', 'board_edge', 'support_clearance', 'collision'])
def test_path_rejects_j3_invalid_envelope(failure):
    obj, q, geometry, checked = offline_path_fixture()
    target = q.copy(); target[2] += 22*mod.QUANTUM
    if failure == 'target_far': target[2] = q[2]+math.radians(3.6)
    elif failure == 'joint_limit': obj.hi[2] = q[2]+math.radians(4.7)
    elif failure == 'board_edge': geometry['Y'][:3,3] += (obj.fk(q[:6])@geometry['X'])[:3,1] * .4
    elif failure == 'support_clearance': geometry['Y'][:3,3] -= (obj.fk(q[:6])@geometry['X'])[:3,0] * .18
    elif failure == 'collision': obj.validity = lambda _: SimpleNamespace(valid=False)
    with pytest.raises(RuntimeError): obj.path(q, target, geometry)


@pytest.mark.parametrize('response', ['valid_j3', 'j3_overshoot', 'j3_wrong_direction', 'j2_moved', 'wrong_sdk_axis', 'gripper_changed'])
def test_actual_j3_response_and_wire_rejections(monkeypatch, response):
    obj = guarded(monkeypatch)
    base, target = vectors()
    obj.args = SimpleNamespace(execute=True, joint=3)
    obj.started_ns = 9_000_000_000
    obj.guard = lambda: None
    obj.stationary = lambda stamp: {}
    obj.snapshot = lambda label: {'Y': np.eye(4), 'X': np.eye(4), 'source_stamp_ns': 9_990_000_000}
    obj.path = lambda *args: {}
    obj.driver_connection = lambda: None
    obj.stable_hold_recording = lambda seconds: None
    obj.baseline_wire_q = base
    obj.expected_words = mod.wire_words(base)
    obj.expected_gripper = base[6]
    obj.publish_count = 0
    obj.event = lambda *args, **kwargs: None
    sent = []
    def publish(command):
        sent.append(command)
        actual = target.copy()
        reference = target.copy()
        if response == 'j3_overshoot': actual[2] = base[2]+math.radians(4.9)
        elif response == 'j3_wrong_direction': actual[2] = base[2]-math.radians(.51)
        elif response == 'j2_moved': actual[1] += math.radians(.41)
        elif response == 'wrong_sdk_axis': reference[0] += mod.QUANTUM
        elif response == 'gripper_changed': reference[6] -= .0001
        obj.accepted.append(message(actual, 10_000_000_001, 'sdk_measured'))
        obj.reference.append(message(reference, 10_000_000_001, 'sdk_manual'))
    obj.publisher = SimpleNamespace(get_num_connections=lambda: 1, publish=publish)
    if response == 'valid_j3': obj.execute_step('outbound', target)
    else:
        with pytest.raises(RuntimeError): obj.execute_step('outbound', target)
    assert len(sent) == obj.publish_count == 1


@pytest.mark.parametrize('source_ns,accepted', [(9_650_000_000, True), (9_649_999_999, False), (10_000_000_001, False), (7_999_999_999, False)])
def test_prepublication_board_age_and_epoch(monkeypatch, source_ns, accepted):
    obj = guarded(monkeypatch)
    base, target = vectors()
    obj.args = SimpleNamespace(execute=True, joint=3)
    obj.started_ns = 9_000_000_000
    obj.guard = lambda: None
    obj.stationary = lambda stamp: {}
    obj.snapshot = lambda label: {'Y': np.eye(4), 'X': np.eye(4), 'source_stamp_ns': source_ns}
    obj.path = lambda *args: {}
    obj.driver_connection = lambda: None
    obj.baseline_wire_q = base
    obj.expected_words = mod.wire_words(base)
    obj.expected_gripper = base[6]
    obj.publish_count = 0
    messages = []
    class Published(Exception): pass
    obj.publisher = SimpleNamespace(get_num_connections=lambda: 1, publish=messages.append)
    def event(name, **values):
        if name == 'command': raise Published()
    obj.event = event
    if accepted:
        with pytest.raises(Published): obj.execute_step('outbound', target)
        assert len(messages) == obj.publish_count == 1
    else:
        with pytest.raises(RuntimeError, match='precommand board source'): obj.execute_step('outbound', target)
        assert not messages and obj.publish_count == 0


@pytest.mark.parametrize('stale_count,success', [(1, True), (2, True), (3, False)])
def test_fresh_reacquisition_is_bounded_and_sends_once(monkeypatch, stale_count, success):
    obj = guarded(monkeypatch)
    base, target = vectors()
    obj.args = SimpleNamespace(execute=True, joint=3)
    obj.started_ns = 9_000_000_000
    obj.guard = lambda: None
    obj.stationary = lambda stamp: {}
    labels = []
    def snapshot(label):
        labels.append(label)
        precommand_count = sum('precommand' in name for name in labels)
        source = 9_649_999_999 if 0 < precommand_count <= stale_count else 9_990_000_000
        return {'Y': np.eye(4), 'X': np.eye(4), 'source_stamp_ns': source}
    obj.snapshot = snapshot
    obj.path = lambda *args: {}
    obj.driver_connection = lambda: None
    obj.baseline_wire_q = base
    obj.expected_words = mod.wire_words(base)
    obj.expected_gripper = base[6]
    obj.publish_count = 0
    messages = []
    class Published(Exception): pass
    obj.publisher = SimpleNamespace(get_num_connections=lambda: 1, publish=messages.append)
    def event(name, **values):
        if name == 'command': raise Published()
    obj.event = event
    original_deadline = obj.deadline
    if success:
        with pytest.raises(Published): obj.execute_step('outbound', target)
        assert len(messages) == obj.publish_count == 1
        assert sum('precommand' in x for x in labels) == stale_count+1
    else:
        with pytest.raises(RuntimeError, match='three bounded acquisitions'): obj.execute_step('outbound', target)
        assert not messages and obj.publish_count == 0
        assert sum('precommand' in x for x in labels) == 3
    assert obj.deadline == original_deadline


@pytest.mark.parametrize('direction', [-1, 1])
def test_return_reproves_original_absolute_interval_without_expansion(direction):
    obj, initial, geometry, checked = offline_path_fixture()
    outbound = initial.copy(); outbound[2] += direction*22*mod.QUANTUM
    first = obj.path(initial, outbound, geometry)
    first_values = np.asarray(checked).copy(); checked.clear()
    # The board is still fixed, but the camera is now at the outbound position.
    returned = obj.path(outbound, initial, geometry)
    return_values = np.asarray(checked)
    assert first['response_absolute_joint3_range_rad'] == returned['response_absolute_joint3_range_rad']
    assert np.min(return_values[:,2]) == np.min(first_values[:,2])
    assert np.max(return_values[:,2]) == np.max(first_values[:,2])
    assert np.min(return_values[:,2]) == obj.joint3_response_bounds[0]
    assert np.max(return_values[:,2]) == obj.joint3_response_bounds[1]
    # In particular, the return upper limit never becomes outbound + 4.8 degrees.
    assert np.max(return_values[:,2]) == initial[2]+math.radians(4.8)


@pytest.mark.parametrize('side', [-1, 1])
def test_return_current_or_target_outside_original_interval_is_rejected(side):
    obj, initial, geometry, checked = offline_path_fixture()
    outbound = initial.copy(); outbound[2] += side*math.radians(2.)
    escaped = initial.copy(); escaped[2] += side*math.radians(4.81)
    # This would have been inside the old, recentered +/-4.8-degree interval.
    assert abs(escaped[2]-outbound[2]) < math.radians(4.8)
    with pytest.raises(RuntimeError, match='original absolute'):
        obj.path(escaped, initial, geometry)
    with pytest.raises(RuntimeError, match='original absolute'):
        obj.path(outbound, escaped, geometry)
    assert checked == []


@pytest.mark.parametrize('side', [-1, 1])
def test_each_feedback_latches_escape_even_if_latest_sample_returns_inside(monkeypatch, side):
    obj = guarded(monkeypatch)
    base, _ = vectors()
    obj.closed = False
    obj.pair_changed = threading.Condition(obj.lock)
    obj.write_bag = lambda *args: None
    escaped = base.copy(); escaped[2] += side*math.radians(4.81)
    obj.receive('/alicia_d/accepted_joint_states', message(escaped, 9_999_999_999, 'sdk_measured'))
    assert 'original absolute' in obj.fault
    obj.receive('/alicia_d/accepted_joint_states', message(base, 10_000_000_000, 'sdk_measured'))
    with pytest.raises(RuntimeError, match='original absolute'):
        obj.guard()


def test_fixed_envelope_is_required_before_geometry_approval():
    obj, initial, geometry, checked = offline_path_fixture()
    obj.joint3_response_bounds = None
    with pytest.raises(RuntimeError, match='has not been initialized'):
        obj.path(initial, initial, geometry)
    assert not checked
