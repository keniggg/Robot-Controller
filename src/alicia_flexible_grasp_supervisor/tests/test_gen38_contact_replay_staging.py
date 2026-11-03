"""Production receipt wiring with original CAD/SAT and external-only storage.

The affine FK and identity bias prediction are explicit adapter-unit doubles.
The real continuous checker, CAD boxes, SAT, retimer input, exceptions and
external store execute here. The service test also runs the actual handler and
MoveIt sequence wrapper with a recorded model/plan and inert planning adapters.
This is not a GEN35 chain, live MoveIt, or execution proof.
"""
import ast
import base64
from collections import OrderedDict
from collections.abc import Mapping
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import io
import json
import math
import os
from pathlib import Path
import time
import threading
import sys
import xml.etree.ElementTree as ET
from types import SimpleNamespace as NS

import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools/tests'))
from storage_os_adapter import storage_os, tmp_path
import rospy
from moveit_msgs.msg import RobotState, RobotTrajectory

from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_flexible_grasp_supervisor.srv import CheckContactSequenceRequest
from alicia_flexible_grasp.grasp import gripper_geometry
from alicia_flexible_grasp.robot import contact_sequence_precheck as production_precheck
from alicia_flexible_grasp.robot import observation_path_guard as production_guard
from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner

from test_gen38_contact_replay_receipt import m, inputs, Time as NanosecondTime


W = Path(__file__).resolve().parents[3]
PRODUCTION = W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot'
BASELINE = W/'.runtime/gen37_baseline_source'/PRODUCTION.relative_to(W)
REAL_PREDICT_CONTACT_COMMAND_PATH = production_precheck.predict_contact_command_path
REAL_POSE_VALUES = production_precheck.pose_values


@pytest.fixture(autouse=True)
def adapter_boundaries(monkeypatch):
    # These toy fixtures isolate the receipt adapter; the service test below
    # restores the real bias predictor and uses the recorded URDF/plan/FK.
    monkeypatch.setattr(production_precheck, 'predict_contact_command_path',
                        lambda nominal, *_: (deepcopy(nominal), {}))
    monkeypatch.setattr(production_precheck, 'pose_values', lambda value: value)


class Time(NanosecondTime):
    def to_sec(self):
        return self.ns*1e-9


def kernels(production):
    if production:
        assert Path(production_precheck.__file__).resolve() == PRODUCTION/'contact_sequence_precheck.py'
        assert Path(production_guard.__file__).resolve() == PRODUCTION/'observation_path_guard.py'
        assert Path(production_precheck.validate_contact_prediction.__code__.co_filename).resolve() == (
            PRODUCTION/'contact_sequence_precheck.py')
        assert production_precheck.validate_observation_trajectory is production_guard.validate_observation_trajectory
        ns = dict(vars(gripper_geometry))
        ns.update(vars(production_guard))
        ns.update(vars(production_precheck))
        return ns
    # Only the immutable GEN37 error-semantics comparator is extracted. Every
    # GEN38 subject under test above is imported from the production package.
    path = PRODUCTION.parent/'grasp/gripper_geometry.py'
    wanted = {'GripperGeometry', '_readonly_vector', '_finite_number', '_validated_rotation',
              'parse_tool_axis', 'gripper_box_centers', '_stage_boxes', '_box_corners', '_obb_overlap'}
    ns = dict(np=np, math=math, dataclass=dataclass, __name__=__name__)
    tree = ast.parse(path.read_text())
    body = [n for n in tree.body if (isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in wanted)
            or (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name)
                and t.id.startswith('ANALYTICAL_') for t in n.targets))]
    exec(compile(ast.Module(body=body, type_ignores=[]), str(path), 'exec'), ns)
    guard = BASELINE/'observation_path_guard.py'
    ns.update(OrderedDict=OrderedDict, deepcopy=deepcopy, Mapping=Mapping, hashlib=hashlib,
              io=io, time=time, ET=ET, stamp_nanoseconds=lambda stamp: stamp.to_nsec())
    tree = ast.parse(guard.read_text())
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,
        (ast.FunctionDef, ast.ClassDef, ast.Assign))], type_ignores=[]), str(guard), 'exec'), ns)
    precheck = BASELINE/'contact_sequence_precheck.py'
    ns.update(__file__=str(precheck), os=os, Path=Path, ARM_NAMES=m.ARM_NAMES,
        DiagnosticStore=m.DiagnosticStore, ReplayStorageUnavailable=m.ReplayStorageUnavailable,
        best_effort_failure_receipt=m.best_effort_failure_receipt,
        contact_failure_data=m.contact_failure_data,
        JointTrajectoryPoint=lambda **kwargs: NS(effort=[], time_from_start=Time(0), **kwargs),
        pose_values=lambda value: value)
    # Bias compensation is outside this adapter unit. The identity projection
    # is fixed for both GEN37 and production comparisons; it never skips SAT.
    ns['predict_contact_command_path'] = lambda nominal, *_: (deepcopy(nominal), {})
    names = {'ContactPrecheckError', '_emit_contact_replay_failure', '_ContactReplayLogStore',
             '_contact_replay_failure_sink', 'validate_contact_prediction',
             'validate_selected_pregrasp_path'}
    tree = ast.parse(precheck.read_text())
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,
        (ast.FunctionDef, ast.ClassDef)) and n.name in names], type_ignores=[]), str(precheck), 'exec'), ns)
    return ns


class AffineFk:
    names = m.ARM_NAMES
    limits = np.tile([-math.pi, math.pi], (6, 1))

    def __init__(self, model):
        self.model_sha256 = hashlib.sha256(model.encode()).hexdigest()

    def __call__(self, q):
        T = np.eye(4); T[:3, 3] = [float(q[0]), 0., .3]
        return T

    def point_motion_radii(self, _):
        return np.ones(6)


class Retimer:
    def _retime_cartesian_plan(self, curve):
        curve.joint_trajectory.header.seq = 22
        curve.joint_trajectory.points[1].time_from_start = Time(2000000111)
        curve.joint_trajectory.points[1].velocities = [.0123]*6
        curve.joint_trajectory.points[1].accelerations = [.00123]*6
        return curve

    def _same_joint_path_geometry(self, before, after):
        return [p.positions for p in before.joint_trajectory.points] == [
            p.positions for p in after.joint_trajectory.points]


def scene_fixture(ns, collide=True):
    values = inputs(); curve = values['checked_curve']
    curve.joint_trajectory.points = curve.joint_trajectory.points[:2]
    for i, p in enumerate(curve.joint_trajectory.points):
        p.positions = [.1*i, 0., 0., 0., 0., 0.]
        p.velocities = p.accelerations = [0.]*6
        p.time_from_start = Time(i*1000000000)
    sample = values['following_sample']
    sample['accepted_positions_rad'] = sample['sdk_positions_rad'] = [0.]*6
    g = ns['GripperGeometry'](.05, .002, np.array([.0434, .0286, .06]),
                              np.array([.1175, .155, .0774]), .003)
    fk = AffineFk(values['model_xml'])
    scene = values['scene']
    endpoint = fk(curve.joint_trajectory.points[-1].positions)
    scene.center = ns['_stage_boxes'](endpoint, g, .0498, 'y', 'z')[0][1]
    if not collide: scene.center = np.array([2., 2., 2.])
    scene.size = np.array([.008]*3)
    scene.normal = np.array([0., 0., 1.]); scene.rotation = np.eye(3)
    state = NS(joint_state=NS(name=list(m.ARM_NAMES), position=[0.]*6))
    plan = NS(plan_id=scene.plan_id, poses=[[.1, 0., .3, 0., 0., 0., 1.]]*4,
              contact_geometry_evidence='{}', diagnostic='CONTACT_EXECUTION_PLAN',
              model_choice='unknown_tabletop')
    return values, curve, g, fk, scene, state, plan


def sink(ns, values, g, scene, plan, audit):
    return ns['_contact_replay_failure_sink'](audit, plan, scene, values['model_xml'],
        values['binding']['control_epoch_ns'], values['binding']['selection'],
        values['following_sample'], values['config']['geometry'], values['config']['grasp'],
        dict(open_position_m=.05), g, .0498, 'current_accepted_open_posture')


def call_contact(ns, fixture, failure_sink=None):
    values, curve, g, fk, scene, state, plan = fixture
    kwargs = {} if failure_sink is None else dict(failure_sink=failure_sink)
    return ns['validate_contact_prediction'](curve, fk, values['following_sample'], plan.poses[2],
        scene, g, .0498, Retimer(), state, lambda *_args, **_kwargs: 0, lambda: 30., **kwargs)


def test_production_first_sample_preserves_original_collision_type_text_and_flags():
    original, staged = kernels(False), kernels(True)
    errors = []
    for ns in (original, staged):
        values, curve, g, fk, scene, _, _ = scene_fixture(ns)
        hold = NS(positions=[0.]*6, velocities=[0.]*6, accelerations=[0.]*6)
        with pytest.raises(ns['ObservationPathError']) as caught:
            ns['validate_observation_trajectory'](curve, scene, fk, g, .0498, hold)
        errors.append(caught.value)
    assert type(errors[0]).__name__ == type(errors[1]).__name__
    assert str(errors[0]) == str(errors[1]) == 'target OBB collision at bridge-end/left_finger'
    assert errors[0].candidate_rejection_code == errors[1].candidate_rejection_code == 'TARGET_OBB_COLLISION'
    assert errors[0].candidate_rejection_at_start == errors[1].candidate_rejection_at_start is False
    first = errors[1].contact_replay_sample
    assert first['available'] and first['label'] == 'bridge-end' and first['body'] == 'left_finger'
    np.testing.assert_allclose(first['joint_positions_rad'], [.1, 0., 0., 0., 0., 0.], rtol=0., atol=1e-15)
    np.testing.assert_allclose(first['T_base_tool0'], scene_fixture(staged)[3]([.1, 0, 0, 0, 0, 0]), rtol=0., atol=1e-15)


def test_production_contact_failure_saves_exact_retimed_curve_and_keeps_rejection(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros'))
    original, staged = kernels(False), kernels(True)
    f0, f1 = scene_fixture(original), scene_fixture(staged)
    with pytest.raises(original['ContactPrecheckError']) as baseline: call_contact(original, f0)
    audit = {}; callback = sink(staged, f1[0], f1[2], f1[4], f1[6], audit)
    with pytest.raises(staged['ContactPrecheckError']) as caught: call_contact(staged, f1, callback)
    assert caught.value.code == baseline.value.code == 'CONTACT_SEQUENCE_PRECHECK_FAILED'
    assert str(caught.value) == str(baseline.value)
    assert isinstance(caught.value.__cause__, staged['ObservationPathError'])
    assert 'contact_prediction' not in audit  # the interrupted assignment never completed
    receipt = audit['contact_failure_replay']
    assert receipt['status'] == 'stored' and receipt['stage'] == 'grasp' and receipt['curve'] == 'nominal'
    ref = receipt['reference']; restored = m.DiagnosticStore(ref['store_root']).get(ref)
    assert restored['checked_curve']['header']['seq'] == 22
    point = restored['checked_curve']['points'][1]
    assert point['time_from_start_ns'] == 2000000111
    assert point['velocities'] == [.0123]*6 and point['accelerations'] == [.00123]*6
    assert f1[1].joint_trajectory.header.seq == 17  # no mutation of nominal input
    assert restored['failure'] == dict(code=caught.value.code, message=str(caught.value), stage='grasp')
    assert restored['first_failed_sample']['curve'] == 'nominal'
    assert restored['first_failed_sample']['label'] == 'bridge-end'
    assert restored['config']['checker_options']['max_checks'] == 8192
    assert restored['scene']['source_ns'] == restored['binding']['source_stamp_ns']
    assert restored['execution_authorized'] is False


@pytest.mark.parametrize('storage', ['unset', 'outside_archive', 'io_error'])
def test_production_unavailable_storage_keeps_original_collision_and_reports_status(tmp_path, monkeypatch, storage):
    if storage == 'unset': monkeypatch.delenv('ROS_LOG_DIR', raising=False)
    elif storage == 'outside_archive': monkeypatch.setenv('ROS_LOG_DIR', str(W/'.runtime/forbidden_replay_fallback'))
    else:
        monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros'))
        def broken(*_args, **_kwargs): raise OSError('injected external write failure')
        monkeypatch.setattr(production_precheck.DiagnosticStore, 'put', broken)
    ns = kernels(True); f = scene_fixture(ns); audit = {}
    callback = sink(ns, f[0], f[2], f[4], f[6], audit)
    with pytest.raises(ns['ContactPrecheckError']) as caught: call_contact(ns, f, callback)
    assert caught.value.code == 'CONTACT_SEQUENCE_PRECHECK_FAILED'
    assert str(caught.value).startswith('nominal continuous geometry: target OBB collision at ')
    assert audit['contact_failure_replay']['status'] == 'unavailable'
    assert audit['contact_failure_replay']['original_error_message'] == str(caught.value)
    assert not (W/'.runtime/forbidden_replay_fallback').exists()


def test_production_pregrasp_failure_retains_same_exception_and_audit_before_assignment(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros'))
    ns = kernels(True); f = scene_fixture(ns); audit = {}; values, curve, g, fk, scene, state, plan = f
    callback = sink(ns, values, g, scene, plan, audit)
    with pytest.raises(ns['ObservationPathError']) as caught:
        ns['validate_selected_pregrasp_path'](curve, fk, values['following_sample'], scene, g,
            .0498, Retimer(), state, lambda *_args, **_kwargs: 0, lambda: 30., failure_sink=callback)
    assert str(caught.value) == 'target OBB collision at bridge-end/left_finger'
    receipt = audit['contact_failure_replay']
    assert receipt['status'] == 'stored' and receipt['stage'] == 'pregrasp' and receipt['curve'] == 'command'
    ref = receipt['reference']; restored = m.DiagnosticStore(ref['store_root']).get(ref)
    assert restored['checked_curve']['header']['seq'] == 17  # this helper does not retime
    assert restored['failure']['code'] == 'MOVEIT_CHECK_ERROR'  # unchanged sequence wrapper default


def test_production_context_capture_fault_is_only_diagnostic(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros'))
    ns = kernels(True); f = scene_fixture(ns); audit = {}
    # A malformed *diagnostic* context must not become a new planning gate.
    incomplete = NS(max_inner_gap_m=.05)
    callback = sink(ns, f[0], incomplete, f[4], f[6], audit)
    with pytest.raises(ns['ContactPrecheckError']) as caught: call_contact(ns, f, callback)
    assert caught.value.code == 'CONTACT_SEQUENCE_PRECHECK_FAILED'
    assert audit['contact_failure_replay']['status'] == 'unavailable'
    assert 'context capture unavailable' in audit['contact_failure_replay']['diagnostic_error_message']


def test_production_success_does_not_call_failure_sink_or_create_receipt():
    ns = kernels(True); f = scene_fixture(ns, collide=False); calls = []
    evidence = call_contact(ns, f, lambda *_args: calls.append(True))
    assert not calls and evidence['nominal_geometry'] and evidence['predicted_command_geometry']


def test_production_budget_stop_has_explicit_missing_fk_without_fabrication():
    ns = kernels(True); values, curve, g, fk, scene, _, _ = scene_fixture(ns)
    hold = NS(positions=[0.]*6, velocities=[0.]*6, accelerations=[0.]*6)
    with pytest.raises(ns['ObservationPathError']) as caught:
        ns['validate_observation_trajectory'](curve, scene, fk, g, .0498, hold, max_checks=0)
    sample = caught.value.contact_replay_sample
    assert sample['available'] is False and 'T_base_tool0' not in sample
    assert str(caught.value) == 'continuous path validation computation budget exhausted'


def test_actual_service_and_sequence_keep_failed_retimed_curve_after_exception_flattening(tmp_path, monkeypatch):
    """Recorded geometry, real handler/checker/wrapper; no planner or RPC client.

    The old failed retimed curve was not recorded. This explicitly reconstructed
    curve tests today's receipt, and does not claim to recover that old curve.
    GEN55 versions the old-box rejection separately from complete CAD checks;
    neither the reconstruction nor the CAD result changes the historical result.
    """
    from unittest import mock
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'ros'))
    monkeypatch.setattr(production_precheck, 'predict_contact_command_path',
                        REAL_PREDICT_CONTACT_COMMAND_PATH)
    monkeypatch.setattr(production_precheck, 'pose_values', REAL_POSE_VALUES)
    data = json.loads((Path(__file__).parent/'fixtures/contact_command_collision_20260930.json').read_text())
    plan = Grasp6DPlan(); plan.deserialize(base64.b64decode(data['plan_roswire_base64']))
    curve = RobotTrajectory(); curve.deserialize(base64.b64decode(data['nominal_roswire_base64']))
    for index, point in enumerate(curve.joint_trajectory.points):
        point.time_from_start = rospy.Duration(index*3)
        point.velocities = point.accelerations = [0.]*6
    sample = deepcopy(data['sample'])
    sample.update(accepted_gripper_opening_m=data['opening_m'],
                  accepted_stamp_ns=100000000001, sdk_stamp_ns=100000000000)
    params = {'/robot_description': data['robot_description'],
              '/grasp_mode/selection': dict(mode='unknown', strategy='observe_return', generation=4),
              '/grasp_6d/remote/gripper_geometry': data['geometry'],
              '/grasp': dict(unknown_contact_cartesian_compensation_enabled=True,
                             unknown_lift_after_close_enabled=False),
              '/gripper': dict(open_position_m=.05, open_skip_tolerance_m=.001)}
    monkeypatch.setattr(rospy, 'get_time', lambda: 100.)
    monkeypatch.setattr(rospy, 'get_param', lambda key, default=None: deepcopy(params.get(key, default)))

    class ReceiptPlanner(MoveItPlanner):
        @staticmethod
        def _planning_now_sec():
            return 100.

        def _plan_pose_from_start_state(self, *args, **kwargs):
            return deepcopy(curve), 'inert recorded planning adapter'

        def _plan_cartesian_from_start_state(self, *args, **kwargs):
            return deepcopy(curve), 'inert recorded planning adapter'

        def _retime_cartesian_plan(self, original):
            original.joint_trajectory.header.seq = 22
            original.joint_trajectory.points[1].time_from_start = rospy.Duration(3, 111)
            return original

        def _same_joint_path_geometry(self, before, after):
            return [p.positions for p in before.joint_trajectory.points] == [
                p.positions for p in after.joint_trajectory.points]

        def _sequence_stage_joint_path_metrics(self, *_):
            return dict(path_cost=.5, max_delta=.2, joint_duration_lower_bound_sec=1.)

    planner = ReceiptPlanner.__new__(ReceiptPlanner)  # never instantiate MoveIt clients
    planner.ready = True; planner.error = None
    planner.manipulator = NS(clear_pose_targets=lambda: None)
    state = RobotState(); state.joint_state.name = list(m.ARM_NAMES)+['right_finger']
    state.joint_state.position = list(sample['accepted_positions_rad'])+[data['opening_m']]
    planner.robot = NS(get_current_state=lambda: deepcopy(state))
    node = production_precheck.ContactSequencePrecheckGateway()
    node._control_reference_epoch_ns = 1
    node._planner_operation_lock = threading.RLock
    node._contact_precheck_sample = lambda *args, **kwargs: deepcopy(sample)
    node._contact_precheck_scene_digest = lambda: 'inert-frozen-planning-scene'
    node._unknown_pregrasp_velocity_limits = lambda _: None
    node._ensure_planner = lambda: planner
    collision_calls = []
    node._contact_command_collision_check = lambda *args, **kwargs: collision_calls.append(True)
    real_fk = production_precheck.SerialUrdfFk
    class LegacyBoxFk(real_fk):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            assert self.finger_cad_model is not None
            self.finger_cad_model = None
    # Preserve the real receipt/exception chain in explicit old-box mode.
    # The CAD proof is restored for the separate current-contract check below.
    with mock.patch.object(production_precheck, 'SerialUrdfFk', LegacyBoxFk):
        response = node.handle_contact_sequence_precheck(CheckContactSequenceRequest(
            plan=plan, deadline=rospy.Time.from_sec(110.)))
    assert not response.success and response.failure_code == 'CONTACT_SEQUENCE_PRECHECK_FAILED'
    assert response.failed_stage == 'grasp', response.message
    assert 'continuous geometry: target OBB collision at ' in response.message
    audit = json.loads(response.evidence_json)
    assert audit['success'] is False and audit['execution_authorized'] is False
    assert 'contact_prediction' not in audit and not hasattr(node, '_contact_precheck_cache')
    assert not collision_calls  # the real finite gripper collision fails first
    receipt = audit['contact_failure_replay']
    assert receipt['status'] == 'stored' and receipt['stage'] == 'grasp'
    ref = receipt['reference']; restored = m.DiagnosticStore(ref['store_root']).get(ref)
    assert restored['checked_curve']['header']['seq'] == 22
    assert restored['checked_curve']['points'][1]['time_from_start_ns'] == 3000000111
    assert restored['checked_curve']['points'][1]['velocities'] == [0.]*6
    assert restored['checked_curve']['points'][1]['accelerations'] == [0.]*6
    assert restored['failure']['message'] == response.message
    assert restored['failure']['stage'] == restored['first_failed_sample']['stage'] == response.failed_stage
    assert restored['binding']['plan_id'] == plan.plan_id
    assert restored['binding']['plan_sha256'] == production_guard.committed_plan_digest(plan)
    assert restored['binding']['source_stamp_ns'] == plan.header.stamp.to_nsec()
    original_replay_sources = {
        name: hashlib.sha256((PRODUCTION/name).read_bytes()).hexdigest()
        for name in ('contact_sequence_precheck.py', 'observation_path_guard.py')}
    assert {name: restored['config']['replay_source_sha256'][name]
            for name in original_replay_sources} == original_replay_sources
    assert restored['config']['replay_source_sha256'] == {
        **original_replay_sources,
        'finger_cad_separation.py': hashlib.sha256(
            (PRODUCTION/'finger_cad_separation.py').read_bytes()).hexdigest(),
        'gripper_geometry.py': hashlib.sha256(
            (PRODUCTION.parent/'grasp'/'gripper_geometry.py').read_bytes()).hexdigest()}
    assert restored['execution_authorized'] is False

    continuous = mock.Mock(wraps=production_guard.validate_observation_trajectory)
    collision = mock.Mock(return_value=1)
    node._contact_command_collision_check = collision
    with mock.patch.object(production_precheck, 'validate_observation_trajectory', continuous):
        cad_response = node.handle_contact_sequence_precheck(CheckContactSequenceRequest(
            plan=plan, deadline=rospy.Time.from_sec(110.)))
    assert cad_response.success, cad_response.message
    assert continuous.call_count == 2
    collision.assert_called_once()
    cad_audit = json.loads(cad_response.evidence_json)
    assert cad_audit['execution_authorized'] is False
    assert cad_audit['prediction_cache_hit'] is False
    assert 'contact_failure_replay' not in cad_audit
    for name in ('nominal_geometry', 'predicted_command_geometry'):
        checked = cad_audit['contact_prediction'][name]
        assert checked['segments'] == len(curve.joint_trajectory.points)-1
        assert checked['target_cad_projection']['sufficient_separation_proofs'] > 0
    for call in continuous.call_args_list:
        checked_curve = call.args[0].joint_trajectory
        assert checked_curve.header.seq == 22
        assert checked_curve.points[1].time_from_start.to_nsec() == 3000000111

    # Force a new check so a previous prediction cache cannot conceal rejection.
    node._contact_precheck_cache.clear()
    continuous.reset_mock()
    collision.reset_mock()
    collision.side_effect = production_precheck.ContactPrecheckError('isolated whole-arm collision')
    with mock.patch.object(production_precheck, 'validate_observation_trajectory', continuous):
        rejected = node.handle_contact_sequence_precheck(CheckContactSequenceRequest(
            plan=plan, deadline=rospy.Time.from_sec(110.)))
    assert not rejected.success and rejected.failure_code == 'CONTACT_SEQUENCE_PRECHECK_FAILED'
    assert rejected.failed_stage == 'grasp'
    assert 'isolated whole-arm collision' in rejected.message
    assert continuous.call_count == 2
    collision.assert_called_once()
    rejected_audit = json.loads(rejected.evidence_json)
    assert rejected_audit['success'] is False and rejected_audit['execution_authorized'] is False
    assert 'contact_prediction' not in rejected_audit
    assert not node._contact_precheck_cache
