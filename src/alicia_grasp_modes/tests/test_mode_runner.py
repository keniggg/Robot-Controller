"""Mode-aware single-attempt admission and audit binding without live services."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import types

import pytest

from alicia_grasp_modes.runner_contract import (
    selection_audit_error, direct_audit_error, audit_matches_preview)

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'mode_aware_grasp_runner.py'
spec = importlib.util.spec_from_file_location('mode_runner_under_test', str(SCRIPT))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def evidence():
    selection = dict(mode='unknown', strategy='direct', generation=2, stamp_ns=19000000000)
    poses = [types.SimpleNamespace(
        position=types.SimpleNamespace(x=.1*i, y=.02, z=.3),
        orientation=types.SimpleNamespace(x=0., y=0., z=0., w=1.)) for i in range(4)]
    preview = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(secs=20, nsecs=10), frame_id='base_link'),
        poses=poses, plan_id='a'*24, diagnostic='CONTACT_EXECUTION_PLAN', valid=True,
        model_choice='unknown_tabletop', required_open_width_m=.04)
    gate = dict(ok=True, failure_code='', failure_reason='', required_open_width_m=.04,
                center_distance_m=.001, support_clearance_m=.003, jaw_alignment=1.,
                motion_cost=.1, geometry_cost=.1, failed_gate='', passed_gate_count=6)
    report = dict(
        plan_id=preview.plan_id, grasp_mode_selection=selection, snapshot_stamp_ns=20000000010,
        execution_strategy='direct', planning_origin='current_view_contact_snapshot',
        observation_motion_performed=False, outcome=dict(preview_valid=True),
        moveit_selection=dict(policy='BOUNDED_APERTURE_THEN_HARDWARE_DURATION', checked_count=3, reachable_count=2),
        replay_geometry=dict(available=True, object_points_count=1000, object_points_sha256='b'*64),
        selected=dict(selected=True, moveit=dict(reachable=True, collision_free=True,
            within_joint_limits=True, ik_valid=True, planning_success=True, failure_code=''),
            final_execution_sequence_checked=True, final_strict_plan_id=preview.plan_id,
            final_geometry_gate=gate, execution_sequence=dict(available=True, kind='near_field_contact',
                stages=[dict(stage=name, frame_id='base_link', position_m=[p.position.x,.02,.3],
                    quaternion_xyzw=[0.,0.,0.,1.], linear_from_prior_state=i>0)
                    for i, (name,p) in enumerate(zip(('pregrasp','approach','grasp','lift'), poses))])))
    return selection, report, preview


def test_real_contact_contract_is_accepted_without_observation_audit():
    selection, report, preview = evidence()
    assert selection_audit_error(report, selection, 20000000000) == ''
    assert direct_audit_error(report) == ''
    assert audit_matches_preview(report, preview, True)
    assert module.original.FreshPreviewRunner.audit_error(report)


@pytest.mark.parametrize('key,value', [('generation',3), ('strategy','two_stage'), ('mode','carton')])
def test_identical_contact_geometry_does_not_authorize_another_selection(key, value):
    selection, report, _ = evidence()
    changed = dict(selection, **{key:value})
    assert selection_audit_error(report, changed, 20000000000)


@pytest.mark.parametrize('mutation', ['observation', 'weak_moveit', 'stale_final_plan', 'missing_geometry',
                                     'partial_geometry', 'fake_geometry_ok', 'missing_points'])
def test_partial_or_observation_evidence_cannot_authorize_direct(mutation):
    _, report, _ = evidence()
    selected = report['selected']
    if mutation == 'observation':
        selected['execution_sequence']['kind'] = 'far_field_observation'
        selected['execution_sequence']['stages'] = selected['execution_sequence']['stages'][:1]
    elif mutation == 'weak_moveit':
        selected['moveit']['collision_free'] = None
    elif mutation == 'stale_final_plan':
        selected['final_strict_plan_id'] = 'c'*24
    elif mutation == 'missing_geometry':
        del selected['final_geometry_gate']
    elif mutation == 'partial_geometry':
        selected['final_geometry_gate']['passed_gate_count'] = 5
    elif mutation == 'fake_geometry_ok':
        selected['final_geometry_gate']['ok'] = 'true'
    elif mutation == 'missing_points':
        report['replay_geometry']['object_points_count'] = 0
    assert direct_audit_error(report)


@pytest.mark.parametrize('mutation', ['grasp_pose', 'lift_pose', 'stamp', 'width', 'phase', 'frame', 'short_position'])
def test_all_contact_poses_and_exact_evidence_match_published_plan(mutation):
    _, report, preview = evidence()
    if mutation == 'grasp_pose':
        preview.poses[2].position.x += .001
    elif mutation == 'lift_pose':
        preview.poses[3].position.z += .001
    elif mutation == 'stamp':
        # A float seconds conversion must not silently erase source nanoseconds.
        report['snapshot_stamp_ns'] = 20000000000
    elif mutation == 'width':
        preview.required_open_width_m = .045
    elif mutation == 'phase':
        preview.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    elif mutation == 'frame':
        report['selected']['execution_sequence']['stages'][3]['frame_id'] = 'camera_link'
    elif mutation == 'short_position':
        report['selected']['execution_sequence']['stages'][0]['position_m'] = [0.]
    assert not audit_matches_preview(report, preview, True)


@pytest.mark.parametrize('mode', ['carton','unknown'])
def test_two_stage_validates_mode_specific_observation_band_and_original_gates(mode):
    selection, report, preview = evidence()
    selection['mode'] = mode
    selection['strategy'] = 'two_stage'
    report['moveit_selection'] = dict(policy='FIRST_REACHABLE_BY_AUTHORITATIVE_RANK',
                                     checked_count=1, reachable_count=1)
    selected = report['selected']
    selected['execution_sequence']['kind'] = 'far_field_observation'
    selected['execution_sequence']['stages'] = selected['execution_sequence']['stages'][:1]
    selected['execution_sequence']['stages'][0]['stage'] = 'observation'
    selected['observation_view'] = dict(actual_camera_target_distance_m=.20,
                                      min_camera_target_distance_m=.18, max_camera_target_distance_m=.22)
    selected['observation_envelope'] = dict(ok=True, minimum_support_clearance_m=.003)
    selected['moveit']['joint_max_delta_rad'] = .3
    if mode == 'unknown':
        selected['observation_view'] = dict(actual_camera_target_distance_m=.30,
            min_camera_target_distance_m=.22, max_camera_target_distance_m=.30)
    preview.diagnostic = 'FAR_FIELD_OBSERVATION_PLAN'
    runner = module.ModeAwareGraspRunner()
    runner.selection, runner.minimum_stamp_ns = selection, 20000000000
    assert runner.audit_error(report) == ''
    assert runner.audit_matches_preview(report, preview)
    selected['observation_view']['actual_camera_target_distance_m'] = .31
    assert runner.audit_error(report)
    selected['observation_view']['actual_camera_target_distance_m'] = .30 if mode == 'unknown' else .20
    selected['observation_envelope']['minimum_support_clearance_m'] = .002
    assert runner.audit_error(report)
    selected['observation_envelope']['minimum_support_clearance_m'] = .003
    selected['moveit']['joint_max_delta_rad'] = 3.1
    if mode == 'unknown':
        assert runner.audit_error(report)
    else:
        assert not runner.audit_error(report)


def fake_run(monkeypatch, start):
    selection, report, preview = evidence()
    runner = module.ModeAwareGraspRunner('unknown', 'direct')
    params = {module.SELECTION_PARAM: deepcopy(selection), '/grasp_6d/plan_validity_sec':120.}
    calls = []
    monkeypatch.setattr(module.rospy, 'init_node', lambda *a, **k: None)
    monkeypatch.setattr(module.rospy, 'wait_for_service', lambda *a, **k: None)
    monkeypatch.setattr(module.rospy, 'Subscriber', lambda *a, **k: None)
    monkeypatch.setattr(module.rospy, 'get_param', lambda name, default=None: params.get(name, default))
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time(20)))
    runner.wait_for_preview = lambda timeout: preview
    runner.committed_audits[preview.plan_id] = report
    runner.wait_for_execution_authority = lambda *a: types.SimpleNamespace(success=True)
    def request(trigger):
        calls.append(('inference', trigger))
        return types.SimpleNamespace(success=True, message='ok')
    def execute(flag, plan_id):
        calls.append(('start', flag, plan_id))
        assert calls == [('inference', True), ('start', True, preview.plan_id)]
        return start(runner, params)
    services = {'/grasp_6d/request_plan':request, '/grasp_6d/replan_execution':lambda *a: None,
                '/grasp/current_plan':lambda *a: types.SimpleNamespace(success=True,
                     message='plan_id=%s validation=VALID' % preview.plan_id), '/grasp/start':execute}
    monkeypatch.setattr(module.rospy, 'ServiceProxy', lambda name,*a: services[name])
    return runner, params, calls


def test_runner_keeps_inference_until_bound_task_response(monkeypatch):
    runner, _, calls = fake_run(monkeypatch,
        lambda runner, params: types.SimpleNamespace(success=True, message='done'))
    assert runner.run() == 0
    assert calls[-1] == ('inference', False)
    assert not runner.execution_inflight


def test_ambiguous_execution_response_does_not_revoke_inflight_target(monkeypatch):
    def disconnected(*args):
        raise RuntimeError('response connection lost after execution started')
    runner, _, calls = fake_run(monkeypatch, disconnected)
    with pytest.raises(RuntimeError):
        runner.run()
    runner.stop_candidate_computation()
    assert runner.execution_inflight
    assert ('inference', False) not in calls


def test_selection_change_before_start_cannot_launch_or_stop_new_mode(monkeypatch):
    runner, params, calls = fake_run(monkeypatch, lambda *a: pytest.fail('stale selection executed'))
    def changed(*args):
        params[module.SELECTION_PARAM] = dict(params[module.SELECTION_PARAM], generation=3)
        return types.SimpleNamespace(success=True)
    runner.wait_for_execution_authority = changed
    assert runner.run() == 5
    assert calls == [('inference', True)]
