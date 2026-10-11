"""GEN15 exact SDK handoff, followed by the real strict dispatch/rejection loop.

The recorded second-view endpoint was not retained. Endpoint geometry below is
an existing separately recorded physical fixture, explicitly a transport stand-
in. These tests do not reconstruct the lost goal or prove a GEN15 path.
"""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import ast
import hashlib
import os
import json
import math

import numpy as np
import pytest

from test_remote_modes import configured, base, module
from test_reachable_observation_integration import context, recorded_inputs, recorded_proposal
from alicia_grasp_modes import reachable_observation_views as proposals

FIXTURE = Path(__file__).with_name('fixtures') / 'gen15_recorded_seed_handoff.json'


def recorded_feedback():
    raw = FIXTURE.read_bytes()
    assert hashlib.sha256(raw).hexdigest() == 'b3c62600b5507f8cdd2802df270854a77423194a6409af7c31ff4b6a76de0ce9'
    return json.loads(raw)


def ros_stamp(ns):
    return base.rospy.Time(ns // 10**9, ns % 10**9)


def joint_message(row):
    return NS(name=list(row['name']), position=list(row['position']),
        header=NS(stamp=ros_stamp(row['source_stamp_ns'])))


def test_recorded_feedback_is_exactly_one_sdk_count_with_roundoff_not_extra_motion():
    data = recorded_feedback(); feedback = data['joint_feedback']
    before = feedback['seed']; after = feedback['accepted_after']
    assert before['source_stamp_ns'] == 1791716949198060573
    assert after['source_stamp_ns'] == 1791716949478806613
    np.testing.assert_array_equal(before['position'][1:6], after['position'][1:6])
    delta = after['position'][0] - before['position'][0]
    count = 2. * math.pi / 4096.
    # Strict <= one_count would reject this recorded one-count decrement.
    assert abs(delta) > count
    assert abs(delta + count) < abs(np.spacing(before['position'][0]))
    fresh = feedback['fresh_after_search']
    assert 0 < fresh['probe_clock_ns'] - fresh['latest_by_topic']['/joint_states']['source_stamp_ns'] < 50_000_000
    request = data['qualified_measurement_request']
    assert request['state'] == 'qualified' and request['selection']['generation'] == 15
    assert len(set(request['source_stamps_ns'])) == 3
    assert request['target_epoch'] == data['source_binding']['target_epoch']
    assert request['target_track_id'] == data['source_binding']['target_track_id']
    assert max(request['source_stamps_ns']) < data['source_binding']['snapshot_stamp_ns']


@pytest.mark.parametrize('recorded_one_count', [False, True])
def test_recorded_seed_handoff_reaches_strict_rejection_without_motion_authority(
        configured, monkeypatch, recorded_inputs, recorded_proposal, recorded_one_count):
    baseline = os.environ.get('GEN15_SEED_HANDOFF_BASELINE_SOURCE')
    if baseline:
        tree = ast.parse(Path(baseline).read_text())
        owner = next(n for n in tree.body if isinstance(n, ast.ClassDef)
            and n.name == 'ModeAwareRemoteGrasp6DNode')
        function = next(n for n in owner.body if isinstance(n, ast.FunctionDef)
            and n.name == '_joint_space_observation_candidates')
        namespace = dict(vars(module))
        exec(compile(ast.Module(body=[function], type_ignores=[]), baseline, 'exec'), namespace)
        monkeypatch.setattr(module.ModeAwareRemoteGrasp6DNode,
            '_joint_space_observation_candidates', namespace[function.name])
    data = recorded_feedback(); feedback = data['joint_feedback']
    # Preserve the independent physical scene; only the SDK handoff values and
    # times are GEN15. No claim that this endpoint was GEN15's discarded one.
    node, params, source, prepared, reference = context(configured, monkeypatch, recorded_inputs)
    names = ['Joint%d' % i for i in range(1, 7)]
    params['/alicia_controller/constraints'] = {n: {'trajectory': .2, 'goal': .035} for n in names}
    params['/robot/observation_tracking_contract_enabled'] = True
    params['/alicia_controller/stop_trajectory_duration'] = .5
    params['/gui/joint_direct_mode'] = False
    node._measurement_remote_epoch_ns = data['qualified_measurement_request']['control_epoch_ns']
    now = [feedback['seed']['source_stamp_ns'] + 1_000_000]
    monkeypatch.setattr(module.rospy.Time, 'now', lambda: ros_stamp(now[0]))
    node.latest_joint_state = joint_message(feedback['seed'])
    calls = []
    endpoint = deepcopy(recorded_proposal)
    def generated(**kwargs):
        calls.append(kwargs)
        np.testing.assert_array_equal(kwargs['current_joint_positions'], feedback['seed']['position'][:6])
        next_row = deepcopy(feedback['fresh_after_search']['latest_by_topic']['/joint_states'])
        if not recorded_one_count:
            next_row['position'][:6] = feedback['seed']['position'][:6]
        node.latest_joint_state = joint_message(next_row)
        now[0] = feedback['fresh_after_search']['probe_clock_ns']
        return (deepcopy(endpoint),), dict(elapsed_sec=2.0001683659938863,
            status='budget_exhausted', candidate_count=1, execution_authority=False)
    monkeypatch.setattr(proposals, 'generate_reachable_observation_views', generated)
    rows, report = node._joint_space_observation_candidates(prepared)
    assert rows, report  # Old exact q equality drops the recorded one-count case here.
    assert len(calls) == 1 and len(rows) == 1
    assert rows[0]['execution_authority'] is False and rows[0]['complete_path_checked'] is False
    profile = node._contact_stage_profile(None, None)
    sequence = NS(pregrasp=reference, approach=deepcopy(reference), grasp=deepcopy(reference),
        lift=deepcopy(reference), adaptive_stage_profile=profile, observation_view_audit={})
    variants, rejected = node._joint_space_observation_variants(reference, prepared.geometry,
        [0., 0., -1.], prepared.snapshot, prepared, (dict(sequence=sequence, preference_index=0),))
    assert len(variants) == 1 and not rejected and len(calls) == 1
    # Run the actual existing strict loop on the retained variant. Ticket
    # ownership transport is a local fixture; it never fabricates path proof.
    binding = source['binding']
    prepared.ticket = NS(request_id=binding['request_id'], generation=binding['generation'],
        target_epoch=binding['target_epoch'], snapshot_stamp_sec=prepared.snapshot.stamp_ns / 1e9)
    prepared.near_field = False
    checked_tickets = []
    def current_ticket(ticket):
        assert ticket is prepared.ticket
        checked_tickets.append(ticket)
    monkeypatch.setattr(node, '_require_stream_ticket_current', current_ticket)
    monkeypatch.setattr(node, '_require_stream_ticket_current_locked', current_ticket)
    monkeypatch.setattr(node, '_require_unknown_observation_prepared', lambda p: current_ticket(p.ticket))
    # Source lifetime is the independent scene fixture's current clock.
    node._stream_source_clock = lambda: prepared.ticket.snapshot_stamp_sec + .05
    node.execution_plan_validity_sec = 120.
    node.mujoco_selection_snapshot_reserve_sec = 30.
    node.observation_following_guard_required = True
    node._information_driven_observation_comparison = lambda: False
    runtime = dict(prepared=prepared, grasp_pose=reference, observation_sequence=variants[0]['sequence'],
        observation_variants=variants, soft_evidence={})
    node._stable_variant_runtime = {(31, 1): runtime}
    # This one-variant test retains base scheduling/budget mechanics and does
    # not spend or impersonate the separate advisory IK query service.
    monkeypatch.setattr(node, '_order_observation_checks', base.RemoteGrasp6DNode._order_observation_checks)
    strict_calls = []
    def strict(_self, target, observation=False):
        assert observation
        strict_calls.append((target, deepcopy(node.latest_joint_state)))
        return base.MoveItResult(False, 0., 0., 'fixture full path collision',
            failure_code='MOVEIT_COLLISION'), {}, ''
    monkeypatch.setattr(base.RemoteGrasp6DNode, '_strict_moveit_evaluation', strict)
    outcome = node._check_moveit_stable_candidate(NS(track_id=31, variant_index=1))
    assert len(strict_calls) == 1 and checked_tickets
    np.testing.assert_array_equal(base.pose_matrix(strict_calls[0][0]), base.pose_matrix(variants[0]['sequence'].pregrasp))
    expected_q = feedback['fresh_after_search']['latest_by_topic']['/joint_states']['position'][:6] if recorded_one_count else feedback['seed']['position'][:6]
    np.testing.assert_array_equal(strict_calls[0][1].position[:6], expected_q)
    assert not outcome.reachable
    checked = runtime['observation_orientation_search']['variant_results']
    assert len(checked) == 1 and checked[0]['search_family'] == 'joint_space_fk'
    assert checked[0]['checked'] and checked[0]['failure_code'] == 'MOVEIT_COLLISION'
    assert not any(getattr(p, 'valid', False) for p in node.rich_plan_pub.messages)
