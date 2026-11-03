"""Early view reports retain current box diagnostics without granting contact."""
from copy import deepcopy
import json
from types import SimpleNamespace

from test_remote_modes import configured
from test_measured_observation_planning import preflight_context
from test_gen31_unqualified_precision_routing import current_candidate
from alicia_grasp_modes.active_observation_policy import evidence_digest


BOX_FIELDS = ('regular_box_inference', 'regular_box_planning_enclosure',
              'current_measured_tool_for_box_precision')


def emitted_trigger(node, snapshot):
    report = {}
    node._observe_return_audit(report, SimpleNamespace(snapshot=snapshot,
        remote_diagnostics={'candidate_role': 'measured_observation'}))
    # Exercise the durable JSON report boundary, including nested candidates.
    encoded = json.loads(json.dumps(report, allow_nan=False))
    proof = encoded['observe_return']['trigger_evidence']
    assert proof['evidence_sha256'] == evidence_digest(
        {key: value for key, value in proof.items() if key != 'evidence_sha256'})
    assert proof['snapshot_stamp_ns'] == snapshot.stamp_ns
    assert proof['contact_or_motion_authorized'] is False
    assert proof['contact_search_completed'] is False
    assert encoded['observation_motion_performed'] is False
    return proof


def test_actual_unqualified_box_candidate_survives_early_trigger_and_report(configured, monkeypatch):
    node, ticket, snapshot, geometry, transform, diagnostics, _, _ = current_candidate(configured, monkeypatch)
    captured = node._regular_box_snapshot
    result = captured['result']
    diagnostics.update(regular_box_inference=dict(result.audit, ok=result.ok, code=result.code),
        regular_box_planning_enclosure=captured['enclosure'],
        current_measured_tool_for_box_precision=captured['current_tool'])
    expected = deepcopy(diagnostics['regular_box_inference'])
    assert node._maybe_prepare_observation_from_contact_geometry(
        ticket, snapshot, geometry, transform, (), diagnostics)
    proof = emitted_trigger(node, snapshot)
    assert proof['policy'] == 'current_request_box_precision_view_preflight'
    assert proof['regular_box_inference'] == expected
    candidate = proof['regular_box_inference']['precision_view_candidate']
    assert candidate == proof['targeted_precision_views'][0]['precision_view_candidate']
    assert candidate['motion_authorized'] is False
    assert proof['regular_box_planning_enclosure'] is None
    assert proof['current_measured_tool_for_box_precision'] is None
    assert node._current_inferred_box() is None
    assert not any(plan.valid for plan in node.rich_plan_pub.messages)
    # A later result or report mutation must not rewrite the source-bound proof.
    diagnostics['regular_box_inference']['precision_view_candidate'].clear()
    diagnostics['regular_box_inference']['code'] = 'later request'
    proof['regular_box_inference'].clear()
    assert emitted_trigger(node, snapshot)['regular_box_inference'] == expected


def test_measured_gap_retains_rejected_box_reason_without_using_it_as_authority(configured, monkeypatch):
    node, ticket, snapshot, geometry, transform, candidates, diagnostics, _ = preflight_context(configured, monkeypatch)
    # Transport fixture only: the production measured-gap decision still uses
    # the real synthetic surface, not these failure diagnostics.
    rejected = dict(ok=False, code='BOX_MEASURED_GEOMETRY_CONFLICT',
        reason='measured target extends outside inferred regular prism',
        measured_outside_count=4, source={'source_stamp_ns': snapshot.stamp_ns})
    diagnostics.update(regular_box_inference=rejected,
        regular_box_planning_enclosure=None,
        current_measured_tool_for_box_precision={'available': False, 'reason': 'not qualified'})
    assert node._maybe_prepare_observation_from_contact_geometry(
        ticket, snapshot, geometry, transform, candidates, diagnostics)
    proof = emitted_trigger(node, snapshot)
    assert proof['policy'] == 'current_request_measured_view_deficit_preflight'
    assert node._observe_return_trigger == 'BILATERAL_SURFACE_EVIDENCE_MISSING'
    assert proof['regular_box_inference'] == rejected
    assert proof['current_measured_tool_for_box_precision'] == diagnostics['current_measured_tool_for_box_precision']
    assert 'targeted_precision_views' not in proof
    assert proof['measured_missing_sides']


def test_missing_box_diagnostics_remain_absent_not_fabricated(configured, monkeypatch):
    node, ticket, snapshot, geometry, transform, candidates, diagnostics, _ = preflight_context(configured, monkeypatch)
    assert not any(key in diagnostics for key in BOX_FIELDS)
    assert node._maybe_prepare_observation_from_contact_geometry(
        ticket, snapshot, geometry, transform, candidates, diagnostics)
    proof = emitted_trigger(node, snapshot)
    assert not any(key in proof for key in BOX_FIELDS)
    assert proof['policy'] == 'current_request_measured_view_deficit_preflight'


def test_recorded_box_failure_does_not_override_available_contact(configured, monkeypatch):
    node, ticket, snapshot, geometry, transform, _, diagnostics, _ = preflight_context(configured, monkeypatch)
    diagnostics['regular_box_inference'] = {'ok': False, 'code': 'BOX_TOP_NOT_RECTANGULAR'}
    assert not node._maybe_prepare_observation_from_contact_geometry(
        ticket, snapshot, geometry, transform, (object(),), dict(diagnostics, materialized_count=1))
    assert node._observe_return_phase == 'initial_contact'
    assert node._observe_return_trigger_evidence is None
