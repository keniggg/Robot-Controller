"""Real station joins and supplemental delivery; every motion hook stays inert."""
from copy import deepcopy
from types import SimpleNamespace as NS
import json

import pytest

from test_station_compute_wait_task import (
    station, configured, frame, deliver, advance, feedback, base, at_ns)
from alicia_flexible_grasp.grasp.rich_plan_integrity import compute_plan_id
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest

KEY = 'station_supplemental_observation_computation'


def computed(c):
    n = c.node; witnesses = []; parent = None
    for ns in (100_000_000_000, 100_400_000_000, 100_800_000_000,
               101_200_000_000, 101_600_000_000):
        c.mono[0] += (ns + 50_000_000 - c.now[0]) / 1e9
        c.now[0] = ns + 50_000_000
        feedback(c, ns - 20_000_000); feedback(c, ns + 20_000_000)
        row = frame(c, ns, parent); deliver(c, row)
        parent = row['status']['station_compute_wait']
        witnesses.append(deepcopy(n._station_contact_sources[ns]))
    plan = deepcopy(c.plan)
    plan.header.stamp = at_ns(101_600_000_000)
    plan.object_geometry.header = deepcopy(plan.header)
    plan.plan_id = compute_plan_id(plan)
    receipt = dict(version=1, scope='registered_station_supplemental_observation_provenance',
        motion_authorized=False, metric_geometry_authority=False,
        plan_id=plan.plan_id, plan_sha256=committed_plan_digest(plan),
        plan_source_ns=plan.header.stamp.to_nsec(),
        ticket=dict(request_id=23, generation=9, target_epoch=4),
        sources=witnesses, current=deepcopy(witnesses[-1]), issued_ns=c.now[0],
        snapshot_policy=dict(purpose='supplementary_observation', phase='observation_prefix',
                             frame_count=5, maximum_span_sec=12.))
    report = {KEY: receipt, 'observe_return': {'phase': 'supplemental_observation'}}
    # This fixture enters the real supplemental consumer but stops before any
    # admission service, path transfer or movement. Those original gates must run.
    n._observe_return_session['max_observations'] = 3
    return plan, report, parent


def clock_notice(c, parent):
    m = base.Grasp6DPlan(); m.header.stamp = at_ns(c.now[0]); m.header.frame_id = 'base_link'
    m.valid = False; m.candidate_source = 'near_field_pending'
    m.diagnostic = 'TARGET_PROCESSING_PENDING: ' + json.dumps(dict(
        station_compute_wait=parent, station_compute_recovery_chain=None, clock_only=True))
    m.target_track_id = c.plan.target_track_id
    return m


def recover(c, parent):
    ns = 102_500_000_000
    c.mono[0] += (ns + 50_000_000 - c.now[0]) / 1e9; c.now[0] = ns + 50_000_000
    feedback(c, ns-20_000_000); feedback(c, ns+20_000_000)
    row = frame(c, ns, parent); deliver(c, row)
    assert c.node._station_task_record and not c.node._station_task_record['pending']


@pytest.mark.parametrize('audit_order', ['before_preview', 'after_pending', 'after_recovery'])
def test_clock_notice_and_delayed_audit_do_not_lose_supplement(station, monkeypatch, audit_order):
    c = station; n = c.node; plan, report, parent = computed(c)
    if audit_order == 'before_preview': n._observe_return_reports[plan.plan_id] = report
    n.grasp6d_preview_plan_cb(plan)
    advance(c, parent['geometry_deadline_ns'] + 20_000_000)
    n.grasp6d_preview_plan_cb(clock_notice(c, parent))
    if audit_order == 'after_pending': n._observe_return_reports[plan.plan_id] = report
    assert n.latest_grasp6d_preview_plan.valid is False
    assert n._before_near_field_terminal_recovery(None, c.plan, {}, None, None) == {'waiting': True}
    recover(c, parent)
    if audit_order == 'after_recovery': n._observe_return_reports[plan.plan_id] = report
    calls = []
    monkeypatch.setattr(n, '_diagnostic_observation_checkpoint',
                        lambda p, cfg, label: calls.append(label) or False)
    result = n._before_near_field_terminal_recovery(None, c.plan, {}, lambda: None, lambda: None)
    assert calls == ['supplemental observation admission']
    assert result == {'failed': True}
    assert n.latest_grasp6d_preview_plan.valid is False
    assert n._bound_execution_plan == c.plan
