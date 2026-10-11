"""Whole live scheduling wrapper regressions, with no ROS/hardware submission."""
from dataclasses import replace
from types import SimpleNamespace as NS
import pytest

from test_remote_modes import configured, module, base
from test_observation_fk_first_strict import loop_context
from test_information_first_path_coverage import ordinary_search
from alicia_grasp_modes.silhouette_observation_order import ObservationCheckSchedule


def test_schedule_inspection_does_not_dispatch_or_freeze_incumbent_transition():
    rows = [object() for _ in range(4)]
    evidence = {'dispatched_original_indices': []}
    state = {'qualified': False, 'validations': 0}
    def validate(): state['validations'] += 1
    schedule = ObservationCheckSchedule(rows, [2, 0, 3, 1], evidence, validate, lambda: state['qualified'])
    assert schedule.initial_order == (rows[2], rows[0], rows[3], rows[1])
    assert min(range(len(schedule.initial_order))) == 0
    assert evidence['dispatched_original_indices'] == [] and state['validations'] == 0
    schedule.reorder_before_incumbent((rows[3], rows[2], rows[0], rows[1]))
    assert next(schedule) is rows[3]
    state['qualified'] = True
    assert tuple(schedule) == tuple(rows[:3])
    assert evidence['dispatched_original_indices'] == [3, 0, 1, 2]
    assert evidence['full_path_incumbent_found']


def test_started_schedule_cannot_be_rewritten():
    rows=[object(), object()]
    schedule=ObservationCheckSchedule(rows,[1,0],{'dispatched_original_indices':[]},lambda:None,lambda:False)
    next(schedule)
    with pytest.raises(ValueError): schedule.reorder_before_incumbent(rows)


@pytest.mark.parametrize('has_fk', [False, True])
def test_outer_family_wrapper_keeps_dynamic_coverage_until_full_qualification(configured, monkeypatch, tmp_path, has_fk):
    node, prepared, variants, runtime, candidate, clock, checked = ordinary_search(configured, monkeypatch, tmp_path)
    monkeypatch.setattr(node, '_joint_space_observation_active', lambda: True)
    if has_fk:
        variants[2]['search_family']='joint_space_fk'
        variants[3]['search_family']='joint_space_fk'
    for v in variants: v.setdefault('search_family', 'camera_orientation_lattice')
    # No fresh comparison selects the real FK fallback wrapper; all original
    # source-bound ordering and actual strict loop still execute.
    node._observation_information_comparison = None
    outcome=node._check_moveit_stable_candidate(candidate)
    assert not outcome.reachable and checked
    assert len(checked)==4
    assert runtime['observation_orientation_search']['checked_variant_count']==4
    assert runtime['observation_orientation_search']['unchecked_variant_count']==4
    assert runtime['observation_orientation_search']['search_complete'] is False
    assert len(runtime['observation_orientation_search']['variant_results'])==8


def test_fk_first_failure_family_balancing_keeps_lazy_return_to_information_rank(configured, monkeypatch, tmp_path):
    node, prepared, runtime, rows, candidate, checker, events, hints, clock = loop_context(
        configured, monkeypatch, tmp_path, count=8, first_failure='following')
    result = checker(candidate)
    assert result.reachable
    strict = [value for kind, value in events if kind=='strict']
    assert strict[:3]==[1,2,0]
    assert len(hints)==1
    coverage=runtime['unknown_observation_scheduling']['first_incumbent_coverage']
    assert coverage['full_path_incumbent_found']
    assert coverage['family_balanced_first_path_order'][0] != 0
    assert coverage['dispatched_original_indices'][0] != 0
    assert coverage['dispatched_original_indices'][1] == 0
    assert len(set(strict))==len(strict)


def test_empty_dispatch_is_incomplete_and_counts_all_unchecked_candidates(configured, monkeypatch, tmp_path):
    node, _, variants, runtime, candidate, *_ = ordinary_search(configured, monkeypatch, tmp_path)
    monkeypatch.setattr(node, '_order_observation_checks', lambda _: iter(()))
    result=node._check_moveit_stable_candidate(candidate)
    assert not result.reachable and 'checked 0/8' in result.reason
    report=runtime['observation_orientation_search']
    assert report['configured_variant_count']==8 and report['checked_variant_count']==0
    assert report['unchecked_variant_count']==8 and report['search_complete'] is False


def test_gen7_recorded_116_four_fk_rows_reach_real_outer_strict_loop(configured, monkeypatch, tmp_path):
    from pathlib import Path
    from copy import deepcopy
    import json
    import hashlib
    import numpy as np
    from alicia_grasp_modes.silhouette_observation_order import transform_fingerprint
    fixture=Path(__file__).parent/'fixtures/gen7_mixed_family_empty_dispatch.json'
    assert hashlib.sha256(fixture.read_bytes()).hexdigest() == 'ea5cb329b4a0a18c6e14b63606cb90dbe002c6e0fa03439fa04ee8f0f7227aad'
    data=json.loads(fixture.read_text()); frozen=data['rows']
    assert len(frozen)==116 and sum(r['search_family']=='joint_space_fk' for r in frozen)==4
    node, prepared, runtime, rows, candidate, checker, events, hints, clock = loop_context(
        configured, monkeypatch, tmp_path, count=116)
    statuses={}
    for row, captured in zip(rows, frozen):
        matrix=np.asarray(captured['base_from_tool'])
        pose=base.make_pose_stamped('base_link',matrix[:3,3],base.quaternion_from_matrix(matrix),stamp=prepared.stamp)
        row['sequence'].pregrasp=pose
        row.update({key:captured[key] for key in ['search_family','camera_target_distance_m',
            'camera_tilt_offset_deg','camera_roll_offset_deg','preference_index']})
        row['observation_view']['information_utility']=deepcopy(captured['information_utility'])
        row['sequence'].observation_view_audit=deepcopy(row['observation_view'])
        statuses[transform_fingerprint(base.pose_matrix(pose))]=captured['support_status']
    runtime['grasp_pose']=rows[0]['sequence'].pregrasp
    # Source wrapper and service replies remain controlled. Recorded exact
    # endpoint transforms, families and support tiers exercise full dispatch;
    # this does not assert any of these recorded paths is executable.
    schedule=NS(evaluate=lambda pose:dict(status=statuses[transform_fingerprint(pose)]),
        report=lambda:dict(candidates_removed=0))
    monkeypatch.setattr(node,'_observation_support_schedule',lambda _:schedule)
    def rejected(_self,target,observation=False):
        index=next(i for i,v in enumerate(rows) if v['sequence'].pregrasp is target)
        events.append(('strict',index));clock[0]+=.8
        return base.MoveItResult(False,0.,0.,'controlled original rejection',failure_code='MOVEIT_UNREACHABLE'),{},''
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_strict_moveit_evaluation',rejected)
    before=[base.pose_matrix(r['sequence'].pregrasp).copy() for r in rows]
    outcome=checker(candidate)
    assert not outcome.reachable
    actual=[i for kind,i in events if kind=='strict']
    assert len(actual)==4 and rows[actual[0]]['search_family']=='joint_space_fk'
    assert len(set(actual))==4
    report=runtime['observation_orientation_search']
    assert report['configured_variant_count']==116
    assert report['checked_variant_count']==4 and report['unchecked_variant_count']==112
    assert report['search_complete'] is False
    assert len(report['variant_results'])==116
    assert all(np.array_equal(base.pose_matrix(r['sequence'].pregrasp),v) for r,v in zip(rows,before))
    assert not any(message.valid for message in node.rich_plan_pub.messages)
