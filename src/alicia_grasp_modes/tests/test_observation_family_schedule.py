"""Fair proposal scheduling preserves source strata and all existing check gates."""
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from test_remote_modes import configured, module, base
from test_observe_return_remote import setup_observe_return

FK='joint_space_fk'
LATTICE='camera_orientation_lattice'
POLICY='measured_support_strata_then_fk_lattice_round_robin'
RECORD=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261009T014405.965804Z_6ffa3ee69c35414abb29d86c19a84a1b/keyframes_audits/25113e950719ebc45f10837670177f392545e7b4f75a169a5c0dc59278c52b5b.json')


def variant(index,family=LATTICE,support='sufficient'):
    audit=dict(check_scheduling_index=index,check_scheduling_policy='original_source_rank',
        check_scheduling_metrics=dict(measured_support_schedule=dict(status=support)),
        endpoint_schedule_proves_joint_motion=False,endpoint_schedule_proves_path_visibility=False,
        endpoint_schedule_is_global_optimum=False)
    pose=base.PoseStamped();pose.pose.position.x=index
    return dict(search_family=family,sequence=NS(pregrasp=pose,observation_view_audit=deepcopy(audit)),
        observation_view=deepcopy(audit),camera_tilt_offset_deg=45.,camera_roll_offset_deg=0.,preference_index=index)


def interleave(rows):return module.ModeAwareRemoteGrasp6DNode._interleave_observation_families(rows)


def test_recorded_212_candidates_give_both_families_an_early_turn_without_mutating_source():
    if not RECORD.exists():pytest.skip('sealed local experimental audit is not distributed')
    raw=RECORD.read_bytes();assert hashlib.sha256(raw).hexdigest()==RECORD.stem
    source=json.loads(raw)['observation_failure_diagnostics']['observation_candidates']
    ordered=tuple(sorted(source,key=lambda v:v['observation_view']['check_scheduling_index']))
    before=deepcopy(ordered)
    assert len(ordered)==212
    assert [i for i,v in enumerate(ordered) if v['search_family']==FK]==[140,141]
    result=interleave(ordered)
    assert result[0] is ordered[140] and result[1] is ordered[0]
    assert result[2] is ordered[141] and result[3] is ordered[1]
    assert {id(v) for v in result}=={id(v) for v in ordered} and len(result)==212
    for family in (FK,LATTICE):
        assert [id(v) for v in result if v['search_family']==family]==[id(v) for v in ordered if v['search_family']==family]
    assert ordered==before


def test_weaker_support_never_jumps_a_stronger_support_stratum():
    rows=tuple([variant(0),variant(1),variant(2,FK,'unknown'),variant(3,LATTICE,'unknown'),
                variant(4,LATTICE,'insufficient'),variant(5,FK,'insufficient')])
    result=interleave(rows)
    assert [v['preference_index'] for v in result]==[0,1,2,3,5,4]
    assert Counter(map(id,result))==Counter(map(id,rows))


@pytest.mark.parametrize('families',[[],[FK,FK],[LATTICE,LATTICE],[FK,LATTICE,'future_family']])
def test_absent_family_or_unrecognized_third_family_keeps_exact_original_order(families):
    rows=tuple(variant(i,family) for i,family in enumerate(families))
    assert interleave(rows) is rows


def test_implicit_legacy_lattice_family_is_retained():
    rows=(variant(0),variant(1,FK),variant(2))
    rows[0].pop('search_family');rows[2].pop('search_family')
    assert interleave(rows)==(rows[1],rows[0],rows[2])


def ordinary(configured):
    node,params,clock=setup_observe_return(configured)
    node._observe_return_phase='observation_prefix'
    node._observe_return_trigger='BILATERAL_SURFACE_EVIDENCE_MISSING'
    node._observe_return_paused=False
    node.grasp_config={}
    return node


def test_ordinary_bilateral_wrapper_preserves_exact_endpoints_and_records_old_new_ranks(configured,monkeypatch):
    node=ordinary(configured);rows=tuple(variant(i,FK if i==2 else LATTICE) for i in range(4))
    poses=[r['sequence'].pregrasp for r in rows]
    runtime={'prepared':NS(ticket=object()),
        'unknown_observation_scheduling':{'policy':'original_source_rank','candidate_count':4,'pruned_candidate_count':0}}
    node._active_unknown_observation_check_runtime=runtime
    monkeypatch.setattr(node,'_order_unknown_observation_checks',lambda candidates,**kwargs:rows)
    result=tuple(node._order_observation_checks(rows))
    assert [r['preference_index'] for r in result]==[2,0,1,3]
    report=runtime['unknown_observation_scheduling']['family_check_scheduling']
    assert report['ordered_original_indices']==[2,0,1,3]
    assert not report['execution_authority'] and not report['strict_budget_extended'] and report['candidates_removed']==0
    for index,row in enumerate(result):
        audit=row['observation_view']
        assert row['sequence'].pregrasp is poses[row['preference_index']]
        assert audit==row['sequence'].observation_view_audit
        assert audit['check_scheduling_index']==index and audit['check_scheduling_policy']==POLICY
        assert audit['family_original_check_index']==row['preference_index']
        assert not audit['endpoint_schedule_proves_joint_motion'] and not audit['endpoint_schedule_proves_path_visibility']


@pytest.mark.parametrize('mode',['silhouette','initial_contact','observing_contact','returned_contact','direct','carton','other_trigger'])
def test_other_modes_never_enter_family_balancing(configured,monkeypatch,mode):
    node=ordinary(configured);rows=(variant(0),variant(1,FK))
    if mode=='silhouette':node._observe_return_diagnostic_observation_only=True
    elif mode=='direct':node._mode_selection['strategy']='direct'
    elif mode=='carton':node._mode_selection['mode']='carton'
    elif mode=='other_trigger':node._observe_return_trigger='ANOTHER_REASON'
    else:node._observe_return_phase=mode
    monkeypatch.setattr(node,'_order_unknown_observation_checks',lambda candidates:rows)
    monkeypatch.setattr(base.RemoteGrasp6DNode,'_order_observation_checks',lambda self,candidates:rows)
    monkeypatch.setattr(node,'_balance_observation_family_checks',lambda *a:pytest.fail('unrelated mode entered balancing'))
    assert node._order_observation_checks(rows)==rows


@pytest.mark.parametrize('per_check,expected_count',[(1.01,3),(.1,6)])
def test_fairness_cannot_extend_three_second_or_six_check_budget_and_still_calls_full_path_gate(per_check,expected_count):
    from test_remote_grasp6d_streaming import streaming_node,MutableClock,snapshot,remote_node
    node=streaming_node(clock=MutableClock(50.),start_worker=False)
    node.execution_plan_validity_sec=120.;node.observation_following_guard_required=True
    node._order_observation_checks=interleave
    try:
        node.start_streaming();node.submit_stream_snapshot(snapshot(49.8));ticket=node._stream_worker_ticket
        node.mujoco_selection_snapshot_reserve_sec=30.;elapsed=MutableClock(0.);node._observation_tilt_clock=elapsed
        rows=tuple(variant(i,FK if i in (7,8) else LATTICE) for i in range(10))
        runtime=dict(prepared=NS(ticket=ticket),grasp_pose=rows[0]['sequence'].pregrasp,
            observation_sequence=rows[0]['sequence'],observation_variants=rows,soft_evidence={})
        node._stable_variant_runtime={(3,1):runtime};strict=[];qualified=[]
        def check(pose,observation=False):
            assert observation is True
            strict.append(int(pose.pose.position.x));elapsed.value+=per_check
            return remote_node.MoveItResult(True,1.,1.,'planned'),{},''
        def full_gate(prepared,result,pose):
            qualified.append(int(pose.pose.position.x))
            return remote_node.MoveItResult(False,1.,1.,'original full-path gate rejected',failure_code='UNKNOWN_OBSERVATION_PATH_VIEW_INVALID'),{'ok':False}
        node._strict_moveit_evaluation=check;node._observation_following_support_evaluation=full_gate
        result=node._check_moveit_stable_candidate(NS(track_id=3,variant_index=1))
        assert strict==qualified and strict[:2]==[7,0] and len(strict)==expected_count
        assert not result.reachable
        assert node._observation_tilt_search_budget['checked_count']==expected_count
        count=len(strict);node._check_moveit_stable_candidate(NS(track_id=3,variant_index=1))
        assert len(strict)==count  # Ranking another time cannot renew the shared request budget.
        assert base.OBSERVATION_TILT_STRICT_BUDGET_SEC==3.
        assert base.OBSERVATION_TILT_MAX_STRICT_CHECKS==6
        assert base.OBSERVATION_COMPARISON_AFTER_INCUMBENT_SEC==8.
    finally:node.shutdown_streaming_worker()
