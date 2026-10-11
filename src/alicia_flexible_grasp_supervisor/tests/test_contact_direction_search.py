"""Ordering regression; no ROS, IK, trajectory execution or hardware claims."""
import ast
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT/'src/alicia_flexible_grasp/grasp/contact_direction_search.py'
spec = importlib.util.spec_from_file_location('staged_contact_direction_search', HELPER)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
contact_direction_priorities = module.contact_direction_priorities


def candidate(track, proposal, rank=None, bucket=0):
    payload = NS(candidate_source='tabletop_geometry', audit={'proposal_source_index': proposal})
    stable = NS(candidate_source='tabletop_geometry', target_epoch=4, target_track_id='g1-t4')
    return NS(track_id=track, variant_index=0, stable_candidate=stable, payload=payload,
              evaluation_request_id=40, evaluation_snapshot_stamp_sec=1791679099.959358,
              evaluation_context_revision='recorded-request', rank=track if rank is None else rank,
              bucket=bucket)


def order(batch, phase='observing_contact'):
    priorities = {(c.track_id, c.variant_index): c.bucket for c in batch}
    old = lambda c: (c.rank, c.track_id, c.variant_index)
    hints = contact_direction_priorities(batch, old, priorities, observation_phase=phase)
    return sorted(batch, key=lambda c: (priorities[(c.track_id,c.variant_index)],
                  hints.get((c.track_id,c.variant_index),0), *old(c)))


def tracks(batch):
    return [c.track_id for c in batch]


def test_three_directions_interleave_without_mutating_candidates_or_group_order():
    batch = [candidate(i, p) for i,p in enumerate([2,2,0,0,1,1],1)]
    assert tracks(order(batch)) == [1,3,5,2,4,6]
    assert tracks(batch) == list(range(1,7))
    assert {id(c) for c in order(batch)} == {id(c) for c in batch}


def test_one_direction_preserves_original_rank_exactly():
    batch = [candidate(i,0,7-i) for i in range(1,7)]
    assert tracks(order(batch)) == list(range(6,0,-1))


def test_endpoint_hint_buckets_remain_absolute_priority():
    batch = [candidate(1,0,bucket=1),candidate(2,0),candidate(3,0),
             candidate(4,1),candidate(5,1,bucket=1),candidate(6,0,bucket=1)]
    assert tracks(order(batch)) == [2,4,3,1,5,6]


@pytest.mark.parametrize('phase',['','initial_contact','direct','observing','returning',None])
def test_initial_one_shot_and_other_phases_keep_original_rank(phase):
    batch = [candidate(i,p) for i,p in enumerate([0,0,1,1],1)]
    assert tracks(order(batch,phase)) == [1,2,3,4]


@pytest.mark.parametrize('phase',['observing_contact','returned_contact'])
def test_only_post_observation_contact_phases_enable_direction_rounds(phase):
    batch = [candidate(i,p) for i,p in enumerate([0,0,1,1],1)]
    assert tracks(order(batch,phase)) == [1,3,2,4]


@pytest.mark.parametrize('bad',[None,True,-1,'1',1.0,{},[]])
def test_bad_proposal_keys_fall_back_entire_batch(bad):
    batch=[candidate(1,0),candidate(2,0),candidate(3,bad)]
    assert tracks(order(batch))==[1,2,3]


@pytest.mark.parametrize('field', ['mixed_source','missing_audit','request','stamp','context','epoch','track'])
def test_mixed_or_unbound_batches_keep_original_order(field):
    batch=[candidate(1,0),candidate(2,0),candidate(3,1)]
    c=batch[-1]
    if field=='mixed_source':c.payload.candidate_source='graspnet'
    elif field=='missing_audit':c.payload.audit={}
    elif field=='request':c.evaluation_request_id+=1
    elif field=='stamp':c.evaluation_snapshot_stamp_sec+=1
    elif field=='context':c.evaluation_context_revision='other'
    elif field=='epoch':c.stable_candidate.target_epoch+=1
    elif field=='track':c.stable_candidate.target_track_id='g1-t5'
    assert tracks(order(batch))==[1,2,3]


def test_duplicate_runtime_key_has_no_partial_mapping():
    batch=[candidate(1,0),candidate(1,1)]
    assert contact_direction_priorities(batch,lambda c:(c.rank,),{},observation_phase='observing_contact')=={}


def test_exact_gen2_recorded_membership_all_three_directions_get_first_round():
    fixture=json.loads((ROOT/'tests/fixtures/gen2_contact_direction_membership.json').read_text())
    batch=[]
    for row in fixture['candidates']:
        c=candidate(row['track_id'],row['proposal_source_index'],row['recorded_soft_rank'],row['endpoint_priority'])
        c.variant_index=row['variant_index'];batch.append(c)
    result=order(batch)
    assert len(batch)==len(result)==38
    assert {c.payload.audit['proposal_source_index'] for c in result[:3]}=={0,1,2}
    for group in (0,1,2):
        prior=sorted([c for c in batch if c.payload.audit['proposal_source_index']==group],key=lambda c:(c.rank,c.track_id,c.variant_index))
        assert [(c.track_id,c.variant_index) for c in result if c.payload.audit['proposal_source_index']==group]==[(c.track_id,c.variant_index) for c in prior]


def pipeline_candidate(track, proposal):
    from test_grasp6d_pipeline import scored_candidate
    c=scored_candidate(track,0)
    payload=NS(candidate_source='tabletop_geometry',audit={'proposal_source_index':proposal})
    return replace(c,stable_candidate=replace(c.stable_candidate,candidate_source='tabletop_geometry',source_lineage=('tabletop_geometry',),payload=payload))


def pipeline_rank(batch):
    hints=contact_direction_priorities(batch,lambda c:(c.track_id,c.variant_index),{},observation_phase='observing_contact')
    return lambda c:(hints.get((c.track_id,c.variant_index),0),c.track_id,c.variant_index)


def test_actual_selector_stops_at_original_continuation_gate():
    from test_grasp6d_pipeline import bounded_moveit_select,failed_moveit_result
    batch=[pipeline_candidate(i,p) for i,p in enumerate([0,0,1,1,2,2],1)]
    visited=[]
    def check(c):visited.append(c.track_id);return failed_moveit_result('collision_free')
    result=bounded_moveit_select(batch,check,exhaustive=True,ranking_key=pipeline_rank(batch),
        continue_checking=lambda:len(visited)<3,continuation_stop_reason='ORIGINAL_DEADLINE')
    assert visited==[1,3,5]
    assert result.selected is None and result.terminated_early
    assert result.termination_reason=='ORIGINAL_DEADLINE'
    assert result.shortlist_count==6 and len(result.checked)==3


def test_actual_selector_keeps_hard_gate_and_does_not_call_checker_for_collision():
    from test_grasp6d_pipeline import bounded_moveit_select,failed_moveit_result
    batch=[pipeline_candidate(i,p) for i,p in enumerate([0,0,1,1],1)]
    batch[2]=replace(batch[2],latest_safety=replace(batch[2].latest_safety,collision_free=False))
    visited=[]
    def check(c):visited.append(c.track_id);return failed_moveit_result('collision_free')
    result=bounded_moveit_select(batch,check,exhaustive=True,ranking_key=pipeline_rank(batch))
    assert 3 not in visited and result.selected is None
    assert len(result.checked)==3


def test_actual_selector_still_requires_complete_reachability_result():
    from test_grasp6d_pipeline import bounded_moveit_select,reachable_moveit_result,failed_moveit_result
    batch=[pipeline_candidate(i,p) for i,p in enumerate([0,0,1,1],1)]
    def check(c):return reachable_moveit_result() if c.track_id==3 else failed_moveit_result('collision_free')
    result=bounded_moveit_select(batch,check,exhaustive=True,ranking_key=pipeline_rank(batch))
    assert result.selected.track_id==3
    assert result.selected.moveit_result.collision_free is True


def test_actual_remote_ranking_statements_read_phase_and_only_run_in_direct_branch():
    """Execute real staged ranking AST; do not load ROS or mock a full callback."""
    tree=ast.parse((ROOT/'scripts/remote_grasp6d_node.py').read_text())
    blocks=[n for n in ast.walk(tree) if isinstance(n,ast.If) and isinstance(n.test,ast.Name)
            and n.test.id=='direct_near_field' and any(isinstance(x,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='direction_priorities' for t in x.targets) for x in n.body)]
    assert len(blocks)==1
    batch=[candidate(i,p) for i,p in enumerate([0,0,1,1],1)]
    compiled=compile(ast.fix_missing_locations(ast.Module(body=[blocks[0]],type_ignores=[])),'staged_remote_rank','exec')
    for direct,phase,expected in [(True,'observing_contact',[1,3,2,4]),(True,'initial_contact',[1,2,3,4]),(False,'observing_contact',[1,2,3,4])]:
        env={'self':NS(_observe_return_phase=phase,_direct_contact_endpoint_priorities=lambda *args:{}),
             'direct_near_field':direct,'moveit_candidates':batch,'prepared':None,'acceptance_diagnostics':{},
             'moveit_ranking_key':lambda c:(c.rank,), 'contact_direction_priorities':contact_direction_priorities}
        exec(compiled,env)
        assert tracks(sorted(batch,key=env['moveit_ranking_key']))==expected
