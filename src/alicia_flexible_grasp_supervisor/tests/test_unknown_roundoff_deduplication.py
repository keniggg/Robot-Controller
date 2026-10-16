"""Recorded duplicate IK families may be pruned, never granted cached authority."""
import importlib.util
import json
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
import test_remote_grasp6d_node as remote_test_fixture

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('unknown_roundoff_node', ROOT.parent/'alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py')
module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
base=module.original


def fixture_node():
    node=object.__new__(module.ModeAwareRemoteGrasp6DNode)
    node._mode_selection={'mode':'unknown'};node.near_field_planning_active=True
    node._stable_variant_runtime={};prepared=object();candidates=[]
    rows=json.loads((ROOT/'tests/fixtures/unknown_roundoff_families_20260923.json').read_text())['families']
    factory=remote_test_fixture.RemoteGrasp6DNodeTest()
    for index,family in enumerate(rows):
        candidate=factory._scored_for_moveit_dedupe(track_id=index+1,source_index=index)
        candidate=replace(candidate,stable_candidate=replace(candidate.stable_candidate,center_base_xyz=(.1,0.,.2)))
        stages={}
        for row in family['stages']:
            pose=base.PoseStamped();ns=row['stamp_ns'];pose.header.stamp=base.rospy.Time(ns//10**9,ns%10**9)
            pose.header.frame_id=row['frame_id']
            for field,axes,values in ((pose.pose.position,'xyz',row['position_m']),
                                      (pose.pose.orientation,'xyzw',row['quaternion_xyzw'])):
                for axis,value in zip(axes,values):setattr(field,axis,value)
            stages[row['stage']]=pose
        node._stable_variant_runtime[(candidate.track_id,0)]={'prepared':prepared,'sequence':SimpleNamespace(**stages)}
        candidates.append(candidate)
    return node,prepared,candidates


def test_recorded_duplicate_families_are_pruned_without_modifying_representatives(monkeypatch):
    monkeypatch.setattr(base.rospy,'loginfo',lambda *a:None)
    node,prepared,candidates=fixture_node();audit={}
    original=base.RemoteGrasp6DNode._dedupe_exact_contact_sequences_for_moveit(node,candidates,prepared)
    assert len(original)==4
    result=node._dedupe_exact_contact_sequences_for_moveit(candidates,prepared,acceptance_diagnostics=audit)
    assert [x.track_id for x in result]==[1,3]
    assert result[0] is candidates[0] and result[1] is candidates[2]
    report=audit['unknown_roundoff_proposal_deduplication']
    assert report['motion_result_reuse'] is False
    assert len(report['duplicates'])==2
    assert all(d['maximum_transform_difference']<=8*np.finfo(float).eps for d in report['duplicates'])


@pytest.mark.parametrize('change',['pregrasp','approach','grasp','lift','stamp','frame','width','context','target','source','missing','gate','request','prepared','center','invalid_quaternion'])
def test_distinct_or_incomplete_evidence_is_never_merged(monkeypatch,change):
    monkeypatch.setattr(base.rospy,'loginfo',lambda *a:None)
    node,prepared,all_candidates=fixture_node();candidates=all_candidates[:2]
    candidate=candidates[1];runtime=node._stable_variant_runtime[(2,0)];sequence=runtime['sequence']
    if change in ('pregrasp','approach','grasp','lift'):getattr(sequence,change).pose.position.x+=1e-12
    elif change=='stamp':sequence.pregrasp.header.stamp.nsecs+=1
    elif change=='frame':sequence.pregrasp.header.frame_id='different'
    elif change=='width':candidate=replace(candidate,stable_candidate=replace(candidate.stable_candidate,required_open_width_m=.041))
    elif change=='context':candidate=replace(candidate,evaluation_context_revision='other')
    elif change=='target':candidate=replace(candidate,stable_candidate=replace(candidate.stable_candidate,target_epoch=8))
    elif change=='source':candidate=replace(candidate,stable_candidate=replace(candidate.stable_candidate,candidate_source='graspnet',source_lineage=('graspnet',)))
    elif change=='missing':del sequence.lift
    elif change=='gate':candidate=replace(candidate,latest_safety=replace(candidate.latest_safety,geometry_valid=False))
    elif change=='request':candidate=replace(candidate,evaluation_request_id=4)
    elif change=='prepared':runtime['prepared']=object()
    elif change=='center':candidate=replace(candidate,stable_candidate=replace(candidate.stable_candidate,center_base_xyz=(.100001,0.,.2)))
    elif change=='invalid_quaternion':
        for axis in 'xyzw':setattr(sequence.grasp.pose.orientation,axis,0.)
    candidates[1]=candidate
    assert len(node._dedupe_exact_contact_sequences_for_moveit(candidates,prepared))==2


@pytest.mark.parametrize('mode,near',[('carton',True),('unknown',False)])
def test_other_modes_retain_original_exact_policy(mode,near):
    node,prepared,candidates=fixture_node();node._mode_selection={'mode':mode};node.near_field_planning_active=near
    assert len(node._dedupe_exact_contact_sequences_for_moveit(candidates,prepared))==4
