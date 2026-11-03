"""Endpoint IK is a search hint; full-path failures still reject execution."""
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIKResponse
from test_remote_grasp6d_streaming import remote_node
from test_grasp6d_pipeline import scored_candidate, reachable_moveit_result
from alicia_flexible_grasp.grasp.grasp6d_pipeline import bounded_moveit_select


def make_case(monkeypatch):
    node=remote_node.RemoteGrasp6DNode.__new__(remote_node.RemoteGrasp6DNode)
    state=JointState();state.header.stamp=rospy.Time(100)
    state.name=['Joint%d'%j for j in range(1,7)];state.position=[0.]*6
    node.latest_joint_state=state
    node._direct_near_field_deadline_gate=lambda _:lambda:True
    node._endpoint_probe_clock=lambda:0.
    monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:rospy.Time(100)))
    monkeypatch.setattr(rospy,'wait_for_service',lambda *a,**k:None)
    calls=[]
    def service(req):
        calls.append(deepcopy(req))
        assert req.ik_request.avoid_collisions is True
        assert req.ik_request.timeout.to_sec()<=.025
        assert req.ik_request.group_name=='alicia'
        assert req.ik_request.ik_link_name=='tool0'
        p=req.ik_request.pose_stamped.pose.position
        r=GetPositionIKResponse();r.error_code.val=(-31 if p.x==1 and p.z==2 else 1)
        r.solution=deepcopy(req.ik_request.robot_state)
        return r
    monkeypatch.setattr(rospy,'ServiceProxy',lambda *a,**k:service)
    candidates=[scored_candidate(i,float(i)) for i in (1,2,3)]
    node._stable_variant_runtime={}
    for c in candidates:
        poses=[]
        for stage in (1,2):
            p=PoseStamped();p.header.frame_id='base_link';p.header.stamp=rospy.Time(90)
            p.pose.position.x=c.track_id;p.pose.position.z=stage;p.pose.orientation.w=1;poses.append(p)
        node._stable_variant_runtime[(c.track_id,0)]={'sequence':NS(grasp=poses[0],approach=poses[1])}
    return node,candidates,calls


def test_hints_reorder_without_mutating_poses_or_deleting_candidates(monkeypatch):
    n,cs,calls=make_case(monkeypatch);before=deepcopy(n._stable_variant_runtime);audit={}
    hints=n._direct_contact_endpoint_priorities(cs,None,audit)
    assert hints=={(2,0):0,(3,0):0}
    ranked=sorted(cs,key=lambda c:(hints.get((c.track_id,c.variant_index),1),*n._direct_near_field_moveit_rank_key(c)))
    assert [c.track_id for c in ranked]==[2,3,1]
    assert audit['endpoint_ik_search_order']['candidates_removed']==0
    assert audit['endpoint_ik_search_order']['execution_authority'] is False
    for k,r in n._stable_variant_runtime.items():
        assert r['sequence']==before[k]['sequence']
    assert len(calls)==6


def test_positive_hint_does_not_authorize_failed_full_path(monkeypatch):
    n,cs,_=make_case(monkeypatch);h=n._direct_contact_endpoint_priorities(cs,None,{})
    checked=[]
    def strict(c):
        checked.append(c.track_id)
        if c.track_id!=1:
            return remote_node.MoveItResult(reachable=False,joint_path_cost=0.,joint_max_delta_rad=0.,failure_code='MOVEIT_UNREACHABLE',reason='Cartesian fraction below existing threshold')
        return reachable_moveit_result()
    result=bounded_moveit_select(cs,strict,top_n=3,first_reachable_by_rank=True,
        ranking_key=lambda c:(h.get((c.track_id,c.variant_index),1),*n._direct_near_field_moveit_rank_key(c)))
    assert checked==[2,3,1]
    assert result.selected.track_id==1


def test_stale_or_missing_joint_seed_performs_no_ik(monkeypatch):
    n,cs,calls=make_case(monkeypatch);n.latest_joint_state.header.stamp=rospy.Time(99);audit={}
    assert n._direct_contact_endpoint_priorities(cs,None,audit)=={}
    assert calls==[] and audit['endpoint_ik_search_order']['status']=='joint_seed_stale'
    n.latest_joint_state=None
    assert n._direct_contact_endpoint_priorities(cs,None,{})=={} and calls==[]


def test_phase_expired_or_budget_exhausted_keeps_unknown_tail(monkeypatch):
    n,cs,calls=make_case(monkeypatch);n._direct_near_field_deadline_gate=lambda _:lambda:False
    assert n._direct_contact_endpoint_priorities(cs,None,{})=={} and calls==[]
    n._direct_near_field_deadline_gate=lambda _:lambda:True
    times=iter([0.,6.,6.]);n._endpoint_probe_clock=lambda:next(times);audit={}
    assert n._direct_contact_endpoint_priorities(cs,None,audit)=={} and calls==[]
    assert audit['endpoint_ik_search_order']['status']=='budget_exhausted'


def test_failed_service_is_unknown_not_candidate_rejection(monkeypatch):
    n,cs,calls=make_case(monkeypatch)
    monkeypatch.setattr(rospy,'ServiceProxy',lambda *a,**k:lambda _:(_ for _ in ()).throw(RuntimeError('lost service')))
    assert n._direct_contact_endpoint_priorities(cs,None,{})=={}
    assert len(cs)==3
    assert all(r['endpoint_ik_search_probe']['status']=='unknown' for r in n._stable_variant_runtime.values())


def test_hints_are_recomputed_for_each_request(monkeypatch):
    n,cs,calls=make_case(monkeypatch)
    n._direct_contact_endpoint_priorities(cs,None,{})
    n._direct_contact_endpoint_priorities(cs,None,{})
    assert len(calls)==12
