"""Pure method/callback-boundary demonstration; services are fail-only stubs."""
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch,Mock
import json,hashlib
from test_remote_grasp6d_streaming import remote_node as r
from test_contact_height_window_proposals import fixture
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen31_contact_analysis')
node,plan,prepared,runtime,_,_,_,_,_=fixture()
node.near_field_strategy='single_snapshot_direct';node.near_field_planning_active=True
node._near_field_phase_id=7;node._near_field_phase_deadline_sec=1791552516.268740177
prepared=NS(near_field=True,near_field_phase_id=7,near_field_phase_deadline_sec=1791552516.268740177,ticket=NS(request_id=25,generation=13,target_epoch=4))
for name in ('_configured_near_field_strategy','_direct_near_field_active','_direct_near_field_deadline_sec','_direct_near_field_candidate_deadline_sec'):
 setattr(node,name,getattr(r.RemoteGrasp6DNode,name).__get__(node))
node._execution_plan_validity_now_sec=lambda:1791552440.82
before=dict(active=node._direct_near_field_active(prepared),deadline=node._direct_near_field_candidate_deadline_sec(prepared,{}))
node.near_field_planning_active=False;node._near_field_phase_deadline_sec=0.
after=dict(active=node._direct_near_field_active(prepared),deadline=node._direct_near_field_candidate_deadline_sec(prepared,{}))
node.contact_sequence_precheck_enabled=True
node._require_stream_ticket_current=Mock(side_effect=r.StreamResultCancelled('TARGET_EPOCH_STALE'))
evidence=dict(plan_id=plan.plan_id,scope='read_only_candidate_constant_bias_prediction',execution_authorized=False,success=False)
calls=[]
def service(req):
 calls.append(dict(deadline=req.deadline.to_sec(),plan_id=req.plan.plan_id))
 return NS(evidence_json=json.dumps(evidence),success=False,failure_code='CONTACT_SEQUENCE_PRECHECK_FAILED',message='stub original geometry rejection')
with patch.object(r.rospy,'wait_for_service'),patch.object(r.rospy,'ServiceProxy',return_value=service),patch.object(r.rospy,'loginfo'):
 result,_=r.RemoteGrasp6DNode._final_contact_sequence_evaluation(node,plan,[],prepared,{},'cancelled_diagnostic',deadline_sec=after['deadline'])
out=dict(scope='OFFLINE_ACTUAL_METHODS_WITH_FAIL_ONLY_ROS_STUBS',before=before,after=after,service_calls=calls,ticket_recheck_calls=node._require_stream_ticket_current.call_count,result=dict(reachable=result.reachable,failure_code=result.failure_code),actual_record_timing=dict(active_phase2_start_ns=1791552426268758535,active_phase2_deadline_ns=1791552516268740177,task_failed_ns=1791552440805040000,inactive_phase_received_ns=1791552440806591987,strict_sample_sec=1791552440.844078,strict_result_sec=1791552441.609554),limitation='This proves the code boundary can pass zero as unbounded after cancellation. Exact final strict caller-entry timestamp was not logged; actual asynchronous ordering is not replayed.',source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(__file__),Path('src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py')]})
p=N/'cancelled_deadline_counterfactual.json';p.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
