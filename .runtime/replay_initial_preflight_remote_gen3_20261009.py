#!/usr/bin/env python3
"""Recorded remote callback replay. No ROS master, network, or hardware I/O."""
from copy import deepcopy
from collections import deque
import threading
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

WORK=Path(__file__).resolve().parents[1]
STAGE=WORK/'.runtime/initial_precommit_staging_20261009/src/alicia_grasp_modes'
sys.path[:0]=[str(STAGE/'src'),str(STAGE/'tests'),str(WORK/'src/alicia_flexible_grasp_supervisor/tests'),str(WORK/'.runtime')]
from _pytest.monkeypatch import MonkeyPatch
from cv_bridge import CvBridge
from std_msgs.msg import String
import rospy
from test_unknown_pending_identity import consumer_for,module,Publisher,deliver
from test_initial_preflight_remote import request,admit_measurement
from test_task_modes import module as task_module
from build_gen3_initial_preflight_fixture_20261009 import read_fixture_events,SOURCES


def run(folder):
    events=list(read_fixture_events(folder));metadata=json.loads((folder/'fixture.json').read_text())
    plans=[(e,m) for e,m in events if e['topic']=='/grasp_6d/plan_enriched' and m.valid]
    plan=deepcopy(plans[-1][1]);selection=metadata['selection']
    audits=[]
    for path in (Path(metadata['source_record'])/'keyframes_audits').glob('*.json'):
        value=json.loads(path.read_text())
        if value.get('plan_id')==plan.plan_id and value.get('outcome',{}).get('code')=='PLAN_READY':
            audits.append(path)
    assert len(audits)==1
    audit_path=audits[0];audit_text=audit_path.read_text()
    exact_rows={role:{} for role in SOURCES};times={role:{} for role in SOURCES}
    for e,msg in events:
        role=e.get('source_role');topic=e['topic']
        key={'/perception/object':'object','/perception/object_mask':'mask','/perception/unknown/detector_status':'status'}.get(topic)
        if role and key:
            exact_rows[role][key]=json.loads(msg.data) if key=='status' else msg
            times[role][key]=e['bag_received_ns']
    epoch=next(m.stamp.to_nsec() for e,m in events if e['topic']=='/alicia_d/control_reference_epoch')
    results=[];durations=[]
    for order in itertools.permutations(('object','mask','status')):
        with MonkeyPatch().context() as patch:
            now=[min(times['acquired'].values())];clock=[now[0]/1e9]
            params={module.SELECTION_PARAM:dict(selection,stamp_ns=str(selection['stamp_ns'])),
                '/perception':{'detector':'unknown_tabletop','yolo_model_choice':'unknown_tabletop'},
                '/gui/joint_direct_mode':False}
            patch.setattr(rospy,'get_param',lambda name,default=None:params.get(name,default))
            patch.setattr(rospy,'get_param_cached',lambda name,default=None:params.get(name,default))
            patch.setattr(rospy.Time,'now',staticmethod(lambda:rospy.Time(now[0]//10**9,now[0]%10**9)))
            n=consumer_for(NS(bridge=CvBridge()),clock,params);n._ros_source_clock_ns=lambda:now[0]
            # Explicit recorded context seed; the four retained frames cannot
            # reconstruct omitted identity history. Input messages stay exact.
            g,t=plan.target_track_id.split('-');n._target_identity_generation=int(g[1:]);n.target_instance_epoch=int(t[1:])
            n._measurement_remote_epoch_ns=epoch
            n._initial_preflight_ack_pub=Publisher();n.execution_plan_validity_sec=30.
            task=task_module.ModeAwareGraspTaskNode.__new__(task_module.ModeAwareGraspTaskNode)
            task._initialize_mode_boundary();task.active=False;task.set_state=lambda *a,**kw:None
            assert task.reset_mode_cb(None).success
            task._mode_bound_selection=deepcopy(task._mode_selection)
            task._following_evidence_lock=threading.RLock();task._following_epoch_ns=epoch
            task._following_accepted_history=deque(maxlen=128);task._following_sdk_history=deque(maxlen=128)
            class LinkedPublisher(Publisher):
                def __init__(self, callback):super().__init__();self.callback=callback
                def publish(self,msg):super().publish(msg);self.callback(deepcopy(msg))
            n.geometry_pub=LinkedPublisher(task.target_geometry_cb)
            n._initial_preflight_ack_pub=LinkedPublisher(task._initial_preflight_ack)
            task._initial_preflight_request_pub=NS(publish=n._initial_preflight_request_cb)
            delivered_feedback=set()
            def advance(ns):
                now[0]=ns;clock[0]=ns/1e9
                for e,msg in events:
                    if (e['topic']=='/alicia_d/accepted_joint_states' and e['bag_received_ns']<=ns
                            and e['ordinal'] not in delivered_feedback):
                        n._station_compute_accepted_cb(deepcopy(msg));delivered_feedback.add(e['ordinal'])
                        task._following_accepted_history.append(deepcopy(msg));task._initial_preflight_feedback(msg)
            def emit(role,keys=order):
                # For permutation tests co-admit unchanged inputs at the latest
                # original triplet receive time; no claim of subscriber order.
                advance(max(times[role].values()))
                started=time.perf_counter();deliver(n,exact_rows[role],keys)
                durations.append((time.perf_counter()-started)*1000)
                for key in keys:
                    msg=deepcopy(exact_rows[role][key])
                    if key=='object':task.obj_cb(msg)
                    elif key=='mask':task._initial_preflight_mask(msg)
                    else:task._unknown_support_status_cb(String(data=json.dumps(msg)))
            emit('acquired');assert n._unknown_identity_reference is not None
            emit('reference');assert n._unknown_identity_reference['source_ns']==SOURCES['reference']
            assert n._current_stream_target_identity().track_id==plan.target_track_id
            n._observe_return_phase='observation_prefix';n._observe_return_diagnostic_observation_only=False
            n._observe_return_prefix=deepcopy(plan);n._observe_return_origin_prefix=deepcopy(plan)
            n.latest_preview_rich_plan=deepcopy(plan);n._unknown_identity_publication_started=True
            q=exact_rows['reference']['status']['initial_station_measurement']['q_station_rad']
            task._observe_return_audit_cb(String(data=json.dumps(dict(report_path=str(audit_path),
                report_sha256=hashlib.sha256(audit_path.read_bytes()).hexdigest()))))
            task._initial_preflight_begin(NS(plan_id=plan.plan_id));task._freeze_execution_plan(plan)
            task.active=True;task.stage=3
            assert task._initial_preflight_arm(plan,dict(plan_validity_sec=30.))
            token=deepcopy(task._initial_preflight_token['request'])
            scene=NS(n=n,token=token,clock=clock,plan=plan)
            ack=json.loads(n._initial_preflight_ack_pub.messages[-1].data);assert ack['accepted'],ack
            identity=n._current_stream_target_identity();rich_count=len(n.rich_plan_pub.messages)
            emit('gap');geometry=n.geometry_pub.messages[-1]
            assert n._unknown_identity_pending and n._current_stream_target_identity()==identity
            assert task._initial_preflight_pending and not task._execution_authority_revoked
            assert task.latest_obj is None and task.latest_target_geometry is None
            attempts=[]
            result=task._validate_bound_plan_locked(plan,{})
            assert not result.ok and result.code=='TARGET_MEASUREMENT_PENDING' and not attempts
            assert geometry.failure_reason.startswith('TARGET_MEASUREMENT_PENDING:')
            assert geometry.header.stamp.to_nsec()==SOURCES['gap']
            assert len(n.rich_plan_pub.messages)==rich_count
            blocked=request(scene,'commit');assert not blocked['accepted']
            assert blocked['pending_source_ns']==SOURCES['gap']
            emit('recovery');assert n._unknown_identity_pending is None
            assert n.latest_object.header.stamp.to_nsec()==SOURCES['recovery']
            assert n._current_stream_target_identity()==identity
            assert task._initial_preflight_pending is None and not task._execution_authority_revoked
            assert task.latest_obj.header.stamp.to_nsec()==SOURCES['recovery']
            assert task._initial_preflight_replan
            recomputations=[]
            def inert_preflight(pose,execute):
                assert not execute;recomputations.append('fresh_preflight');return NS(success=True,message='offline')
            assert task._initial_preflight_wrap_move(inert_preflight)(object(),False).success
            assert recomputations==['fresh_preflight'] and not task._initial_preflight_replan
            admit_measurement(scene);task._initial_preflight_commit()
            reply=json.loads(n._initial_preflight_ack_pub.messages[-1].data);assert reply['accepted'],reply
            assert task._initial_preflight_token['state']=='committed'
            assert not task._initial_preflight_attempt['physical_action_submitted']
            service_spy=[]
            def inert_strict(*args,**kwargs):service_spy.append('inert_strict');return NS(success=True)
            strict=task._initial_preflight_wrap_strict(inert_strict)
            assert strict(object(),True).success
            assert task._initial_preflight_attempt['physical_action_submitted']
            assert strict(object(),True).success and len(service_spy)==2
            deliver(n,exact_rows['gap'],order)
            assert n._measurement_remote_request['state']=='executing'
            assert n._current_stream_target_identity()==identity
            results.append(dict(order=list(order),arm=ack['accepted'],pending_source_ns=SOURCES['gap'],
                replay_generated_geometry_reason=geometry.failure_reason,exact_recovery_source_ns=SOURCES['recovery'],
                original_measurement_lease_admitted=True,commit=reply['accepted'],old_gap_replay_ignored=True,
                task_received_replay_geometry=True,task_pending_action_calls=attempts,task_fresh_preflight_recomputed=True,
                inert_strict_service_spy_calls=len(service_spy),inert_first_submission_latched=True))
    return dict(schema_version=1,fixture=str(folder),fixture_sha256=hashlib.sha256((folder/'fixture.json').read_bytes()).hexdigest(),
        recorded_audit_path=str(audit_path),recorded_audit_sha256=hashlib.sha256(audit_path.read_bytes()).hexdigest(),
        plan_id=plan.plan_id,recorded_plan_source_ns=plan.header.stamp.to_nsec(),recorded_context_track=plan.target_track_id,
        results=results,callback_triplet_ms=dict(count=len(durations),max=max(durations),mean=sum(durations)/len(durations)),
        limitations=['Recorded g1-t11 identity/epoch and original published plan are explicit context seeds; omitted history is not reconstructed.',
            'Original object, mask, status, feedback and epoch bytes are unchanged. Permutations are synthetic subscriber schedules.',
            'Fresh geometry pending notification is replay-generated and delivered directly to the actual task callback; no successful metric recovery ObjectGeometry is invented.',
            'Task fresh preflight wrapper runs with an inert planner response. The strict-wrapper spy is called twice to verify first submission latch and later delegation, without the absent full metric geometry/path gate; these calls are not physical execution.',
            'Existing remote measurement lease parser and admission run offline; producer dual ACK and physical motion are not executed.',
            'Only consumer callbacks timed; no ROS transport, TF/planning workload, or source production included.'],
        physical_actions=0,ros_master_io=False,live_source_modified=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--fixture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();value=run(args.fixture);args.output.write_text(json.dumps(value,indent=2,sort_keys=True)+'\n')
    print(json.dumps(dict(output=str(args.output),permutations=len(value['results']),callback_triplet_ms=value['callback_triplet_ms'],physical_actions=0)))
