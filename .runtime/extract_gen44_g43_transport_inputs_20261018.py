#!/usr/bin/env python3
"""Export small exact source/plan/precheck evidence from the closed G43 bag."""
import base64
import hashlib
import io
import json
from pathlib import Path
import socket
import time
W=Path(__file__).resolve().parent.parent
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
G=E/'grasp_records/20261018T052442.803648Z_891dd95b290a47f8a5ee478e99e37329'
OUT=N/'verification/gen44_g43_replay_inputs_r3'
FIXTURE=W/'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen44_g43_stage0_replay.json'
SOURCES=[1792301138087765216,1792301138286787748,1792301138488024473]
boundary=[]
class OfflineSocket(socket.socket):
    def __new__(cls,*a,**k):
        boundary.append('socket');raise RuntimeError('sealed bag only')
socket.socket=OfflineSocket
import rosbag
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb')as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()
def put(path,value):
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    assert json.loads(path.read_text())==value
    return dict(path=str(path),sha256=sha(path))
def plain(v):
    if hasattr(v,'to_nsec'):return v.to_nsec()
    if hasattr(v,'__slots__'):return {k:plain(getattr(v,k))for k in v.__slots__}
    if isinstance(v,(list,tuple)):return[plain(x)for x in v]
    return v
def wire(msg):
    b=io.BytesIO();msg.serialize(b);raw=b.getvalue()
    return dict(type=msg._type,md5sum=msg._md5sum,wire_sha256=hashlib.sha256(raw).hexdigest(),
                wire_base64=base64.b64encode(raw).decode())
def main():
    assert E.is_dir() and not OUT.exists()
    m=json.loads((G/'manifest.json').read_text());assert m['recorder_closed']and m['state']=='complete'
    b=next(x for x in m['files']if x['path']=='keyframes.bag');assert sha(G/'keyframes.bag')==b['sha256']
    OUT.mkdir(exist_ok=False)
    rows={ns:{}for ns in SOURCES};acks=[]
    topics=['/perception/object','/perception/object_mask','/perception/unknown/detector_status',
            '/grasp_mode/observe_return_measurement_ack']
    with rosbag.Bag(str(G/'keyframes.bag'),'r')as bag:
        for topic,msg,t in bag.read_messages(topics=topics):
            if topic.endswith('measurement_ack'):
                d=json.loads(msg.data)
                if d.get('qualified')is True:acks.append(dict(topic=topic,received_ns=t.to_nsec(),value=d))
                continue
            if topic.endswith('detector_status'):
                d=json.loads(msg.data);ns=d.get('source_stamp_ns')
                if ns in rows:rows[ns]['status']=dict(topic=topic,received_ns=t.to_nsec(),value=d,**wire(msg))
                continue
            ns=msg.header.stamp.to_nsec()
            if ns not in rows:continue
            if topic.endswith('object_mask'):
                raw=bytes(msg.data)
                rows[ns]['mask_summary']=dict(topic=topic,received_ns=t.to_nsec(),source_stamp_ns=ns,
                    width=msg.width,height=msg.height,encoding=msg.encoding,step=msg.step,
                    data_sha256=hashlib.sha256(raw).hexdigest(),nonzero_pixels=sum(x!=0for x in raw),
                    raw_mask_retained_in_sealed_bag=True,mask_reconstructed_or_fabricated=False)
            else:rows[ns]['object']=dict(topic=topic,received_ns=t.to_nsec(),message=plain(msg),**wire(msg))
    for ns,row in rows.items():
        assert set(row)=={'object','mask_summary','status'},(ns,list(row))
        assert row['object']['message']['detected']is True
        assert row['mask_summary']['nonzero_pixels']>0
    original=N/'verification/gen43_actual_aperture_failure_r1/frozen_plan.json'
    p=json.loads(original.read_text());assert p['message']['header']['stamp']==SOURCES[-1]
    prepath=N/'verification/gen43_precheck_stage0_aperture_replay_r1/original_precheck.json'
    pre=json.loads(prepath.read_text());assert pre['precheck']['success']is True
    cfg=G/'perception_execution_configuration.json'
    reached=N/'verification/gen43_actual_aperture_failure_r1/runtime_aperture_report.json'
    correction=N/'verification/gen43_actual_aperture_failure_r1/runtime_compensation_report.json'
    fixture=dict(kind='closed_G43_source_transport_plan_and_stage0_precheck_fixture',
        record=str(G),record_manifest_sha256=sha(G/'manifest.json'),bag_sha256=b['sha256'],
        source_stamp_ns=SOURCES,recorded_producer_transport_summaries={str(ns):row for ns,row in rows.items()},
        recorded_qualified_measurement_ack=acks,
        plan_wire=dict(type=p['message_type'],md5sum=p['md5sum'],wire_sha256=p['wire_sha256'],wire_base64=p['wire_base64']),
        plan_id=p['message']['plan_id'],plan_source_stamp_ns=SOURCES[-1],
        original_service_request=dict(deadline_sec=pre['deadline_sec'],source_stamp_ns=pre['source_stamp_ns'],
            request_id=pre['request']['request_id'],selection=pre['precheck']['selection']),
        original_precheck_sample=pre['precheck']['source_sample'],
        original_precheck=dict(success=True,code=pre['consumer_outcome'],
            held_pregrasp_selection=pre['precheck']['held_pregrasp_selection'],
            held_approach_prediction=pre['precheck']['held_approach_prediction'],
            contact_prediction=pre['precheck']['contact_prediction'],
            stage_start_prediction=pre['precheck']['stage_start_prediction'],
            model_sha256=pre['precheck']['model_sha256'],opening_m=pre['precheck']['opening_m'],
            source_received_wall_ns=pre['recorded_wall_ns']),
        config=json.loads(cfg.read_text())['parameters']['/grasp'],
        robot_model_path='deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf',
        model_sha256=pre['precheck']['model_sha256'],
        actual_reached_pregrasp=json.loads(reached.read_text()),
        actual_pregrasp_compensation=json.loads(correction.read_text()),
        historical_rigid_projection_result=dict(code='MEASURED_APERTURE_CAD_COLLISION',
            reason='projected_contact: right_finger enters support clearance',
            required_open_width_m=.04654902486742669,
            projected_contact_minimum_support_clearance_m=.0014817959331234476),
        providers=dict(raw_image_segmentation_reexecuted=False,
            recorded_producer_object_and_status_wire_retained=True,
            recorded_mask_header_counts_and_data_hash_retained=True,
            producer_callback_execution_not_replayed=True,
            original_MoveIt_trajectory_recorded=False,
            test_MoveIt_trajectory_provider='explicitly simulated, no historical full trajectory claim',
            hardware_or_future_following_verified=False),
        original_inputs={str(f):sha(f)for f in [original,prepath,cfg,reached,correction]},
        artifact_helper_sha256=sha(Path(__file__)))
    artifact=put(FIXTURE,fixture)
    result=put(OUT/'result.json',dict(passed=True,fixture=artifact,source_count=3,
        bag_manifest_verified=True,read_topics=topics,network_boundary_attempts=boundary,
        ros_init_called=False,temperature_read=False,hardware_contacted=False,production_or_existing_test_modified=False))
    assert not boundary
    print(json.dumps(result,ensure_ascii=False))
if __name__=='__main__':main()
