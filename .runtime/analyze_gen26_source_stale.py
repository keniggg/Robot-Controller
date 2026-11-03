"""Sealed gen26 timing and station evidence; no ROS calls, no temperature data."""
import hashlib,json,re
from pathlib import Path
import numpy as np
import rosbag,rospy

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z')
D=BASE/'gen26_handoff_analysis'
A=json.loads((D/'arrival_and_terminal.json').read_text())
S=json.loads((D/'detector_status.json').read_text())
R=Path(A['record'])
failed=A['first_loss']['message'];f=failed['first_loss'];ft=failed['frame_timing'];now=f['observed_stamp_ns']
source=f['source_stamp_ns'];ref=f['last_good_source_stamp_ns']
before=[r for r in S if r['message'].get('state')=='ready' and r['message'].get('source_stamp_ns',0)<source]
recent=before[-20:]
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ns_time(ns):return rospy.Time(ns//10**9,ns%10**9)
def compact(r):
    s=r['message'];t=s.get('frame_timing',{})
    return dict(source_stamp_ns=s.get('source_stamp_ns'),bag_received_ns=r['received_ns'],state=s['state'],detail=s.get('detail'),color_seq=s.get('color_seq'),depth_seq=s.get('depth_seq'),source_age_at_processing_start_sec=t.get('processing_start_source_age_sec'),source_age_at_status_sec=t.get('status_source_age_sec'),processing_wall_sec=t.get('processing_wall_sec'),processing_thread_cpu_sec=t.get('processing_thread_cpu_sec'),stage_wall_sec=t.get('stage_wall_sec'),stage_thread_cpu_sec=t.get('stage_thread_cpu_sec'),raw_arrivals=t.get('raw_arrivals'),rgb_identity_hold=s.get('rgb_identity_hold'))
joints=[];camera=[]
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
    for topic,msg,t in bag.read_messages(topics=['/alicia_d/accepted_joint_states','/grasp_record/camera_runtime_metadata'],start_time=ns_time(now-7_000_000_000),end_time=ns_time(now+1_000_000_000)):
        if topic.endswith('accepted_joint_states'):
            if msg.header.frame_id=='sdk_measured':joints.append(dict(bag_received_ns=t.to_nsec(),source_stamp_ns=msg.header.stamp.to_nsec(),names=list(msg.name),positions=list(msg.position)))
        else:
            d=json.loads(msg.data);parameters=d.get('parameters',{});profile=parameters.get('runtime_profile',{})
            camera.append(dict(bag_received_ns=t.to_nsec(),metadata={k:v for k,v in d.items() if k!='parameters'},camera_geometry={k:parameters.get(k) for k in ('width','height','fx','fy','cx','cy','depth_scale','fps','publish_fps','intrinsics_source')},stationary_temporal=profile.get('stationary_temporal'),runtime_profile_geometry_activation_stamp_ns=profile.get('geometry_activation_stamp_ns')))
q=np.asarray([x['positions'] for x in joints]);qspan=np.ptp(q,axis=0).tolist() if len(q) else []
plan_views=[]
seen=set()
for p in sorted((R/'keyframes_audits').glob('*.json')):
    a=json.loads(p.read_text());selected=a.get('selected') or {};plan=a.get('plan_id')
    if not plan or plan in seen or 'observation_view' not in selected:continue
    seen.add(plan)
    for v in A['views']:
        if v['arrival_request']['view_plan_id']!=plan:continue
        endpoint=selected['observation_orientation_search']['variant_results'][0]['following_support']['unknown_observation_view']['arrival_projection_endpoint']
        ack=next(x for x in v['protocol_acks'] if x['message']['state']=='requalifying' and x['message']['participant']=='remote' and x['message']['qualified'])
        plan_views.append(dict(view_index=v['view_index'],plan_id=plan,request_id=a['request_id'],source_stamp_ns=a['snapshot_stamp_ns'],audit_path=str(p),audit_sha256=sha(p),strict_arrival_projection_endpoint=endpoint,arrival_stamp_ns=v['arrival_request']['arrival_stamp_ns'],qualified_ack_bag_received_ns=ack['received_ns'],qualified_source_stamps_ns=ack['message']['source_stamps_ns'],arrival_to_remote_ack_receipt_sec=(ack['received_ns']-v['arrival_request']['arrival_stamp_ns'])*1e-9))
lines=(BASE/'arrival_projection_deployment/consumers.log').read_text(errors='replace').splitlines()
contact=[];terminal=[]
for line in lines:
    if 'final candidate rejected:' in line:contact.append(line)
    if 'Unknown target first loss generation=26' in line or '[Grasp] FAILED' in line:terminal.append(line)
def statistics(key):
    values=[r['message']['frame_timing'][key] for r in recent if key in r['message'].get('frame_timing',{})]
    return dict(n=len(values),minimum=float(np.min(values)),median=float(np.median(values)),p95=float(np.percentile(values,95)),maximum=float(np.max(values))) if values else None
result=dict(generation=26,record=str(R),sealed_manifest=A['sealed_manifest'],record_manifest_sha256=A['manifest_sha256'],views=sorted(plan_views,key=lambda x:x['view_index']),three_observation_arrivals_qualified=True,contact_execution_started=False,contact_strict_rejection_count=len(contact),contact_strict_rejection_lines=contact,terminal_lines=terminal,first_loss=dict(source_stamp_ns=source,last_good_source_stamp_ns=ref,observed_stamp_ns=now,reason=f['reason'],failed_stage=ft['failed_stage'],capture_gap_to_last_good_sec=(source-ref)*1e-9,last_good_age_at_processing_start_sec=(ft['processing_start_ros_ns']-ref)*1e-9,last_good_age_at_failure_sec=(now-ref)*1e-9,original_frame_timing=ft),last_twenty_ready_frames=[compact(r) for r in recent],ready_processing_statistics=dict(processing_wall_sec=statistics('processing_wall_sec'),processing_thread_cpu_sec=statistics('processing_thread_cpu_sec')),stationary_joint_window=dict(start_ns=now-7_000_000_000,end_ns=now+1_000_000_000,sample_count=len(joints),joint_peak_to_peak_rad=qspan,first=joints[0] if joints else None,last=joints[-1] if joints else None),camera_metadata_window=camera,findings=[
    'The selected source reached exact synchronizer at21.564ms and processing started at42.367ms; this source was fresh at entry.',
    'The failure is post_refine after1.866s wall/.563s current-thread CPU. The recorded RGB composite stage consumed1.709s wall/.421s CPU.',
    'At status build the latest pending source was only68.158ms old, so newer exact RGB-D continued to arrive during the processing of the older source.',
    'Main exact TF lookup was0.089ms and decode2.304ms. The existing normal-frame completion token was created validly and correctly rejected after current-source .8s deadline; this is not an old-reference-only expiry.',
    'The RGB timing interval also includes _prepare_locked_rgb_hold_reference, including cached parameter context, static-mount TF checks, joint bracketing, appearance and hashes. It does not isolate GrabCut alone.',
    'RGB wall minus current-thread CPU is1.288s, demonstrating off-current-thread elapsed time, but not identifying GIL, lock wait, descheduling, page fault or IO. No contemporaneous scheduler/stack trace is available here.',
    'The rgb_identity_hold rejected diagnostic also appears in earlier ready frames. Its retained text is not evidence of a new first-loss identity rejection.',
    'Contact candidates were still being rejected by original continuous target/finger geometry before source loss; a completed all-candidate outcome is not available.'],limitations=['Only sealed non-temperature metadata, text audits and logs were read.','No new camera stream, hardware queries, pixel replay, robot control or production changes.','Camera metadata is a periodic parameter snapshot, not an exact per-frame SDK/filter receipt; do not assign its filter status to a specific pixel source.','Unproven scheduling mechanisms are not labeled as root cause.'],input_sha256={str(D/'arrival_and_terminal.json'):sha(D/'arrival_and_terminal.json'),str(D/'detector_status.json'):sha(D/'detector_status.json')},script_sha256=sha(__file__))
p=D/'diagnosis_summary.json';p.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(p),sha256=sha(p),views=[{k:v for k,v in x.items() if k in ('view_index','arrival_to_remote_ack_receipt_sec','qualified_source_stamps_ns')} for x in result['views']],contact_rejection_count=len(contact),q_samples=len(joints),q_span=qspan,camera_metadata_samples=len(camera),ready_statistics=result['ready_processing_statistics']),ensure_ascii=False,indent=2))
