#!/usr/bin/env python3
"""Read-only, run-bounded GEN54 queue/deadline audit. No ROS imports."""
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import re
import struct

E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
C = W/'.runtime/gen54_release_candidate_r1'
R = E/'grasp_records/20261019T031844.566032Z_249db38c06c54219b4cc46bfc70fab51'
OUT = N/'verification/gen54_actual_pipeline_analysis_r1'
LOW, HIGH = 1792379924.566032, 1792380061.152256
AUDIT_SHA = '57648911633e0f25eb53a1a8602d2c84d648004eaaf63240f024bb7ede2736fd'
CONSUMER = N/'gen54_runtime/compute_gateway_deployment_r1/consumers.log'
GATEWAY = E/'ros_log/full_validated_gen52_restart_20261019T013414Z/core_ros/40a36d9c-cb5d-11f1-a61a-77e20f59d0e2/motion_gateway-7.log'

def sha(b): return hashlib.sha256(b).hexdigest()
def pick(d, keys): return {k:d[k] for k in keys if k in d}
def identity(p):
    b=p.read_bytes()
    return dict(path=str(p), bytes=len(b), sha256=sha(b))
def write(name,obj):
    p=OUT/name
    p.write_text(json.dumps(obj,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)+'\n')
    return identity(p)
def f32(v): return struct.unpack('f',struct.pack('f',v))[0]
def elapsed_fields(d,path=''):
    rows=[]
    if isinstance(d,dict):
        for k,v in d.items():
            p=path+'.'+k if path else k
            if k in ('elapsed_sec','waited','attempts','iterations') and isinstance(v,(int,float,bool)):
                rows.append(dict(path=p,value=v))
            elif isinstance(v,(dict,list)): rows.extend(elapsed_fields(v,p))
    elif isinstance(d,list):
        for i,v in enumerate(d): rows.extend(elapsed_fields(v,path+'[%d]'%i))
    return rows

if not E.is_dir(): raise RuntimeError('SanStand unavailable; no fallback')
OUT.mkdir(parents=True,exist_ok=True)
ap=R/'keyframes_audits'/('%s.json'%AUDIT_SHA)
ab=ap.read_bytes()
assert sha(ab)==AUDIT_SHA
a=json.loads(ab)
assert a['request_id']==17 and a['generation']==7 and a['target_epoch']==4

consumer_events=[]; deliveries=[]; heights=[]; selected_bytes=[]
offset=0
with CONSUMER.open('rb') as stream:
    for line_no,b in enumerate(stream,1):
        here=offset;offset+=len(b)
        line=b.decode('utf-8','replace')
        m=re.search(r'\[(179\d+\.\d+)\]',line)
        if not m or not LOW<=float(m[1])<=HIGH: continue
        t=float(m[1])
        if not any(marker in line for marker in (
            'contact precheck delivery:', 'contact height retry:', 'measurement request:',
            'measurement acknowledged:', 'Observation endpoint compensation completed:',
            'observation selected', 'NEAR_FIELD_DIRECT_TIMEOUT', 'near-field candidate rejected',
            'remote 6D pipeline metrics:', 'OBSERVE_RETURN_SESSION_HANDOFF')):
            continue
        if 'remote 6D pipeline metrics: ' in line and json.loads(line.split('remote 6D pipeline metrics: ',1)[1]).get('request_id')!=17:
            continue
        ref=dict(path=str(CONSUMER),line=line_no,byte_offset=here,bytes=len(b),sha256=sha(b))
        selected_bytes.append(b)
        event=dict(wall_sec=t,source=ref)
        for marker in ('remote 6D contact precheck delivery: ', 'remote 6D contact height retry: ',
                       'Observation endpoint compensation completed: ', 'Observe-return measurement acknowledged: ',
                       'Observe-return measurement request: ', 'remote 6D pipeline metrics: ',
                       'OBSERVE_RETURN_SESSION_HANDOFF '):
            if marker not in line:continue
            d=json.loads(line.split(marker,1)[1]);event['kind']=marker.rstrip(': ')
            if 'precheck delivery' in marker:
                assert d['request']==dict(request_id=17,generation=7,target_epoch=4)
                x=pick(d,('plan_id','operation','deadline_sec','request_wall_ns','request_monotonic_ns',
                    'recorded_wall_ns','recorded_monotonic_ns','response','consumer_error','consumer_outcome',
                    'diagnostic_call_id','plan_wire_sha256','source_stamp_ns','phase_id','request',
                    'execution_authorized','future_following_verified'))
                x.update(source=ref,log_wall_sec=t,response_present=d.get('response') is not None)
                x['remaining_at_request_sec']=d['deadline_sec']-d['request_wall_ns']/1e9
                response=d.get('response')
                x['rpc_elapsed_monotonic_sec']=((response['received_monotonic_ns']-d['request_monotonic_ns'])/1e9 if response else None)
                if response:
                    pre=d['precheck']
                    x['strict_precheck']=pick(pre,('scope','success','failed_stage','failure_code','reason',
                        'checked_stage_names','opening_m','opening_source','contact_failure_replay',
                        'model_sha256','epoch_ns','execution_authorized','hardware_response_verified'))
                    x['strict_substage_timing_and_iterations']=elapsed_fields(pre)
                    x['source_sample']=pick(pre.get('source_sample',{}),('accepted_stamp_ns','sdk_stamp_ns',
                        'accepted_gripper_opening_m','measurement_kind','epoch_ns'))
                deliveries.append(x);event['data']=x
            elif 'height retry' in marker:
                s=d['search']
                x=pick(d,('plan_id','deadline_sec','eligible','first_ineligible_condition','original_failure_code',
                    'source_stamp_ns','request','refinement_translation_m'))
                x['search']=pick(s,('policy','maximum_proposals','evaluated_proposal_indices','deadline_extended',
                    'reason','strict_result_reused','upper_interior_reserve_m','selected_geometry_revalidation'))
                x['proposals']=[]
                for idx,tr in zip(s['evaluated_proposal_indices'],s['proposals']):
                    pr=pick(tr,('plan_id','accepted','reason','result','registration_plus_placement_displacement_m','geometry_gate'))
                    pr['proposal_index']=idx
                    pr['placement']=pick(tr['placement'],('normal_translation_m','policy','upper_interior_reserve_m',
                        'unchanged_translation_evidence_limit_m','allowed_normal_translation_upper_m'))
                    pr['height_margin']=pick(tr.get('final_contact_height_margin',{}),('ok','maximum_symmetric_normal_translation_m',
                        'translation_evidence_limit_m','allowed_normal_translation_intervals_m'))
                    pre=tr.get('contact_sequence_precheck') or {}
                    pr['precheck_plan_id']=pre.get('plan_id')
                    pr['precheck_is_bound_to_trial']=pre.get('plan_id')==pr.get('plan_id')
                    x['proposals'].append(pr)
                x.update(source=ref,log_wall_sec=t)
                heights.append(x);event['data']=x
            elif 'endpoint compensation completed' in marker:
                event['data']=pick(d,('physical_motion_submitted','real_grasp_success','complete_execution_path_checked',
                    'arrival_measurement_required','plan_id','plan_digest','session_id','view_index','code','no_motion',
                    'completion_stamp_ns','accepted_stamp_ns'))
            else:
                event['data']=d
            break
        else:
            event.update(kind='consumer_event',text=line.split(']: ',1)[-1].strip())
        consumer_events.append(event)

assert len(deliveries)==9 and sum(x['response_present'] for x in deliveries)==8
assert len(heights)==2 and heights[0]['search']['evaluated_proposal_indices']==[0,1,2,3]
assert heights[1]['search']['evaluated_proposal_indices']==[0,1,2]
parent_map={h['plan_id']:(i+1,None,0.) for i,h in enumerate(heights)}
for i,h in enumerate(heights):
    for p in h['proposals']: parent_map[p['plan_id']]=(i+1,p['proposal_index'],p['placement']['normal_translation_m'])
for d in deliveries:
    family,index,delta=parent_map[d['plan_id']]
    d.update(family_rank=family,height_proposal_index=index,normal_translation_m=delta)
    if d['response_present']: assert 'predicted_command continuous geometry: target OBB collision at 13-end/right_finger'==d['response']['message']

gateway=[];offset=0
with GATEWAY.open('rb') as stream:
    for line_no,b in enumerate(stream,1):
        here=offset;offset+=len(b);line=b.decode('utf-8','replace')
        m=re.search(r'(2026-10-18 \d\d:\d\d:\d\d,\d+)',line)
        if not m:continue
        t=dt.datetime.strptime(m[1],'%Y-%m-%d %H:%M:%S,%f').replace(tzinfo=dt.timezone(dt.timedelta(hours=-7))).timestamp()
        if not LOW<=t<=HIGH or not any(x in line for x in ('check_pose_strict request','check_pose_strict result',
                'plan_and_execute_pose_strict request','plan_and_execute_pose_strict result',
                'submitted trajectory duration=', 'check_pose_sequence_strict result')):continue
        gateway.append(dict(wall_sec=t,text=line.strip(),source=dict(path=str(GATEWAY),line=line_no,
            byte_offset=here,bytes=len(b),sha256=sha(b))))

# The stable rows, not the overwritten display rows, retain each variant's lineage.
import numpy as np
obb=np.array(a['replay_geometry']['R_base_obb']);size=np.array(a['replay_geometry']['obb_size_xyz_m'])
width_by_source={r['source_index']:r['required_open_width_m'] for r in a['rows'] if r.get('candidate_source')=='tabletop_geometry'}
queue=[]
for stable_index,r in enumerate(a['stable_evaluations']):
    if (r.get('endpoint_ik_search_probe') or {}).get('status')!='endpoint_pair_found':continue
    soft=r['latest_soft_evidence'];track=r['tracking']['track_id'];variant=r['variant_index']
    grasp=next(s for s in r['execution_sequence']['stages'] if s['stage']=='grasp')
    x,y,z,w=grasp['quaternion_xyzw']
    rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                       [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                       [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    # Production jaw tool axis is Y. Absolute projection is flip-invariant.
    jaw=rotation[:,1]
    projected=float(np.abs(obb.T.dot(jaw)).dot(size))
    required=width_by_source[r['source_index']]
    rank=[max(math.floor(f32(required)/.001),math.floor(f32(projected)/.001)),
        soft['contact_start_orientation_delta_rad'],soft['contact_start_translation_delta_m'],r['pre_moveit_score'],track,variant]
    queue.append(dict(stable_evaluation_index=stable_index,track_id=track,evaluation_variant_index=variant,
        source_index=r['source_index'],lineage_binding=r['lineage_binding'],required_open_width_m=required,
        full_obb_projected_width_m=projected,rank_key=rank,endpoint_hint=r['endpoint_ik_search_probe'],
        adaptive_stage_profile=r['adaptive_stage_profile'],final_strict_plan_id=r.get('final_strict_plan_id'),
        actually_checked=(track,variant) in ((9,0),(3,0))))
queue.sort(key=lambda x:x['rank_key'])
assert len(queue)==14 and [(x['track_id'],x['evaluation_variant_index']) for x in queue[:2]]==[(9,0),(3,0)]
for i,q in enumerate(queue,1):q['rank']=i
queue_report=dict(order_reconstruction='unchanged production millimetre width band, pose delta, score, track and variant; all endpoint hint buckets equal; one proposal means direction priorities equal',
    limitation='untried candidates have analytic and endpoint IK evidence only; full strict success is unknown',candidates=queue)

complete=[d for d in deliveries if d['response_present']]
elapsed=[d['rpc_elapsed_monotonic_sec'] for d in complete]
phase=a['observe_return']['reached_view']
phase_start=phase['start_stamp_ns']/1e9
selection_deadline=deliveries[0]['deadline_sec']
phase_deadline=phase['deadline_ns']/1e9
family_timing=[]
for family in (1,2):
    ds=[d for d in complete if d['family_rank']==family]
    start=ds[0]['request_wall_ns']/1e9;end=ds[-1]['response']['received_wall_ns']/1e9
    total=sum(d['rpc_elapsed_monotonic_sec'] for d in ds)
    family_timing.append(dict(family_rank=family,completed_rpc_count=len(ds),start_wall_sec=start,
        last_response_wall_sec=end,completed_rpc_sum_sec=total,span_sec=end-start,
        between_rpc_local_or_other_work_sec=end-start-total))
first=complete[0]['request_wall_ns']/1e9;last=complete[-1]['response']['received_wall_ns']/1e9
timing=dict(phase_start_wall_sec=phase_start,phase_deadline_wall_sec=phase_deadline,
    phase_budget_sec=phase_deadline-phase_start,publication_reserve_sec=phase_deadline-selection_deadline,
    selection_deadline_wall_sec=selection_deadline,contact_snapshot_wall_sec=a['snapshot_stamp_sec'],
    phase_start_to_first_strict_rpc_sec=first-phase_start,snapshot_to_first_strict_rpc_sec=first-a['snapshot_stamp_sec'],
    phase_start_to_snapshot_sec=a['snapshot_stamp_sec']-phase_start,
    first_strict_rpc_to_last_response_sec=last-first,completed_rpc_sum_sec=sum(elapsed),
    completed_rpc_mean_sec=sum(elapsed)/len(elapsed),completed_rpc_min_sec=min(elapsed),completed_rpc_max_sec=max(elapsed),
    inter_rpc_local_or_other_work_sec=last-first-sum(elapsed),
    last_reply_remaining_selection_sec=selection_deadline-last,
    expired_local_call_lateness_sec=deliveries[-1]['request_wall_ns']/1e9-selection_deadline,
    preparation_stages_ms=a['remote_diagnostics']['ros_prepare_stages_ms'],
    preparation_stage_sum_sec=sum(a['remote_diagnostics']['ros_prepare_stages_ms'].values())/1000,
    endpoint_ik_search=a['pipeline_funnel']['acceptance_diagnostics'].get('endpoint_ik_search_order'),
    family_timings=family_timing,
    interpretation='RPC durations include gateway computation, internal service waits and transport; they are not CPU times. Between-RPC deltas include registration/evidence/gates/serialization and other local work, not an isolated CPU measurement. Held-pregrasp optimizer timers are available, but complete stage-by-stage wall timers are not.')
timing['held_pregrasp_selection_total_sec']=sum(next((r['value'] for r in d['strict_substage_timing_and_iterations'] if r['path']=='held_pregrasp_selection.elapsed_sec'),0.) for d in complete)
timing['sample_acquisition_total_sec']=sum(r['value'] for d in complete for r in d['strict_substage_timing_and_iterations'] if r['path'].startswith('following_sample_acquisition') and r['path'].endswith('.elapsed_sec'))
timing['sample_acquisition_waited_count']=sum(r['value'] is True for d in complete for r in d['strict_substage_timing_and_iterations'] if r['path'].startswith('following_sample_acquisition') and r['path'].endswith('.waited'))

tabletop=a['remote_diagnostics']['tabletop_geometry']
result=dict(schema=1,scope='closed_actual_gen54_run_read_only_pipeline_analysis',
    run_path=str(R),time_filter=dict(lower_inclusive_wall_sec=LOW,upper_inclusive_wall_sec=HIGH),
    source_audit=identity(ap),result_summary=json.loads((R/'result_summary.json').read_text()),
    outcome=dict(terminal='NEAR_FIELD_DIRECT_TIMEOUT',completed_strict_rpc_count=8,local_deadline_rejections_without_rpc_response=1,
        actually_checked_candidate_families=2,shortlist_candidate_count=14,unchecked_candidate_families=12,
        first_family_fourth_height_actually_checked=True,second_family_fourth_height_checked=False,
        strict_success_count=0,known_safe_untried_candidate_proven=False,contact_motion_authorized=False),
    counts=dict(request_id=a['request_id'],request_id_is_not_candidate_count=True,
        proposal_count=tabletop['proposal_count'],materialized_count=tabletop['materialized_count'],
        refined_yaw_proposal_count=tabletop['open_aperture_refined_proposal_count'],
        stable_recheck=a['stable_recheck'],selection=a['moveit_selection'],pipeline_source_counts=a['pipeline_funnel']['source_counts']),
    geometry_branch=dict(policy=tabletop['regular_box_inference']['policy'],code=tabletop['regular_box_inference']['code'],
        model_sha256=tabletop['regular_box_inference']['model_sha256'],size_xyz_m=a['replay_geometry']['obb_size_xyz_m'],
        inferred_nominal_size_xyz_m=tabletop['regular_box_inference']['inferred_size_xyz_m'],
        prior_gen53_surface_feasibility_is_not_evidence_for_this_geometry=True),
    observation_handoff=pick(a['observe_return'],('session_id','view_index','view_plan_id','qualified_source_stamps_ns',
        'measurement_receipt_sha256','reached_view','phase','view_source_stamp_ns')),
    timing=timing,delivery_attempts=deliveries,height_retry_diagnostics=heights,
    consumer_events=consumer_events,gateway_events=gateway,
    cautions=['Eight replies are geometry rejections. The ninth attempted delivery has no response and is not a ninth geometric rejection.',
        'The second family third proposal inherited the preceding precheck object in diagnostic runtime; its plan ID differs. Its actual result is MOVEIT_TIMEOUT.',
        '28 stable variants were deduplicated into 14. Only two were strict checked. Unchecked is not rejected or proven safe.',
        'Observation view execution completed, then fresh three-frame qualification completed. No contact strict candidate succeeded.',
        'The 43.329/43.577 second gateway numbers are hardware execution lower bounds of preliminary nominal plans, not planning wall time.'])
artifacts=[write('actual_queue.json',queue_report),write('actual_pipeline_analysis.json',result)]
source_paths=[ap,R/'result_summary.json',R/'manifest.json',Path(__file__),
    C/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    C/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py']
write('sources.json',dict(files=[identity(p) for p in source_paths if p.is_file()],
    consumer_selected_lines_sha256=sha(b''.join(selected_bytes)),artifacts=artifacts))
print(json.dumps(dict(artifacts=artifacts,timing=timing,outcome=result['outcome'],queue=[(q['rank'],q['track_id'],q['evaluation_variant_index']) for q in queue]),ensure_ascii=False,indent=2))
