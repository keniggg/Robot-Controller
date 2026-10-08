#!/usr/bin/env python3
"""Closed bag initial support diagnostics; no ROS master or device access."""
from collections import Counter
from dataclasses import fields
import json
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge
import replay_matched_shadow_geometry_20261005 as replay

ROOT=replay.support.EXT/'ros_log/unknown_first_loss_readonly_20261005T041849Z_58490'
OUT=ROOT/'offline_initial_support_20261005T0430Z'
CONFIG=replay.support.EXT/'ros_log/full_ros_resume_20261005T012259Z/matched_shadow_current_pipeline_config.json'
PREVIOUS=replay.support.EXT/'ros_log/rgb_observation_bootstrap_20261005T034713Z'
if not replay.support.MOUNT.is_mount():
    raise RuntimeError('SanStand unavailable; no fallback')
report=json.loads((ROOT/'report.json').read_text())
meta=json.loads((ROOT/'metadata.json').read_text())
if report['archive_bag_closed'] is not True or report['writer']['archive_complete'] is not True:
    raise ValueError('unclosed/incomplete bag')
camera=meta['camera_parameters']
cfg=replay.tabletop.Config(**{f.name:meta['unknown_perception_parameters'][f.name]
    for f in fields(replay.tabletop.Config) if f.name in meta['unknown_perception_parameters']})
args=replay.geometry_arguments(json.loads(CONFIG.read_text())['/grasp_6d']['remote'])
statuses=[];feedback=[];epochs=[]
with rosbag.Bag(str(ROOT/'evidence.bag')) as bag:
    for topic,msg,t in bag.read_messages(topics=['/perception/unknown/detector_status',
        '/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch']):
        if topic=='/perception/unknown/detector_status':
            value=json.loads(msg.data)
            if value.get('generation')==6:
                statuses.append(dict(value,bag_receipt_ns=t.to_nsec()))
        elif topic=='/alicia_d/accepted_joint_states':
            if msg.header.frame_id=='sdk_measured':
                values=dict(zip(msg.name,msg.position))
                feedback.append((msg.header.stamp.to_nsec(),[values['Joint'+str(i)] for i in range(1,7)]))
        else:
            epochs.append(msg.stamp.to_nsec())
sources=[row for row in statuses if row.get('source_stamp_ns')]
first,last=sources[0]['source_stamp_ns'],sources[-1]['source_stamp_ns']
chosen={row['source_stamp_ns'] for row in sources if row['detail']!='perception_failure: support_plane_inlier_ratio_low'}
for fraction in (0.,.25,.5,.75,1.):
    chosen.add(min(sources,key=lambda row:abs(row['source_stamp_ns']-(first+fraction*(last-first))))['source_stamp_ns'])
chosen=sorted(chosen)
frames={n:{} for n in chosen}; bridge=CvBridge()
with rosbag.Bag(str(ROOT/'evidence.bag')) as bag:
    for topic,msg,t in bag.read_messages(topics=['/unknown_loss/exact_rgb','/unknown_loss/exact_depth']):
        ns=msg.header.stamp.to_nsec()
        if ns in frames:
            key='rgb' if topic.endswith('rgb') else 'depth'
            if key in frames[ns]:
                raise ValueError('duplicate sampled exact source')
            frames[ns][key]=bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8' if key=='rgb' else 'passthrough')
            frames[ns][key+'_frame_id']=msg.header.frame_id
if any(set(frame)!={'rgb','depth','rgb_frame_id','depth_frame_id'} for frame in frames.values()):
    raise ValueError('missing sampled exact RGB-D source')
OUT.mkdir(exist_ok=False)
cv2.setNumThreads(1)
statuses_by_source={r['source_stamp_ns']:r for r in sources}
previous_capture=json.loads((PREVIOUS/'result.json').read_text())
previous_replay=json.loads((PREVIOUS/'offline_geometry_20261005T0348Z/report.json').read_text())
old_entry=previous_capture['rows'][0]
with np.load(old_entry['source_file'],allow_pickle=False) as old:
    old_depth=old['depth_raw'].astype(np.float32)*float(old['depth_scale'])
old_plane=np.array(previous_replay['rows'][0]['replay']['selected']['metrics']['support_plane_base'])
ring_y,ring_x=np.indices(old_depth.shape);radius=np.hypot(ring_x-old_depth.shape[1]/2.,ring_y-old_depth.shape[0]/2.)
scale=min(old_depth.shape[1]/640.,old_depth.shape[0]/480.)
ring=(radius>=70*scale)&(radius<=170*scale)
rows=[]
for ns in chosen:
    frame=frames[ns];rgb,raw=frame['rgb'],frame['depth']
    fitting=[]
    def tracer(f,event,arg):
        if f.f_code is replay.tabletop._fit_plane.__code__:
            if event=='exception' and isinstance(arg[1],ValueError):
                fitting.append(dict(ok=False,reason=str(arg[1]),
                    measured_inlier_ratio=f.f_locals.get('ratio'),sample_count=len(f.f_locals['points'])))
            elif event=='return' and isinstance(arg,tuple):
                fitting.append(dict(ok=True,measured_inlier_ratio=arg[1],residual_m=arg[2],sample_count=arg[3]))
        return tracer
    prior_trace=sys.gettrace()
    try:
        sys.settrace(tracer)
        result,masks=replay.replay_stage(raw,rgb,camera,np.eye(4),cfg,
            float(meta['unknown_perception_parameters'].get('minimum_geometry_quality',.5)),args)
    finally:
        sys.settrace(prior_trace)
    row=dict(source_stamp_ns=ns,relative_to_first_gen6_seconds=(ns-first)*1e-9,
        recorded_detail=statuses_by_source[ns]['detail'],source_rgb_sha256=replay.support.array_sha(rgb),
        source_depth_sha256=replay.support.array_sha(raw),original_support_fit_attempts=fitting,replay=result)
    depth=raw.astype(np.float32)*camera['depth_scale']
    valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
    oldvalid=np.isfinite(old_depth)&(old_depth>=cfg.depth_min_m)&(old_depth<=cfg.depth_max_m)
    both=ring&valid&oldvalid
    points=replay.tabletop._project(depth,[camera[k] for k in ('fx','fy','cx','cy')])
    distance=np.einsum('ijk,k->ij',points,old_plane[:3])+old_plane[3]
    row['camera_local_depth_comparison']=dict(reference_source_stamp_ns=old_entry['source_stamp_ns'],
        current_ring_valid_fraction=float(valid[ring].mean()),previous_ring_valid_fraction=float(oldvalid[ring].mean()),
        both_valid_pixels=int(both.sum()),depth_delta_mm_percentiles=np.percentile((depth-old_depth)[both]*1000,[0,10,50,90,100]).tolist(),
        current_points_within_original_2mm_band_about_old_camera_plane=float((np.abs(distance[ring&valid])<=cfg.support_threshold_m).mean()),
        old_plane_used_only_as_diagnostic_reference_not_current_support=True)
    if ns in (chosen[0],chosen[-1]) or result.get('geometry',{}).get('ok'):
        name=str(ns)+'_rgb.png';cv2.imwrite(str(OUT/name),rgb);row['rgb_artifact']=dict(path=name,sha256=replay.support.sha(OUT/name))
    for key,mask in masks.items():
        name=str(ns)+'_'+key+'.png';cv2.imwrite(str(OUT/name),mask)
        row.setdefault('masks',{})[key]=dict(path=name,sha256=replay.support.sha(OUT/name))
    rows.append(row)
confirmations=[r for r in sources if r['detail'].startswith('initial_instance_confirming')]
def quantiles(values):
    values=[v for v in values if v is not None]
    return None if not values else dict(zip(('min','p50','p95','max'),map(float,np.percentile(values,[0,50,95,100]))))
timing={k:quantiles([r.get('frame_timing',{}).get(k) for r in sources]) for k in
    ('processing_start_source_age_sec','processing_wall_sec','processing_thread_cpu_sec','status_source_age_sec')}
qs=np.array([q for ns,q in feedback])
result=dict(schema=1,bag=str(ROOT/'evidence.bag'),archive_report_sha256=replay.support.sha(ROOT/'report.json'),
    metadata_sha256=replay.support.sha(ROOT/'metadata.json'),script_sha256=replay.support.sha(__file__),
    configuration_source=str(CONFIG),configuration_sha256=replay.support.sha(CONFIG),
    robot_commands_sent=False,motion_authorized=False,metric_geometry_authority=False,
    first_gen6_source_ns=first,last_gen6_source_ns=last,gen6_source_span_sec=(last-first)*1e-9,
    gen6_status_counts=dict(Counter(r['detail'] for r in statuses)),gen6_source_count=len(sources),
    initial_confirming_sources=[r['source_stamp_ns'] for r in confirmations],
    initial_confirming_gap_seconds=np.diff([r['source_stamp_ns'] for r in confirmations]).astype(float).tolist(),
    first_confirmation_delay_sec=(confirmations[0]['source_stamp_ns']-first)*1e-9,
    timing_quantiles=timing,accepted_feedback=dict(count=len(feedback),q_span_rad=np.ptp(qs,axis=0).tolist(),
        current_first_q=qs[0].tolist(),difference_from_previous_capture_q=(qs[0]-np.array(old_entry['q_source'])).tolist()),
    recorded_epochs=sorted(set(epochs)),previous_frame_initial_support_accepted=11,previous_frame_count=12,
    selected_source_count=len(chosen),rows=rows,
    limitations=['Exact later generation 6 anchor 1791173975525453329 and failed source1791173976326127767 are outside this bag; no event replay claimed.',
        'Current camera metadata was captured at recorder start, not per-frame stationary-temporal activity evidence.',
        'Original single-frame support + initial selector + RGB refinement + pre-snapshot geometry are replayed in camera_link; no source-time TF, live private cache or multiframe motion permission is synthesized.',
        'RANSAC rejected ratios are read by Python trace at the original exception, without changing thresholds or accepting rejected geometry.',
        'Earlier camera-plane agreement is diagnostic only; it is never treated as current support or a corrected depth.'])
result['initial_confirming_gap_seconds']=[v*1e-9 for v in result['initial_confirming_gap_seconds']]
shutil.copy2(__file__,OUT/Path(__file__).name)
(OUT/'report.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
print(json.dumps(dict(report=str(OUT/'report.json'),sha256=replay.support.sha(OUT/'report.json'),
    counts=result['gen6_status_counts'],timing=timing,replays=[dict(source=r['source_stamp_ns'],recorded=r['recorded_detail'],
    stop=r['replay']['stop_stage'],reason=r['replay']['reason'],fits=r['original_support_fit_attempts'],
    geometry=r['replay'].get('geometry')) for r in rows])))
