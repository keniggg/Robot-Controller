#!/usr/bin/env python3
"""Exact recorded RGB-D support fits and asynchronous filter diagnostics; no ROS I/O."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import rosbag
import rospy
import tf2_ros
from tf.transformations import quaternion_matrix

WORK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORK/'src/alicia_grasp_modes/src'))
from alicia_grasp_modes import tabletop
spec=importlib.util.spec_from_file_location('old_support_audit',WORK/'.runtime/analyze_current_support_rgbd_20261007.py')
helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for part in iter(lambda:f.read(1024*1024),b''): h.update(part)
    return h.hexdigest()


def percentile(a):
    return dict(zip(('min','p10','p50','p90','p95','p99','max'),np.percentile(a,[0,10,50,90,95,99,100]).tolist())) if len(a) else None


def serialize(msg):
    f=io.BytesIO();msg.serialize(f);return f.getvalue()


def fit_diagnostic(points,cfg,u,v,center,seed=None):
    original=np.random.RandomState
    try:
        if seed is not None:
            np.random.RandomState=lambda ignored:original(seed)
        outcome,qualified_plane=helper.original_fit(points,cfg)
    finally:
        np.random.RandomState=original
    model=qualified_plane if qualified_plane is not None else outcome['original_locals'].get('best_plane')
    result=dict(outcome=outcome,all_valid_points=int(len(points)))
    if model is None:return result
    model=np.asarray(model);signed=np.einsum('ij,j->i',points,model[:3])+model[3]
    if model[3]<0:model=-model;signed=-signed
    sample=np.linspace(0,len(points)-1,min(len(points),cfg.support_max_samples),dtype=int)
    result.update(model_camera=model.tolist(),full_ring_ratio=float(np.mean(abs(signed)<=cfg.support_threshold_m)),
        full_ring_signed_residual_mm=percentile(signed*1000),full_ring_abs_residual_mm=percentile(abs(signed)*1000),
        sample_abs_residual_mm=percentile(abs(signed[sample])*1000),
        full_ring_fractions_within_mm={str(limit):float(np.mean(abs(signed)*1000<=limit)) for limit in (.5,1.,1.5,2.,2.5,3.,4.,5.)},
        covered_quadrants=tabletop._support_quadrants(abs(signed)<=cfg.support_threshold_m,u,v,center),
        quadrant_abs_residual_mm={label:percentile(abs(signed[(u<center[0] if left else u>=center[0])&(v<center[1] if top else v>=center[1])])*1000)
            for label,left,top in [('top_left',True,True),('top_right',False,True),('bottom_left',True,False),('bottom_right',False,False)]})
    return result


def main(bag_path,out):
    if not Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('external_mount_missing')
    if not str(out.resolve()).startswith('/media/zhuyupei/SanStand/实验记录/Robot-Controller/'):raise ValueError('external_output_required')
    out.mkdir(parents=True,exist_ok=True)
    cv2.setNumThreads(1)
    statuses=[];meta=[];first_task_failure_ns=None;tfbuf=tf2_ros.Buffer(cache_time=rospy.Duration(600))
    with rosbag.Bag(str(bag_path)) as bag:
        for topic,msg,received in bag.read_messages(topics=['/perception/unknown/detector_status','/grasp_record/camera_runtime_metadata','/tf','/tf_static','/grasp/state']):
            if topic=='/tf' or topic=='/tf_static':
                for transform in msg.transforms:
                    (tfbuf.set_transform_static if topic=='/tf_static' else tfbuf.set_transform)(transform,'recorded_exact_bag')
            elif topic=='/grasp/state':
                if msg.active and msg.state=='FAILED' and first_task_failure_ns is None:
                    first_task_failure_ns=msg.header.stamp.to_nsec() if hasattr(msg,'header') else msg.stamp.to_nsec()
            elif topic.endswith('detector_status'):
                row=json.loads(msg.data)
                if row.get('generation')==3 and row.get('detail') in ('tracked','support_measurement_pending_revalidation'):
                    statuses.append((row['source_stamp_ns'],row,received.to_nsec()))
            else:meta.append((received.to_nsec(),json.loads(msg.data)))
    assert first_task_failure_ns is not None
    statuses=[r for r in statuses if r[0]<first_task_failure_ns]
    meta=[r for r in meta if r[0]<first_task_failure_ns]
    statuses.sort(key=lambda r:r[0]);gaps=[i for i,r in enumerate(statuses) if r[1]['detail']=='support_measurement_pending_revalidation']
    chosen_gaps=[gaps[0],gaps[len(gaps)//2],gaps[-1]]
    selected={};pairs=[]
    for i in chosen_gaps:
        group=[]
        before=next((j for j in range(i-1,-1,-1) if statuses[j][1]['detail']=='tracked'),None)
        after=next((j for j in range(i+1,len(statuses)) if statuses[j][1]['detail']=='tracked'),None)
        for j,role in [(before,'before_control'),(i,'failure'),(after,'after_control')]:
            if j is None:continue
            selected[statuses[j][0]]=statuses[j];group.append(dict(source_ns=statuses[j][0],role=role))
        pairs.append(dict(failure_source_ns=statuses[i][0],frames=group))
    images={ns:{} for ns in selected}
    with rosbag.Bag(str(bag_path)) as bag:
        for topic,msg,received in bag.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']):
            ns=msg.header.stamp.to_nsec()
            if ns in images:
                key='rgb' if '/color/' in topic else 'depth'
                raw=serialize(msg);header,array=helper.decode_image(raw)
                images[ns][key]=(header,array,hashlib.sha256(raw).hexdigest())
    camera=meta[0][1]['parameters'];cfg=tabletop.Config()
    policy=dict(configuration=asdict(cfg),source='production Config defaults; launch has no support overrides; exact success metrics checked below',
        source_config_sha256=digest(tabletop.__file__),camera_profile=camera['runtime_profile'],camera_parameters={k:v for k,v in camera.items() if k!='runtime_profile'})
    result=dict(schema_version=1,scope='three_recorded_support_ratio_gaps_and_nearest_successes',bag=str(bag_path),bag_sha256=digest(bag_path),
        first_task_failure_ns=first_task_failure_ns,script_sha256=digest(__file__),original_fit_helper_sha256=digest(helper.__file__),production_changes=False,motion_authorized=False,ros_master_access=False,
        policy=policy,pairs=pairs,frames=[],limitations=[
            'Camera runtime metadata is a 1 Hz asynchronous parameter sample, explicitly not an image time association; no exact-frame filter causality is claimed.',
            'Published depth is already spatially/regionally processed; unfiltered native depth is not in this bag.',
            'RANSAC seed sensitivity is diagnostic only; no alternate model is installed or admitted as execution geometry.',
            'Support settings are production defaults. Exact successful fits reproduce recorded metrics to expose configuration or coordinate mismatches.'])
    intrinsics=np.asarray([camera[k] for k in ('fx','fy','cx','cy')])
    owner,owner_regions,owner_replay=helper.diagnostic_owner(camera,dict(regional_stationary_evidence=dict(regions=camera['runtime_profile']['stationary_temporal']['regions'])),(camera['height'],camera['width']))
    result['pixel_owner_reproduction']=owner_replay
    arrays={}
    for ns,(unused,status,received) in sorted(selected.items()):
        pair=images[ns]
        assert set(pair)=={'rgb','depth'},('exact_pair_missing',ns)
        assert pair['rgb'][0]['source_stamp_ns']==pair['depth'][0]['source_stamp_ns']==ns
        depth=pair['depth'][1].astype(np.float32)*float(camera['depth_scale'])
        stamp=rospy.Time(ns//10**9,ns%10**9)
        tf=tfbuf.lookup_transform('base_link','camera_link',stamp).transform
        T=quaternion_matrix([tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w]);T[:3,3]=[tf.translation.x,tf.translation.y,tf.translation.z]
        center=tabletop.project_base_to_pixel(np.asarray(status['anchor_position_base']),T,intrinsics)
        v,u=np.indices(depth.shape);scale=min(depth.shape[1]/640.,depth.shape[0]/480.);radius=np.hypot(u-center[0],v-center[1])
        valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
        row=dict(source_ns=ns,recorded_detail=status['detail'],source_age_sec=status['frame_timing']['status_source_age_sec'],
            source_headers={key:pair[key][0] for key in pair},serialized_message_sha256={key:pair[key][2] for key in pair},
            anchor_base=status['anchor_position_base'],ring_center_px=center.tolist(),T_base_camera=T.tolist(),rings=[],
            recorded_support_metrics={k:v for k,v in status.get('geometry_metrics',{}).items() if k.startswith('support_')})
        sequence=True
        for outer in (170,140,110):
            ring=valid&(radius>=70*scale)&(radius<=outer*scale);uv=(u[ring],v[ring]);points=tabletop._project_pixels(depth[ring],*uv,intrinsics)
            diag=fit_diagnostic(points,cfg,*uv,center);diag.update(outer_base_px=outer,outer_actual_px=outer*scale,consulted_by_original_sequence=sequence,
                valid_fraction=float(ring.sum()/max(1,np.count_nonzero((radius>=70*scale)&(radius<=outer*scale)))),
                actual_regional_ownership_valid_point_fraction={region['name']:float(np.count_nonzero(ring&(owner==index))/max(1,ring.sum())) for index,region in enumerate(owner_regions)})
            if sequence and diag['outcome']['original_fit_qualified']:
                row['replayed_selected_outer_base_px']=outer;sequence=False
                recorded=row['recorded_support_metrics']
                row['success_reproduction_check']=dict(recorded_available=bool(recorded),
                    ratio_delta=diag['outcome']['ratio']-recorded.get('support_current_audit_inlier_ratio',diag['outcome']['ratio']),
                    residual_m_delta=diag['outcome']['residual_m']-recorded.get('support_current_audit_residual_m',diag['outcome']['residual_m']),
                    outer_px_delta=outer*scale-recorded.get('support_current_audit_ring_outer_radius_px',outer*scale))
            if diag['consulted_by_original_sequence']:
                sensitivities=[]
                for seed in (1,3,7,11,23,29,37,43):
                    sd=fit_diagnostic(points,cfg,*uv,center,seed=seed)
                    sensitivities.append(dict(seed=seed,qualified=sd['outcome']['original_fit_qualified'],sample_ratio=sd['outcome']['original_locals'].get('ratio'),full_ring_ratio=sd.get('full_ring_ratio')))
                diag['offline_seed_sensitivity']=sensitivities
            row['rings'].append(diag)
        row['replayed_support_ratio_available']=not sequence
        before=[m for m in meta if m[0]<=ns];after=[m for m in meta if m[0]>ns]
        row['asynchronous_metadata_brackets']=[dict(received_ns=m[0],time_offset_sec=(m[0]-ns)/1e9,temporal=m[1]['parameters']['runtime_profile']['stationary_temporal']) for m in ([before[-1]] if before else [])+([after[0]] if after else [])]
        npz=out/('source_%d.npz'%ns);np.savez_compressed(str(npz),rgb=pair['rgb'][1],depth_raw=pair['depth'][1],depth_scale=camera['depth_scale'],T_base_camera=T,intrinsics=intrinsics)
        row['source_extract']=dict(path=str(npz),sha256=digest(npz))
        result['frames'].append(row);arrays[ns]=depth
    for pair in pairs:
        fail_ns=pair['failure_source_ns'];fail=arrays[fail_ns];stats=[]
        for entry in pair['frames']:
            if entry['role']=='failure':continue
            reference=arrays[entry['source_ns']];valid=(fail>=cfg.depth_min_m)&(reference>=cfg.depth_min_m)&(fail<=cfg.depth_max_m)&(reference<=cfg.depth_max_m)
            stats.append(dict(source_ns=entry['source_ns'],role=entry['role'],shared_valid_pixels=int(valid.sum()),depth_delta_mm=percentile((fail[valid]-reference[valid])*1000)))
        pair['pixel_temporal_differences']=stats
    temporal=[]
    for received,entry in meta:
        t=entry['parameters']['runtime_profile']['stationary_temporal'];h=t.get('pair_history',{})
        raw=h.get('last_complete_raw_frames',{})
        temporal.append(dict(received_ns=received,active=t.get('active'),filtered_fraction=t.get('filtered_fraction'),fully_active=t.get('fully_active'),regional_output_active=t.get('regional_output_active'),history_epoch=h.get('history_epoch'),
            history_reason=h.get('reason'),history_state=h.get('state'),rejected_packet_count=h.get('rejected_packet_count'),
            actual_fps_metadata={k:v.get('metadata_raw',{}).get('actual_fps') for k,v in raw.items()},
            regions=[{k:r[k] for k in ('name','filter_applied','history_reset','qualified','warming','visual') if k in r} for r in t.get('regions',[])]))
    result['asynchronous_temporal_metadata']=temporal
    result['temporal_summary']=dict(samples=len(meta),metadata_has_exact_image_time_association=False,
        fully_active_count=sum(bool(t['fully_active']) for t in temporal),legacy_whole_frame_active_field_count=sum(bool(t['active']) for t in temporal),regional_output_active_count=sum(bool(t['regional_output_active']) for t in temporal),
        filtered_fraction=percentile([t['filtered_fraction'] for t in temporal]),unique_history_epochs=len({t['history_epoch'] for t in temporal}),
        history_reason_counts=dict(Counter(t['history_reason'] for t in temporal)),
        regions={name:dict(samples=len(rows),filter_applied_count=sum(bool(r.get('filter_applied')) for r in rows),
            history_reset_count=sum(bool(r.get('history_reset')) for r in rows),
            visual_reason_counts=dict(Counter(r.get('visual',{}).get('reason') for r in rows)),
            motion_p90_px=percentile([r['visual']['motion_p90_px'] for r in rows if 'motion_p90_px' in r.get('visual',{})]))
            for name in ('center','top_left','top_right','bottom_left','bottom_right')
            for rows in [[r for t in temporal for r in t['regions'] if r['name']==name]]})
    payload=(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n').encode();path=out/'result.json';path.write_bytes(payload)
    brief=[dict(source_ns=r['source_ns'],detail=r['recorded_detail'],available=r['replayed_support_ratio_available'],
        ratios=[round(x['outcome']['original_locals']['ratio'],6) for x in r['rings']],full_ring_ratios=[round(x['full_ring_ratio'],6) for x in r['rings']],reproduction=r.get('success_reproduction_check')) for r in result['frames']]
    print(json.dumps(dict(result=str(path),sha256=hashlib.sha256(payload).hexdigest(),frames=brief,temporal_summary=result['temporal_summary']),ensure_ascii=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--bag',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();main(args.bag,args.output)
