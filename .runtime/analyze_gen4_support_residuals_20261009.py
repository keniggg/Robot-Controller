#!/usr/bin/env python3
"""Limited sealed GEN4 depth replay; no ROS master or robot I/O."""
import argparse
from collections import Counter
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import rosbag
import rospy
import tf2_ros
from tf.transformations import quaternion_matrix

WORK=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('prior_support_replay',WORK/'.runtime/analyze_gen3_support_residuals_20261009.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
from alicia_flexible_grasp.vision.pair_history_continuity import interleaved_pair_reason,_stream,_consecutive


def main(record,first_loss_path,out):
    if not Path('/media/zhuyupei/SanStand').is_mount():raise RuntimeError('SanStand not mounted')
    if not str(out.resolve()).startswith('/media/zhuyupei/SanStand/实验记录/Robot-Controller/'):raise ValueError('external output required')
    manifest=json.loads((record/'manifest.json').read_text())
    assert manifest['state']=='complete' and manifest['recorder_closed'] is True
    bag_path=record/'keyframes.bag';bag_sha=old.digest(bag_path)
    assert next(f['sha256'] for f in manifest['files'] if f['path']=='keyframes.bag')==bag_sha
    first=json.loads(first_loss_path.read_text());interpretation=first['interpretation']
    failed_ns=[r['payload']['source_stamp_ns'] for r in first['preceding_five_statuses'] if r['payload']['detail']=='support_measurement_pending_revalidation']
    assert len(failed_ns)==4
    last_good=interpretation['last_good_source_ns'];terminal=interpretation['failing_source_ns']
    out.mkdir(parents=True,exist_ok=True);cv2.setNumThreads(1)
    tfbuf=tf2_ros.Buffer(cache_time=rospy.Duration(600));statuses={};metadata=[]
    with rosbag.Bag(str(bag_path)) as bag:
        for topic,msg,received in bag.read_messages(topics=['/perception/unknown/detector_status','/grasp_record/camera_runtime_metadata','/tf','/tf_static']):
            if topic in ('/tf','/tf_static'):
                for tf in msg.transforms:
                    (tfbuf.set_transform_static if topic=='/tf_static' else tfbuf.set_transform)(tf,'original_GEN4_bag')
            elif topic.endswith('detector_status'):
                row=json.loads(msg.data);ns=row.get('source_stamp_ns')
                if row.get('generation')==4 and ns and ns<=terminal:statuses[ns]=(row,received.to_nsec())
            elif received.to_nsec()<=terminal+2000000000:metadata.append((received.to_nsec(),json.loads(msg.data)))
    before=sorted(ns for ns,(r,_) in statuses.items() if ns<=last_good and r.get('detail')=='tracked')[-2:]
    # Terminal source never reached the production fit: audit it only as a
    # counterfactual, explicitly separated from the four real fit failures.
    selected=sorted(set(before+failed_ns+[terminal]));images={ns:{} for ns in selected}
    with rosbag.Bag(str(bag_path)) as bag:
        for topic,msg,received in bag.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw']):
            ns=msg.header.stamp.to_nsec()
            if ns>terminal:break
            if ns in images:
                raw=old.serialize(msg);header,array=old.helper.decode_image(raw)
                images[ns]['rgb' if '/color/' in topic else 'depth']=(header,array,old.hashlib.sha256(raw).hexdigest())
    camera=metadata[0][1]['parameters'];cfg=old.tabletop.Config()
    deployed=json.loads((record/'deployment_sources.json').read_text())['source_sha256']
    key='src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py'
    assert old.digest(WORK/key)==deployed[key]
    intrinsics=np.asarray([camera[k] for k in ('fx','fy','cx','cy')])
    owner,regions,owner_proof=old.helper.diagnostic_owner(camera,dict(regional_stationary_evidence=dict(regions=camera['runtime_profile']['stationary_temporal']['regions'])),(camera['height'],camera['width']))
    result=dict(schema_version=1,record=str(record),bag_sha256=bag_sha,script_sha256=old.digest(__file__),reuse_script_sha256=old.digest(old.__file__),production_changes=False,robot_commands=False,
        first_loss=interpretation,policy=dict(configuration=asdict(cfg),tabletop_sha256=deployed[key],camera=camera,source='unchanged deployed Config defaults; controls verify exact recorded fit metrics'),owner_reproduction=owner_proof,frames=[],temporal_metadata=[],history_predicate_replay=[],limitations=[
        'The bag records published processed depth, not raw pre-temporal SDK depth. No counterfactual filtering result is available.',
        'Runtime metadata samples are asynchronous at approximately 1 Hz; exact-image filter application cannot be inferred from nearest metadata.',
        'Config defaults and unchanged deployed source are used. Two exact successful controls validate ratios/residuals/radii; no claim of independent full ROS parameter snapshot.',
        'Terminal rejected source did not reach production support fit; its offline fit is explicitly counterfactual and cannot renew retired authority.'])
    for ns in selected:
        pair=images[ns];assert set(pair)=={'rgb','depth'},(ns,pair.keys())
        assert pair['rgb'][0]['source_stamp_ns']==pair['depth'][0]['source_stamp_ns']==ns
        status,received=statuses[ns];anchor=status.get('anchor_position_base',first['first_loss']['payload']['first_loss']['anchor_position_base'])
        depth=pair['depth'][1].astype(np.float32)*float(camera['depth_scale'])
        tf=tfbuf.lookup_transform('base_link','camera_link',rospy.Time(ns//10**9,ns%10**9)).transform
        T=quaternion_matrix([tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w]);T[:3,3]=[tf.translation.x,tf.translation.y,tf.translation.z]
        center=old.tabletop.project_base_to_pixel(np.asarray(anchor),T,intrinsics)
        v,u=np.indices(depth.shape);scale=min(depth.shape[1]/640.,depth.shape[0]/480.);radius=np.hypot(u-center[0],v-center[1]);valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
        row=dict(source_ns=ns,role=('counterfactual_after_capture_cutoff' if ns==terminal else 'recorded_failure' if ns in failed_ns else 'recorded_success_control'),recorded_detail=status['detail'],recorded_reason=status.get('source_qualification',{}).get('reason'),source_age_sec=status.get('frame_timing',{}).get('status_source_age_sec'),processing_wall_sec=status.get('frame_timing',{}).get('processing_wall_sec'),source_headers={k:x[0] for k,x in pair.items()},serialized_sha256={k:x[2] for k,x in pair.items()},T_base_camera=T.tolist(),ring_center_px=center.tolist(),recorded_metrics={k:x for k,x in status.get('geometry_metrics',{}).items() if k.startswith('support_')},rings=[])
        pending=True
        for outer in (170,140,110):
            ring=valid&(radius>=70*scale)&(radius<=outer*scale);points=old.tabletop._project_pixels(depth[ring],u[ring],v[ring],intrinsics)
            diag=old.fit_diagnostic(points,cfg,u[ring],v[ring],center)
            diag.update(outer_base_px=outer,consulted_by_original_sequence=pending,ownership_fraction={r['name']:float(np.count_nonzero(ring&(owner==i))/max(1,ring.sum())) for i,r in enumerate(regions)})
            if pending and diag['outcome']['original_fit_qualified']:
                pending=False;row['selected_outer_base_px']=outer
                if row['role']=='recorded_success_control':
                    recorded=row['recorded_metrics'];row['success_reproduction']=dict(ratio_delta=diag['outcome']['ratio']-recorded['support_current_audit_inlier_ratio'],residual_delta_m=diag['outcome']['residual_m']-recorded['support_current_audit_residual_m'],outer_delta_px=outer*scale-recorded['support_current_audit_ring_outer_radius_px'])
            row['rings'].append(diag)
        row['support_fit_qualified']=not pending
        before_meta=[m for m in metadata if m[0]<=ns];after_meta=[m for m in metadata if m[0]>ns]
        row['asynchronous_metadata_bracket']=[dict(received_ns=t,offset_seconds=(t-ns)/1e9,temporal=m['parameters']['runtime_profile']['stationary_temporal']) for t,m in ([before_meta[-1]] if before_meta else [])+([after_meta[0]] if after_meta else [])]
        p=out/('source_%d.npz'%ns);np.savez_compressed(str(p),rgb=pair['rgb'][1],depth_raw=pair['depth'][1],depth_scale=camera['depth_scale'],T_base_camera=T,intrinsics=intrinsics)
        row['extract']=dict(path=str(p),sha256=old.digest(p));result['frames'].append(row)
    seen=set()
    for received,m in metadata:
        if received<last_good-5000000000 or received>terminal+1500000000:continue
        t=m['parameters']['runtime_profile']['stationary_temporal'];h=t.get('pair_history',{})
        center=next(r for r in t['regions'] if r['name']=='center')
        result['temporal_metadata'].append(dict(received_ns=received,time_association=m.get('image_time_association'),center=center,filtered_fraction=t.get('filtered_fraction'),history=h))
        key=json.dumps(h,sort_keys=True)
        if key in seen:continue
        seen.add(key)
        previous=h.get('last_complete_raw_frames');rejected=h.get('rejected_raw_frames');after=h.get('next_accepted_raw_frames')
        if not previous or not rejected:continue
        row=dict(recorded_reason=h['reason'],state=h['state'],history_epoch=h['history_epoch'],replayed_first_rejection=interleaved_pair_reason(previous,rejected,h['last_complete_filter_host_monotonic'],h['first_rejection_host_monotonic']))
        if after:row['original_successor_checks']={k:dict(counter_delta=_stream(after[k])[0]-_stream(previous[k])[0],interval_ms=_stream(after[k])[1]-_stream(previous[k])[1],fps=_stream(previous[k])[3],consecutive=_consecutive(_stream(previous[k]),_stream(after[k]))) for k in ('color','depth')}
        result['history_predicate_replay'].append(row)
    result['temporal_summary']=dict(samples=len(result['temporal_metadata']),center_applied=sum(bool(r['center'].get('filter_applied')) for r in result['temporal_metadata']),center_reset=sum(bool(r['center'].get('history_reset')) for r in result['temporal_metadata']),center_visual_reasons=dict(Counter(r['center'].get('visual',{}).get('reason') for r in result['temporal_metadata'])))
    path=out/'result.json';path.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(result=str(path),sha256=old.digest(path),frames=[dict(source_ns=r['source_ns'],role=r['role'],qualified=r['support_fit_qualified'],ratios=[x['outcome']['original_locals'].get('ratio') for x in r['rings']],p90_mm=[x.get('full_ring_abs_residual_mm',{}).get('p90') for x in r['rings']],reproduction=r.get('success_reproduction')) for r in result['frames']],temporal=result['temporal_summary'],history=result['history_predicate_replay']),ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--record',type=Path,required=True);p.add_argument('--first-loss',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();main(a.record,a.first_loss,a.output)
