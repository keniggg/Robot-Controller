#!/usr/bin/env python3
"""Offline A1/saved-baseline/A2 description; no ROS IO, devices, or motion authority."""
import argparse
from collections import Counter
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import sys

import cv2
import numpy as np

WORK=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(WORK/'.runtime/realsense-2.56.5'))
sys.path.insert(0,str(WORK/'src/alicia_grasp_modes/src'))
sys.path.insert(0,str(WORK/'src/alicia_flexible_grasp_supervisor/src'))
import pyrealsense2 as rs
from alicia_grasp_modes import tabletop,rgb_instance,measured_mask_seeds
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy,propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_handoff import _appearance,HandoffPolicy
from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification,rectification_rgb_valid_mask


def load(name):
    spec=importlib.util.spec_from_file_location(name,WORK/'.runtime'/(name+'.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

stage=load('analyze_camera_stages_20261004')
prior=load('analyze_live_rectified_measurements_20261004')
sha=stage.sha;quantiles=stage.quantiles
REFERENCE=prior.BASE


def transform(info):
    x,y,z,w=info['quaternion_xyzw'];assert abs(x*x+y*y+z*z+w*w-1)<1e-6
    T=np.eye(4);T[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                         [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                         [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    T[:3,3]=info['translation'];return T


def associate(rgb,valid_rgb,ref,search,gates):
    proposals=propose_rgb_observation_regions(rgb,policy=search,rgb_valid=valid_rgb)
    alternatives=[]
    for i,c in enumerate(proposals.candidates):
        hist,contour=_appearance(rgb,c.mask.astype(np.uint8))
        color=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(hist*ref[0])))))
        shape=float(cv2.matchShapes(ref[1],contour,cv2.CONTOURS_MATCH_I1,0.))
        alternatives.append(dict(index=i,bbox=list(c.bbox_xywh),pixels=int(c.area_px),
            color_distance=color,contour_distance=shape,eligible=color<=gates.max_colour_distance and shape<=gates.max_contour_distance,
            score=.65*color/gates.max_colour_distance+.35*shape/gates.max_contour_distance))
    eligible=sorted((a for a in alternatives if a['eligible']),key=lambda a:a['score'])
    unique=bool(eligible and (len(eligible)==1 or eligible[1]['score']-eligible[0]['score']>gates.ambiguity_margin))
    mask=proposals.candidates[eligible[0]['index']].mask.astype(bool) if unique else None
    return mask,dict(proposal_reason=proposals.reason,alternatives=alternatives,unique=unique,
                     chosen_index=eligible[0]['index'] if unique else None)


def measured_gate(rgb,z16,scale,K,T,mask,cfg,valid_rgb):
    depth=z16.astype(np.float32)*scale
    try:seg=tabletop.segment(depth,K,T,cfg)
    except ValueError as exc:return dict(support_pass=False,reason=str(exc))
    chosen,reason=tabletop.TargetTracker(cfg).choose(seg,depth.shape)
    out=dict(support_pass=True,support_metrics=seg.metrics,support_plane_base=seg.plane_base.tolist(),
             candidate_count=len(seg.candidates),initial_choice_reason=reason,candidates=[])
    choice=None
    for i,c in enumerate(seg.candidates):
        cm=c.mask>0;iou=float(np.count_nonzero(mask&cm)/np.count_nonzero(mask|cm))
        item=dict(index=i,bbox=list(c.bbox),pixels=int(cm.sum()),quality=c.quality,
                  target_iou=iou,initial_selected=c is chosen)
        if c.quality>=.5:
            refined,trace=prior.refine_with_seed_trace(rgb,depth,K,T,c,cfg,valid_rgb)
            item['refinement']=trace
            if refined is not None:
                rm=refined.mask>0
                item['refined_target_iou']=float(np.count_nonzero(mask&rm)/np.count_nonzero(mask|rm))
        if c is chosen:choice=item
        out['candidates'].append(item)
    if chosen is None:outcome=reason
    elif chosen.quality<.5:outcome='insufficient_geometry_quality'
    elif not choice['refinement']['passed']:outcome='initial_instance_'+choice['refinement']['reason']
    else:outcome='single_frame_initial_refinement_passed'
    out['initial_exact_frame_outcome']=outcome
    out['target_best_measured_candidate_iou']=max((c['target_iou'] for c in out['candidates']),default=0.)
    out['target_measured_iou_gate_pass']=any(c['target_iou']>=.5 and c.get('refinement',{}).get('passed') for c in out['candidates'])
    return out


def traced_plane(points,mask,cfg,space):
    trace_data={};old=sys.gettrace()
    def trace(frame,event,arg):
        if frame.f_code is tabletop._fit_plane.__code__ and event=='exception':
            loc=frame.f_locals
            if 'ratio' in loc:trace_data['rejected_inlier_ratio']=loc['ratio']
            if all(k in loc for k in ['points','normal','offset']):
                distances=np.abs(loc['points']@loc['normal']+loc['offset'])*1000
                trace_data['rejected_fit_all_sample_abs_distance_mm']=quantiles(distances)
        return trace
    try:
        sys.settrace(trace);plane,audit=stage.plane_fit(points,mask,cfg)
    finally:sys.settrace(old)
    if 'plane_optical' in audit:audit['plane_'+space]=audit.pop('plane_optical')
    audit.update(trace_data);return plane,audit


def original_initial_support(z16,scale,K,cfg):
    # This is the camera-local original support precondition only. It needs no
    # base transform and does not fabricate one to run downstream geometry.
    depth=z16.astype(np.float32)*scale;points=tabletop._project(depth,K)
    valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
    y,x=np.indices(depth.shape);factor=min(depth.shape[1]/640.,depth.shape[0]/480.)
    radius=np.hypot(x-depth.shape[1]/2.,y-depth.shape[0]/2.)
    attempts=[]
    for outer in (170,140,110):
        ring=valid&(radius>=70*factor)&(radius<=outer*factor)
        plane,audit=traced_plane(points,ring,cfg,'camera_link')
        audit['outer_radius_px']=outer*factor;attempts.append(audit)
        if plane is not None:
            supporting=ring&(np.abs(np.einsum('ijk,k->ij',points,plane[:3])+plane[3])<=cfg.support_threshold_m)
            covered=tabletop._support_quadrants(supporting,x,y,np.array([depth.shape[1]/2.,depth.shape[0]/2.]))
            audit['covered_quadrants']=covered
            return dict(passed=covered>=3 and audit['residual_mm']<=cfg.support_threshold_m*1000,attempts=attempts)
        if audit['error']!='support_plane_inlier_ratio_low':break
    return dict(passed=False,reason=attempts[-1]['error'],attempts=attempts)


def camera_stats(z16,scale,K,mask,cfg,ring):
    depth=z16.astype(float)*scale;valid=(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
    points=tabletop._project(depth,K)
    plane,audit=traced_plane(points,valid&ring,cfg,'camera_link')
    heights=points[mask&valid]@plane[:3]+plane[3] if plane is not None else np.array([])
    return dict(mask_pixels=int(mask.sum()),valid_pixels=int(np.count_nonzero(mask&valid)),
        valid_fraction=float(np.count_nonzero(mask&valid)/mask.sum()),zero_pixels=int(np.count_nonzero(mask&(z16==0))),
        optical_z_mm=quantiles(depth[mask&valid]*1000),support=audit,
        local_support_height_mm=quantiles(heights*1000),foreground_4_to_120mm_pixels=(int(np.count_nonzero((heights>=cfg.foreground_min_m)&(heights<=cfg.foreground_max_m))) if plane is not None else None)),plane


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--capture',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();cv2.setNumThreads(1);assert rs.__version__=='2.56.5'
    root=args.capture;folder=root/'stages';s=json.loads((folder/'summary.json').read_text());outer=json.loads((root/'result.json').read_text())
    for item in s['files']:
        f=folder/item['name'];assert f.stat().st_size==item['bytes'] and sha(f)==item['sha256']
    assert sum(p.stat().st_size for p in folder.iterdir() if p.is_file())<=s['maximum_artifact_bytes']
    assert sha(root/'camera_preset_ab_observer.py')==outer['wrapper_sha256']
    reference=json.loads((REFERENCE/'result.json').read_text());refrow=reference['rows'][2]
    assert sha(refrow['source_file'])==refrow['source_file_sha256']
    with np.load(refrow['source_file'],allow_pickle=False) as z:ref_rgb=z['color_bgr'].copy()
    search=CenterSearchPolicy(center_xy=(.5,.5),radius_fraction=.125);gates=HandoffPolicy();cfg=tabletop.Config(**refrow['config'])
    rv=rectification_rgb_valid_mask(refrow['source_context']['camera'],ref_rgb.shape[:2])
    proposals=propose_rgb_observation_regions(ref_rgb,policy=search,rgb_valid=rv)
    region=proposals.selected
    assert refrow['observation_interest_ready'] and list(region.bbox_xywh)==refrow['interest_volume']['bbox_xywh']
    ref=_appearance(ref_rgb,region.mask.astype(np.uint8))
    first=json.loads((folder/'sample_01.json').read_text());mapper=PairedPinholeRectification(first['runtime_profile']['color_intrinsics'],sdk=rs)
    assert mapper.contract==first['runtime_profile']['rectification']
    K=np.array([mapper.output_intrinsics[k] for k in ['fx','fy','cx','cy']])
    native_info=first['sdk_frames']['depth']['intrinsics'];color_info=first['sdk_frames']['color']['intrinsics']
    nr,cr=stage.rays(native_info),stage.rays(color_info);nm,cm=stage.intrinsics(native_info),stage.intrinsics(color_info)
    R=np.array(first['depth_to_color']['rotation_column_major']).reshape(3,3,order='F');t=np.array(first['depth_to_color']['translation_m'])
    a,b,a2=[p['sensor_after'] for p in s['phases']]
    report=dict(schema=1,scope=__doc__,script_sha256=sha(__file__),source_capture=str(root),
        source_result_sha256=sha(root/'result.json'),source_summary_sha256=sha(folder/'summary.json'),
        archived_wrapper_sha256=sha(root/'camera_preset_ab_observer.py'),all_source_files_and_array_hashes_verified=True,
        reference=dict(result=str(REFERENCE/'result.json'),result_sha256=sha(REFERENCE/'result.json'),
            source_file_sha256=refrow['source_file_sha256'],row_index=2,source_stamp_ns=refrow['source_stamp_ns'],
            purpose='generic recorded RGB interest; current appearance hypothesis only'),
        config=asdict(cfg),config_scope='Previous live rectified observation original measurement configuration; not a newly captured runtime perception parameter snapshot',
        search_policy=asdict(search),appearance_gates=asdict(gates),map_contract=mapper.contract,
        source_implementation_sha256={str(Path(m.__file__).resolve()):sha(m.__file__) for m in [tabletop,rgb_instance,measured_mask_seeds]},
        protocol=dict(experiment_valid=s['experiment_valid'],capture_complete=s['capture_complete'],counts=s['counts'],
            guard_faults=s['guard_faults'],errors=s['errors'],restoration_succeeded=s['restoration']['succeeded'],
            high_accuracy_parameters_restored=s['initial_sensor']==s['restoration']['sensor_after'],A1_A2_sensor_snapshots_equal=a==a2,
            original_read_restored=s['original_read_restored'],original_sync_restored=s['original_sync_restored'],
            actual_artifact_bytes=sum(p.stat().st_size for p in folder.iterdir() if p.is_file()),maximum_artifact_bytes=s['maximum_artifact_bytes'],
            elapsed_read_seconds=s['last_read_monotonic']-s['first_read_monotonic'],post_restore_warmup=outer.get('post_restore_warmup'),
            visual_nodes_restored=outer.get('visual_nodes_restored'),clock_scope=s['clock_scope']),
        sensor_difference={k:dict(A1=v,B=b['advanced_settings']['parameters'].get(k),A2=a2['advanced_settings']['parameters'].get(k))
                           for k,v in a['advanced_settings']['parameters'].items() if b['advanced_settings']['parameters'].get(k)!=v},
        phase_sensor_options={p['label']:p['sensor_after']['options'] for p in s['phases']},
        limitations=[
            'Whole protocol is incomplete. This is descriptive evidence from preserved frames, not a qualified causal experiment or grasp authorization.',
            'Actual q brackets refer to host read-completion ROS anchors, not calibrated SDK exposure timestamps. Exact TF is used only when recorded for that same anchor; no latest or neighboring TF replacement.',
            'Generic appearance is a target hypothesis, not proof of identity, an active selection, frozen volume, or measured geometry authority.',
            'Raw zeros are counted in SDK K/D/extrinsics mapped footprints at explicit assumed color Z. Range endpoints are sensitivity samples, not a continuous coverage proof. Raw and RGB pixel indices are never equated.',
            'Aligned Z16 retains native axial Z per SDK convention. Local plane residuals/heights are measured consistency statistics, not independently known object thickness or absolute depth accuracy.',
            'Final-chain A/B comparison includes the existing unknown-only stationary temporal policy. Raw and aligned pre-filter stages are the primary matcher comparison.',
            'Auto exposure and auto white balance remain enabled. Actual exposure/gain metadata is unsupported; option readback equality does not establish exposure or illumination equality. RGB brightness and appearance changes are recorded, not corrected away.',
            'A1/B/A2 is one short ordered sequence. Stochastic holes, RGB motion, lighting drift and unmeasured physical accuracy remain possible confounders.',
            'Original returned arrays are compared to deterministic central rectification from saved inputs. Observer code preserves original return objects, but no simultaneous observer-free run proves zero timing effect.',
            'Sparse offline per-frame segment/choose/refine is not the continuous online lifecycle, full cross-view handoff, or a successful grasp.'
        ],rows=[])
    for index in range(1,25):
        info=json.loads((folder/f'sample_{index:02}.json').read_text())
        assert info['sdk_frames']['depth']['intrinsics']==native_info and info['sdk_frames']['color']['intrinsics']==color_info
        assert info['depth_to_color']==first['depth_to_color']
        with np.load(folder/f'sample_{index:02}.npz',allow_pickle=False) as z:data={k:z[k] for k in z.files}
        for name,array in data.items():assert stage.array_sha(array)==info['arrays'][name]['sha256']
        evidence=info['source_evidence'];row=dict(index=index,phase=info['phase'],complete=info['complete'],
            source_evidence=evidence,guard_at_capture=info['guard_at_capture'],sdk_frames=info['sdk_frames'],
            pair_timing=info['production_read_diagnostics'].get('pair_timing'),
            temporal=info['runtime_profile'].get('stationary_temporal'),
            stationary_rgb=info['production_read_diagnostics'].get('stationary_rgb_evidence'),
            capture_overhead_ms=info['capture_overhead_sec']*1000,read_duration_ms=(info['read_completed_monotonic']-info['read_started_monotonic'])*1000)
        report['rows'].append(row)
        if not info['complete']:
            row['excluded_reason']='production rejected RGB-depth timing pair; preserved raw arrays cannot be a synchronized measurement';continue
        scale=info['depth_scale_m'];assert info['runtime_profile']['rectification']==mapper.contract
        assert info['runtime_profile']['geometry_intrinsics']==mapper.output_intrinsics
        rgb,verify_final=mapper.apply(data['raw_rgb'],data['post_stationary_depth'])
        clipped=data['post_stationary_depth'].copy();z=clipped.astype(np.float32)*scale;clipped[(z<cfg.depth_min_m)|(z>cfg.depth_max_m)]=0
        _,verify_final=mapper.apply(data['raw_rgb'],clipped)
        assert np.array_equal(rgb,data['final_rgb']) and np.array_equal(verify_final,data['final_depth'])
        row['original_final_rgb_and_depth_reconstructed_exactly']=True
        mask,assoc=associate(rgb,mapper.rgb_valid,ref,search,gates);row['appearance']=assoc
        if mask is None:continue
        aligned_mask=mask.ravel()[mapper.target_indices].reshape(data['raw_depth'].shape)
        assert aligned_mask.shape==(color_info['height'],color_info['width'])
        gray=cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY);yy,xx=np.indices(mask.shape)
        ys,xs=np.nonzero(mask);center=(float(np.mean(xs)),float(np.mean(ys)))
        inner=float(np.hypot(xs.max()-xs.min()+1,ys.max()-ys.min()+1));outer_radius=min(170.,3*inner)
        radius=np.hypot(xx-center[0],yy-center[1]);ring=(radius>=inner)&(radius<=outer_radius)
        row['outline']=dict(pixels=int(mask.sum()),aligned_grid_selected_pixels=int(aligned_mask.sum()),
            centroid=center,bbox=[int(xs.min()),int(ys.min()),int(xs.max()-xs.min()+1),int(ys.max()-ys.min()+1)],
            mask_sha256=stage.array_sha(mask),unobserved_rgb_pixels=int(np.count_nonzero(mask&~mapper.rgb_valid)))
        row['brightness']=dict(target_gray=quantiles(gray[mask]),local_ring_gray=quantiles(gray[ring&mapper.rgb_valid]))
        row['local_support_ring']=dict(inner_px=inner,outer_px=outer_radius)
        row['stages']={};stage_arrays={}
        for name in ['aligned_depth','post_spatial_depth','pre_stationary_depth','post_stationary_depth','final_depth']:
            rect=data[name] if name=='final_depth' else mapper.apply(data['raw_rgb'],data[name])[1]
            stage_arrays[name]=rect;row['stages'][name],_=camera_stats(rect,scale,K,mask,cfg,ring)
        row['deltas']={left+'__'+right:stage.delta(data[left],data[right],aligned_mask,scale)
            for left,right in [('aligned_depth','post_spatial_depth'),('post_spatial_depth','pre_stationary_depth'),('pre_stationary_depth','post_stationary_depth')]}
        nominal=float(np.median(data['aligned_depth'][aligned_mask&(data['aligned_depth']>0)])*scale)
        footprints=[];row['raw_mapped_footprints']=[]
        for depth in sorted(set([.03,.06,.12,nominal,.5,1.,2.])):
            footprint,_=stage.projected_native_footprint(aligned_mask,np.repeat(depth,int(aligned_mask.sum())),cr,nm,R,t)
            footprints.append(footprint);positive=footprint&(data['raw_depth']>0)
            row['raw_mapped_footprints'].append(dict(assumed_color_z_m=depth,pixels=int(footprint.sum()),
                positive=int(positive.sum()),positive_fraction=float(positive.sum()/footprint.sum()),
                raw_z_mm=quantiles(data['raw_depth'][positive]*scale*1000),nominal=depth==nominal))
        vicinity=cv2.dilate(np.logical_or.reduce(footprints).astype(np.uint8),np.ones((11,11),np.uint8)).astype(bool)
        scalar=stage.raw_point_raster(data['raw_depth'],scale,native_info,cm,R,t,vicinity)
        actual=data['aligned_depth'];row['alignment']=dict(
            scalar_raw_corner_coverage=int(np.count_nonzero(scalar[aligned_mask])),actual_positive=int(np.count_nonzero(actual[aligned_mask])),
            actual_zero_despite_raw_coverage=int(np.count_nonzero(aligned_mask&(scalar>0)&(actual==0))),
            actual_zero_without_raw_coverage=int(np.count_nonzero(aligned_mask&(scalar==0)&(actual==0))),
            actual_positive_without_scalar_coverage=int(np.count_nonzero(aligned_mask&(scalar==0)&(actual>0))))
        # Direct positive raw sample geometry: use actual measured native Z,
        # extrinsics and SDK color projection; never assign depth to zeros.
        raw=data['raw_depth'];positive=(raw>0)&vicinity
        native_points=nr[positive]*(raw[positive]*scale)[:,None];color_points=native_points@R.T+t
        uv=np.rint(stage.project(color_points,cm)).astype(int)
        inside=(uv[:,0]>=0)&(uv[:,0]<aligned_mask.shape[1])&(uv[:,1]>=0)&(uv[:,1]<aligned_mask.shape[0])
        hits=inside.copy();hits[inside]&=aligned_mask[uv[inside,1],uv[inside,0]]
        # Independently fit original raw SDK points over mapped local RGB ring.
        aligned_ring=ring.ravel()[mapper.target_indices].reshape(raw.shape)
        footprint,_=stage.projected_native_footprint(aligned_ring,np.repeat(nominal,int(aligned_ring.sum())),cr,nm,R,t)
        ringraw=footprint&(raw>0);rawpoints=nr*(raw*scale)[:,:,None]
        rawplane,rawaudit=traced_plane(rawpoints,ringraw,cfg,'native_optical')
        h=native_points[hits]@rawplane[:3]+rawplane[3] if rawplane is not None else np.array([])
        row['raw_positive_measurements']=dict(count=int(hits.sum()),raw_z_mm=quantiles(native_points[hits,2]*1000),
            color_z_mm=quantiles(color_points[hits,2]*1000),support=rawaudit,
            local_support_height_mm=quantiles(h*1000),foreground_4_to_120mm_pixels=(int(np.count_nonzero((h>=.004)&(h<=.12))) if rawplane is not None else None))
        row['original_initial_support']={name:original_initial_support(stage_arrays[name],scale,K,cfg) for name in ['aligned_depth','final_depth']}
        if evidence['evidence_complete']:
            tf=evidence['exact_tf'];assert tf['source_stamp_ns']==info['anchor_ros_stamp_ns']
            T=transform(tf);row['measurement_replay']={name:measured_gate(rgb,stage_arrays[name],scale,K,T,mask,cfg,mapper.rgb_valid)
                for name in ['aligned_depth','final_depth']}
        else:row['measurement_replay_unavailable']='exact anchor TF unavailable; no replacement pose'
        print(json.dumps(dict(index=index,phase=row['phase'],aligned=row['stages']['aligned_depth']['valid_fraction'],
                             final=row['stages']['final_depth']['valid_fraction'],tf=evidence['evidence_complete'])),flush=True)
    report['phase_summary']={}
    for phase in ['A1','B','A2']:
        rows=[r for r in report['rows'] if r['phase']==phase];usable=[r for r in rows if 'stages' in r]
        summary=dict(attempted=len(rows),complete_pairs=sum(r['complete'] for r in rows),appearance_unique=len(usable),
            exact_anchor_evidence=sum(r['source_evidence']['evidence_complete'] for r in rows),
            temporal_active=sum(bool(r.get('temporal',{}).get('active')) for r in usable),
            capture_overhead_ms=quantiles(np.array([r['capture_overhead_ms'] for r in rows])),
            raw_nominal_positive_fraction=quantiles(np.array([next(f['positive_fraction'] for f in r['raw_mapped_footprints'] if f['nominal']) for r in usable])),
            raw_target_height_median_mm=quantiles(np.array([r['raw_positive_measurements']['local_support_height_mm']['p50'] for r in usable if r['raw_positive_measurements']['local_support_height_mm'] is not None])),
            raw_support_fit_failure_count=sum('error' in r['raw_positive_measurements']['support'] for r in usable),
            target_gray_median=quantiles(np.array([r['brightness']['target_gray']['p50'] for r in usable])),
            ring_gray_median=quantiles(np.array([r['brightness']['local_ring_gray']['p50'] for r in usable])),stages={})
        for name in ['aligned_depth','final_depth']:
            stages=[r['stages'][name] for r in usable];replays=[r['measurement_replay'][name] for r in usable if 'measurement_replay'in r]
            summary['stages'][name]=dict(valid_fraction=quantiles(np.array([v['valid_fraction'] for v in stages])),
                target_foreground_4_to_120mm_counts=[v['foreground_4_to_120mm_pixels'] for v in stages],
                local_support_residual_mm=quantiles(np.array([v['support']['residual_mm'] for v in stages if 'residual_mm'in v['support']])),
                local_support_inlier_ratio=quantiles(np.array([v['support']['inlier_ratio'] for v in stages if 'inlier_ratio'in v['support']])),
                original_initial_support_passes=sum(r['original_initial_support'][name]['passed'] for r in usable),
                original_initial_support_failure_reasons=dict(Counter(r['original_initial_support'][name].get('reason') for r in usable if not r['original_initial_support'][name]['passed'])),
                exact_tf_measurement_frames=len(replays),initial_outcomes=dict(Counter(v.get('initial_exact_frame_outcome',v.get('reason')) for v in replays)),
                maximum_target_iou=max((v.get('target_best_measured_candidate_iou',0.) for v in replays),default=0.),
                target_measured_gate_passes=sum(v.get('target_measured_iou_gate_pass',False) for v in replays))
        report['phase_summary'][phase]=summary
    assert Path('/media/zhuyupei/SanStand').is_mount(),'no storage fallback'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as out:json.dump(report,out,ensure_ascii=False,indent=2,allow_nan=False);out.write('\n')
    print(json.dumps(dict(output=str(args.output),sha256=sha(args.output),phase_summary=report['phase_summary']),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
