#!/usr/bin/env python3
"""Offline measured native/aligned support diagnostics; no ROS or device IO."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import warnings

import cv2
import numpy as np

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('ab_analysis',HERE/'analyze_camera_preset_ab_20261004.py')
ab=importlib.util.module_from_spec(spec);spec.loader.exec_module(ab)
tt=ab.tabletop
q=ab.quantiles
sha=ab.sha


def fit_audit(points,cfg):
    """Observe the unchanged fit, including its rejected model; never promote it."""
    state={}
    def trace(frame,event,arg):
        if frame.f_code is tt._fit_plane.__code__ and event in ('exception','return'):
            loc=frame.f_locals
            if all(k in loc for k in ('normal','offset','ratio')):
                state.update(plane=np.r_[loc['normal'],loc['offset']].copy(),ratio=float(loc['ratio']))
        return trace
    old=sys.gettrace();error=None;passed=False
    try:
        sys.settrace(trace)
        plane,ratio,residual,count=tt._fit_plane(points,cfg)
        state.update(plane=plane,ratio=ratio);passed=True
    except ValueError as exc:error=str(exc)
    finally:sys.settrace(old)
    plane=state.get('plane')
    out=dict(passed_original_gate=passed,reason=error,count=len(points),sampled_inlier_ratio=state.get('ratio'),
             admission_scope='Original _fit_plane numerical thresholds only; no source/quadrants/multi-radius lifecycle or target admission.',
             full_support_admission_evaluated=False,grasp_authority=False)
    if plane is not None:
        if plane[3]<0:plane=-plane
        distance=points@plane[:3]+plane[3]
        out.update(model=plane.tolist(),all_point_signed_residual_mm=q(distance*1000),
                   all_point_abs_residual_mm=q(abs(distance)*1000),
                   all_point_inlier_ratio=float(np.mean(abs(distance)<=cfg.support_threshold_m)))
    return plane,out


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--capture',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args();cv2.setNumThreads(1)
    root=args.capture;folder=root/'stages';summary=json.loads((folder/'summary.json').read_text())
    revised=root/'offline_comparison_revised_20261005.json';previous=json.loads(revised.read_text())
    for f in summary['files']:
        p=folder/f['name'];assert p.stat().st_size==f['bytes'] and sha(p)==f['sha256']
    cfg=tt.Config(**previous['config'])
    report=dict(schema=1,source=str(root),source_summary_sha256=sha(folder/'summary.json'),
        revised_report_sha256=sha(revised),script_sha256=sha(__file__),
        dependency_sha256={str(Path(m.__file__).resolve()):sha(m.__file__) for m in (ab,ab.stage,tt)},
        original_config=previous['config'],all_manifest_hashes_verified=True,
        scope='Offline native/raw and actual aligned/spatial/final values only. Diagnostic subsets and temporal summaries never become admitted geometry.',
        limitations=['A/B protocol was incomplete. No recovered TF, no physical depth ground truth, no calibrated exposure-time pose or per-frame AE metadata.',
        'Same native pixel coordinate across phases assumes approximately stationary camera/scene; recorded joint guard is stable but cannot prove optical/mechanical stationarity.',
        'A-derived reference, common pixels and temporal medians below are diagnostic comparisons only, not new support/target evidence.',
        'Native 3D is transformed using actual extrinsics and color axial Z. SDK aligned Z retains source depth axial Z and is rasterized, so stage comparisons are not pointwise-isomorphic or a pure alignment-error estimate.'],rows=[])
    infos=[];arrays=[];raw_points=[];raw_rings=[];fits=[]
    for i in range(1,25):
        info=json.loads((folder/f'sample_{i:02}.json').read_text())
        if not info['complete']:continue
        with np.load(folder/f'sample_{i:02}.npz',allow_pickle=False) as z:data={k:z[k].copy() for k in z.files}
        for name,a in data.items():assert ab.stage.array_sha(a)==info['arrays'][name]['sha256']
        infos.append(info);arrays.append(data)
    first=infos[0];mapper=ab.PairedPinholeRectification(first['runtime_profile']['color_intrinsics'],sdk=ab.rs)
    K=np.array([mapper.output_intrinsics[k] for k in ('fx','fy','cx','cy')]);scale=first['depth_scale_m']
    ray=ab.stage.rays(first['sdk_frames']['depth']['intrinsics'])
    R=np.asarray(first['depth_to_color']['rotation_column_major']).reshape(3,3,order='F');t=np.asarray(first['depth_to_color']['translation_m'])
    w,h=mapper.output_intrinsics['width'],mapper.output_intrinsics['height'];factor=min(w/640,h/480)
    yy,xx=np.indices((h,w));radius=np.hypot(xx-w/2,yy-h/2)
    fixed_ring=(radius>=70*factor)&(radius<=140*factor)
    report['fixed_diagnostic_support_ring']=dict(center_xy=[w/2,h/2],inner_px=70*factor,outer_px=140*factor,
        scope='One identical camera-image annulus for stage comparison, not the complete initial support multi-radius lifecycle.')
    for info,data in zip(infos,arrays):
        assert info['depth_scale_m']==scale and info['depth_to_color']==first['depth_to_color']
        assert info['sdk_frames']['depth']['intrinsics']==first['sdk_frames']['depth']['intrinsics']
        assert info['sdk_frames']['color']['intrinsics']==first['sdk_frames']['color']['intrinsics']
        assert info['runtime_profile']['geometry_intrinsics']==mapper.output_intrinsics
        optical=(ray*(data['raw_depth']*scale)[...,None])@R.T+t
        z=optical[...,2];u=K[0]*optical[...,0]/np.where(z>0,z,1)+K[2];v=K[1]*optical[...,1]/np.where(z>0,z,1)+K[3]
        rad=np.hypot(u-w/2,v-h/2)
        ring=(data['raw_depth']>0)&(z>=cfg.depth_min_m)&(z<=cfg.depth_max_m)&(rad>=70*factor)&(rad<=140*factor)
        points=np.stack((z,-optical[...,0],-optical[...,1]),axis=-1)
        plane,audit=fit_audit(points[ring],cfg);raw_points.append(points);raw_rings.append(ring);fits.append(plane)
        row=dict(index=info['sample_index'],phase=info['phase'],native_original_support=audit,
                 scale_m=scale,stages={},source_evidence_complete=info['source_evidence']['evidence_complete'])
        for name in ('aligned_depth','post_spatial_depth','pre_stationary_depth','post_stationary_depth','final_depth'):
            rect=data[name] if name=='final_depth' else mapper.apply(data['raw_rgb'],data[name])[1]
            valid=fixed_ring&(rect*scale>=cfg.depth_min_m)&(rect*scale<=cfg.depth_max_m)
            _,row['stages'][name]=fit_audit(tt._project(rect.astype(float)*scale,K)[valid],cfg)
        row['spatial_ring_change']=ab.stage.delta(data['aligned_depth'],data['post_spatial_depth'],
            fixed_ring.ravel()[mapper.target_indices].reshape(data['aligned_depth'].shape),scale)
        report['rows'].append(row)
    accepted_A=[f for f,row in zip(fits,report['rows']) if row['phase']!='B' and row['native_original_support']['passed_original_gate']]
    assert accepted_A
    reference=np.median(accepted_A,axis=0);reference/=np.linalg.norm(reference[:3])
    report['diagnostic_reference_A_median_plane']=reference.tolist()
    A=np.array([a['raw_depth'] for info,a in zip(infos,arrays) if info['phase']!='B'])
    a_fraction=np.mean(A>0,axis=0);a_stable=a_fraction>=.8
    for row,points,ring in zip(report['rows'],raw_points,raw_rings):
        row['diagnostic_A_reference_signed_residual_mm']=q((points[ring]@reference[:3]+reference[3])*1000)
        row['diagnostic_A_reference_subsets']={}
        for label,subset in [('A_observed_at_least_80pct',a_stable),('A_less_than_20pct',a_fraction<.2),('A_20_to_80pct',(a_fraction>=.2)&~a_stable)]:
            selected=ring&subset
            plane,stats=fit_audit(points[selected],cfg)
            stats['signed_residual_to_A_reference_mm']=q((points[selected]@reference[:3]+reference[3])*1000)
            row['diagnostic_A_reference_subsets'][label]=stats
    report['phase_summary']={};temporal_regions=[];temporal_maps={}
    for phase in ('A1','B','A2'):
        indices=[i for i,info in enumerate(infos) if info['phase']==phase]
        raw=np.asarray([arrays[i]['raw_depth'] for i in indices],dtype=float)*scale
        raw[raw==0]=np.nan
        with warnings.catch_warnings():
            warnings.simplefilter('ignore',RuntimeWarning)
            med=np.nanmedian(raw,axis=0);mad=np.nanmedian(abs(raw-med),axis=0);std=np.nanstd(raw,axis=0)
        count=np.sum(np.isfinite(raw),axis=0)
        temporal_region=np.logical_or.reduce([raw_rings[i] for i in indices])&(count>=math.ceil(len(indices)*.75))
        temporal_points=(ray*np.nan_to_num(med)[...,None])@R.T+t
        temporal_points=np.stack((temporal_points[...,2],-temporal_points[...,0],-temporal_points[...,1]),axis=-1)
        _,median_audit=fit_audit(temporal_points[temporal_region],cfg)
        median_audit.update(diagnostic_aggregate_only=True,grasp_authority=False,
                            represents_a_real_single_frame=False)
        temporal_regions.append(temporal_region);temporal_maps[phase]=(std,mad)
        rows=[report['rows'][i] for i in indices]
        report['phase_summary'][phase]=dict(frames=len(indices),
            native_original_gate_passes=sum(r['native_original_support']['passed_original_gate'] for r in rows),
            native_sampled_inlier_ratio=q([r['native_original_support']['sampled_inlier_ratio'] for r in rows]),
            native_all_abs_residual_p90_mm=q([r['native_original_support']['all_point_abs_residual_mm']['p90'] for r in rows]),
            temporal_native_support_pixels=int(temporal_region.sum()),temporal_per_pixel_std_mm=q(std[temporal_region]*1000),
            temporal_per_pixel_mad_mm=q(mad[temporal_region]*1000),
            diagnostic_temporal_median_support=median_audit,
            aligned_gate_passes=sum(r['stages']['aligned_depth']['passed_original_gate'] for r in rows),
            final_gate_passes=sum(r['stages']['final_depth']['passed_original_gate'] for r in rows))
    common=a_stable&np.logical_and.reduce(temporal_regions)
    report['common_native_temporal_comparison']=dict(pixel_count=int(common.sum()),
        mask_sha256=ab.stage.array_sha(common),
        selection='Same native pixels in all three phase support domains, observed in >=75% of each phase and >=80% of all A frames.',
        interpretation='Within-phase axial depth variability at identical sampled locations; not independent ground-truth error or an exposure-controlled causal estimate.',
        phases={phase:dict(std_mm=q(std[common]*1000),mad_mm=q(mad[common]*1000))
                for phase,(std,mad) in temporal_maps.items()})
    strict_common=np.logical_and.reduce(raw_rings)
    report['strict_all_frames_common_native_temporal_comparison']=dict(pixel_count=int(strict_common.sum()),
        mask_sha256=ab.stage.array_sha(strict_common),
        selection='Identical native pixels with actual positive support samples in every one of the 23 complete frames; raw_rings intersection.',
        limitations='Conditional on surviving all frames; does not represent new B pixels. Unequal A1/B/A2 frame counts 7/8/8. No physical ground truth or exposure control.',
        phases={phase:dict(std_mm=q(std[strict_common]*1000),mad_mm=q(mad[strict_common]*1000))
                for phase,(std,mad) in temporal_maps.items()})
    assert Path('/media/zhuyupei/SanStand').is_mount(),'no storage fallback'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
    print(json.dumps({'output':str(args.output),'sha256':sha(args.output),'phase_summary':report['phase_summary']},ensure_ascii=False))


if __name__=='__main__':main()
