#!/usr/bin/env python3
"""Offline additional search on frozen original samples; never changes gates."""
import argparse
from dataclasses import fields
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile
import zlib
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/alicia_grasp_modes/src'))
from alicia_grasp_modes import tabletop as t


def evaluate(points,plane,threshold):
    residual=points@plane[:3]+plane[3]
    mask=abs(residual)<=threshold
    return int(mask.sum()),float(np.sqrt(np.mean(residual[mask]**2))) if mask.any() else float('inf'),mask


def offset_consensus(points,normal,threshold):
    normal=np.asarray(normal,dtype=float);normal/=np.linalg.norm(normal)
    projection=np.sort(points@normal)
    end=np.searchsorted(projection,projection+2*threshold,side='right')
    count=end-np.arange(len(projection));start=int(count.argmax());stop=int(end[start])
    plane=np.r_[normal,-float((projection[start]+projection[stop-1])*.5)]
    return plane


def refine(points,plane,threshold,steps=6):
    best=plane.copy();count,rms,mask=evaluate(points,best,threshold)
    for _ in range(steps):
        if mask.sum()<3:break
        center=points[mask].mean(axis=0)
        _,_,vh=np.linalg.svd(points[mask]-center,full_matrices=False)
        candidate=offset_consensus(points,vh[-1],threshold)
        n,r,m=evaluate(points,candidate,threshold)
        if (n,-r)>(count,-rms):best,count,rms=candidate,n,r
        if np.array_equal(mask,m):break
        mask=m
    return best


def search(points,initial,reference,threshold):
    rng=np.random.RandomState(17)
    best=initial.copy();count,rms,_=evaluate(points,best,threshold)
    for _ in range(4096):
        ix=rng.randint(len(points),size=3)
        if len(set(ix.tolist()))<3:continue
        a,b,c=points[ix];n=np.cross(b-a,c-a);length=np.linalg.norm(n)
        if length<1e-9:continue
        n/=length;plane=np.r_[n,-n@a]
        c0,r0,_=evaluate(points,plane,threshold)
        if (c0,-r0)>(count,-rms):best,count,rms=plane,c0,r0
    ransac=refine(points,best,threshold)
    candidates={'expanded_4096_ransac_local_refinement':ransac,
        'original_normal_optimal_offset':offset_consensus(points,initial[:3],threshold),
        'reference_normal_optimal_offset_diagnostic_only':offset_consensus(points,reference[:3],threshold)}
    # Finite diagnostic orientation grid; no global optimum claim. The center
    # is the current measured model, not an imported support-plane authority.
    normal=ransac[:3]/np.linalg.norm(ransac[:3]);axis1,axis2=t._plane_axes(normal)
    best=ransac.copy();count,rms,_=evaluate(points,best,threshold)
    for a in np.deg2rad(np.arange(-2.,2.00001,.2)):
        for b in np.deg2rad(np.arange(-2.,2.00001,.2)):
            n=normal+np.tan(a)*axis1+np.tan(b)*axis2
            plane=offset_consensus(points,n,threshold)
            c0,r0,_=evaluate(points,plane,threshold)
            if (c0,-r0)>(count,-rms):best,count,rms=plane,c0,r0
    candidates['local_orientation_grid_with_optimal_offsets']=refine(points,best,threshold)
    return candidates


def main():
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--replay',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();s=json.loads((a.capture/'summary.json').read_text())
    assert s['archive_closed'] and s['complete_without_writer_error']
    ext=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller').resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount() and ext in a.output.resolve().parents
    a.output.mkdir(parents=True,exist_ok=False)
    replay=json.loads(a.replay.read_text());cfg=t.Config(**replay['config']);K=replay['intrinsics'];bridge=CvBridge()
    report=dict(schema_version=1,source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        original_replay=str(a.replay),original_replay_sha256=hashlib.sha256(a.replay.read_bytes()).hexdigest(),
        scope='Additional finite search only on the exact original row-major 1800 samples; no input/gate modification or production use; rejected best models are diagnostic only',
        search=dict(random_seed=17,random_trials=4096,offset_interval_width_m=2*cfg.support_threshold_m,
                    local_grid_degrees=[-2.,2.,.2],local_refinement_steps=6),rows=[])
    with zipfile.ZipFile(a.capture/'diagnostic.zip') as z:
        meta=json.loads(z.read(next(n for n in z.namelist() if n.startswith('metadata/'))))
        camera=meta['parameters']['/camera']
        for source in replay['rows']:
            if source['detail']!='support_measurement_pending_revalidation':continue
            ns=source['source_stamp_ns'];msg=Image().deserialize(zlib.decompress(z.read('pairs/%d_depth.rosmsg.zlib'%ns)))
            assert msg.header.stamp.to_nsec()==ns
            depth=np.asarray(bridge.imgmsg_to_cv2(msg,'passthrough'),dtype=np.float32)*float(camera['depth_scale'])
            F=np.asarray(source['T_base_camera']);ref=np.asarray(source['reference_support_plane_base'])
            anchor=np.asarray(source['anchor_position_base']);footprint=np.asarray(source['reference_support_footprint_base'])
            ref_camera=F.T@ref;center=np.asarray(source['center_uv'])
            v,u=np.indices(depth.shape);scale=min(depth.shape[1]/640.,depth.shape[0]/480.)
            radius=np.hypot(u-center[0],v-center[1]);valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
            for oldring in source['rings']:
                start=time.monotonic();outer=oldring['outer_base_px'];ring=valid&(radius>=70*scale)&(radius<=outer*scale)
                pu,pv=u[ring],v[ring];points=np.asarray(t._project_pixels(depth[ring],pu,pv,K),dtype=float)
                samples=points[np.linspace(0,len(points)-1,min(len(points),cfg.support_max_samples),dtype=int)]
                initial=np.asarray(oldring['fit']['original_function_locals']['best_plane'])
                row=dict(source_stamp_ns=ns,outer_base_px=outer,
                    original_ratio=oldring['fit']['original_function_locals']['ratio'],
                    original_sample_sha256=hashlib.sha256(samples.tobytes()).hexdigest(),candidates={})
                for name,plane in search(samples,initial,ref_camera,cfg.support_threshold_m).items():
                    plane=t._normalized_plane(plane)
                    if plane[3]<0:plane=-plane
                    n,rms,inside=evaluate(samples,plane,cfg.support_threshold_m)
                    fulln,fullrms,fullinside=evaluate(points,plane,cfg.support_threshold_m)
                    quadrant=t._support_quadrants(fullinside,pu,pv,center)
                    model=t._normalized_plane(np.linalg.solve(F.T,plane))
                    entry=dict(sample_inlier_ratio=n/len(samples),sample_inlier_count=n,
                        sample_residual_m=rms,full_ring_inlier_ratio=fulln/len(points),
                        full_ring_inlier_residual_m=fullrms,quadrants=quadrant,plane_camera=plane.tolist(),
                        original_fit_quality_pass=bool(n>=cfg.support_min_points and n/len(samples)>=cfg.support_min_ratio and rms<=cfg.support_threshold_m and quadrant>=3))
                    try:
                        _,agreement=t._validate_support_plane_update(ref,model,anchor,cfg,footprint)
                        entry.update(reference_consistency_pass=True,agreement=agreement)
                    except ValueError as exc:entry.update(reference_consistency_pass=False,reference_error=str(exc))
                    entry['all_original_contracts_pass']=entry['original_fit_quality_pass'] and entry['reference_consistency_pass']
                    row['candidates'][name]=entry
                row['elapsed_sec']=time.monotonic()-start;report['rows'].append(row)
                print(json.dumps(dict(source=ns,outer=outer,original=row['original_ratio'],
                    extra={k:dict(ratio=v['sample_inlier_ratio'],full=v['full_ring_inlier_ratio'],passed=v['all_original_contracts_pass']) for k,v in row['candidates'].items()},elapsed=row['elapsed_sec'])),flush=True)
    (a.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
