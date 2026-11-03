#!/usr/bin/env python3
"""Offline finite model comparison on sealed gen23 depth; never grants authority."""
import argparse
from dataclasses import asdict, fields
import hashlib
import importlib.util
import json
from pathlib import Path
import time
import zipfile
import numpy as np

W = Path(__file__).resolve().parents[1]
C = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/aligned_grasp_20261007T112309.952794Z/recorded_retry_20261007T120118Z/raw_support')
spec = importlib.util.spec_from_file_location('offline_depth_decoder', W/'.runtime/analyze_current_support_rgbd_20261007.py')
a = importlib.util.module_from_spec(spec); spec.loader.exec_module(a)
t = a.table


def score(points, plane, threshold):
    d = np.einsum('ij,j->i', points, plane[:3]) + plane[3]
    mask = abs(d) <= threshold
    return int(mask.sum()), float(np.sqrt(np.mean(d[mask]**2))) if mask.any() else None, mask


def candidates(points):
    """The original 128 trials, original seed and original order, no new seed."""
    rng = np.random.RandomState(17); planes=[]; origins=[]; normals=[]
    for i in range(128):
        ids = rng.randint(len(points),size=3)
        if len(set(ids.tolist())) < 3: continue
        x,y,z=points[ids]; n=np.cross(y-x,z-x); length=np.linalg.norm(n)
        if length < 1e-9: continue
        n/=length; planes.append((i,np.r_[n,-float(n@x)])); origins.append(x); normals.append(n)
    return planes,np.asarray(origins),np.asarray(normals)


def best_offset(points, normal, threshold):
    """For an existing normal, exact finite sliding 2*t projection interval."""
    projected = np.sort(np.einsum('ij,j->i',points,normal))
    ends=np.searchsorted(projected,projected+2*threshold,side='right')
    counts=ends-np.arange(len(projected)); index=int(np.argmax(counts))
    end=int(ends[index]-1)
    # Midpoint avoids placing both extreme observations on a rounding boundary.
    offset=-(projected[index]+projected[end])*.5
    return np.r_[normal,offset]


def timed(fn,repeats=5):
    wall=[];cpu=[]
    for _ in range(repeats):
        w=time.perf_counter(); c=time.thread_time(); fn()
        wall.append(time.perf_counter()-w);cpu.append(time.thread_time()-c)
    return dict(repeats=repeats,wall_sec=wall,thread_cpu_sec=cpu)


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args()
    out=Path(args.output)
    ext=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
    if ext not in out.resolve().parents or out.exists():raise ValueError('new_external_output_required')
    report=json.loads((C/'analysis/report.json').read_text())
    if report['production_tabletop_sha256']!=a.sha(Path(t.__file__).read_bytes()):raise ValueError('production_changed')
    result=dict(diagnostic_only=True,motion_authorized=False,production_modified=False,
                source_sha256=a.sha(Path(__file__).read_bytes()),
                production_tabletop_sha256=report['production_tabletop_sha256'],
                replay_report_sha256=a.sha((C/'analysis/report.json').read_bytes()),
                archive_sha256=report['archive_sha256'],rows=[],
                limitations=['These first-15-second pending frames recovered in production; terminal loss after 28 seconds is outside RGB-D capture.',
                    'Candidate comparison cannot prove global optimality or independently certify source/pose qualification.',
                    'Reference normal is only a diagnostic candidate; all scores use current-frame original measured points.',
                    'No new seed, threshold, ring, freshness, sampling, target identity or production behavior is changed.',
                    'Offline timing includes current host load and is not the original execution timeline.'])
    with zipfile.ZipFile(C/'diagnostic.zip') as z:
        entries={e['name']:e for e in json.loads(z.read('entry_manifest.json'))}
        def read(name):
            b=z.read(name);e=entries[name]
            if len(b)!=e['bytes'] or a.sha(b)!=e['sha256']:raise ValueError('archive_entry_hash:'+name)
            return b
        metadata=json.loads(read('metadata/000000.json')) if 'metadata/000000.json' in entries else json.loads(read(next(n for n in entries if n.startswith('metadata/'))))
        cam=metadata['parameters']['/camera'];rawcfg=metadata['parameters']['/unknown_perception']
        cfg=t.Config(**{f.name:rawcfg[f.name] for f in fields(t.Config) if f.name in rawcfg})
        result['config']=asdict(cfg);K=[cam[k] for k in ('fx','fy','cx','cy')]
        for row in report['pending_replay']:
            ns=row['source_stamp_ns']; pair=json.loads(read('pairs/%s.json'%ns))['streams']['depth']
            raw=a.bounded_original_message(read('pairs/%s_depth.rosmsg.zlib'%ns),pair['serialized_bytes'])
            if a.sha(raw)!=pair['serialized_sha256']:raise ValueError('depth_hash')
            header,pixels=a.decode_image(raw)
            if header['source_stamp_ns']!=ns:raise ValueError('source_mismatch')
            depth=pixels.astype(np.float32)*float(cam['depth_scale'])
            status=json.loads(json.loads(read(row['status_entry']))['original_status_data'])
            T=np.asarray(row['tf_matrix']);ref=np.asarray(status['reference_support_plane_base']);center=np.asarray(row['projected_anchor_center_uv'])
            uv=np.indices(depth.shape);v,u=uv;radius=np.hypot(u-center[0],v-center[1]);scale=min(depth.shape[1]/640,depth.shape[0]/480)
            valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
            rr=dict(source_stamp_ns=ns,status_entry=row['status_entry'],rings=[])
            result['rows'].append(rr)
            for ringreport in row['rings']:
                outer=ringreport['outer_base_px'];mask=valid&(radius>=70*scale)&(radius<=outer*scale)
                full=t._project_pixels(depth[mask],u[mask],v[mask],K)
                P=np.asarray(full,dtype=float)[np.linspace(0,len(full)-1,min(len(full),cfg.support_max_samples),dtype=int)]
                original=np.asarray(ringreport['fit']['original_locals']['best_plane'])
                count,resid,_=score(P,original,cfg.support_threshold_m)
                if count!=ringreport['fit']['original_locals']['best_count']:raise ValueError('original_sample_score_mismatch')
                planes,origins,normals=candidates(P)
                pool=[('original_final',original)]
                rawscored=sorted([(score(P,plane,cfg.support_threshold_m)[0],i,plane) for i,plane in planes],key=lambda x:-x[0])
                for _,i,plane in rawscored[:4]:
                    current=plane.copy()
                    for j in range(2):
                        selected=score(P,current,cfg.support_threshold_m)[2]
                        if selected.sum()<3:break
                        mu=P[selected].mean(axis=0);_,_,vh=np.linalg.svd(P[selected]-mu,full_matrices=False)
                        current=np.r_[vh[-1],-float(vh[-1]@mu)]
                        pool.append(('original_trial_%d_svd_%d'%(i,j+1),current.copy()))
                for i,plane in planes:
                    pool.append(('original_trial_%d_exact_offset'%i,best_offset(P,plane[:3],cfg.support_threshold_m)))
                pool.append(('original_final_exact_offset',best_offset(P,original[:3],cfg.support_threshold_m)))
                refcamera=t._normalized_plane(T.T@ref)
                pool.append(('immutable_reference_evaluated_on_current',refcamera))
                pool.append(('immutable_reference_normal_exact_current_offset',best_offset(P,refcamera[:3],cfg.support_threshold_m)))
                ranked=sorted([(score(P,plane,cfg.support_threshold_m)[:2],name,plane) for name,plane in pool],key=lambda x:(-x[0][0],x[0][1] if x[0][1] is not None else float('inf')))
                bestscore,name,best=ranked[0];fullcount,fullresidual,fullmask=score(full,best,cfg.support_threshold_m)
                quadrants=t._support_quadrants(fullmask,u[mask],v[mask],center)
                try:
                    _,agreement=t._validate_support_plane_update(ref,t._normalized_plane(np.linalg.solve(T.T,best)),np.asarray(status['anchor_position_base']),cfg,np.asarray(status['reference_support_footprint_base']))
                except ValueError as e:agreement={'error':str(e)}
                def original_run():
                    try:t._fit_plane(full,cfg)
                    except ValueError as e:
                        if str(e)!='support_plane_inlier_ratio_low':raise
                # Identical arithmetic/order trials as a bounded vectorized diagnostic,
                # not a production algorithm replacement. Pre-generation time excluded.
                scalar=np.stack([np.abs(np.einsum('ij,j->i',P-x,n))<=cfg.support_threshold_m for x,n in zip(origins,normals)],axis=1)
                def vector_eval():
                    parts=[]
                    for lo in range(0,len(normals),16):
                        d=np.einsum('ijk,jk->ij',P[:,None,:]-origins[None,lo:lo+16,:],normals[lo:lo+16])
                        parts.append(abs(d)<=cfg.support_threshold_m)
                    return np.concatenate(parts,axis=1)
                vector=vector_eval()
                def scalar_eval():
                    return np.stack([np.abs(np.einsum('ij,j->i',P-x,n))<=cfg.support_threshold_m for x,n in zip(origins,normals)],axis=1)
                rr['rings'].append(dict(outer_base_px=outer,point_count=len(full),sample_sha256=a.sha(P.tobytes()),
                    original_count=count,original_ratio=count/len(P),best_model=name,best_plane=best.tolist(),
                    best_count=bestscore[0],best_ratio=bestscore[0]/len(P),sample_residual_m=bestscore[1],
                    full_ratio=fullcount/len(full),full_residual_m=fullresidual,quadrants=quadrants,reference_agreement=agreement,
                    threshold_and_coverage_and_reference_passed=(bestscore[0]>=len(P)*cfg.support_min_ratio and quadrants>=3 and 'error' not in agreement),
                    tested_models=len(pool),passing_model_names=[n for s,n,_ in ranked if s[0]>=len(P)*cfg.support_min_ratio],
                    original_fit_timing=timed(original_run,3),scalar_distance_timing=timed(scalar_eval,3),
                    vector16_distance_timing=timed(vector_eval,3),distance_masks_bitwise_equal=bool(np.array_equal(scalar,vector))))
            print(json.dumps({'source':ns,'original_max':max(r['original_ratio'] for r in rr['rings']),
                              'best_max':max(r['best_ratio'] for r in rr['rings']),
                              'any_full_contract':any(r['threshold_and_coverage_and_reference_passed'] for r in rr['rings'])}),flush=True)
    out.mkdir(parents=True,exist_ok=False);b=a.encoded(result);(out/'report.json').write_bytes(b)
    (out/'SHA256SUMS').write_text(a.sha(b)+'  report.json\n');print(str(out),a.sha(b))


if __name__=='__main__':main()
