#!/usr/bin/env python3
"""Offline fixed local/geometric motion controls for seed-quality risk review."""
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np

W=Path(__file__).resolve().parents[1]
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/regional_production_20261007T011805.778547Z')
QUALITY=W/'.runtime/analyze_visual_seed_quality_20261007.py'
spec=importlib.util.spec_from_file_location('quality_risk_fixed_helper',QUALITY)
q=importlib.util.module_from_spec(spec);spec.loader.exec_module(q)
QUALITIES=(.02,.12)


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def patches(bounds):
    x0,y0,x1,y1=bounds;cx=(x0+x1)/2;cy=(y0+y1)/2;w=x1-x0;h=y1-y0
    return [('quadrant_tl',(x0,y0,cx,cy)),('quadrant_tr',(cx,y0,x1,cy)),
            ('quadrant_bl',(x0,cy,cx,y1)),('quadrant_br',(cx,cy,x1,y1)),
            ('left_strip',(x0,y0,x0+w/4,y1)),('right_strip',(x1-w/4,y0,x1,y1)),
            ('top_strip',(x0,y0,x1,y0+h/4)),('bottom_strip',(x0,y1-h/4,x1,y1))]


def field(shape,bounds):
    yy,xx=np.indices(shape,dtype=np.float32);x0,y0,x1,y1=bounds
    # Fixed 8-pixel smooth transition; no mask depends on points or flow.
    edge=np.minimum.reduce([xx-x0,x1-1-xx,yy-y0,y1-1-yy])
    t=np.clip(edge/8.,0,1)
    return .5-.5*np.cos(np.pi*t)


def controls(raw,bounds):
    h,w=raw.shape[:2];yy,xx=np.indices((h,w),dtype=np.float32)
    yield dict(name='identity',kind='identity'),raw.copy(),np.zeros((h,w),np.float32),np.eye(3)
    for name,box in patches(bounds):
        weight=field((h,w),box)
        for direction in ((.8,0.),(0.,.8)):
            dx,dy=direction
            moved=cv2.remap(raw,xx-dx*weight,yy-dy*weight,cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT_101)
            yield dict(name=name+('_x' if dx else '_y'),kind='local_translation',bounds_xyxy=list(box),
                interior_translation_full_px=list(direction),edge_transition_full_px=8),moved,weight,None
    x0,y0,x1,y1=bounds;cx=(x0+x1)/2;cy=(y0+y1)/2
    for angle in (.1,-.1,.2,-.2):
        H=np.vstack([cv2.getRotationMatrix2D((cx,cy),angle,1.),[0,0,1.]])
        moved=cv2.warpPerspective(raw,H,(w,h),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT_101)
        yield dict(name='rotation_'+str(angle),kind='rotation',angle_degrees=angle,center_full_px=[cx,cy],matrix=H.tolist()),moved,None,H
    shift=np.array([[1.,0.,-cx],[0.,1.,-cy],[0.,0.,1.]])
    for axis in ('x','y'):
        for strength in (.003,.006):
            P=np.eye(3);P[2,0 if axis=='x' else 1]=strength/((x1-x0 if axis=='x' else y1-y0)/2)
            H=np.linalg.inv(shift)@P@shift
            moved=cv2.warpPerspective(raw,H,(w,h),flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT_101)
            yield dict(name='projective_'+axis+'_'+str(strength),kind='projective',normalized_strength=strength,matrix=H.tolist()),moved,None,H


def pixel_displacement(shape,H):
    yy,xx=np.indices(shape,dtype=np.float64);one=np.ones(shape)
    raw=np.stack([xx,yy,one],axis=-1);after=raw@H.T
    return np.linalg.norm(after[:,:,:2]/after[:,:,2:]-raw[:,:,:2],axis=2)


def coverage_stats(points,scale,bounds):
    xy=points.reshape(-1,2);raw=(xy+.5)*scale-.5+np.array(bounds[:2])
    x0,y0,x1,y1=bounds
    idsx=np.minimum(3,((raw[:,0]-x0)/(x1-x0)*4).astype(int));idsy=np.minimum(3,((raw[:,1]-y0)/(y1-y0)*4).astype(int))
    cells=np.bincount(idsy*4+idsx,minlength=16)
    return raw,dict(grid4x4_counts=cells.tolist(),occupied_grid4x4_cells=int(np.count_nonzero(cells)),
        maximum_grid4x4_fraction=float(cells.max()/len(points)))


def evaluate(arrays,meta):
    _,_,scale=q.r.validate_snapshot(arrays,meta)
    anchor=arrays['anchor_gray'];raw=arrays['anchor_raw_color'];bounds=meta['bounds_xyxy']
    x0,y0,x1,y1=bounds
    seeds={}
    for quality in QUALITIES:
        pts,seed=q.seed(anchor,quality)
        if quality==.02:assert np.array_equal(pts,arrays['seed_points'])
        xy,spatial=coverage_stats(pts,scale,bounds)
        seeds[quality]=(pts,seed,xy,spatial)
    out=[];images={}
    for cfg,moved,weight,H in controls(raw,bounds):
        _,gray,_=q.r.gray_for(moved,bounds)
        if cfg['kind']=='local_translation':pixel_motion=.8*weight
        else:pixel_motion=pixel_displacement(raw.shape[:2],H)
        roi_motion=pixel_motion[y0:y1,x0:x1]
        row=dict(control=cfg,known_synthetic_roi=dict(fraction_over_025=float(np.mean(roi_motion>.25)),
            maximum_full_px=float(roi_motion.max()),motion_p90_full_px=float(np.percentile(roi_motion,90))),qualities={})
        for quality in QUALITIES:
            points,seed,xy,spatial=seeds[quality]
            d=dict(seed=seed,spatial_coverage=spatial)
            if seed['seed_eligible']:
                metrics=q.r.lk(anchor,gray,points,scale)['metrics']
                local_motion=cv2.remap(pixel_motion.astype(np.float32),xy[:,0].astype(np.float32).reshape(1,-1),
                    xy[:,1].astype(np.float32).reshape(1,-1),cv2.INTER_LINEAR).ravel()
                d.update(pair=metrics,seed_locations_with_synthetic_motion_over_025=int(np.count_nonzero(local_motion>.25)),
                    seed_locations_with_synthetic_motion_over_06=int(np.count_nonzero(local_motion>.6)),
                    seed_nominal_motion_fraction_over_025=float(np.mean(local_motion>.25)))
            row['qualities'][str(quality)]=d
        base=row['qualities']['0.02'].get('pair',{});high=row['qualities']['0.12'].get('pair',{})
        row['baseline_reject_high_quality_accept']=bool(cfg['kind']!='identity' and base.get('tracks_eligible')
            and not base.get('p90_motion_gate_passed') and high.get('tracks_eligible') and high.get('p90_motion_gate_passed'))
        if cfg['kind']=='identity':
            assert all(x['pair']['p90_motion_gate_passed'] for x in row['qualities'].values())
        if row['baseline_reject_high_quality_accept'] and cfg['kind'] not in images:
            images[cfg['kind']]=(raw,moved,meta,seeds,row)
        out.append(row)
    return out,images


def figure(output,name,payload):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    raw,moved,meta,seeds,row=payload
    fig,axes=plt.subplots(1,2,figsize=(12,5),dpi=150)
    for ax,quality in zip(axes,QUALITIES):
        ax.imshow(cv2.cvtColor(raw,cv2.COLOR_BGR2RGB));xy=seeds[quality][2]
        ax.scatter(xy[:,0],xy[:,1],s=13,c='cyan',edgecolors='black',linewidth=.3)
        box=row['control'].get('bounds_xyxy',meta['bounds_xyxy']);x0,y0,x1,y1=box
        ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,fill=False,color='yellow',linewidth=1.3))
        d=row['qualities'][str(quality)];ax.set_title('quality %.2f: %d points, %d quadrants\nSynthetic pair P90 %.3f px; %s' % (
            quality,d['seed']['seed_points_count'],d['pair']['covered_quadrants'],d['pair']['motion_p90_px'],
            'PAIR PASS' if d['pair']['p90_motion_gate_passed'] else 'MOTION REJECT'))
    fig.suptitle(name+' | '+meta['region']+' | '+row['control']['name']+'\nSynthetic control only; same original gates, no production permission',fontsize=10)
    fig.tight_layout();fig.savefig(output/(name+'.png'));plt.close(fig)


def main():
    cv2.setNumThreads(1)
    capture=ROOT/'visual_snapshots';summary,entries=q.r.sealed_entries(capture)
    frozen={str(capture/'summary.json'):sha(capture/'summary.json'),str(QUALITY):sha(QUALITY),
        str(Path(__file__).resolve()):sha(Path(__file__).resolve()),str(q.REPLAY_PATH):sha(q.REPLAY_PATH),str(q.r.VISUAL_PATH):sha(q.r.VISUAL_PATH)}
    seen={};rows=[];pictures={}
    for entry in entries:
        path=capture/entry['name'];assert sha(path)==entry['sha256'];frozen[str(path)]=entry['sha256']
        with np.load(path,allow_pickle=False)as a:arrays={k:a[k].copy() for k in a.files}
        meta=json.loads(arrays['metadata_json'].tobytes())
        q.r.validate_snapshot(arrays,meta)
        assert meta['source']==entry['source'] and meta['source_implementation']==summary['source_implementation']
        original,_=q.seed(arrays['anchor_gray'],.02);assert np.array_equal(original,arrays['seed_points'])
        key=meta['region']+':'+hashlib.sha256(arrays['anchor_raw_color'].tobytes()).hexdigest()
        if key in seen:seen[key].append(entry['name']);continue
        seen[key]=[entry['name']]
        cases,images=evaluate(arrays,meta)
        rows.append(dict(file=entry['name'],region=meta['region'],anchor_source=meta['pre_observe']['anchor_source'],anchor_group_key=key,controls=cases))
        for kind,payload in images.items():
            if kind not in pictures:pictures[kind]=(entry['name'].replace('.npz','')+'_'+kind,payload)
    regressions=[dict(file=row['file'],region=row['region'],**case) for row in rows for case in row['controls'] if case['baseline_reject_high_quality_accept']]
    result=dict(schema=1,diagnostic_only=True,motion_authorized=False,production_modified=False,
        input_sha256=frozen,opencv_version=cv2.__version__,qualities=list(QUALITIES),source_snapshots=32,
        unique_anchor_region_groups=len(rows),duplicate_groups=seen,rows=rows,
        baseline_reject_high_quality_accept_count=len(regressions),
        regression_counts=dict(Counter(x['region']+'/'+x['control']['kind'] for x in regressions)),regressions=regressions,
        methodology='Fixed 8 geometric masks x 2 directions at0.8px; fixed rotations +/-0.1,+/-0.2deg; fixed centered projective strengths0.003/0.006 onx/y; identity. No controls selected by observed flow; repeated identical anchors deduplicated by full raw RGB SHA+region.',
        limitations=['Synthetic controls are not physical observations or calibrated movement.',
            'Local controls use smooth backward remap fields; interior displacement0.8px, 8px boundary taper. Reported nominal seed movement samples that field, not a measured forward correspondence.',
            'Four-quadrant minimum coverage does not ensure uniform sampling within each quadrant or weak-texture moving-region coverage.',
            'Every original point-count, tracked fraction, quadrant, and0.25px gate remains unchanged. Pair outcomes do not reconstruct the original0.35s temporal window.',
            'Selected capped recorded scenes do not estimate deployment miss rates; a counterexample establishes risk, not its prevalence.'])
    assert all(sha(Path(p))==v for p,v in frozen.items())
    assert Path('/media/zhuyupei/SanStand').is_mount()
    output=ROOT/'quality_seed_local_motion_risk_v1';output.mkdir(exist_ok=False)
    for name,payload in list(pictures.values())[:2]:figure(output,name,payload)
    with (output/'report.json').open('x')as f:json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(dict(report=str(output/'report.json'),sha256=sha(output/'report.json'),unique_anchors=len(rows),regressions=len(regressions),by_kind=result['regression_counts'])))


if __name__=='__main__':main()
