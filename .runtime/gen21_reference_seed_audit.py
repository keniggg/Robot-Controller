"""Exact ordinary RGB refinement seed-label audit, sealed GEN21 sources only."""
from pathlib import Path
from dataclasses import fields
import hashlib,json,sys,inspect
import numpy as np
WORK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORK/'src/alicia_grasp_modes/src'))
from alicia_grasp_modes import tabletop,rgb_instance,measured_mask_seeds
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen21_analysis')
assert Path('/media/zhuyupei/SanStand').is_mount()
manifest=json.loads((OUT/'replay_inputs/manifest.json').read_text())
params=json.loads((OUT/'replay_inputs/unknown_perception_configuration.json').read_text())['parameters']
cfg=tabletop.Config(**{k:v for k,v in params.items() if k in {f.name for f in fields(tabletop.Config)}})
rows=manifest['files'];a=next(r for r in rows if r['role']=='anchor');f=dict(np.load(a['path']))
def arrays(f):
    return f['depth_raw'].astype(np.float32)*float(f['depth_scale']),f['K'],f['T_base_camera']
tracker=tabletop.TargetTracker(cfg);result=tabletop.segment_with_support_fallback(*arrays(f),cfg)
anchor,_=tracker.choose(result,f['depth_raw'].shape)
previous=next(r for r in rows if r['role']=='previous_ready_reference_probe');prev=dict(np.load(previous['path']))
prev_mask=prev[next(k for k in prev if 'mask' in k)]>0
reports=[]
side_source=inspect.getsource(measured_mask_seeds._side_seed_colour_evidence)
needle='    if np.count_nonzero(ambiguous_side) < max(25, .1 * np.count_nonzero(core)):'
assert side_source.count(needle)==1
side_source=side_source.replace(needle,"    report.update(diagnostic_candidate_demotions=int(np.count_nonzero(ambiguous_side)), diagnostic_minimum_demotions=max(25, .1 * np.count_nonzero(core)), diagnostic_input_core_pixels=int(np.count_nonzero(core)), diagnostic_side_pixels=int(np.count_nonzero(side)))\n"+needle)
namespace=measured_mask_seeds.__dict__.copy()
exec(compile(side_source,'<gen21_readonly_side_seed_count>','exec'),namespace)
measured_mask_seeds._side_seed_colour_evidence=namespace['_side_seed_colour_evidence']
original=rgb_instance.confidence_labels
for row in [previous,next(r for r in rows if r['role']=='last_pre_motion')]:
    f=dict(np.load(row['path']));depth,K,T=arrays(f);recorded=f[next(k for k in f if 'mask' in k)]>0
    result=tabletop.segment_with_support_fallback(depth,K,T,cfg,anchor=anchor,last=tracker.last,
        reference_plane_base=tracker.plane_base,prefer_current_support=True)
    raw,_=tracker.choose(result,depth.shape)
    measured={}
    yy,xx=np.nonzero(raw.mask);margin=max(8,int(round(16*min(depth.shape[1]/640.,depth.shape[0]/480.))))
    x0,y0=max(0,int(xx.min())-margin),max(0,int(yy.min())-margin)
    def labels(*args,**kwargs):
        core,support,evidence=original(*args,**kwargs)
        measured.update(evidence)
        full=np.zeros(depth.shape,bool);full[y0:y0+core.shape[0],x0:x0+core.shape[1]]=core
        measured['core']=full
        return core,support,evidence
    rgb_instance.confidence_labels=labels
    refined=rgb_instance.refine_candidate(f['color'],depth,K,T,raw,cfg,rgb_valid=f['rgb_valid'])
    rgb_instance.confidence_labels=original
    core=measured.pop('core');mask=refined.mask>0
    plane=T.T@np.asarray(raw.metrics['support_plane_base']);points=tabletop._project(depth,K)
    heights=points@plane[:3]+plane[3]
    valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)&np.all(np.isfinite(points),axis=-1)
    groups={}
    for name,selected in [('mask',mask),('added_vs_previous_recorded_mask',mask&~prev_mask),('core',core),('core_added_vs_previous',core&~prev_mask)]:
        h=heights[selected&valid];ys,xs=np.nonzero(selected)
        groups[name]=dict(pixels=int(selected.sum()),valid_depth_pixels=int(np.count_nonzero(selected&valid)),
            missing_depth_pixels=int(np.count_nonzero(selected&~valid)),
            within_support_2mm=int(np.count_nonzero(selected&valid&(np.abs(heights)<=cfg.support_threshold_m))),
            positive_2_to_4mm=int(np.count_nonzero(selected&valid&(heights>cfg.support_threshold_m)&(heights<cfg.foreground_min_m))),
            eligible_foreground=int(np.count_nonzero(selected&valid&(heights>=cfg.foreground_min_m)&(heights<=cfg.foreground_max_m))),
            height_percentiles_m=np.percentile(h,[0,5,50,95,100]).tolist() if len(h) else [],
            bbox_xyxy=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)] if len(xs) else None)
    reports.append(dict(source_ns=row['source_stamp_ns'],mask_exact_recorded_match=bool(np.array_equal(recorded,mask)),
        xor_pixels=int(np.count_nonzero(recorded^mask)),raw_metrics=raw.metrics,seed_evidence=measured,refined_metrics=refined.metrics,groups=groups))
def default(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)
report=dict(production_modified=False,ros_calls=False,source_manifest_sha256=hashlib.sha256((OUT/'replay_inputs/manifest.json').read_bytes()).hexdigest(),reports=reports)
p=OUT/'reference_seed_label_audit_r2.json';p.write_text(json.dumps(report,default=default,indent=2)+'\n')
print(json.dumps({'report':str(p),'summaries':[dict(source_ns=r['source_ns'],exact=r['mask_exact_recorded_match'],groups=r['groups'],seed_evidence=r['seed_evidence']) for r in reports]},default=default))
