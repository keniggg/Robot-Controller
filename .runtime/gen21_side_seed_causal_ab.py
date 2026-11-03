"""Causal offline GEN21 current side-evidence denominator counterfactual."""
from pathlib import Path
from dataclasses import fields
import inspect,hashlib,json,sys,time
import numpy as np
import cv2
WORK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORK/'src/alicia_grasp_modes/src'))
from alicia_grasp_modes import tabletop,rgb_instance,measured_mask_seeds,observation_arrival_confirmation as arrival
from alicia_grasp_modes.rgb_observation_handoff import _appearance
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen21_analysis')
assert Path('/media/zhuyupei/SanStand').is_mount()
inp=OUT/'replay_inputs';manifest=json.loads((inp/'manifest.json').read_text())
params=json.loads((inp/'unknown_perception_configuration.json').read_text())['parameters']
cfg=tabletop.Config(**{k:v for k,v in params.items() if k in {f.name for f in fields(tabletop.Config)}})
entries=manifest['files'];frames={r['source_stamp_ns']:dict(np.load(r['path'])) for r in entries}
def arrays(f):return f['depth_raw'].astype(np.float32)*float(f['depth_scale']),f['K'],f['T_base_camera']
a=frames[next(r['source_stamp_ns'] for r in entries if r['role']=='anchor')]
tracker=tabletop.TargetTracker(cfg);r=tabletop.segment_with_support_fallback(*arrays(a),cfg)
anchor,_=tracker.choose(r,a['depth_raw'].shape)
f=frames[next(r['source_stamp_ns'] for r in entries if r['role']=='last_pre_motion')]
r=tabletop.segment_with_support_fallback(*arrays(f),cfg,anchor=anchor,last=anchor,
    reference_plane_base=tracker.plane_base,prefer_current_support=True)
last,_=tracker.choose(r,f['depth_raw'].shape)
original=measured_mask_seeds._side_seed_colour_evidence
source=inspect.getsource(original)
old='np.count_nonzero(ambiguous_side) < max(25, .1 * np.count_nonzero(core))'
assert source.count(old)==1
changed=source.replace(old,'np.count_nonzero(ambiguous_side) < 25')
ns=measured_mask_seeds.__dict__.copy();exec(compile(changed,'<gen21_side_seed_counterfactual>','exec'),ns)
counterfactual=ns['_side_seed_colour_evidence'];baseline_refine=arrival.refine_candidate
reports=[]
for label,side in [('production',original),('absolute_minimum_only',counterfactual)]:
    measured_mask_seeds._side_seed_colour_evidence=side
    reference=rgb_instance.refine_candidate(f['color'],*arrays(f),last,cfg,rgb_valid=f['rgb_valid'])
    recorded=f[next(k for k in f if 'mask' in k)]
    hist,contour=_appearance(f['color'],reference.mask)
    cv2.imwrite(str(OUT/('side_seed_'+label+'_reference775_mask.png')),reference.mask)
    helper=arrival.ObservationArrivalConfirmation(cfg,anchor,last,tracker.plane_base,hist,contour,.5)
    observed=[]
    for row in entries:
        if row['role'] not in ('arrival','first_one_of_three','recorded_arrival_not_online_evaluated'):continue
        frame=frames[row['source_stamp_ns']];evidence={};started=time.monotonic()
        result=tabletop.segment_with_support_fallback(*arrays(frame),cfg,anchor=anchor,last=last,
            reference_plane_base=tracker.plane_base,prefer_current_support=True)
        def refine(*args,**kwargs):
            candidate=baseline_refine(*args,**kwargs);h,c=_appearance(frame['color'],candidate.mask)
            evidence.update(colour_distance=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(h*hist))))),
                contour_distance=float(cv2.matchShapes(contour,c,cv2.CONTOURS_MATCH_I1,0.)),
                mask_sha256=hashlib.sha256(candidate.mask.tobytes()).hexdigest(),metrics=candidate.metrics)
            return candidate
        arrival.refine_candidate=refine
        try:
            confirmed,candidate,staged,info=helper.observe(result,frame['color'],*arrays(frame),row['source_stamp_ns'],rgb_valid=frame['rgb_valid'])
            accepted=True;reason=info['reason']
        except Exception as exc:accepted=False;confirmed=False;reason=str(exc)
        finally:arrival.refine_candidate=baseline_refine
        observed.append(dict(source_stamp_ns=row['source_stamp_ns'],online_evaluated=row['role']!='recorded_arrival_not_online_evaluated',
            identity_passed=accepted,confirmed=confirmed,reason=reason,seconds=time.monotonic()-started,**evidence))
    reports.append(dict(variant=label,reference_mask_pixels=int(np.count_nonzero(reference.mask)),
        reference_equals_recorded=bool(np.array_equal(reference.mask,recorded)),reference_metrics=reference.metrics,
        reference_mask_sha256=hashlib.sha256(reference.mask.tobytes()).hexdigest(),observations=observed))
measured_mask_seeds._side_seed_colour_evidence=original
def default(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)
report=dict(production_modified=False,no_ros=True,manifest_sha256=hashlib.sha256((inp/'manifest.json').read_bytes()).hexdigest(),
    source_sha256={str(Path(m.__file__)):hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for m in (tabletop,rgb_instance,measured_mask_seeds,arrival)},
    counterfactual_diff='Only max(25, 0.1 * all_core_pixels) -> 25; every local evidence predicate unchanged.',reports=reports,
    limitations=['Sparse saved sources: not a reproduction of callback scheduling or capture-deadline completion.',
        'Reference re-created from its own true current RGB-D; no prior mask/image substituted.'])
p=OUT/'side_seed_causal_ab_r1.json';p.write_text(json.dumps(report,default=default,indent=2)+'\n')
print(json.dumps({'report':str(p),'variants':[dict(variant=x['variant'],reference_pixels=x['reference_mask_pixels'],reference_exact=x['reference_equals_recorded'],
    demoted=x['reference_metrics']['support_colour_demoted_side_pixels'],passed=sum(r['identity_passed'] for r in x['observations']),
    rows=[{k:v for k,v in r.items() if k not in ('metrics','mask_sha256')} for r in x['observations']]) for x in reports]},default=default))
