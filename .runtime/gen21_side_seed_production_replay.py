"""New production imports: exact three reference frames and saved arrival frames."""
from pathlib import Path
from dataclasses import fields
import json,hashlib,sys,time
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
modules=[tabletop,rgb_instance,measured_mask_seeds,arrival]
def hashes():return {str(Path(m.__file__)):hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for m in modules}
stem=sys.argv[1] if len(sys.argv)>1 else 'side_seed_production_replay'
before=hashes();tracker=tabletop.TargetTracker(cfg);references=[]
def arrays(f):return f['depth_raw'].astype(np.float32)*float(f['depth_scale']),f['K'],f['T_base_camera']
for role in ('anchor','previous_ready_reference_probe','last_pre_motion'):
    row=next(r for r in manifest['files'] if r['role']==role);f=dict(np.load(row['path']))
    kwargs={} if tracker.anchor is None else dict(anchor=tracker.anchor,last=tracker.last,reference_plane_base=tracker.plane_base,prefer_current_support=True)
    result=tabletop.segment_with_support_fallback(*arrays(f),cfg,**kwargs)
    raw,reason=tracker.choose(result,f['depth_raw'].shape);assert raw is not None
    candidate=rgb_instance.refine_candidate(f['color'],*arrays(f),raw,cfg,rgb_valid=f['rgb_valid'])
    assert candidate.quality>=.5
    recorded=f[next(k for k in f if 'mask' in k)]
    references.append(dict(role=role,source_ns=row['source_stamp_ns'],raw_tracking=reason,
        recorded_pixels=int(np.count_nonzero(recorded)),new_pixels=int(np.count_nonzero(candidate.mask)),
        xor_pixels=int(np.count_nonzero((recorded>0)^(candidate.mask>0))),quality=candidate.quality,
        raw_position_base=raw.position_base.tolist(),plane=tracker.plane_base.tolist(),
        current_support_audit=result.metrics,metrics=candidate.metrics,
        source_npz_sha256=row['sha256'],mask_sha256=hashlib.sha256(candidate.mask.tobytes()).hexdigest()))
    cv2.imwrite(str(OUT/(stem+'_'+str(row['source_stamp_ns'])+'_mask.png')),candidate.mask)
hist,contour=_appearance(f['color'],candidate.mask)
helper=arrival.ObservationArrivalConfirmation(cfg,tracker.anchor,tracker.last,tracker.plane_base,hist,contour,.5)
observations=[]
for row in manifest['files']:
    if row['role'] not in ('arrival','first_one_of_three','recorded_arrival_not_online_evaluated'):continue
    f=dict(np.load(row['path']));started=time.monotonic()
    r=tabletop.segment_with_support_fallback(*arrays(f),cfg,anchor=tracker.anchor,last=tracker.last,
        reference_plane_base=tracker.plane_base,prefer_current_support=True)
    info={}
    try:
        confirmed,c,staged,info=helper.observe(r,f['color'],*arrays(f),row['source_stamp_ns'],rgb_valid=f['rgb_valid'])
        passed=True;reason=info['reason']
    except ValueError as exc:passed=False;confirmed=False;reason=str(exc)
    observations.append(dict(source_ns=row['source_stamp_ns'],online_evaluated=row['role']!='recorded_arrival_not_online_evaluated',
        identity_passed=passed,confirmed=confirmed,reason=reason,evidence=info,seconds=time.monotonic()-started))
after=hashes()
def default(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)
report=dict(source_sha256=before,source_after_sha256=after,source_unchanged=before==after,
    no_ros_calls=True,source_frames_unchanged=True,recording=manifest['record'],
    manifest_sha256=hashlib.sha256((inp/'manifest.json').read_bytes()).hexdigest(),references=references,
    observations=observations,identity_passed=sum(r['identity_passed'] for r in observations),
    limitations=['Actual production image/geometry path; no substitute old mask or injected successful identity.',
        'Sparse saved pairs do not reproduce real-time callback scheduling or guarantee capture deadline.',
        'No hardware trial or physical grasp success is claimed.'])
p=OUT/(stem+'.json');p.write_text(json.dumps(report,default=default,indent=2)+'\n')
print(json.dumps({'path':str(p),'source_unchanged':report['source_unchanged'],
    'references':[{k:v for k,v in r.items() if k in ('role','source_ns','recorded_pixels','new_pixels','xor_pixels','quality')} for r in references],
    'passed':report['identity_passed'],'total':len(observations),'online_exact':[dict(source_ns=r['source_ns'],passed=r['identity_passed']) for r in observations if r['online_evaluated']]}))
