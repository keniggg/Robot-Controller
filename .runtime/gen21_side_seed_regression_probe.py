"""Offline component layout of current side contradiction; no rule change."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
import cv2
W=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_grasp_modes/tests'),str(W/'src/alicia_flexible_grasp_supervisor/src')]
from alicia_grasp_modes import measured_mask_seeds as seeds,rgb_instance,tabletop
from test_rgb_instance import make_candidate
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen21_analysis')
assert Path('/media/zhuyupei/SanStand').is_mount()
old=seeds._side_seed_colour_evidence;captured=[]
def observed(bgr,core,support,heights,upper,cosine):
    result,evidence=old(bgr,core,support,heights,upper,cosine)
    demoted=core&~result
    n,labels,stats,centres=cv2.connectedComponentsWithStats(demoted.astype(np.uint8),8)
    captured.append(dict(core=int(core.sum()),demoted=int(demoted.sum()),components=[dict(pixels=int(stats[k,4]),bbox_xywh=stats[k,:4].tolist(),
        median_bgr=np.median(bgr[labels==k],axis=0).tolist(),median_height=float(np.median(heights[labels==k])),
        median_cosine=float(np.median(cosine[labels==k]))) for k in sorted(range(1,n),key=lambda k:int(stats[k,4]),reverse=True)]))
    return result,evidence
seeds._side_seed_colour_evidence=observed
data=dict(np.load(W/'src/alicia_grasp_modes/tests/fixtures/shadow_jump_20260923.npz'))
reports=[]
for i in (0,1,2):
    captured.clear();candidate=make_candidate(data,i)
    r=rgb_instance.refine_candidate(data['bgr'][i],data['depth'][i].astype(np.float32)*float(data['depth_scale']),data['intrinsics'],data['T'][i],candidate,tabletop.Config())
    reports.append(dict(kind='shadow_jump',index=i,capture=list(captured),retained_dark=int(np.count_nonzero(r.mask[285:294,313:366]))))
manifest=json.loads((OUT/'replay_inputs/manifest.json').read_text())
tracker=tabletop.TargetTracker(tabletop.Config())
for role in ('anchor','previous_ready_reference_probe','last_pre_motion'):
    row=next(r for r in manifest['files'] if r['role']==role);f=dict(np.load(row['path']));depth=f['depth_raw'].astype(np.float32)*float(f['depth_scale'])
    kwargs={} if tracker.anchor is None else dict(anchor=tracker.anchor,last=tracker.last,reference_plane_base=tracker.plane_base,prefer_current_support=True)
    segmented=tabletop.segment_with_support_fallback(depth,f['K'],f['T_base_camera'],tracker.config,**kwargs)
    raw,_=tracker.choose(segmented,depth.shape);captured.clear()
    r=rgb_instance.refine_candidate(f['color'],depth,f['K'],f['T_base_camera'],raw,tracker.config,rgb_valid=f['rgb_valid'])
    reports.append(dict(kind=role,source_ns=row['source_stamp_ns'],capture=list(captured),mask_pixels=int(np.count_nonzero(r.mask))))
seeds._side_seed_colour_evidence=old
p=OUT/'side_seed_regression_component_probe.json';p.write_text(json.dumps(dict(source_sha256=hashlib.sha256(Path(seeds.__file__).read_bytes()).hexdigest(),reports=reports),indent=2)+'\n')
print(json.dumps(reports))
