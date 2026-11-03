"""Offline sealed GEN22 exact frames; in-memory crop-only counterfactual."""
from pathlib import Path
from dataclasses import fields
from copy import deepcopy
import hashlib
import inspect
import json
import sys
import time
import numpy as np
import cv2

WORK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORK/'src/alicia_grasp_modes/src'))
from alicia_grasp_modes import tabletop, rgb_instance, observation_arrival_confirmation as arrival
from alicia_grasp_modes.rgb_observation_handoff import _appearance

OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen22_analysis')
assert Path('/media/zhuyupei/SanStand').is_mount()
inp=OUT/'replay_inputs'
manifest=json.loads((inp/'manifest.json').read_text())
config_values=json.loads((inp/'unknown_perception_configuration.json').read_text())['parameters']
cfg=tabletop.Config(**{k:v for k,v in config_values.items() if k in {f.name for f in fields(tabletop.Config)}})
entries=manifest['files']
frames={row['source_stamp_ns']:dict(np.load(row['path'])) for row in entries}
anchor_row=next(row for row in entries if row['role']=='anchor')
last_row=next(row for row in entries if row['role']=='last_pre_motion')
def arrays(frame):
    k=np.asarray(frame['K'])
    intrinsics=[k[0,0],k[1,1],k[0,2],k[1,2]] if k.shape==(3,3) else k
    return frame['depth_raw'].astype(np.float32)*float(frame['depth_scale']),intrinsics,frame['T_base_camera']
anchor_frame=frames[anchor_row['source_stamp_ns']]
tracker=tabletop.TargetTracker(cfg)
r=tabletop.segment_with_support_fallback(*arrays(anchor_frame),cfg)
anchor,reason=tracker.choose(r,anchor_frame['depth_raw'].shape)
status=json.loads((inp/'anchor_status.json').read_text())
if 'message' in status:status=status['message']
anchor_error=float(np.max(np.abs(anchor.position_base-np.array(status['anchor_position_base']))))
plane_error=float(np.max(np.abs(tracker.plane_base-np.array(status['reference_support_plane_base']))))
assert anchor_error<1e-10 and plane_error<1e-10,(anchor_error,plane_error)
last_frame=frames[last_row['source_stamp_ns']]
r=tabletop.segment_with_support_fallback(*arrays(last_frame),cfg,anchor=anchor,last=anchor,
    reference_plane_base=tracker.plane_base,prefer_current_support=True)
last_status=json.loads((inp/'last_pre_motion_status.json').read_text())['message']
# GEN22 pre-second-view READY is an actual fragment-group final_raw. Reconstruct
# its exact current measured mask as the production group final_raw does, from
# recorded refined mask + unaltered depth; do not choose an arbitrary fragment.
assert last_status['geometry_metrics']['observation_fragment_group']['final_original_geometry_passed']
depth,K,T=arrays(last_frame);points=tabletop._project(depth,K)
last_plane=np.asarray(last_status['geometry_metrics']['support_plane_base'])
pc=T.T@last_plane;height=points@pc[:3]+pc[3]
measured=(last_frame['recorded_refined_mask']>0)&np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)&(height>=cfg.foreground_min_m)&(height<=cfg.foreground_max_m)
xyz=points[measured].astype(float);p_cam=np.median(xyz,axis=0);p_base=(T@np.r_[p_cam,1.])[:3]
centered=xyz-np.mean(xyz,axis=0);_,_,vh=np.linalg.svd(centered,full_matrices=False)
extent=np.sort(np.percentile(centered@vh.T,98,axis=0)-np.percentile(centered@vh.T,2,axis=0))[::-1]
yy,xx=np.nonzero(measured)
last=tabletop.Candidate(measured.astype(np.uint8)*255,np.array([xx.mean(),yy.mean()]),(int(xx.min()),int(yy.min()),int(xx.max()-xx.min()+1),int(yy.max()-yy.min()+1)),p_cam,p_base,extent,float(p_cam[0]),float(last_status['geometry_quality']),deepcopy(last_status['geometry_metrics']),tabletop._measured_support_footprint(xyz@T[:3,:3].T+T[:3,3],last_plane))
last_position_error=float(np.max(abs(p_base-np.asarray(last_status['position_base']))))
last_extent_error=float(np.max(abs(extent-np.asarray(last_status['geometry_metrics']['observed_surface_extent_m']))))
assert last_position_error<1e-10 and last_extent_error<1e-10
assert len(xyz)==last_status['geometry_metrics']['observation_fragment_group']['final_measured_points']
mask_key=next(k for k in last_frame if 'mask' in k)
hist,contour=_appearance(last_frame['color'],last_frame[mask_key])
original_instance=rgb_instance._instance_mask_from_seed
results=[]
baseline_refine=arrival.refine_candidate
for variant,instance in [('production',original_instance)]:
    rgb_instance._instance_mask_from_seed=instance
    helper=arrival.ObservationArrivalConfirmation(cfg,anchor,last,tracker.plane_base,hist,contour,.5)
    for row in entries:
        if row['role'] not in ('arrival','first_one_of_three','recorded_arrival_not_online_evaluated'):continue
        ns=row['source_stamp_ns'];frame=frames[ns];measured={}
        try:
            t=time.monotonic()
            r=tabletop.segment_with_support_fallback(*arrays(frame),cfg,anchor=anchor,last=last,
                reference_plane_base=tracker.plane_base,prefer_current_support=True)
            def refine(*a,**kw):
                candidate=baseline_refine(*a,**kw)
                h,c=_appearance(frame['color'],candidate.mask)
                measured.update(colour_distance=float(np.sqrt(max(0.,1.-np.sum(np.sqrt(h*hist))))),
                    contour_distance=float(cv2.matchShapes(contour,c,cv2.CONTOURS_MATCH_I1,0.)),
                    rgb_pixels=int(np.count_nonzero(candidate.mask)),metrics=candidate.metrics,
                    mask_sha256=hashlib.sha256(candidate.mask.tobytes()).hexdigest())
                cv2.imwrite(str(OUT/('crop_ab_'+variant+'_'+str(ns)+'_mask.png')),candidate.mask)
                return candidate
            arrival.refine_candidate=refine
            confirmed,candidate,staged,evidence=helper.observe(r,frame['color'],*arrays(frame),ns,rgb_valid=frame['rgb_valid'])
            reason=evidence['reason'];passed=True
        except Exception as exc:
            reason=str(exc);confirmed=False;passed=False
        finally:arrival.refine_candidate=baseline_refine
        results.append(dict(variant=variant,source_ns=ns,identity_passed=passed,confirmed=confirmed,
            reason=reason,seconds=time.monotonic()-t,raw_candidate_count=len(r.candidates),
            raw_matches=len(tabletop._matching_candidates(r.candidates,anchor,last,cfg)),**measured))
rgb_instance._instance_mask_from_seed=original_instance
def default(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError(type(value).__name__)
report=dict(no_ros_calls=True,production_modified=False,manifest_sha256=hashlib.sha256((inp/'manifest.json').read_bytes()).hexdigest(),
    source_sha256={str(Path(m.__file__)):hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for m in (tabletop,rgb_instance,arrival)},
    reference_source_ns=last_row['source_stamp_ns'],reference_mask_key=mask_key,reference_mask_pixels=int(np.count_nonzero(last_frame[mask_key])),
    anchor_error=anchor_error,plane_error=plane_error,last_position_error=last_position_error,last_extent_error=last_extent_error,last_measured_points=len(xyz),last_reconstruction='Production fragment-group final_raw from recorded RGB mask plus actual current depth and recorded support plane; position/extent/count checked against READY status.',config=config_values,results=results,
    limitations=['Reference is last producer READY before second executing transition. Private dictionary is not separately recorded.','All eleven saved second-arrival sources are adjacent real frames, not exact online assessed source frames. This is geometric/image replay, not exact failed-frame, real-time or hardware validation.'])
path=OUT/'arrival_identity_replay_r1.json';path.write_text(json.dumps(report,default=default,indent=2)+'\n')
print(json.dumps({'report':str(path),'reference_source_ns':report['reference_source_ns'],
    'results':[{k:v for k,v in row.items() if k not in ('metrics',)} for row in results]},default=default))
