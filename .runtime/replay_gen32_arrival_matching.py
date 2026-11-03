"""Actual segmentation/arrival on selected sealed exact sources; no authority."""
from pathlib import Path
from dataclasses import asdict
import json,hashlib,time,sys
import numpy as np,cv2
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes import rgb_instance as rgbmod
from alicia_grasp_modes import arrival_fragment_group as groupmod
from alicia_grasp_modes.observation_arrival_confirmation import ObservationArrivalConfirmation
from alicia_grasp_modes.rgb_observation_handoff import _appearance,_mask
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen32_handoff_analysis')
meta=json.loads((D/'matching_source_images/manifest.json').read_text());out=D/'arrival_matching_replay';out.mkdir(exist_ok=True)
cfg=table.Config();cv2.setNumThreads(1)
def frame(row):
    a=np.load(row['npz']);camera=row['camera_metadata']['message']['parameters'];K=[camera[k] for k in ('fx','fy','cx','cy')]
    dep=np.asarray(a['depth_raw'],dtype=np.float32)*camera['depth_scale']
    return a,a['rgb'],dep,a['T_base_camera'],K,rectification_rgb_valid_mask(camera,dep.shape)
ar,lr=meta['sources'][:2];a,rgb,dep,T,K,valid=frame(ar)
seg=table.segment_with_support_fallback(dep,K,T,cfg);tracker=table.TargetTracker(cfg);anchor,reason=tracker.choose(seg,dep.shape);assert anchor is not None,reason
status=ar['detector_status']['message'];plane=np.asarray(status['reference_support_plane_base']);checks={}
for key,x,y in [('anchor_position',anchor.position_base,status['anchor_position_base']),('anchor_extent',anchor.extent_m,status['anchor_extent_m']),('anchor_plane',seg.plane_base,plane),('anchor_footprint',anchor.support_footprint_base,status['reference_support_footprint_base'])]:
    checks[key]=float(np.max(np.abs(np.asarray(x)-np.asarray(y))));assert checks[key]<1e-10,(key,checks[key])
a,rgb,dep,T,K,valid=frame(lr);seg=table.segment_with_support_fallback(dep,K,T,cfg,anchor,anchor,plane,prefer_current_support=True)
matches=table._matching_candidates(seg.candidates,anchor,anchor,cfg);assert len(matches)==1,len(matches)
last=matches[0][1];refined=rgbmod.refine_candidate(rgb,dep,K,T,last,cfg,rgb_valid=valid)
checks['last_mask_bitexact']=bool(np.array_equal(refined.mask,a['refined_mask']));assert checks['last_mask_bitexact']
hist,contour=_appearance(rgb,_mask(refined.mask,rgb.shape[:2]));helper=ObservationArrivalConfirmation(cfg,anchor,last,plane,hist,contour,.5)
files=[Path(table.__file__),Path(rgbmod.__file__),Path(groupmod.__file__),Path(sys.modules[ObservationArrivalConfirmation.__module__].__file__)];hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files};rows=[]
for row in meta['sources'][2:]:
    a,rgb,dep,T,K,valid=frame(row);ns=row['source_stamp_ns'];started=time.perf_counter();r=dict(source_stamp_ns=ns,exact_online_failure=True,online_reason=row['detector_status']['message']['observation_measurement']['reason'])
    seg=table.segment_with_support_fallback(dep,K,T,cfg,anchor,last,plane,prefer_current_support=True)
    matches=table._matching_candidates(seg.candidates,anchor,last,cfg);ids={id(c) for _,c in matches};r.update(candidate_count=len(seg.candidates),matching_count=len(matches),support_metrics=seg.metrics,candidates=[])
    pts=table._project(dep,K);spatial=rgbmod._observation_spatial_support(pts,T,K,helper.spatial_reference,cfg);overlay=rgb.copy()
    for i,c in enumerate(seg.candidates):
        ratios=c.extent_m[:2]/np.maximum(anchor.extent_m[:2],.003);distance=float(np.linalg.norm(c.position_base-anchor.position_base));step=float(np.linalg.norm(c.position_base-last.position_base));mask=c.mask>0
        item=dict(index=i,matched=id(c) in ids,bbox_xywh=list(c.bbox),pixels=int(mask.sum()),position_base=c.position_base.tolist(),extent_m=c.extent_m.tolist(),anchor_distance_m=distance,last_step_m=step,xy_extent_ratios=ratios.tolist(),gate_anchor_position=bool(distance<=cfg.association_distance_m),gate_last_position=bool(step<=cfg.association_distance_m),gate_xy_extent=bool(np.all((ratios>=.5)&(ratios<=2.))),quality=c.quality,metrics=c.metrics,support_footprint_base=c.support_footprint_base.tolist(),current_pixels_inside_frozen_footprint=int(np.count_nonzero(mask&spatial['domain_mask'])),current_pixels_outside_frozen_footprint=int(np.count_nonzero(mask&~spatial['domain_mask'])))
        r['candidates'].append(item);cv2.imwrite(str(out/(str(ns)+'_candidate'+str(i)+'.png')),c.mask)
        color=(0,255,0) if item['matched'] else (0,128,255);cv2.drawContours(overlay,cv2.findContours(c.mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2],-1,color,1);cv2.putText(overlay,str(i),tuple(np.asarray(c.center_uv,dtype=int)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1)
    cv2.imwrite(str(out/(str(ns)+'_overlay.png')),overlay)
    calls=[]
    def trace(f,event,arg):
        if event=='call' and f.f_code.co_name=='recover_arrival_fragment_group':calls.append(f.f_code.co_name)
    sys.setprofile(trace)
    try:
        ok,candidate,staged,evidence=helper.observe(seg,rgb,dep,K,T,ns,rgb_valid=valid);r.update(passed=True,confirmed=ok,evidence=evidence)
    except ValueError as exc:r.update(passed=False,reason=str(exc))
    finally:sys.setprofile(None)
    r.update(fragment_group_calls=len(calls),wall_sec=time.perf_counter()-started);rows.append(r)
    print(json.dumps({k:r[k] for k in ['source_stamp_ns','candidate_count','matching_count','fragment_group_calls','passed','reason','wall_sec']},ensure_ascii=False),flush=True)
assert hashes=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
report=dict(scope='EXACT_RECORDED_PIXEL_OFFLINE_DIAGNOSTIC_NO_AUTHORITY',input_manifest_sha256=hashlib.sha256((D/'matching_source_images/manifest.json').read_bytes()).hexdigest(),record=meta['record'],source_sha256=hashes,config=asdict(cfg),reference_checks=checks,original_anchor_source_ns=ar['source_stamp_ns'],premotion_source_ns=lr['source_stamp_ns'],rows=rows,limitations=['Three of eighteen online rejected sources, selected first/middle/last of four exact available saved sources; no absent frame fabricated.','Actual production segmentation, original matcher and arrival helper, no altered thresholds or recovery eligibility.','Premotion reference rebuilt from saved exact source and original mask checked bitexact; original callback clocks/physical execution are not replayed.'])
(out/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print('RESULT',out/'result.json')
