#!/usr/bin/env python3
"""Spatial flow/photometry of retained rectified RGB; NOT SDK gate replay."""
from collections import Counter
import json
from pathlib import Path
import shutil
import cv2
import numpy as np
import check_saved_rgb_hold_overlap_20261005 as base
from alicia_flexible_grasp.vision.visual_stationary import VisualStationaryWindow

CAP=base.EXT/'ros_log/rgb_observation_bootstrap_20261005T042536Z'
OUT=CAP/'offline_rgb_motion_20261005T0438Z'
if not base.MOUNT.is_mount():
    raise RuntimeError('SanStand unavailable; no fallback')
record=json.loads((CAP/'result.json').read_text())
cv2.setNumThreads(1)
frames=[]
for row in record['rows']:
    path=Path(row['source_file'])
    path.resolve().relative_to(CAP)
    if base.sha(path)!=row['source_file_sha256']:
        raise ValueError('source file hash mismatch')
    with np.load(path,allow_pickle=False) as data:
        rgb=data['color_bgr']
        if base.array_sha(rgb)!=row['source_rgb_sha256']:
            raise ValueError('source RGB hash mismatch')
    h,w=rgb.shape[:2]
    ratio=min(1.,VisualStationaryWindow.MAX_SIDE/float(max(h,w)))
    small=cv2.resize(rgb,(round(w*ratio),round(h*ratio)),interpolation=cv2.INTER_AREA)
    gray=cv2.cvtColor(small,cv2.COLOR_BGR2GRAY)
    valid=base.rectification_rgb_valid_mask(row['source_context']['camera'],rgb.shape[:2])
    frames.append(dict(row=row,rgb=rgb,gray=gray,valid=valid,
        scale=np.array([w/float(gray.shape[1]),h/float(gray.shape[0])],np.float32)))


def summary(values):
    values=np.asarray(values)
    if not values.size:
        return None
    return dict(zip(('min','p50','p90','p95','max'),map(float,np.percentile(values,[0,50,90,95,100]))))


def photometry(first,second,valid):
    a=cv2.cvtColor(first,cv2.COLOR_BGR2GRAY).astype(float)
    b=cv2.cvtColor(second,cv2.COLOR_BGR2GRAY).astype(float)
    y,x=np.indices(a.shape)
    sample=(y%4==0)&(x%4==0)&valid&(a>5)&(a<250)&(b>5)&(b<250)
    av,bv=a[sample],b[sample]
    selected=np.ones(len(av),bool)
    for _ in range(3):
        coefficients=np.linalg.lstsq(np.c_[av[selected],np.ones(selected.sum())],bv[selected],rcond=None)[0]
        residual=bv-(av*coefficients[0]+coefficients[1])
        selected=np.abs(residual)<=np.percentile(np.abs(residual),80)
    return dict(median_gray_difference=float(np.median(b[valid]-a[valid])),
        absolute_gray_difference=summary(np.abs(b[valid]-a[valid])),
        robust_gray_gain=float(coefficients[0]),robust_gray_offset=float(coefficients[1]),
        residual_after_gray_gain_offset=summary(np.abs(residual)),
        samples=len(av),fit_scope='unregistered corresponding observed pixels; diagnostic only')


rows=[];alltracks=[]
for i,(a,b) in enumerate(zip(frames,frames[1:]),1):
    # Reuse the production point seeder and LK parameters without observe():
    # sparse published frames lack raw SDK numbering/clock continuity.
    seeder=VisualStationaryWindow()
    if not seeder._seed(a['gray'],0.,0.):
        raise ValueError('insufficient original seeder texture')
    points=seeder._points
    kwargs=dict(winSize=(15,15),maxLevel=2,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,15,.01))
    forward,status,_=cv2.calcOpticalFlowPyrLK(a['gray'],b['gray'],points,None,**kwargs)
    backward,reverse,_=cv2.calcOpticalFlowPyrLK(b['gray'],a['gray'],forward,None,**kwargs)
    fb=np.linalg.norm((backward-points).reshape(-1,2)*a['scale'],axis=1)
    destination=forward.reshape(-1,2)
    good=(status.ravel()==1)&(reverse.ravel()==1)&(fb<=VisualStationaryWindow.MAX_FORWARD_BACKWARD_PX)
    good&=(destination[:,0]>=0)&(destination[:,0]<b['gray'].shape[1])&(destination[:,1]>=0)&(destination[:,1]<b['gray'].shape[0])
    origin=points.reshape(-1,2)*a['scale'];target=destination*a['scale'];delta=target-origin
    origin,target,delta,fb=origin[good],target[good],delta[good],fb[good]
    amplitude=np.linalg.norm(delta,axis=1)
    h,w=a['rgb'].shape[:2]
    qlabels=(origin[:,0]>=w/2).astype(int)+2*(origin[:,1]>=h/2).astype(int)
    strips=np.minimum(4,(origin[:,0]/w*5).astype(int))
    pooled_median=np.median(delta,axis=0)
    center= (origin[:,0]>=.2*w)&(origin[:,0]<=.8*w)
    center_translation=np.median(delta[center],axis=0)
    row=dict(pair_index=i,previous_source_ns=a['row']['source_stamp_ns'],current_source_ns=b['row']['source_stamp_ns'],
        gap_sec=(b['row']['source_stamp_ns']-a['row']['source_stamp_ns'])*1e-9,
        seeded=len(points),tracked=len(origin),coverage_quadrants=int(sum(np.count_nonzero(qlabels==q)>=4 for q in range(4))),
        displacement_full_rectified_px=summary(amplitude),common_translation_median_xy_px=pooled_median.tolist(),
        common_translation_norm_px=float(np.linalg.norm(pooled_median)),
        residual_about_common_translation_px=summary(np.linalg.norm(delta-pooled_median,axis=1)),
        central_20_to_80_percent_width_translation_xy_px=center_translation.tolist(),
        central_displacement_px=summary(amplitude[center]),
        central_residual_about_central_translation_px=summary(np.linalg.norm(delta[center]-center_translation,axis=1)),
        motion_over_existing_point_scale_025_count=int(np.count_nonzero(amplitude>.25)),
        per_horizontal_fifth=[dict(x_fraction=[s/5.,(s+1)/5.],count=int(np.count_nonzero(strips==s)),
            displacement_px=summary(amplitude[strips==s]),
            motion_over_025_count=int(np.count_nonzero((strips==s)&(amplitude>.25)))) for s in range(5)],
        tracks=[dict(origin_xy=list(map(float,p)),destination_xy=list(map(float,q)),displacement_xy=list(map(float,d)),
            magnitude=float(m),fb=float(f),horizontal_fifth=int(s)) for p,q,d,m,f,s in zip(origin,target,delta,amplitude,fb,strips)])
    valid=np.ones(a['rgb'].shape[:2],bool) if a['valid'] is None else a['valid'].copy()
    if b['valid'] is not None:
        valid&=b['valid']
    row['photometry']=photometry(a['rgb'],b['rgb'],valid)
    centralvalid=valid.copy();centralvalid[:,:int(.2*w)]=False;centralvalid[:,int(.8*w):]=False
    row['central_photometry']=photometry(a['rgb'],b['rgb'],centralvalid)
    rows.append(row);alltracks.extend(row['tracks'])
bystrip=[]
for s in range(5):
    subset=[t for t in alltracks if t['horizontal_fifth']==s]
    bystrip.append(dict(x_fraction=[s/5.,(s+1)/5.],tracks=len(subset),
        displacement_px=summary([t['magnitude'] for t in subset]),
        over025=sum(t['magnitude']>.25 for t in subset),over1=sum(t['magnitude']>1. for t in subset)))
result=dict(capture=str(CAP),capture_sha256=base.sha(CAP/'result.json'),script_sha256=base.sha(__file__),
    original_algorithm_sha256=base.sha(base.WORK/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/visual_stationary.py'),
    robot_commands_sent=False,motion_authorized=False,threshold_changed=False,source_kind='published_rectified_RGB',
    original_image_observations=[
        'Human review of first, middle and last RGB: a person/hands move over the keyboard at the left image edge; target and table layout appear stable.',
        'This is direct visual evidence in this capture, not proof these pixels drove the earlier SDK-internal window.'],
    limitations=[
        '23 adjacent published rectified pairs, about .5-.6 seconds apart; raw SDK anchor/points/continuous source frames from earlier 45 seconds were not retained and this is NOT an exact reproduction of those visual-gate decisions.',
        'Production seeding/LK/forward-backward settings are reused, but no SDK source metadata is fabricated and VisualStationaryWindow.observe is not called.',
        'Common-flow/photometry statistics cannot uniquely separate physical vibration, exposure response, rolling-shutter effects or optical flow bias.',
        'A .25 pixel diagnostic point cutoff does not authorize excluding image regions or loosening the actual permission gate.'],
    summary=dict(pairs=len(rows),source_span_sec=(frames[-1]['row']['source_stamp_ns']-frames[0]['row']['source_stamp_ns'])*1e-9,
        pair_p90_motion_px=summary([r['displacement_full_rectified_px']['p90'] for r in rows]),
        common_translation_norm_px=summary([r['common_translation_norm_px'] for r in rows]),
        central_pair_p90_motion_px=summary([r['central_displacement_px']['p90'] for r in rows]),
        gray_gain=summary([r['photometry']['robust_gray_gain'] for r in rows]),
        gray_offset=summary([r['photometry']['robust_gray_offset'] for r in rows]),
        spatial_tracks=bystrip),rows=rows)
OUT.mkdir(exist_ok=False)
shutil.copy2(__file__,OUT/Path(__file__).name)
(OUT/'report.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
print(json.dumps(dict(report=str(OUT/'report.json'),sha256=base.sha(OUT/'report.json'),summary=result['summary'])))
