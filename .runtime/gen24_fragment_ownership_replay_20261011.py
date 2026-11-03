from pathlib import Path
from dataclasses import fields, replace
from copy import deepcopy
import hashlib,json,time,sys
import cv2,numpy as np
from alicia_grasp_modes import tabletop as t, rgb_instance as ri, arrival_fragment_group as afg
from alicia_grasp_modes import observation_arrival_confirmation as ac
from alicia_grasp_modes.rgb_observation_handoff import _appearance
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen24_analysis')
assert Path('/media/zhuyupei/SanStand').is_mount()
INP=BASE/'replay_inputs';OUT=BASE/'fragment_ownership';OUT.mkdir(exist_ok=True)
manifest=json.loads((INP/'manifest.json').read_text());rows=manifest['files']
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
mods=[t,ri,afg,ac];before={m.__file__:sha(m.__file__) for m in mods}
def data(role):
 row=next(r for r in rows if r['role']==role);return dict(np.load(row['path'])),row

def arrays(f):
 k=f['K'];k=[k[0,0],k[1,1],k[0,2],k[1,2]] if k.shape==(3,3) else k
 return f['depth_raw'].astype(np.float32)*float(f['depth_scale']),k,f['T_base_camera']
params=json.loads((INP/'unknown_perception_configuration.json').read_text())['parameters']
cfg=t.Config(**{k:v for k,v in params.items() if k in {f.name for f in fields(t.Config)}})
tracker=t.TargetTracker(cfg)
a,ar=data('anchor');aseg=t.segment_with_support_fallback(*arrays(a),cfg)
raw,reason=tracker.choose(aseg,a['depth_raw'].shape)
as_=json.loads((INP/'anchor_status.json').read_text())['message']
assert np.max(abs(raw.position_base-as_['anchor_position_base']))<1e-9
assert np.max(abs(tracker.plane_base-as_['reference_support_plane_base']))<1e-9
ref,rr=data('last_pre_motion');rs=json.loads((INP/'last_pre_motion_status.json').read_text())['message']
rseg=t.segment_with_support_fallback(*arrays(ref),cfg,anchor=tracker.anchor,last=tracker.last,reference_plane_base=tracker.plane_base,prefer_current_support=True)
matches=t._matching_candidates(rseg.candidates,tracker.anchor,tracker.last,cfg)
assert len(matches)==1, len(matches)
tracker.last=matches[0][1]
assert np.max(abs(tracker.last.extent_m-rs['geometry_metrics']['depth_seed_extent_m']))<1e-9
hist,contour=_appearance(ref['color'],ref['recorded_refined_mask'])
appearance=FrozenObservationAppearance(ref['color'],ref['recorded_refined_mask'],ref['rgb_valid'],rr['source_stamp_ns'],hist,contour)
helper=ac.ObservationArrivalConfirmation(cfg,tracker.anchor,tracker.last,tracker.plane_base,hist,contour,.5,appearance_reference=appearance)
f,row=data('first_failure');depth,K,T=arrays(f)
seg=t.segment_with_support_fallback(depth,K,T,cfg,anchor=tracker.anchor,last=tracker.last,reference_plane_base=tracker.plane_base,prefer_current_support=True)
captured={}
def trace(frame,event,arg):
 if frame.f_code.co_name=='_recover_matching_fragments' and frame.f_code.co_filename==afg.__file__ and event=='exception':captured.update(frame.f_locals)
 return trace
sys.settrace(trace)
start=time.monotonic()
try:
 result=helper.observe(seg,f['color'],depth,K,T,row['source_stamp_ns'],rgb_valid=f['rgb_valid'],appearance_deadline_monotonic=start+.8,appearance_check_current=lambda:None)
 reason='PASSED'
except ValueError as e:reason=str(e)
finally:sys.settrace(None)
cores=captured['cores'];instances=captured['instances'];ind=captured['independent'];native=[c.mask>0 for _,c in captured['matches']]
retention=[[float(np.count_nonzero(i&c))/np.count_nonzero(c) for c in cores]for i in instances]
counts=[[int(np.count_nonzero(i&c))for c in cores]for i in instances]
frame=captured['frame'];h=captured['heights'];rgb=f['color']
def stats(mask):
 y,x=np.nonzero(mask);v=mask&np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
 return dict(pixels=int(mask.sum()),bbox_xyxy=[int(x.min()),int(y.min()),int(x.max()+1),int(y.max()+1)] if len(x) else None,valid_depth=int(v.sum()),height_mm_quantiles=(np.percentile(h[v]*1000,[0,10,50,90,100]).tolist() if v.any() else None),outside_domain=int(np.count_nonzero(mask&~frame.spatial['domain_mask'])))
failures=[]
for i,instance in enumerate(instances):
 for j,core in enumerate(cores):
  lost=core&~instance
  failures.append(dict(instance=i,core=j,**stats(lost)))
for i in range(len(cores)):
 img=rgb.copy();cv2.drawContours(img,cv2.findContours(instances[i].astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0],-1,(0,255,0),1)
 for j,core in enumerate(cores):img[core]=(255,0,0) if j==0 else (0,0,255)
 cv2.imwrite(str(OUT/('instance%d_cores.png'%i)),img)
 cv2.imwrite(str(OUT/('instance%d.png'%i)),instances[i].astype('uint8')*255)
 cv2.imwrite(str(OUT/('native%d.png'%i)),native[i].astype('uint8')*255)
 cv2.imwrite(str(OUT/('core%d.png'%i)),cores[i].astype('uint8')*255)
np.savez_compressed(OUT/'masks.npz',rgb=rgb,depth=depth,heights=h,domain=frame.spatial['domain_mask'],cores=np.array(cores),instances=np.array(instances),native=np.array(native))
report=dict(input_manifest_sha256=sha(INP/'manifest.json'),reference_source_ns=rr['source_stamp_ns'],source_ns=row['source_stamp_ns'],reason=reason,wall_sec=time.monotonic()-start,anchor_position_error_m=float(np.max(abs(raw.position_base-as_['anchor_position_base']))),last_seed_extent_error_m=float(np.max(abs(tracker.last.extent_m-rs['geometry_metrics']['depth_seed_extent_m']))),candidate_count=len(seg.candidates),matching_count=len(captured['matches']),support=seg.metrics,independent=ind,retained_counts=counts,retained_fractions=retention,core_stats=[stats(c) for c in cores],native_stats=[stats(c)for c in native],instance_stats=[stats(i)for i in instances],unowned_stats=failures,pairwise_iou=float(np.count_nonzero(instances[0]&instances[1])/np.count_nonzero(instances[0]|instances[1])),appearance=helper.last_appearance_evidence,source_sha256=before,source_after_sha256={m.__file__:sha(m.__file__)for m in mods},script_sha256=sha(__file__),limitations=['Only first failed online frame has exact recorded RGBD.','This calls production arrival/fragment chain with exact sealed inputs; no ROS timing/authority is replayed.','Recorded config provides detect_hz and 7/4mm support policy; other geometry fields use deployment defaults.'])
def enc(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 raise TypeError(type(v).__name__)
(OUT/'first_failure_replay.json').write_text(json.dumps(report,indent=2,default=enc)+'\n')
print(json.dumps({k:report[k]for k in ['reason','candidate_count','matching_count','retained_counts','retained_fractions','pairwise_iou','core_stats','unowned_stats']}))
