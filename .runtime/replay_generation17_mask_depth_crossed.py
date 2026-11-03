"""Offline factorial diagnostic only; deliberately mixed frames never authorize a plan."""
from pathlib import Path
import json, hashlib
import numpy as np
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/resumed_observe_return_grasp_20260930T040740Z')
base=OUT/'generation17_width_replay.py'
scope={'__file__':str(base),'__name__':'offline_diagnostic_inputs'}
exec(compile(base.read_text().split('rows=[]\nfor mode,window')[0],str(base),'exec'),scope)
images,windows=scope['images'],scope['windows']; bridge=scope['bridge'];objects=scope['objects'];camera=scope['camera'];identity=scope['identity'];fusion=scope['fusion'];geom=scope['geom'];K=scope['K'];estimate=scope['estimate_object_geometry'];fuse=scope['fuse_stable_samples'];Sample=scope['RgbdSample'];transform=scope['transform']
reference=scope['original'][-1]
def data(ns):
 m=images[ns]; obj=objects[ns]
 return dict(color=bridge.imgmsg_to_cv2(m['/supervisor/camera/color/image_raw'],'bgr8'),depth=bridge.imgmsg_to_cv2(m['/supervisor/camera/depth/image_raw'],'passthrough'),mask=bridge.imgmsg_to_cv2(m['/perception/object_mask'],'mono8'),bbox=(obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height),obj=obj)
ref=data(reference); rows=[]
for window in windows:
 ns=window[-1];cur=data(ns)
 for label,dep,mask,bbox in [('both_current',cur['depth'],cur['mask'],cur['bbox']),('fixed_reference_mask_current_depth',cur['depth'],ref['mask'],ref['bbox']),('current_mask_fixed_reference_depth',ref['depth'],cur['mask'],cur['bbox'])]:
  sample=Sample(cur['color'],dep,mask,bbox,cur['obj'],ns/1e9,'camera_link',np.zeros(6),ns,identity.epoch,identity)
  snap=fuse([sample],**fusion)
  row={'label':label,'source_stamp_ns':ns,'reference_stamp_ns':reference,'deliberate_counterfactual':label!='both_current','single_frame_diagnostic_only':True,'snapshot_ok':snap.ok,'mask_pixels':int(np.count_nonzero(mask))}
  if snap.ok:
   est=estimate(snap.depth_raw,snap.target_depth_raw,snap.object_mask,snap.bbox,K,camera['depth_scale'],transform(ns),snap.source_mode,**geom)
   row.update(geometry_ok=est.ok,reason=est.failure_reason)
   if est.ok:row.update(size_mm=(est.size_xyz_m*1000).tolist(),short_width_mm=float(min(est.size_xyz_m[:2])*1000),points=len(est.object_points_base))
  rows.append(row)
summary={}
for label in sorted(set(r['label'] for r in rows)):
 x=np.array([r['short_width_mm'] for r in rows if r['label']==label and r.get('geometry_ok')]);summary[label]={'n':len(x),'min_mm':float(x.min()),'max_mm':float(x.max()),'std_mm':float(x.std()),'median_mm':float(np.median(x))}
report={'scope':'offline input-factor isolation, not production or executable measurements','robot_motion':False,'parameters_changed':False,'source_archive':str(scope['BAG']),'source_inputs_script_sha256':hashlib.sha256(base.read_bytes()).hexdigest(),'counterfactual_limits':['Fixed masks/depth are diagnostic substitutions across different source frames and must never be supplied to runtime planning.','Every row is a single-frame diagnostic; no contact evidence or temporal stability is certified.','Support fit and bbox may respond to the selected mask; result isolates that whole processing dependence.'],'summary':summary,'rows':rows}
(OUT/'generation17_mask_depth_crossed.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(summary,ensure_ascii=False))
