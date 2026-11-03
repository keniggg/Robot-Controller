"""Original locked-support/refinement gates on corrected same-source TF replay."""
import json,hashlib
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np,cv2,rosbag
from cv_bridge import CvBridge
from alicia_grasp_modes.tabletop import Config,_audit_current_support,segment_with_support_fallback,_matching_candidates
from alicia_grasp_modes.rgb_instance import refine_candidate
p=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/feedforward_speed_grasp_20260929T082708Z')
assert Path('/media/zhuyupei/SanStand').is_mount()
a=json.loads((p/'offline_support_motion_replay/report.json').read_text());td=sorted(p.glob('isolated_accepted_tf_*'))[-1];t=json.loads((td/'report.json').read_text());transforms={r['source_ns']:r for r in t['rows']};bridge=CvBridge();cv2.setNumThreads(1)
c=json.loads((p/'offline_support_motion_replay/camera_metadata.json').read_text())[0]['metadata']['parameters'];K=[c[n] for n in ['fx','fy','cx','cy']];cfg=Config()
anchor=NS(position_base=np.array(a['anchor_position_base']),extent_m=np.array(a['first_loss']['anchor_extent_m']),support_footprint_base=np.array(a['reference_footprint_base']));ref=np.array(a['reference_plane_base']);depth={};rgb={}
formal=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20260929T082708.745729Z_2762c918c2cc450d8a1770cc17f763fe/keyframes.bag')
for file in [p/'scene_diagnostic.bag',formal]:
 with rosbag.Bag(str(file)) as b:
  for topic,msg,_ in b.read_messages(topics=['/supervisor/camera/depth/image_raw','/supervisor/camera/color/image_raw']):
   ns=msg.header.stamp.to_nsec()
   if ns not in transforms:continue
   if 'depth' in topic:depth.setdefault(ns,bridge.imgmsg_to_cv2(msg,'passthrough').astype(np.float32)*c['depth_scale'])
   else:rgb.setdefault(ns,bridge.imgmsg_to_cv2(msg,'bgr8'))
rows=[];last=anchor
for ns,item in transforms.items():
 F=np.array(item['T_base_camera']);r=dict(source_ns=ns,seconds_after_terminal=item['seconds_after_terminal'])
 try:
  audit=_audit_current_support(depth[ns],K,F,cfg,anchor,ref);r['support']=audit
 except Exception as e:r['support_error']=str(e)
 try:
  segmentation=segment_with_support_fallback(depth[ns],K,F,cfg,anchor=anchor,last=last,reference_plane_base=ref)
  matching=_matching_candidates(segmentation.candidates,anchor,last,cfg);r['matching_count']=len(matching)
  if len(matching)==1:
   candidate=refine_candidate(rgb[ns],depth[ns],K,F,matching[0][1],cfg);r['rgb_instance_accepted']=True;r['position_base']=candidate.position_base.tolist();r['quality']=candidate.quality;last=candidate
 except Exception as e:r['segmentation_error']=str(e)
 rows.append(r)
summary=dict(rows=len(rows),support_available=sum(r.get('support',{}).get('support_current_audit_available') is True for r in rows),support_rejections=[{k:r[k] for k in ['source_ns','support_error']} for r in rows if 'support_error' in r],maximum_support_error_m=max(r.get('support',{}).get('support_reference_max_distance_m',0) for r in rows),rgb_accepted=sum(r.get('rgb_instance_accepted') is True for r in rows),first_loss_replay=next(r for r in rows if r['source_ns']==a['first_loss']['source_stamp_ns']))
result=dict(scope='same images and immutable anchor/footprint; actual isolated RSP output; original full support and RGB gates',original_threshold_m=cfg.support_max_local_shift_m,summary=summary,rows=rows,motion_authorized=False,production_parameters_changed=False,source_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(td/'segmentation_report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(summary,indent=2))
