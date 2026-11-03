#!/usr/bin/env python3
"""Bounded offline explanation and two original-frame scientific overlays."""
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path
import zipfile

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

path=Path(__file__).with_name('analyze_current_support_rgbd_20261007.py')
spec=importlib.util.spec_from_file_location('current_support_replay',path)
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/current_support_rgbd_20261007T021840Z')
OUT=BASE/'original_support_analysis_v1'
report=json.loads((OUT/'report.json').read_text()); summary=json.loads((BASE/'summary.json').read_text())
rows=report['rows']; fits=[next(r for r in row['analysis']['rings'] if r['outer_base_px']==row['analysis']['selected_outer_base_px']) for row in rows]
validbase=[r for r in rows if 'plane_base' in r['analysis']]
comparisons=[]
for left,right in itertools.combinations(validbase,2):
 p=np.array(left['analysis']['plane_base']);q=np.array(right['analysis']['plane_base'])
 if p[:3]@q[:3]<0:q=-q
 e=[r['original_source_pose_evidence']['feedback']['epoch_stamp_ns'] for r in (left,right)]
 if e[0] is None or e[0]!=e[1]:continue
 comparisons.append(dict(left_stamp_ns=left['source_stamp_ns'],right_stamp_ns=right['source_stamp_ns'],same_control_reference_epoch=e[0],
  normal_angle_deg=float(np.degrees(np.arccos(np.clip(p[:3]@q[:3],-1,1)))),equation_offset_delta_mm=float((q[3]-p[3])*1000)))
feedback=[r['original_source_pose_evidence']['feedback']['before'] for r in rows if r['original_source_pose_evidence']['feedback']['source_bracket_available']]
qs=np.array([[dict(zip(row['names'],row['positions']))['Joint%d'%i] for i in range(1,7)] for row in feedback])
interpret=dict(schema=1,diagnostic_only=True,target_identity=None,motion_authorized=False,grasp_verified=False,
 report_sha256=a.sha((OUT/'report.json').read_bytes()),analysis_script_sha256=a.sha(path.read_bytes()),
 summary_script_sha256=a.sha(Path(__file__).read_bytes()),
 ten_current_camera_support_rings_pass_original_gates=True,
 inlier_ratio_range=[min(r['fit']['ratio'] for r in fits),max(r['fit']['ratio'] for r in fits)],
 inlier_residual_mm_range=[min(r['fit']['residual_m'] for r in fits)*1000,max(r['fit']['residual_m'] for r in fits)*1000],
 support_quadrants=sorted({r['quadrants'] for r in fits}),original_first_ring_used_count=10,
 all_valid_ring_points_owned_by_current_center_region=all(sum(n for name,n in r['actual_region_ownership_valid_points'].items() if name!='center')==0 for r in fits),
 center_filter_applied_count=sum(r['region_filter_applied_valid_fraction']==1 for r in fits),
 center_filter_not_applied_count=sum(r['region_filter_applied_valid_fraction']==0 for r in fits),
 exact_tf_feedback_complete_count=len(validbase),
 tf_library_header_delta_ns_counts={str(v):sum(r['original_source_pose_evidence']['tf']['delta_ns']==v for r in rows) for v in (0,1)},
 missing_optional_parameter_records=report['optional_defaults'],
 same_epoch_base_plane_comparisons=comparisons,
 measured_joint_before_endpoint_peak_to_peak_rad=dict(zip(['Joint%d'%i for i in range(1,7)],np.ptp(qs,axis=0).tolist())),
 statements=[
  'All ten quality-blind fixed-cadence samples pass the original measured plane gates at the initial widest ring; this is current scene evidence, not a repaired capture-history claim.',
  'Six sampled center regions were warming and four applied their qualified temporal filter. Both groups passed here; this small nonrandom sample does not establish a causal filter effect.',
  'Current original generic segmentation runs only for four exact-returned-TF plus accepted-feedback-complete frames. It yields measured components; no component was selected or assigned old identity.',
  'Six exact lookup requests returned a transform header +1ns. These original responses were preserved, were not relabelled, and were excluded from base-coordinate segmentation and comparisons.',
  'Seven absent /unknown_perception replies are explicit default-configuration provenance, not SDK timeout or corrupt frame evidence.',
  'Plane equation offset differences are not target-height errors. Static single-view observations do not validate cross-view fusion, continuous acquisition, a target lock or a grasp.',
  'An original RGB image and measured component outlines help visually inspect completeness; no RGB-specific instance mask or identity qualification was newly accepted.'
 ])
fig,axes=plt.subplots(1,2,figsize=(12,5),dpi=130)
with zipfile.ZipFile(BASE/'diagnostic.zip') as archive:
 for ax,index in zip(axes,(0,9)):
  pair=json.loads(archive.read(summary['pairs'][index]['entry']))
  ims={}
  for image in pair['images']:
   raw=a.bounded_original_message(archive.read(image['entry']),image['serialized_bytes'])
   assert a.sha(raw)==image['serialized_sha256']
   ims[image['stream']]=a.decode_image(raw)[1]
  rgb=ims['color'][...,::-1];h,w=rgb.shape[:2];scale=min(w/640,h/480)
  ax.imshow(rgb)
  for radius in (70,170):ax.add_patch(plt.Circle((w/2,h/2),radius*scale,fill=False,color='#00CFFF',lw=.9))
  if index==9:
   tf=rows[index]['original_source_pose_evidence']['tf'];x,y,z,qw=tf['quaternion_xyzw']
   T=np.eye(4);T[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*qw),2*(x*z+y*qw)],
     [2*(x*y+z*qw),1-2*(x*x+z*z),2*(y*z-x*qw)],[2*(x*z-y*qw),2*(y*z+x*qw),1-2*(x*x+y*y)]]
   T[:3,3]=tf['translation']
   result=a.table.segment(ims['depth'].astype(np.float32)*report['depth_scale_m'],report['camera_intrinsics'],T,a.table.Config(**report['config']))
   for candidate in result.candidates:
    contours,_=cv2.findContours(candidate.mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
     xy=contour[:,0,:];ax.plot(xy[:,0],xy[:,1],color='#FFDF00',lw=.9)
  ax.set_title('Pair %d: original RGB%s'%(index,' + measured depth components' if index==9 else ''))
  ax.set_xlim(0,w);ax.set_ylim(h,0);ax.set_xlabel('Published rectified pixel u');ax.set_ylabel('v')
fig.suptitle('Current support ring (cyan); outlines are measured components, not target identity')
fig.tight_layout()
image=OUT/'current_rgb_and_measured_components.png'
if image.exists():raise ValueError('new figure only')
fig.savefig(image);plt.close(fig)
interpret['figure_sha256']=a.sha(image.read_bytes())
raw=a.encoded(interpret);p=OUT/'interpretation.json'
if p.exists():raise ValueError('new interpretation only')
p.write_bytes(raw)
(OUT/'interpretation_SHA256SUMS').write_text(a.sha(raw)+'  interpretation.json\n'+interpret['figure_sha256']+'  '+image.name+'\n')
print(json.dumps(dict(path=str(p),sha256=a.sha(raw),base_comparisons=comparisons)))
