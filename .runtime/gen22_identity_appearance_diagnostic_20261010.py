from pathlib import Path
import json,hashlib
import cv2,numpy as np,rosbag
from cv_bridge import CvBridge
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from alicia_grasp_modes.rgb_observation_handoff import _appearance
B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen22_analysis');P=B/'replay_inputs';M=json.loads((P/'manifest.json').read_text());R=Path(M['record']);NS=1791638794944338798
msgs={};bridge=CvBridge()
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
 for topic,msg,received in bag.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask']):
  if msg.header.stamp.to_nsec()==NS:msgs[topic]=(msg,received.to_nsec())
refs=[]
for r in M['files']:
 if r['recorded_refined_mask_available']:
  a=dict(np.load(r['path']));refs.append(dict(role=r['role'],source_ns=r['source_stamp_ns'],color=a['color'],mask=a['recorded_refined_mask']))
qualifiedmeta=dict(source_ns=NS,exact_topics_available=list(msgs),derived_mask=False)
if all(k in msgs for k in ('/supervisor/camera/color/image_raw','/perception/object_mask')):
 rgb=bridge.imgmsg_to_cv2(msgs['/supervisor/camera/color/image_raw'][0],'bgr8');mask=bridge.imgmsg_to_cv2(msgs['/perception/object_mask'][0],'mono8')
 ref=dict(role='first_view_qualified',source_ns=NS,color=rgb,mask=mask);refs.append(ref)
 payload=dict(color=rgb,recorded_refined_mask=mask,source_stamp_ns=np.int64(NS))
 if '/supervisor/camera/depth/image_raw' in msgs:payload['depth_raw']=bridge.imgmsg_to_cv2(msgs['/supervisor/camera/depth/image_raw'][0],'passthrough')
 path=P/('first_view_qualified_'+str(NS)+'.npz');np.savez_compressed(path,**payload);qualifiedmeta.update(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),limitations='RGB appearance-only supplemental extraction; no TF/q in this NPZ, unlike full replay-input frames.')
def summary(rgb,mask):
 lab=cv2.cvtColor(rgb,cv2.COLOR_BGR2LAB)[mask!=0];h,c=_appearance(rgb,mask)
 return dict(pixels=int(np.count_nonzero(mask)),mean_lab=lab.mean(0).tolist(),median_lab=np.median(lab,0).tolist(),lab_10_90=np.percentile(lab,[10,90],axis=0).tolist(),hist=h,contour=c)
def distance(a,b):return float(np.sqrt(max(0.,1.-np.sum(np.sqrt(a*b)))))
for ref in refs:ref['summary']=summary(ref['color'],ref['mask'])
replay=json.loads((B/'arrival_identity_replay_r1.json').read_text());frameindex={r['source_stamp_ns']:r for r in M['files']};rows=[]
for item in replay['results']:
 ns=item['source_ns'];a=dict(np.load(frameindex[ns]['path']));mask=cv2.imread(str(B/('crop_ab_production_'+str(ns)+'_mask.png')),0);sm=summary(a['color'],mask);comp=[]
 for ref in refs:
  hs=sm['hist'];hr=ref['summary']['hist'];marg=[]
  for axis in range(3):
   other=tuple(i for i in range(3) if i!=axis);marg.append(distance(hs.sum(axis=other),hr.sum(axis=other)))
  comp.append(dict(reference_role=ref['role'],reference_source_ns=ref['source_ns'],colour_distance=distance(hs,hr),contour_distance=float(cv2.matchShapes(ref['summary']['contour'],sm['contour'],cv2.CONTOURS_MATCH_I1,0)),marginal_L_a_b_distances=marg))
 rows.append(dict(source_ns=ns,current_rgb_mask={k:v for k,v in sm.items() if k not in ('hist','contour')},comparisons=comp))
# A diagnostic plot preserves the original pixels and overlays only measured mask
# boundaries, with source timestamps and mask provenance in the caption.
last=next(r for r in refs if r['role']=='last_pre_motion');first=replay['results'][0];a=dict(np.load(frameindex[first['source_ns']]['path']));mask=cv2.imread(str(B/('crop_ab_production_'+str(first['source_ns'])+'_mask.png')),0)
fig,axes=plt.subplots(1,2,figsize=(13.2,5.4),dpi=130)
for ax,img,mk,title in [(axes[0],last['color'],last['mask'],'Frozen reference: recorded READY mask\n'+str(last['source_ns'])),(axes[1],a['color'],mask,'Second view: production-replayed mask\n'+str(first['source_ns']))]:
 ax.imshow(cv2.cvtColor(img,cv2.COLOR_BGR2RGB));ax.contour(mk>0,levels=[.5],colors=['lime'],linewidths=.8);ax.set_title(title,fontsize=10);ax.set_xlim(0,img.shape[1]);ax.set_ylim(img.shape[0],0);ax.axis('off')
fig.suptitle('GEN22 identity diagnostic | unchanged gates | adjacent recorded arrival source, not online failure frame',fontsize=10);fig.tight_layout();overlay=B/'reference_vs_second_view_mask_overlay.png';fig.savefig(overlay);plt.close(fig)
report=dict(record=str(R),replay_report_sha256=hashlib.sha256((B/'arrival_identity_replay_r1.json').read_bytes()).hexdigest(),first_view_qualified_extraction=qualifiedmeta,reference_summaries=[dict(role=r['role'],source_ns=r['source_ns'],**{k:v for k,v in r['summary'].items() if k not in ('hist','contour')}) for r in refs],arrival_comparisons=rows,overlay=dict(path=str(overlay),sha256=hashlib.sha256(overlay.read_bytes()).hexdigest()),limitations=['Alternative templates are diagnostic only, not a policy to select a favorable reference.','Marginal LAB distances use original smoothed joint histogram marginals and are not additive causal fractions.','All arrival images here are real adjacent recorded frames; none exactly matches the twelve online assessed source stamps.','No synthetic geometry, intensity normalization, threshold change, ROS or motion.'])
out=B/'appearance_template_and_lab_diagnostic.json';out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(report=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),overlay=str(overlay),qualified=qualifiedmeta,first=rows[0],references=report['reference_summaries']),ensure_ascii=False))
