from pathlib import Path
import json,hashlib
import cv2,numpy as np
from alicia_grasp_modes.rgb_observation_handoff import _appearance
B=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen22_analysis');P=B/'replay_inputs';m=json.loads((P/'manifest.json').read_text());idx={r['source_stamp_ns']:r for r in m['files']};lr=next(r for r in m['files'] if r['role']=='last_pre_motion');a=dict(np.load(lr['path']));hr,_=_appearance(a['color'],a['recorded_refined_mask']);abref=hr.sum(axis=0);rep=json.loads((B/'arrival_identity_replay_r1.json').read_text());orb=json.loads((B/'orb_texture_correspondence_diagnostic.json').read_text());oi={r['source_ns']:r for r in orb['results'] if r['variant']=='recorded_object'};rows=[]
dist=lambda x,y:float(np.sqrt(max(0.,1.-np.sum(np.sqrt(x*y)))))
for item in rep['results']:
 ns=item['source_ns'];f=dict(np.load(idx[ns]['path']));mask=cv2.imread(str(B/('crop_ab_production_'+str(ns)+'_mask.png')),0);rgb=f['color'];hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV);shifted=hsv.copy();shifted[:,:,0]=(shifted[:,:,0].astype(int)+60)%180;hue=rgb.copy();hue[mask>0]=cv2.cvtColor(shifted,cv2.COLOR_HSV2BGR)[mask>0]
 for variant,img in [('recorded_object',rgb),('synthetic_hue_plus60',hue)]:
  hist,_=_appearance(img,mask);ab=hist.sum(axis=0);rows.append(dict(source_ns=ns,variant=variant,ab_joint_distance=dist(ab,abref),lab_joint_distance=dist(hist,hr)))
Hrows=[]
for ns,r in oi.items():
 H=np.asarray(r['homography']);x,y,w,h=cv2.boundingRect(a['recorded_refined_mask']);xx,yy=np.meshgrid(np.linspace(x,x+w,5),np.linspace(y,y+h,5));q=np.c_[xx.ravel(),yy.ravel(),np.ones(25)];den=(q@H.T)[:,2];det=np.linalg.det(H)/(den**3);Hrows.append(dict(source_ns=ns,jacobian_determinant_min=float(det.min()),jacobian_determinant_max=float(det.max()),all_sampled_jacobians_positive=bool(np.all(det>0))))
report=dict(record=m['record'],reference_source_ns=lr['source_stamp_ns'],definition='AB 8x8 marginal of unchanged original smoothed LAB 8x8x8 probability histogram; Hellinger distance. This is a diagnostic distinct from original full LAB guard.',results=rows,homography_25_point_jacobian_checks=Hrows,production_changed=False,threshold_chosen=False,limitations=['A/B marginal removes brightness by definition; it cannot replace original identity alone and cannot distinguish same-chroma different objects.','Hue-shifted controls are synthetic; genuine look-alike objects were not recorded.','Mask consistency, dispersed geometric features, depth/identity binding and later task checks remain separately necessary.'])
out=B/'chroma_and_homography_jacobian_diagnostic.json';out.write_text(json.dumps(report,indent=2)+'\n');print('REPORT',str(out),hashlib.sha256(out.read_bytes()).hexdigest())
for variant in ['recorded_object','synthetic_hue_plus60']:
 vals=[r['ab_joint_distance'] for r in rows if r['variant']==variant];print(variant,min(vals),max(vals))
print('all_jacobian_positive',all(r['all_sampled_jacobians_positive'] for r in Hrows))
