"""Offline counterfactual label matrix; outputs confer no qualification authority."""
from pathlib import Path
from types import SimpleNamespace as NS, FunctionType
from dataclasses import replace
import json, hashlib, sys
import numpy as np
import cv2
W=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(W/'src/alicia_grasp_modes/src'))
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_grasp_modes import tabletop as table, rgb_instance as rgb
from alicia_grasp_modes.measured_mask_seeds import local_surface_normals
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
I=E/'ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen29_attempt1_shadow_frames'
O=I.parent/'gen30_label_channel_matrix'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def mh(a):return hashlib.sha256(np.asarray(a>0,dtype=np.uint8).tobytes()).hexdigest()
meta=json.loads((I/'mask_contract_review_r2/result.json').read_text())
cfg=table.Config(**meta['effective_config'])
q=np.load(I/'source_1792062835164456367/raw_arrays.npz')['object_mask']>0
inputs={str(I/'mask_contract_review_r2/result.json'):sha(I/'mask_contract_review_r2/result.json')}
sources={str(Path(m.__file__)):sha(m.__file__) for m in (table,rgb)}
assert Path('/media/zhuyupei/SanStand').is_mount() and not O.exists()
O.mkdir();rows=[]
for prior in meta['rows'][1:]:
 ns=prior['source_stamp_ns'];p=I/('source_%d'%ns)
 for name in ('raw_arrays.npz','metadata.json','detector_status.json'): inputs[str(p/name)]=sha(p/name)
 a=dict(np.load(p/'raw_arrays.npz'));s=json.loads((p/'detector_status.json').read_text())['payload']
 camera=json.loads((p/'metadata.json').read_text())['camera_parameters']
 depth=a['depth_raw'].astype(np.float32)*float(a['depth_scale_m']);T=a['T_base_camera_link'];K=a['intrinsics'];bgr=a['color_bgr']
 anchor=NS(position_base=np.asarray(s['anchor_position_base']),extent_m=np.asarray(s['anchor_extent_m']),support_footprint_base=np.asarray(s['reference_support_footprint_base']),metrics={'median_height_m':s['reference_object_height_m']})
 last=NS(position_base=np.asarray(prior['prior_position_used']))
 locked=np.asarray(s['reference_support_plane_base']);current=np.asarray(prior['support_plane'])
 if prior['support_source']=='locked_reference':
  audit=table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,locked,prefer_current_support=True)
  # The current independent support fit is recorded by the actual segmentation audit.
  current=np.asarray(audit.metrics.get('current_support_plane_base',current))
 spatial=rgb.freeze_observation_spatial_reference(anchor,locked)
 points=table._project(depth,K);sp=rgb._observation_spatial_support(points,T,K,spatial,cfg)
 valid=(np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m))
 normals,reliable=local_surface_normals(points,depth,valid)
 rgbvalid=rectification_rgb_valid_mask(camera,depth.shape)
 item={'source_stamp_ns':ns,'results':[]}
 for seed_plane_name,seed_plane in [('current',current)]:
  seg=table.segment(depth,K,T,cfg,fit_current_support=False,target_position_base=anchor.position_base,plane_base=seed_plane,target_footprint_base=anchor.support_footprint_base)
  # Front native independently selected by greatest measured overlap with the recorded front seed.
  native_image=cv2.imread(str(I/('mask_contract_review_r2/%d_native_1.png'%ns)),0)>0
  candidate=max(seg.candidates,key=lambda c:int(np.count_nonzero((c.mask>0)&native_image)))
  for label_plane_name,plane in [('locked',locked),('current',current)]:
   c=replace(candidate,metrics={**candidate.metrics,'support_plane_base':plane.tolist()})
   plane_camera=T.T@plane;heights=np.einsum('ijk,k->ij',points,plane_camera[:3])+plane_camera[3]
   steep=reliable&(np.abs(np.einsum('ijk,k->ij',normals,plane_camera[:3]/np.linalg.norm(plane_camera[:3])))<=np.sqrt(.5))
   for policy in (['unchanged'] if label_plane_name=='locked' else ['unchanged','background_consensus_in_domain','background_consensus_all','locked_probable','locked_core']):
    captures=[]
    def grab(image,labels,*args):
     # Find the exact slice from its unique shared pixel data and expected spatial crop.
     ys,xs=np.nonzero(c.mask);margin=16
     box=sp['search_box_xyxy'];x0=max(0,min(int(xs.min())-margin,int(box[0])-margin));y0=max(0,min(int(ys.min())-margin,int(box[1])-margin))
     x1=min(depth.shape[1],max(int(xs.max())+margin+1,int(box[2])+margin));y1=min(depth.shape[0],max(int(ys.max())+margin+1,int(box[3])+margin))
     assert np.array_equal(image,bgr[y0:y1,x0:x1]),(ns,image.shape,(x0,y0,x1,y1))
     global locked_labels
     if label_plane_name=='locked':locked_labels=labels.copy()
     remove=(labels==cv2.GC_BGD)&(locked_labels!=cv2.GC_BGD)
     if policy=='background_consensus_in_domain':remove &= sp['domain_mask'][y0:y1,x0:x1]
     remove[:3]=remove[-3:]=False;remove[:,:3]=remove[:,-3:]=False
     captures.append({'hard_background_pixels':int(np.count_nonzero(labels==cv2.GC_BGD)),'current_steep_background_conflicts':int(remove.sum())})
     if policy.startswith('background_consensus'):labels[remove]=cv2.GC_PR_BGD
     if policy=='locked_probable':
      soft=np.isin(labels,(cv2.GC_PR_BGD,cv2.GC_PR_FGD))&np.isin(locked_labels,(cv2.GC_PR_BGD,cv2.GC_PR_FGD))
      labels[soft]=locked_labels[soft]
     if policy=='locked_core':
      labels[labels==cv2.GC_FGD]=cv2.GC_PR_FGD
      labels[(locked_labels==cv2.GC_FGD)&(labels!=cv2.GC_BGD)]=cv2.GC_FGD
     return cv2.grabCut(image,labels,*args)
    proxy=NS(**{k:getattr(cv2,k) for k in dir(cv2) if not k.startswith('__')});proxy.grabCut=grab
    g=dict(rgb._instance_mask_from_seed.__globals__);g['cv2']=proxy
    fn=FunctionType(rgb._instance_mask_from_seed.__code__,g,argdefs=rgb._instance_mask_from_seed.__defaults__);fn.__kwdefaults__=rgb._instance_mask_from_seed.__kwdefaults__
    def builder(bgr,seed,**kw):return fn(bgr,seed,confidence_labeler=rgb.confidence_labels,**kw)
    row={'seed_plane':seed_plane_name,'label_plane':label_plane_name,'diagnostic_policy':policy,'seed_pixels':int(np.count_nonzero(c.mask)),'native_sha256':mh(c.mask)}
    try:
     out=rgb._refine_candidate_measured(bgr,depth,T,c,cfg,points,plane,plane_camera,heights,{'observation_spatial_support':sp},rgb_valid=rgbvalid,observation_arrival_retry=True,mask_builder=builder)
     m=out.mask>0;row.update(mask_pixels=int(m.sum()),mask_sha256=mh(m),fixed_template_iou=float((m&q).sum()/(m|q).sum()),metrics=out.metrics)
     cv2.imwrite(str(O/('%s_%s_%s_%s.png'%(ns,seed_plane_name,label_plane_name,policy))),out.mask)
    except ValueError as exc:row['error']=str(exc)
    row['grabcut_labels']=captures;item['results'].append(row)
    print(json.dumps({k:v for k,v in row.items() if k!='metrics'}),flush=True)
 rows.append(item)
assert sources=={p:sha(p) for p in sources} and inputs=={p:sha(p) for p in inputs}
report=dict(rows=rows,script_sha256=sha(__file__),input_sha256=inputs,source_sha256=sources,source_unchanged=True,input_unchanged=True,no_ros_calls=True,execution_authority=False,production_modified=False,qualification_gate_bypassed_diagnostic_only=True,limitations=['Plane replacement and background demotion are isolated counterfactuals, not production-qualified models.','No template pixel is added to a candidate; template mask only measures resulting IoU.','Same bounded prior-center proxies as mask_contract_review_r2; not full temporal history.'])
(O/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(O/'result.json',sha(O/'result.json'))
