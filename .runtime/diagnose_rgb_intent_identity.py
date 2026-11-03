"""Archived identity diagnostics only; no ROS or source identity overrides."""
from pathlib import Path
from dataclasses import fields
import json,hashlib,os,sys
import cv2,numpy as np
sys.path.insert(0,'src/alicia_grasp_modes/src')
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy,propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_handoff import _appearance,HandoffPolicy
from alicia_grasp_modes.rgb_observation_volume import RGBObservationVolume,SupportPlaneProvenance,project_rgb_observation_volume
from alicia_grasp_modes.tabletop import _project,project_base_to_pixel
cv2.setNumThreads(1)
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
def sha(x): return hashlib.sha256(x).hexdigest()
def load(r):
 p=Path(r['source_file']);assert sha(p.read_bytes())==r['source_file_sha256'];x=np.load(p,allow_pickle=False)
 assert sha(x['color_bgr'].tobytes())==r['source_rgb_sha256'];assert sha(x['depth_raw'].tobytes())==r['source_depth_sha256']
 return x
def vol(v):
 v=dict(v);p=v.pop('support_provenance');p={k:v for k,v in p.items() if k in {f.name for f in fields(SupportPlaneProvenance) if f.init}}
 v['support_provenance']=SupportPlaneProvenance(**p)
 return RGBObservationVolume(**{k:v for k,v in v.items() if k in {f.name for f in fields(RGBObservationVolume) if f.init}})
a=json.loads((ROOT/'rgb_acquisition_gateway_20261004T044244.594925Z_cd1dada6/result.json').read_text())['frames']['initial'];old=load(a)
regions=propose_rgb_observation_regions(old['color_bgr'],policy=CenterSearchPolicy(**a['interest']['rgb_search_policy'])).candidates
refs=[]
for m,v in zip(a['interest']['rgb_interest_members'],a['interest']['interest_volumes']):
 selected=[r for r in regions if sha(r.mask.tobytes())==m['mask_sha256']];assert len(selected)==1
 r=selected[0];hist,contour=_appearance(old['color_bgr'],r.mask.astype(np.uint8))
 refs.append((m,vol(v),hist,contour,r))
rawdoc=json.loads((ROOT/'rgb_observation_bootstrap_20261004T045837Z/result.json').read_text());output=[]
for index in (0,len(rawdoc['rows'])-1):
 r=rawdoc['rows'][index];x=load(r);intr=np.asarray(x['K']);K=intr if intr.shape==(3,3) else np.array([[intr[0],0,intr[2]],[0,intr[1],intr[3]],[0,0,1.]])
 T=x['T_base_camera'];rgb=x['color_bgr'];projections=[np.asarray(project_rgb_observation_volume(v,T_base_camera=T,camera_matrix=K,image_shape=rgb.shape[:2],working_depth_range_m=v.working_depth_range_m)) for _,v,_,_,_ in refs]
 vertices=np.concatenate(projections);center=(vertices.min(0)+vertices.max(0))/2;radius=np.max(np.linalg.norm(vertices-center,axis=1))/min(rgb.shape[:2])
 policy=CenterSearchPolicy((float(center[0]/rgb.shape[1]),float(center[1]/rgb.shape[0])),radius_fraction=min(1,max(.01,float(radius))))
 props=propose_rgb_observation_regions(rgb,policy=policy);rows=[]
 for ref_i,((m,v,h,c,original_region),projected) in enumerate(zip(refs,projections)):
  pm=np.zeros(rgb.shape[:2],np.uint8);cv2.fillConvexPoly(pm,np.rint(cv2.convexHull(projected.astype(np.float32))).astype(np.int32),1);matches=[]
  for i,region in enumerate(props.candidates):
   hist,contour=_appearance(rgb,region.mask.astype(np.uint8));colour=float(np.sqrt(max(0,1-np.sum(np.sqrt(hist*h)))));shape=float(cv2.matchShapes(c,contour,cv2.CONTOURS_MATCH_I1,0));fraction=float(np.count_nonzero(region.mask&pm)/region.area_px);p=HandoffPolicy();admit=colour<=p.max_colour_distance and shape<=p.max_contour_distance and fraction>=p.min_projected_fraction
   if admit:matches.append(dict(index=i,bbox_xywh=region.bbox_xywh,area_px=region.area_px,colour_distance=colour,contour_distance=shape,projected_fraction=fraction,score=.65*colour/p.max_colour_distance+.35*shape/p.max_contour_distance))
  matches.sort(key=lambda row:row['score']);unique=bool(matches) and (len(matches)==1 or matches[1]['score']-matches[0]['score']>HandoffPolicy().ambiguity_margin)
  olddepth=old['depth_raw'].astype(float)*float(old['depth_scale']);oldintr=np.asarray(old['K']);oldintr=(oldintr[0,0],oldintr[1,1],oldintr[0,2],oldintr[1,2]) if oldintr.shape==(3,3) else tuple(oldintr);points=_project(olddepth,oldintr);valid=original_region.mask&np.isfinite(olddepth)&(olddepth>.03)&(olddepth<2);base=points[valid]@old['T_base_camera'][:3,:3].T+old['T_base_camera'][:3,3];median=np.median(base,axis=0);pixel=project_base_to_pixel(median,T,(K[0,0],K[1,1],K[0,2],K[1,2]))
  rows.append(dict(member=ref_i,original_bbox=m['bbox_xywh'],matches=matches,unique_by_original_rgb_rules=unique,raw_reference_median_base=median.tolist(),raw_reference_median_projected_pixel=pixel.tolist()))
 output.append(dict(frame_index=index,source_stamp_ns=r['source_stamp_ns'],source_id=r['source_id'],source_file_sha256=r['source_file_sha256'],old_generation=a['source_context']['selection']['generation'],current_generation=r['source_context']['selection']['generation'],exact_owner_association_admissible=False,reason='source_id_and_generation_changed; appearance_projection_only_diagnostic',candidate_count=len(props.candidates),reference_results=rows))
report=dict(implementation=dict(python=sys.version,opencv=cv2.__version__,numpy=np.__version__,script_sha256=sha(Path(__file__).read_bytes())),original_reference_id='db61036b32d0afdfae1eb519dfa6e0d12130acd471318d4249cec879d57bb7b0',original_source_sha256=a['source_file_sha256'],rows=output,robot_commands_sent=False,source_binding_bypassed=False,motion_authorized=False)
assert os.path.ismount('/media/zhuyupei/SanStand');out=ROOT/'rgb_observation_bootstrap_20261004T045837Z'/'original_intent_reassociation_diagnostic.json';out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));print('FILE',out)
