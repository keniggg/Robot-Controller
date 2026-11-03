"""Counterfactual proposals only; original common interval and source stay intact."""
import pathlib,json,math,sys
import numpy as np,cv2
from alicia_flexible_grasp.vision import cuboid_rgb_interval as interval
D=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z');O=D/'verification/gen38_cuboid_silhouette_probe_r1';I=D/'verification/gen37_actual_attempt1_source_images_r1';R=D/'verification/gen37_box_source_replay_r1';G=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261017T032113.476278Z_a84cc89fc53e4c2b82680749d78b9b9f')
assert pathlib.Path('/media/zhuyupei/SanStand').is_mount() and 'sitecustomize' in sys.modules
read=lambda p:json.loads(p.read_text());a=read(G/'keyframes_audits/0e5ebbd3c562cfab2617b338045f554ffc0920a23626fbc2a49cc474cc9d798d.json');fit=np.load(R/'exact_current_top_fit.npz');c=read(D/'gen37_runtime/gen37_grasp_scene_r2/camera_configuration.json');K=np.array([c[k] for k in ('fx','fy','cx','cy')]);T=np.array(a['snapshot_transform']['T_base_optical']);q=a['remote_diagnostics']['tabletop_geometry']['regular_box_inference'];s=str(q['source_stamp_ns']);color=cv2.imread(str(I/(s+'_color.png')));mask=cv2.imread(str(I/(s+'_mask.png')),0);depth=np.load(I/(s+'_depth.npy'));pixel=np.median(depth[fit['uv'][fit['keep'],1],fit['uv'][fit['keep'],0]])*c['depth_scale']/min(K[:2])/q['camera_top_cosine'];frozen=read(O/'result.json')['best_example'];original_refit=interval._refit_cuboid
def fixed_source_height(lo,hi,h,hu,A,d,T,K,edges,edge_pixels):
 row=original_refit(lo,hi,h,1e-12,A,d,T,K,edges,edge_pixels)
 row['height_budget']=hu+abs(row['height']-h)
 return row
def frozen_existence_proposal(lo,hi,h,hu,A,d,T,K,edges,edge_pixels):
 B=np.array(frozen['axes']);dims=np.array(frozen['size_xyz_m']);center=np.array(frozen['center']);mid=center@B[:,:2];lower,upper=mid-dims[:2]/2,mid+dims[:2]/2
 model=interval._model(B,lower,upper,dims[2],d,T,K);params=np.r_[(center@A[:,:2]-(lo+hi)/2),dims[:2]-(hi-lo),math.atan2(A[:,1]@B[:,0],A[:,0]@B[:,0]),dims[2]-h]
 return dict(model=model,axes=B,lower=lower,upper=upper,height=dims[2],parameters=params,height_budget=hu+abs(dims[2]-h),score=0.)
rows=[]
try:
 for name,proposal in [('fixed_original_measured_height',fixed_source_height),('frozen_existing_18p867mm_proposal',frozen_existence_proposal)]:
  interval._refit_cuboid=proposal
  r=interval.infer_cuboid_rgb_interval(color,mask,depth,c['depth_scale'],T,K,fit['origin'],fit['tn'],fit['axes'],pixel,q['top_plane_rms_m'],a['replay_geometry']['support_offset_m'],q['top_height_m'])
  summary=dict(case=name,ok=r['ok'],reason=r.get('reason'))
  if r['ok']:
   dims=np.r_[r['upper']-r['lower'],r['nominal_height_m']];yaw=r['yaw_uncertainty'];width=dims[1]*math.cos(yaw)+dims[0]*math.sin(yaw);required=width+2*r['edge_uncertainty']+.001
   summary.update(size_xyz_m=dims.tolist(),nominal_height_m=r['nominal_height_m'],height_uncertainty_m=r['height_uncertainty'],edge_uncertainty_m=r['edge_uncertainty'],yaw_uncertainty_rad=yaw,required_opening_upper_m=float(required),opening_with_independent_reserve_m=float(required+.002),passes_with_reserve=bool(required+.002<=.05),full_rgb_max_px=max(r['audit']['full_silhouette_rgb_distances_px']),common_height_interval_m=r['audit']['common_height_interval_m'],original_height_interval_m=r['audit']['original_common_height_interval_m'])
   interval.validate_cuboid_rgb_interval_source(r['audit'],color,mask,depth,c['depth_scale'],T,K)
   summary['source_validator_passed']=True
  rows.append(dict(summary=summary,result=r))
finally:interval._refit_cuboid=original_refit
out=dict(diagnostic_only=True,production_budget_unchanged=True,source_arrays_unchanged=True,all_raw_raster_responses_retained=True,fit_clearance_total_m=.001,independent_reserve_m=.002,cases=rows)
(O/'fixed_height_proposal_diagnostics.json').write_text(json.dumps(out,ensure_ascii=False,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else x.item())+'\n');print(json.dumps([r['summary'] for r in rows],ensure_ascii=False))
