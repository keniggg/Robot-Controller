#!/usr/bin/env python3
"""Original-pixel geometry diagnostics; no mask edits or identity acceptance."""
from pathlib import Path
import hashlib,json
import cv2,numpy as np
from alicia_grasp_modes.tabletop import _project,Config
from alicia_grasp_modes.measured_mask_seeds import local_surface_normals
F=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification/gen14_arrival_independent')
PRE=1791860176945733070
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def plain(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {k:plain(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [plain(v)for v in x]
 return x
def quantiles(a):return np.quantile(a,[0,.05,.25,.5,.75,.95,1])if len(a)else[]
def main():
 assert Path('/media/zhuyupei/SanStand').is_mount()
 out=F/'visibility_sufficiency.json';assert not out.exists();cv2.setNumThreads(1)
 fixture=json.loads((F/'fixture.json').read_text());camera=fixture['camera'];k=np.array([camera[x]for x in('fx','fy','cx','cy')]);pre=np.load(F/f'{PRE}_source.npz');mask=pre['recorded_mask']>0;depth=pre['depth'];config=Config();plane=np.array(json.loads((F/'original_geometry_reconstruction_v2.json').read_text())['seed']['reference_plane_base'])
 xyz=_project(depth,k);base=xyz@pre['T_base_camera'][:3,:3].T+pre['T_base_camera'][:3,3];height=base@plane[:3]+plane[3];valid=np.isfinite(depth)&(depth>=config.depth_min_m)&(depth<=config.depth_max_m)&np.isfinite(xyz).all(axis=2)
 normals,reliable=local_surface_normals(xyz,depth,valid);normals=normals@pre['T_base_camera'][:3,:3].T
 towards=pre['T_base_camera'][:3,3]-base;sign=np.where(np.sum(normals*towards,axis=2)>=0,1.,-1.);normals*=sign[...,None]
 frozen_measured=mask&valid;frozen_reliable=frozen_measured&reliable;rows=[];deps=[F/'fixture.json',F/'original_geometry_reconstruction_v2.json',F/f'{PRE}_source.npz',Path(__file__)]
 for fr in fixture['frames']:
  ns=fr['source_stamp_ns']
  if ns<1791860212000000000:continue
  path=Path(fr['file']);mp=F/f'{ns}_rejected_proposal_0_mask.png';deps.extend([path,mp]);cur=np.load(path);cm=cv2.imread(str(mp),0)>0
  target=(base-cur['T_base_camera'][:3,3])@cur['T_base_camera'][:3,:3];x=target[:,:,0]
  with np.errstate(divide='ignore',invalid='ignore'):u=k[2]-k[0]*target[:,:,1]/x;v=k[3]-k[1]*target[:,:,2]/x
  canv=frozen_measured&np.isfinite(target).all(axis=2)&(x>0)&(u>=0)&(v>=0)&(u<cm.shape[1]-.5)&(v<cm.shape[0]-.5)
  sy,sx=np.nonzero(canv);ix=np.rint(u[canv]).astype(int);iy=np.rint(v[canv]).astype(int);td=cur['depth'][iy,ix];tv=np.isfinite(td)&(td>=config.depth_min_m)&(td<=config.depth_max_m);inside=cm[iy,ix];delta=x[canv]-td;hh=height[canv];rel=frozen_reliable[canv]
  newray=cur['T_base_camera'][:3,3]-base[canv];newray/=np.linalg.norm(newray,axis=1)[:,None];facing=np.sum(normals[canv]*newray,axis=1)
  classes={}
  for name,take in [('all',np.ones(len(sx),bool)),('old_height_le15mm_diagnostic',hh<=.015),('old_height_gt15mm_diagnostic',hh>.015)]:
   c=dict(source_count=int(take.sum()),local_normals_reliable=int((take&rel).sum()),normal_dot_newray_quantiles=quantiles(facing[take&rel]),geometrically_backfacing_reliable=int((take&rel&(facing<0)).sum()),inside_current_mask=int((take&inside).sum()),outside_current_mask=int((take&~inside).sum()))
   for label,location in [('inside_current_mask',inside),('outside_current_mask',~inside)]:
    sample=take&location;good=sample&tv;c[label+'_depth']=dict(total=int(sample.sum()),no_depth=int((sample&~tv).sum()),valid_depth=int(good.sum()),predicted_minus_measured_depth_quantiles_m=quantiles(delta[good]),measured_closer_gt3mm=int((good&(delta>.003)).sum()),measured_closer_gt7mm=int((good&(delta>.007)).sum()),measured_farther_gt3mm=int((good&(delta<-.003)).sum()),measured_farther_gt7mm=int((good&(delta<-.007)).sum()),near_abs_le3mm=int((good&(abs(delta)<=.003)).sum()))
   classes[name]=c
  rows.append(dict(source_ns=ns,current_full_mask_pixels=int(cm.sum()),current_full_mask_missing_depth=int((cm&(~np.isfinite(cur['depth'])|(cur['depth']<config.depth_min_m)|(cur['depth']>config.depth_max_m))).sum()),reference_projected_to_canvas=int(canv.sum()),classes=classes))
 report=dict(scope='Measured source geometry and current raw depth sufficient-data audit only; no pixels removed, no new mask admitted, no RGB identity or same-object occlusion proof.',premotion_source_ns=PRE,premotion_full_mask_pixels=int(mask.sum()),premotion_valid_depth=int(frozen_measured.sum()),premotion_missing_depth=int((mask&~valid).sum()),premotion_reliable_local_normals=int(frozen_reliable.sum()),normal_algorithm='Original measured_mask_seeds.local_surface_normals: 5x5, >=21 valid samples, RMS<=0.5mm, existing degeneracy/depth-spread gates; normals oriented towards original camera. Sign against new ray is geometric only, not a certified normal-error bound.',diagnostic_splits='15mm height split inherited from prior diagnosis, not a production face selector. 3/7mm depth differences are reported counts, not accepted uncertainty bounds.',rows=rows,dependencies_sha256={str(p):sha(p)for p in dict.fromkeys(deps)},limits=['945 premotion missing-depth pixels must remain unknown; neither object shape nor support plane can fill them.', 'Measured closer depth only proves some foreground lies closer along a nominal ray; calling it self-occlusion additionally requires independent same-object ownership and calibrated robust visibility margins.', 'Backfacing local normals are not sufficient proof of closed-surface self-occlusion; local fits on stereo edge ramps are not a complete mesh.', 'Outside-mask rays with farther measured depth cannot be relabelled occluded because the old surface disappeared. Source silhouette/TF/depth conflict must stay explicit.', 'A source-frozen face mask changes the full-object evidence scope. Existing histogram shortcut and whole-mask H cannot be treated as unchanged acceptance by passing cropped masks.', 'No whole-object identity acceptance, metric reference refresh, station registration or three-frame qualification is produced.'],no_ros_connections=True,motion_commands_sent=False)
 out.write_text(json.dumps(plain(report),indent=2)+'\n');print(json.dumps(plain(dict(path=str(out),sha256=sha(out),premotion={k:v for k,v in report.items()if k.startswith('premotion_')},first=rows[0]))))
if __name__=='__main__':main()
