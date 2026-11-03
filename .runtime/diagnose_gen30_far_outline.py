"""Inspect retained far RGB outlines; no rerun of RGB ownership or changed gates."""
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib,json,os,sys
W=Path(__file__).resolve().parents[1];M=Path('/media/zhuyupei/SanStand')
N=M/'实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
I=N/'gen29_attempt1_shadow_frames';F=N/'gen30_boundary_contract_r1';O=N/'gen30_far_outline_diagnostic_r1'
assert M.is_mount() and not O.exists() and os.environ['ROS_MASTER_URI']=='http://127.0.0.1:1'
sys.path[:0]=[str(W/'src/alicia_grasp_modes/src')]
import cv2,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from alicia_grasp_modes import tabletop as table,rgb_instance as rgb,measured_mask_seeds as seeds
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def serial(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
inputs={}
def read(p):inputs[str(p)]=sha(p);return json.loads(Path(p).read_text())
meta=read(I/'mask_contract_review_r2/result.json');cfg=table.Config(**meta['effective_config'])
receipt=read(F/'result.json');sources={str(m.__file__):sha(m.__file__) for m in (table,rgb,seeds)}
O.mkdir();rows=[];fig,axes=plt.subplots(3,3,figsize=(15,14),constrained_layout=True)
for n,(prior,old) in enumerate(zip(meta['rows'][1:],receipt['rows'])):
 ns=prior['source_stamp_ns'];assert ns==old['source_stamp_ns'];p=I/f'source_{ns}'
 status=read(p/'detector_status.json')['payload'];inputs[str(p/'raw_arrays.npz')]=sha(p/'raw_arrays.npz')
 a=dict(np.load(p/'raw_arrays.npz',allow_pickle=False));depth=a['depth_raw'].astype(np.float32)*float(a['depth_scale_m'])
 K=a['intrinsics'];T=a['T_base_camera_link'];color=a['color_bgr']
 anchor=NS(position_base=np.asarray(status['anchor_position_base']),extent_m=np.asarray(status['anchor_extent_m']),support_footprint_base=np.asarray(status['reference_support_footprint_base']),metrics={'median_height_m':status['reference_object_height_m']})
 last=NS(position_base=np.asarray(prior['prior_position_used']));locked=np.asarray(status['reference_support_plane_base'])
 result=table.segment_with_support_fallback(depth,K,T,cfg,anchor,last,locked,prefer_current_support=True)
 ownerpath=N/'gen30_current_fragment_owner_r2'/f'{ns}_owner.png';inputs[str(ownerpath)]=sha(ownerpath)
 whole=cv2.imread(str(ownerpath),0)>0
 points=table._project(depth,K);plane_camera=T.T@result.plane_base
 heights=np.einsum('ijk,k->ij',points,plane_camera[:3])+plane_camera[3]
 valid=np.isfinite(depth)&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
 normals,reliable=seeds.local_surface_normals(points,depth,valid)
 cosine=np.abs(np.einsum('ijk,k->ij',normals,plane_camera[:3]))
 candidates=[]
 for oldfar in old['far_fragment_counterevidence']:
  index=oldfar['candidate_index'];path=F/f'{ns}_far_{index}.png';inputs[str(path)]=sha(path)
  mask=cv2.imread(str(path),0)>0;outside=mask&~whole;yy,xx=np.nonzero(outside)
  measured=mask&valid&(heights>=cfg.foreground_min_m)&(heights<=cfg.foreground_max_m)
  xyz=points[measured].astype(float);center_camera=np.median(xyz,axis=0);center_base=(T@np.r_[center_camera,1.])[:3]
  centered=xyz-xyz.mean(axis=0);_,_,vh=np.linalg.svd(centered,full_matrices=False)
  extent=np.sort(np.percentile(centered@vh.T,98,axis=0)-np.percentile(centered@vh.T,2,axis=0))[::-1]
  distance=float(np.linalg.norm(center_base-anchor.position_base));step=float(np.linalg.norm(center_base-last.position_base))
  ratios=extent[:2]/np.maximum(anchor.extent_m[:2],.003)
  shift=float(np.linalg.norm(center_base-result.candidates[index].position_base))
  rows_pixels=[dict(y=int(y),x=int(x),raw_depth_m=float(depth[y,x]) if valid[y,x] else None,
   height_m=float(heights[y,x]) if valid[y,x] else None,normal_reliable=bool(reliable[y,x]),
   normal_support_cosine=float(cosine[y,x]) if reliable[y,x] else None,
   rgb_bgr=color[y,x].tolist(),in_native_seed=bool(result.candidates[index].mask[y,x])) for y,x in zip(yy,xx)]
  count,labels,stats,_=cv2.connectedComponentsWithStats(outside.astype(np.uint8),connectivity=8)
  components=[dict(pixels=int(stats[i,cv2.CC_STAT_AREA]),bbox_xywh=stats[i,:4].tolist()) for i in range(1,count)]
  audit=dict(candidate_index=index,original_rgb_output_pixels=int(mask.sum()),measured_points=len(xyz),
   original_final_first_rejection=('rgb_instance_shifted_from_measured_target' if shift>cfg.association_distance_m else None),
   refined_center_shift_from_raw_seed_m=shift,original_shift_limit_m=cfg.association_distance_m,
   downstream_diagnostic_only=True,extent_m=extent,max_surface_extent_pass=bool(extent[0]<=cfg.max_surface_extent_m),
   minimum_measured_points_pass=bool(len(xyz)>=cfg.min_object_points),final_center_base=center_base,
   final_anchor_distance_m=distance,final_last_distance_m=step,final_anchor_extent_ratios=ratios,
   final_matching_if_shift_gate_ignored=bool(distance<=cfg.association_distance_m and step<=cfg.association_distance_m and np.all((ratios>=.5)&(ratios<=2.))),
   outside_owner_pixels=int(outside.sum()),outside_reliable_side_pixels=int(np.count_nonzero(outside&valid&reliable&(cosine<=np.sqrt(.5)))),
   outside_reliable_parallel_pixels=int(np.count_nonzero(outside&valid&reliable&(cosine>np.sqrt(.5)))),
   outside_valid_unreliable_normal_pixels=int(np.count_nonzero(outside&valid&~reliable)),
   connected_difference_regions=components,difference_pixels=rows_pixels)
  candidates.append(audit)
  if index==1:
   yall,xall=np.nonzero(mask|whole);x0,x1=int(xall.min())-12,int(xall.max())+13;y0,y1=int(yall.min())-12,int(yall.max())+13
   extent_plot=(x0-.5,x1-.5,y1-.5,y0-.5);crop=np.s_[y0:y1,x0:x1]
   ax=axes[n,0];ax.imshow(color[crop][:,:,::-1],extent=extent_plot,interpolation='nearest')
   ax.contour(whole.astype(float),[.5],colors=['cyan'],linewidths=.65)
   ax.contour(mask.astype(float),[.5],colors=['magenta'],linewidths=.65)
   for select,colour,label in [(outside&valid&(heights>=cfg.foreground_min_m),'lime','depth >=4mm'),(outside&valid&(heights<cfg.foreground_min_m),'yellow','depth <4mm'),(outside&~valid,'deepskyblue','unknown')]:
    y,x=np.nonzero(select);ax.scatter(x,y,s=9,c=colour,marker='s',label=label)
   ax.set_title(f'{ns}\nRGB: cyan owner / magenta far #1')
   ax.legend(loc='upper right',fontsize=7)
   ax=axes[n,1];im=ax.imshow(np.where(valid[crop],heights[crop]*1000,np.nan),extent=extent_plot,vmin=0,vmax=30,cmap='viridis',interpolation='nearest')
   ax.scatter(xx,yy,s=12,facecolors='none',edgecolors='red',linewidths=.8);ax.set_title(f'Current support height (mm); {outside.sum()} external pixels')
   fig.colorbar(im,ax=ax,shrink=.7)
   ax=axes[n,2];im=ax.imshow(np.where((valid&reliable)[crop],cosine[crop],np.nan),extent=extent_plot,vmin=0,vmax=1,cmap='plasma',interpolation='nearest')
   ax.scatter(xx,yy,s=12,facecolors='none',edgecolors='lime',linewidths=.8);ax.set_title('Reliable normal cos(support); white = unproven')
   fig.colorbar(im,ax=ax,shrink=.7)
   for ax in axes[n]:ax.set_xlim(x0,x1);ax.set_ylim(y1,y0);ax.set_aspect('equal')
 rows.append(dict(source_stamp_ns=ns,candidates=candidates))
fig.suptitle('Finite current-source evidence: differences lie along target perimeter; no new admission',fontsize=14)
fig.savefig(O/'far_outline_evidence.png',dpi=160);plt.close(fig)
assert all(sha(p)==s for p,s in sources.items()) and all(sha(p)==s for p,s in inputs.items())
report=dict(rows=rows,input_sha256=inputs,source_sha256=sources,source_unchanged=True,script_sha256=sha(__file__),
 no_rgb_recovery_rerun=True,no_ros_calls=True,no_motion=True,production_deployed=False,
 original_geometry_gate_not_bypassed=True,diagnostic_after_first_rejection_only=True,
 figure=str(O/'far_outline_evidence.png'))
(O/'result.json').write_text(json.dumps(report,indent=2,default=serial,allow_nan=False)+'\n')
print(O/'result.json',sha(O/'result.json'));print(O/'far_outline_evidence.png')
for row in rows:
 print(row['source_stamp_ns'],[{k:v for k,v in c.items() if k not in ('difference_pixels','connected_difference_regions')} for c in row['candidates']])
