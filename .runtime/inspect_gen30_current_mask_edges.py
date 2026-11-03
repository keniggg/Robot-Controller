"""Fixed visual diagnostic ROIs, never new target geometry or a segmentation seed."""
from pathlib import Path
import hashlib,json,os,sys
W=Path(__file__).resolve().parents[1];M=Path('/media/zhuyupei/SanStand')
C=M/'实验记录/Robot-Controller/ros_log/full_latest_gen29_restart_20261016T000728Z'
I=C/'verification/gen30_current_passive_initial_r1';O=C/'verification/gen30_current_mask_edges_r1'
assert M.is_mount() and not O.exists() and os.environ['ROS_MASTER_URI']=='http://127.0.0.1:1'
sys.path.insert(0,str(W/'src/alicia_grasp_modes/src'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from alicia_grasp_modes import measured_mask_seeds as seeds
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
inputs={str(I/'result.json'):sha(I/'result.json')};report=json.loads((I/'result.json').read_text())
# Chosen once by visually inspecting the saved RGB figure, before reading the
# depth in these regions. These are coarse pixel ROIs, not fitted 3-D corners.
corners={'upper_left':(289,198),'upper_right':(365,214),'front_right':(356,257),'front_left':(272,242)}
edge_roi=(259,232,267,248)
fig,axes=plt.subplots(3,2,figsize=(10,12),constrained_layout=True);rows=[];O.mkdir()
for i,row in enumerate(report['rows']):
 ns=row['source_stamp_ns'];p=I/f'{ns}_arrays.npz';inputs[str(p)]=sha(p);a=dict(np.load(p,allow_pickle=False))
 raw=a['depth_raw'];rgb=a['color_bgr'];mask=a['object_mask']>0;K=a['intrinsics'];T=a['T_base_camera_link'];depth=raw.astype(float)*float(a['depth_scale_m'])
 yy,xx=np.indices(depth.shape);points=np.stack((depth,-(xx-K[2])*depth/K[0],-(yy-K[3])*depth/K[1]),axis=-1)
 plane=T.T@np.array(row['segmentation']['plane_base']);heights=np.einsum('ijk,k->ij',points,plane[:3])+plane[3]
 valid=np.isfinite(depth)&(depth>=.03)&(depth<=2.);normals,reliable=seeds.local_surface_normals(points,depth,valid)
 cosine=np.abs(np.einsum('ijk,k->ij',normals,plane[:3]))
 x0,y0,x1,y1=edge_roi;roi=np.zeros(depth.shape,bool);roi[y0:y1,x0:x1]=True;selected=roi&mask
 edge=dict(mask_pixels=int(selected.sum()),valid_depth_pixels=int(np.count_nonzero(selected&valid)),
  foreground_above_4mm_pixels=int(np.count_nonzero(selected&valid&(heights>=.004))),
  locally_reliable_side_normal_pixels=int(np.count_nonzero(selected&valid&reliable&(cosine<=np.sqrt(.5)))),
  median_rgb_bgr=np.median(rgb[selected],axis=0).tolist() if np.any(selected) else None,
  measured_height_range_mm=[float(np.min(heights[selected&valid])*1000),float(np.max(heights[selected&valid])*1000)] if np.any(selected&valid) else None)
 corner_rows={}
 for name,(x,y) in corners.items():
  sl=np.s_[y-3:y+4,x-3:x+4];corner_rows[name]=dict(pixels=49,valid_depth_pixels=int(valid[sl].sum()),
   mask_pixels=int(mask[sl].sum()),valid_depth_in_mask=int(np.count_nonzero(valid[sl]&mask[sl])))
 rows.append(dict(source_stamp_ns=ns,left_red_table_roi=edge,visible_corner_depth_rois=corner_rows))
 crop=np.s_[185:282,250:380];extent=(249.5,379.5,281.5,184.5)
 axes[i,0].imshow(rgb[crop][:,:,::-1],extent=extent,interpolation='nearest');axes[i,0].contour(mask.astype(float),[.5],colors='lime',linewidths=.8)
 axes[i,1].imshow(np.where(valid[crop],heights[crop]*1000,np.nan),extent=extent,vmin=0,vmax=25,cmap='viridis',interpolation='nearest')
 for ax in axes[i]:
  for name,(x,y) in corners.items():
   ax.add_patch(Rectangle((x-3.5,y-3.5),7,7,facecolor='none',edgecolor='deepskyblue',linewidth=1.))
  ax.add_patch(Rectangle((x0-.5,y0-.5),x1-x0,y1-y0,facecolor='none',edgecolor='magenta',linewidth=1.4))
  ax.set_xlim(250,380);ax.set_ylim(282,185);ax.set_aspect('equal')
 axes[i,0].set_title(str(ns)+'\nFixed visual corner ROIs + red-table patch')
 axes[i,1].set_title('Raw height: white = no depth\nBlue boxes were selected from RGB only')
fig.savefig(O/'edge_roi_evidence.png',dpi=160);plt.close(fig)
out=dict(rows=rows,input_sha256=inputs,source_sha256={str(seeds.__file__):sha(seeds.__file__)},script_sha256=sha(__file__),
 manual_diagnostic_corner_centers_xy=corners,manual_diagnostic_left_table_roi_xyxy=edge_roi,
 scope='Pixel-level inspection only; no generated/edited mask, model, annotation ground truth, new selection or geometry authority.',
 no_segmentation_replay=True,no_box_trace=True,no_ros=True,no_motion=True)
(O/'result.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(rows));print(O/'result.json',sha(O/'result.json'))
