"""Scientific diagnostic panels from sealed depth/mask/RGB, no control calls."""
from pathlib import Path
import hashlib,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen7_handoff_analysis/contact_analysis/depth_filter_pixels')
d=np.load(p/'snapshot_2_pixels.npz');mask=d['mask_before_erosion']>0
v,u=np.indices(mask.shape);z=d['depth_raw'].astype(float)*float(d['depth_scale']);fx,fy,cx,cy=d['intrinsics'];T=d['T_base_optical']
xyz=np.stack(((u-cx)*z/fx,(v-cy)*z/fy,z),axis=-1)@T[:3,:3].T+T[:3,3]
h=(xyz@d['support_normal_base']+float(d['support_offset_m']))*1000
h[z==0]=np.nan
my,mx=np.where(mask);x0=max(0,int(mx.min())-30);x1=min(mask.shape[1],int(mx.max())+31);y0=max(0,int(my.min())-30);y1=min(mask.shape[0],int(my.max())+31)
crop=np.s_[y0:y1,x0:x1]
fig,axes=plt.subplots(1,3,figsize=(13,5),constrained_layout=True)
rgb=d['color_bgr'][...,::-1]
axes[0].imshow(rgb[crop]);axes[0].contour(mask[crop],levels=[.5],colors=['lime'],linewidths=.8);axes[0].set_title('Recorded RGB + published mask boundary')
im=axes[1].imshow(h[crop],vmin=0,vmax=25,cmap='viridis');axes[1].contour(mask[crop],levels=[.5],colors=['red'],linewidths=.8);axes[1].set_title('Raw depth: height above original support (mm)')
fig.colorbar(im,ax=axes[1],shrink=.65)
marked=rgb[crop].copy();low=(h[crop]>4)&(h[crop]<16)&~mask[crop];marked[low]=[255,0,255]
axes[2].imshow(marked);axes[2].contour(mask[crop],levels=[.5],colors=['lime'],linewidths=.8);axes[2].set_title('Magenta: 4-16 mm raw pixels outside mask')
for ax in axes:ax.set_xlabel('crop pixels');ax.set_ylabel('crop pixels')
fig.suptitle('GEN7 source 1791601361780489206; contextual pixels are NOT authorized target geometry',fontsize=11)
out=p/'source_1791601361780489206_pixel_panels.png';fig.savefig(out,dpi=150);plt.close(fig)
r=dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),roi_xyxy=[x0,y0,x1,y1],outside_mask_4_to_16mm_pixels=int(low.sum()),limitation='Context pixels may include support-edge depth artifacts or other objects; manual visual association alone does not authorize a target mask or grasp.')
with (p/'pixel_panel_metadata.json').open('x') as f:json.dump(r,f,indent=2);f.write('\n')
print(json.dumps(r,indent=2))
