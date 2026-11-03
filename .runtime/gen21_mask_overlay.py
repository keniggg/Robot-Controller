"""Plot exact sealed image/mask measurements; never alters source frames."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
WORK=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORK/'src/alicia_grasp_modes/src'))
from alicia_grasp_modes.tabletop import _project
OUT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen21_analysis')
assert Path('/media/zhuyupei/SanStand').is_mount()
manifest=json.loads((OUT/'replay_inputs/manifest.json').read_text())
ab=json.loads((OUT/'crop_ab_replay_r1.json').read_text())
stamps=[1791636547753996610,1791636585362278461,1791636585622787237]
fig,axes=plt.subplots(3,3,figsize=(15,12));rows=[]
for i,ns in enumerate(stamps):
    entry=next(e for e in manifest['files'] if e['source_stamp_ns']==ns)
    f=dict(np.load(entry['path']));rgb=f['color'][...,::-1]
    if i==0:
        k=next(k for k in f if 'mask' in k);a=f[k]>0;b=a.copy()
    else:
        a=cv2.imread(str(OUT/('crop_ab_production_'+str(ns)+'_mask.png')),0)>0
        b=cv2.imread(str(OUT/('crop_ab_union48_counterfactual_'+str(ns)+'_mask.png')),0)>0
    yy,xx=np.nonzero(a|b);box=[max(0,int(xx.min())-18),max(0,int(yy.min())-18),min(rgb.shape[1],int(xx.max())+19),min(rgb.shape[0],int(yy.max())+19)]
    for j,ax in enumerate(axes[i]):
        ax.imshow(rgb);ax.set_xlim(box[0],box[2]);ax.set_ylim(box[3],box[1]);ax.set_xlabel('source pixel x');ax.set_ylabel('source pixel y')
        if j==0:ax.set_title('Exact RGB | '+str(ns))
        elif j==1:
            ax.contour(a,levels=[.5],colors=['cyan'],linewidths=.85)
            ax.set_title('Recorded reference mask' if i==0 else 'Production mask (cyan)')
        else:
            ax.contour(a,levels=[.5],colors=['cyan'],linewidths=.85)
            if i:
                ax.contour(b,levels=[.5],colors=['yellow'],linewidths=.65)
                layer=np.zeros((*a.shape,4));layer[b&~a]=[0,1,0,.8];layer[a&~b]=[1,0,1,.8];ax.imshow(layer)
                ax.set_title('Union48: green added / magenta removed')
            else:ax.set_title('Reference mask unchanged; cyan boundary')
    row=dict(source_ns=ns,input_npz=entry['path'],input_sha256=entry['sha256'],production_pixels=int(a.sum()),union48_pixels=int(b.sum()),crop_xyxy=box)
    if i:
        result=next(r for r in ab['results'] if r['variant']=='production' and r['source_ns']==ns)
        K=f['K'];intr=[K[0,0],K[1,1],K[0,2],K[1,2]] if K.shape==(3,3) else K
        depth=f['depth_raw'].astype(np.float32)*float(f['depth_scale'])
        points=_project(depth,intr);plane=np.asarray(result['metrics']['support_plane_base']);camplane=f['T_base_camera'].T@plane
        heights=points@camplane[:3]+camplane[3]
        for name,mask in [('added',b&~a),('removed',a&~b)]:
            components=[];count,labels,stats,centroids=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
            for c in sorted(range(1,count),key=lambda c:int(stats[c,cv2.CC_STAT_AREA]),reverse=True)[:10]:
                selected=labels==c;valid=np.isfinite(depth[selected])&(depth[selected]>0);h=heights[selected][valid]
                components.append(dict(pixels=int(selected.sum()),bbox_xywh=stats[c,:4].tolist(),centroid_xy=centroids[c].tolist(),
                    median_bgr=np.median(f['color'][selected],axis=0).tolist(),
                    valid_depth_pixels=int(valid.sum()),height_m_percentiles=np.percentile(h,[5,50,95]).tolist() if len(h) else []))
            ys,xs=np.nonzero(mask)
            row[name]=dict(pixels=int(mask.sum()),bbox_xyxy=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)] if len(xs) else None,largest_components=components)
    rows.append(row)
fig.suptitle('GEN21 sealed sources | cyan: original mask | yellow: crop-only counterfactual\nNo source pixel, depth, identity threshold or production file changed',fontsize=13)
fig.tight_layout(rect=[0,0,1,.95]);target=OUT/'reference_current_union48_masks.png';fig.savefig(str(target),dpi=150);plt.close(fig)
report=dict(figure=str(target),figure_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),reference_source_ns=stamps[0],
    source_manifest_sha256=hashlib.sha256((OUT/'replay_inputs/manifest.json').read_bytes()).hexdigest(),
    ab_report_sha256=hashlib.sha256((OUT/'crop_ab_replay_r1.json').read_bytes()).hexdigest(),rows=rows,
    meaning='Green/magenta are measured differences between two computed masks, not hand-labelled truth.')
(OUT/'reference_current_union48_masks.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'figure':str(target),'rows':rows}))
