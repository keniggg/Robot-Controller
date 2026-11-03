"""Diagnostic only: explain original Hough fragment length failure, unchanged gates."""
import hashlib,json,math,os,sys
from itertools import product
from pathlib import Path
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
D=E/'ros_log/full_latest_gen36_restart_20261017T012037Z'
O=D/'verification/gen37_box_source_replay_r1'
I=D/'verification/gen37_actual_attempt1_source_images_r1'
G=E/'grasp_records/20261017T032113.476278Z_a84cc89fc53e4c2b82680749d78b9b9f'
assert Path('/media/zhuyupei/SanStand').is_mount() and 'sitecustomize' in sys.modules
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
import cv2,numpy as np
from alicia_flexible_grasp.vision import outer_rgb_rectangle as outer
cv2.setNumThreads(1)
read=lambda p:json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def serial(x):return x.tolist() if hasattr(x,'tolist') else x.item()
source_before=read(O/'source_sha256_before.json');assert all(sha(W/p)==v for p,v in source_before.items())
a=read(G/'keyframes_audits/0e5ebbd3c562cfab2617b338045f554ffc0920a23626fbc2a49cc474cc9d798d.json')
c=read(D/'gen37_runtime/gen37_grasp_scene_r2/camera_configuration.json'); K=np.array([c[k] for k in ('fx','fy','cx','cy')]); scale=c['depth_scale']
T=np.asarray(a['snapshot_transform']['T_base_optical']);stamp=a['snapshot_stamp_ns']
color=cv2.imread(str(I/(str(stamp)+'_color.png')));mask=cv2.imread(str(I/(str(stamp)+'_mask.png')),0)>0;depth=np.load(I/(str(stamp)+'_depth.npy'))
state=np.load(O/'exact_current_top_fit.npz');origin=state['origin'];tn=state['tn'];A=state['axes'];top=state['top'];pixel=a['remote_diagnostics']['tabletop_geometry']['regular_box_inference']['top_height_m']
current=read(O/'current_qualification_actual.json')['audit'];rms=current['top_plane_rms_m'];pixel=float(np.median(depth[state['uv'][state['keep'],1],state['uv'][state['keep'],0]])*scale)/min(K[:2])/current['camera_top_cosine']
captures=[]
def profile(frame,event,arg):
 if event=='return' and frame.f_code.co_name=='_top_side_proposals' and frame.f_code.co_filename==outer.__file__:captures.append(dict(frame.f_locals))
sys.setprofile(profile);outer._top_side_proposals(color,mask,depth,scale,T,K,origin,tn,A,pixel);sys.setprofile(None)
s=captures[-1];choices=s['choices'];camera=T[:3,3]
def on_plane(uv):
 ray=np.column_stack(((uv[:,0]-K[2])/K[0],(uv[:,1]-K[3])/K[1],np.ones(len(uv))))@T[:3,:3].T
 return camera+ray*(float((origin-camera)@tn)/(ray@tn))[:,None]
# Select only the two raw Hough fragments already listed as the true bottom
# edge by exact-source inspection. This manual diagnostic is not an algorithm
# or a permitted production path. No RGB/depth/mask values are changed.
fragments=[np.array([[258.,438.],[301.,438.]]),np.array([[319.,439.],[362.,439.]])]
pix=np.concatenate(fragments);center=pix.mean(0);_,_,vt=np.linalg.svd(pix-center,full_matrices=False);along=vt[0]
t=(pix-center)@along;pixels=np.array([center+along*t.min(),center+along*t.max()]);metric=on_plane(pixels)@A[:,:2];delta=metric[1]-metric[0]
lower,upper=s['lower'],s['upper'];i=2;across=1;al=0;offset=float(metric[:,across].mean());overlap=min(float(metric[:,al].max()),upper[al])-max(float(metric[:,al].min()),lower[al])
v=pixels[1]-pixels[0];outward=np.array([v[1],-v[0]])/np.linalg.norm(v)
if float((on_plane((pixels.mean(0)+outward)[None])-on_plane(pixels.mean(0)[None]))@A[:,across])<0:outward*=-1
bands=[]
for direction in (-1.,1.):
 px=np.rint(pixels[0]+np.outer(np.linspace(.12,.88,21),v)+direction*3.*outward).astype(int);z=depth[px[:,1],px[:,0]].astype(float)*scale;valid=np.isfinite(z)&(z>0.)
 p=np.column_stack(((px[:,0]-K[2])*z/K[0],(px[:,1]-K[3])*z/K[1],z))@T[:3,:3].T+camera;res=(p-origin)@tn
 bands.append(dict(valid_fraction=float(valid.mean()),top_fraction=float(np.count_nonzero(valid&(np.abs(res)<.0015)))/max(1,int(valid.sum())),above_fraction=float(np.mean(valid&(res>.0025)))))
inside,outside=bands;resolved=inside['valid_fraction']>=.60 and inside['top_fraction']>=.70 and outside['valid_fraction']>=.60 and outside['top_fraction']<=.30
bands_pass=not (inside['valid_fraction']>=.60 and inside['top_fraction']<.70) and outside['above_fraction']<=.30
merged=dict(line_pixels_xy=pixels.tolist(),inside=inside,outside=outside)
gates=dict(length=abs(delta[0])>=.35*(upper[0]-lower[0]),angle=abs(delta[1])<=math.tan(math.radians(8.))*abs(delta[0]),offset=abs(offset-upper[1])<=max(.012,8.*pixel),overlap=overlap>=.35*(upper[0]-lower[0]),depth=bands_pass)
choices[2]=[(offset,overlap,merged,bool(resolved))];results=[]
for selected in product(*choices):
 lo=np.array([selected[3][0],selected[0][0]]);hi=np.array([selected[1][0],selected[2][0]])
 result=outer._infer_outer_rgb_rectangle(color,mask,depth,scale,T,K,origin,tn,A,pixel,rms,proposal=(lo,hi),resolved_sides=[r[3] for r in selected]);results.append(result)
assert all(sha(W/p)==v for p,v in source_before.items())
out=dict(diagnostic_only=True,motion_authorized=False,original_pixel_arrays_unchanged=True,original_choice_count=[len(x) for x in s['choices'][:2]]+[0,len(s['choices'][3])],raw_hough_fragments=fragments,manual_merged_proposal=merged,original_gates=gates,original_35pct_length_requirement_m=.35*(upper[0]-lower[0]),merged_span_m=abs(delta[0]),merged_overlap_m=overlap,existing_outer_revalidations=results,source_unchanged=True,source_count=len(source_before),script_sha256=sha(__file__),limitation='Manual grouping is only cause isolation; production implementation must enumerate all compatible competing fragments and independently verify full current RGB line/corner arms and raw depth, then retain the complete uncertainty family. This does not permit truncating U, dropping data or increasing angle/search limits.')
(O/'fragment_grouping_diagnostic.json').write_text(json.dumps(out,ensure_ascii=False,indent=2,default=serial)+'\n');print(json.dumps(dict(gates=gates,result_codes=[r.get('code',r.get('reason','ok')) for r in results],ok=[r['ok'] for r in results])))
