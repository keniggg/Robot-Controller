"""Diagnostic: original Hough offsets, grouped evidence; unchanged full outer gates."""
import hashlib,json,math,os,sys
from itertools import product
from pathlib import Path
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
D=E/'ros_log/full_latest_gen36_restart_20261017T012037Z'
O=D/'verification/gen37_box_source_replay_r1'; I=D/'verification/gen37_actual_attempt1_source_images_r1'
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
c=read(D/'gen37_runtime/gen37_grasp_scene_r2/camera_configuration.json');K=np.array([c[k] for k in ('fx','fy','cx','cy')]);scale=c['depth_scale']
T=np.asarray(a['snapshot_transform']['T_base_optical']);stamp=a['snapshot_stamp_ns'];camera=T[:3,3]
color=cv2.imread(str(I/(str(stamp)+'_color.png')));mask=cv2.imread(str(I/(str(stamp)+'_mask.png')),0)>0;depth=np.load(I/(str(stamp)+'_depth.npy'))
state=np.load(O/'exact_current_top_fit.npz');origin=state['origin'];tn=state['tn'];A=state['axes'];top=state['top']
current=read(O/'current_qualification_actual.json')['audit'];rms=current['top_plane_rms_m'];uvtop=state['uv'][state['keep']]
pixel=float(np.median(depth[uvtop[:,1],uvtop[:,0]])*scale)/min(K[:2])/current['camera_top_cosine']
captures=[]
def profile(frame,event,arg):
 if event=='return' and frame.f_code.co_name=='_top_side_proposals' and frame.f_code.co_filename==outer.__file__:captures.append(dict(frame.f_locals))
sys.setprofile(profile);outer._top_side_proposals(color,mask,depth,scale,T,K,origin,tn,A,pixel);sys.setprofile(None)
s=captures[-1];choices=s['choices'];lower,upper=s['lower'],s['upper'];original_counts=[len(x) for x in choices]
def on_plane(uv):
 ray=np.column_stack(((uv[:,0]-K[2])/K[0],(uv[:,1]-K[3])/K[1],np.ones(len(uv))))@T[:3,:3].T
 return camera+ray*(float((origin-camera)@tn)/(ray@tn))[:,None]
analysis=read(O/'current_depth_axes.json');fragments=[np.asarray(analysis['hough_lines'][j]['line_pixels_xy']) for j in (3,4)]
checks=[];group=[];evidence=[]
for pixels in fragments:
 metric=on_plane(pixels)@A[:,:2];group.append(metric);delta=metric[1]-metric[0];offset=float(metric[:,1].mean())
 overlap=min(float(metric[:,0].max()),upper[0])-max(float(metric[:,0].min()),lower[0])
 v=pixels[1]-pixels[0];outward=np.array([v[1],-v[0]])/np.linalg.norm(v)
 if float((on_plane((pixels.mean(0)+outward)[None])-on_plane(pixels.mean(0)[None]))@A[:,1])<0:outward*=-1
 bands=[]
 for direction in (-1.,1.):
  px=np.rint(pixels[0]+np.outer(np.linspace(.12,.88,21),v)+direction*3.*outward).astype(int)
  z=depth[px[:,1],px[:,0]].astype(float)*scale;valid=np.isfinite(z)&(z>0.)
  p=np.column_stack(((px[:,0]-K[2])*z/K[0],(px[:,1]-K[3])*z/K[1],z))@T[:3,:3].T+camera;res=(p-origin)@tn
  bands.append(dict(valid_fraction=float(valid.mean()),top_fraction=float(np.count_nonzero(valid&(np.abs(res)<.0015)))/max(1,int(valid.sum())),above_fraction=float(np.mean(valid&(res>.0025)))))
 inside,outside=bands;resolved=inside['valid_fraction']>=.60 and inside['top_fraction']>=.70 and outside['valid_fraction']>=.60 and outside['top_fraction']<=.30
 bandpass=not(inside['valid_fraction']>=.60 and inside['top_fraction']<.70) and outside['above_fraction']<=.30
 ev=dict(line_pixels_xy=pixels.tolist(),inside=inside,outside=outside);evidence.append(ev)
 checks.append(dict(original_offset_m=offset,original_overlap_m=overlap,original_span_m=abs(delta[0]),angle_deg=math.degrees(math.atan2(abs(delta[1]),abs(delta[0]))),original_angle_pass=bool(abs(delta[1])<=math.tan(math.radians(8.))*abs(delta[0])),original_offset_pass=bool(abs(offset-upper[1])<=max(.012,8.*pixel)),depth_bands_pass=bool(bandpass),resolved=bool(resolved)))
 if bandpass:choices[2].append((offset,overlap,ev,bool(resolved)))
group=np.concatenate(group);group_span=float(np.ptp(group[:,0]));group_overlap=min(float(group[:,0].max()),upper[0])-max(float(group[:,0].min()),lower[0]);required=.35*(upper[0]-lower[0])
# Both original offsets are retained as distinct explanations. The only
# counterfactual is that fixed-angle collinear fragment groups, rather than
# each fragment alone, satisfy length/overlap proposal eligibility. The full
# final inference below still checks actual current RGB edge/corner arms,
# all raw depth, every competing response and the original interval budget.
assert all(c['original_angle_pass'] and c['original_offset_pass'] and c['depth_bands_pass'] for c in checks)
assert group_span>=required and group_overlap>=required
assert analysis['mask_envelope_distances'][2]['maximum_distance_px']<=math.sqrt(8.)
results=[];final_captures=[]
def final_profile(frame,event,arg):
 if event=='return' and frame.f_code.co_name=='_infer_outer_rgb_rectangle' and frame.f_code.co_filename==outer.__file__:final_captures.append(dict(frame.f_locals))
for selected in product(*choices):
 lo=np.array([selected[3][0],selected[0][0]]);hi=np.array([selected[1][0],selected[2][0]])
 sys.setprofile(final_profile)
 result=outer._infer_outer_rgb_rectangle(color,mask,depth,scale,T,K,origin,tn,A,pixel,rms,proposal=(lo,hi),resolved_sides=[r[3] for r in selected])
 sys.setprofile(None);final=final_captures[-1]
 yy,xx=np.where(mask);zz=depth[yy,xx].astype(float)*scale;known=np.isfinite(zz)&(zz>0.)
 measured=np.column_stack(((xx-K[2])*zz/K[0],(yy-K[3])*zz/K[1],zz))@T[:3,:3].T+camera
 possible_top=~known|(np.abs((measured-origin)@tn)<=.0015);pixels_all=np.column_stack((xx,yy))[possible_top];known_top=known[possible_top]
 local=final['local_silhouette'];extremes=[int(np.argmin(local[:,1])),int(np.argmax(local[:,0])),int(np.argmax(local[:,1])),int(np.argmin(local[:,0]))]
 causes=[dict(edge=j,extension_m=float(final['extensions'][j]),source_pixel=pixels_all[index],source_has_depth=bool(known_top[index]),source_depth_raw=int(depth[pixels_all[index,1],pixels_all[index,0]])) for j,index in enumerate(extremes)]
 results.append(dict(edge_offsets_m=[r[0] for r in selected],selected_raw_line_evidence=[r[2] for r in selected],resolved_sides=[r[3] for r in selected],nominal_size_xy_m=hi-lo,mask_extension_causes=causes,unchanged_final_outer_inference=result))
assert all(sha(W/p)==v for p,v in source_before.items())
boundary=O/'boundary.jsonl';attempts=boundary.read_text().splitlines() if boundary.exists() else [];assert not attempts
out=dict(diagnostic_only=True,motion_authorized=False,passed=True,original_source_arrays_unchanged=True,original_choices_per_edge=original_counts,diagnostic_choices_per_edge=[len(x) for x in choices],source_count=len(source_before),source_unchanged=True,raw_original_fragments=fragments,fragment_checks=checks,group_span_m=group_span,group_overlap_m=group_overlap,original_35pct_requirement_m=required,whole_edge_original_canny_max_distance_px=analysis['mask_envelope_distances'][2]['maximum_distance_px'],candidate_family=results,all_candidates_rejected=all(not r['unchanged_final_outer_inference']['ok'] for r in results),qualified_model=None,aperture_budget_available=False,external_boundary_attempts=attempts,script_sha256=sha(__file__),conclusion='Fragment-per-line 35% eligibility is a real earlier rejection but is not the only blocker; all original-offset candidate explanations still pass through unchanged final interval gates. No model/U/aperture is issued for rejected explanations.')
(O/'original_fragment_family_diagnostic.json').write_text(json.dumps(out,ensure_ascii=False,indent=2,default=serial)+'\n')
print(json.dumps(dict(group_span_m=group_span,required_m=required,candidate_count=len(results),ok=[r['unchanged_final_outer_inference']['ok'] for r in results],reasons=[r['unchanged_final_outer_inference'].get('reason') for r in results])))
