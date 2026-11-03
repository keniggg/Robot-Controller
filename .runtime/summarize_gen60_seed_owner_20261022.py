"""Frozen output visualization and provenance; no recomputation of seed decisions."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
from pathlib import Path
import hashlib,json
import cv2
import numpy as np
W=Path(__file__).resolve().parents[1]
S=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/actual_attempt_r1')
O=S/'source_owner_diagnosis_r1';O.mkdir(exist_ok=False)
cv2.setNumThreads(1)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def plain(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 if isinstance(x,dict):return {str(k):plain(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [plain(v) for v in x]
 return x
base=json.loads((S/'source_owner_replay_r2/result.json').read_text())
cf=json.loads((S/'extra2_counterfactual_r2/result.json').read_text())
geo=dict(np.load(S/'extra2_counterfactual_r2/owner_geometry.npz'))
frame=dict(np.load(S/'extra2_counterfactual_r2/frame.npz'))
seed=dict(np.load(S/'source_owner_replay_r2/seed_05.npz'))
extra=geo['extra_masks'][1].astype(bool);core=np.zeros(extra.shape,bool)
yx=base['seed_calls'][4]['crop_yx'];sl=tuple(slice(*p) for p in yx);core[sl]=seed['foreground']
assert np.array_equal(extra[sl],seed['seed']) and int(extra.sum())==96 and int(core.sum())==18
final=geo['attempted_refined_mask']>0;foreign=geo['rejected_foreign_component'].astype(bool)
yy,xx=np.nonzero(extra);xyz=seed['points'][seed['seed']];T=frame['T'];base_xyz=xyz@T[:3,:3].T+T[:3,3]
xy=np.stack([xx,yy],axis=-1);order=np.linspace(0,len(xy)-1,7).astype(int)
q=[0,10,50,90,100]
report=dict(scope='Exact failure decomposition and diagnostic-only evidence-omission counterfactual; no accepted frame or grasp',source_stamp_ns=1792649243704485416,online_result='TARGET_LOST: perception_failure: rgb_seed_core_too_small',failed_stage='raw_source_component_evidence extra region 2',original_counts=base['seed_calls'][4],pixel_bbox_inclusive_xy=[int(xx.min()),int(yy.min()),int(xx.max()),int(yy.max())],example_original_xy=xy[order],camera_point_bounds_m=[xyz.min(0),xyz.max(0)],base_point_bounds_m=[base_xyz.min(0),base_xyz.max(0)],support_height_percentiles=dict(zip(map(str,q),np.percentile(seed['heights'][seed['seed']],q))),relationships=dict(extra_intersects_native_union=int(np.count_nonzero(extra&geo['group'])),extra_in_raw_owner=int(np.count_nonzero(extra&geo['raw_owner'])),raw_owner_points=int(geo['raw_owner'].sum()),native_union_points=int(geo['group'].sum()),raw_only_points=int(geo['raw_only'].sum()),extra_regions=[int(a.sum()) for a in geo['extra_masks']],counterfactual_refined_extra_intersection=int(np.count_nonzero(final&extra)),counterfactual_refined_extra_core_intersection=int(np.count_nonzero(final&core)),counterfactual_refined_foreign_intersection=int(np.count_nonzero(final&foreign)),rejected_foreign_component_points=int(foreign.sum()),foreign_is_separate_from_original_owner=not bool(np.any(foreign&geo['raw_owner']))),counterfactual=dict(only_change=cf['counterfactual_bypasses'],next_original_error=cf['owner_error'],appearance_diagnostic=cf['rejected_union_fixed_appearance_diagnostic'],motion_authorized=False,not_an_acceptable_fix=True),interpretation_limits=['Raw-only means omitted by original candidate acceleration/morphology; it does not mean unmeasured or irrelevant.','The extra2 region belongs to the same original raw source component but lies outside native union. This alone cannot distinguish a target rim from a connected competing object.','No additional error was bypassed after the extra2 exception. Foreign component overlap is a real subsequent original rejection in this diagnostic call.','Fixed-template appearance was evaluated only as a diagnostic after ownership rejection; it independently remains below IoU .85.','Image annotations show recorded RGB and exact computed masks. Visual resemblance is not ownership proof.'],production_modified=False,live_ROS_or_network_used=False)
# Quantify first original primary against the same extra region independently.
report['relationships']['original_primary_refined_extra_intersection']=int(np.count_nonzero((frame['primary_mask']>0)&extra))
report['relationships']['original_primary_refined_foreign_intersection']=int(np.count_nonzero((frame['primary_mask']>0)&foreign))
# Draw exact masks on original BGR only for diagnosis, preserve original separately.
rgb=frame['color'];cv2.imwrite(str(O/'exact_rgb.png'),rgb)
overlay=rgb.copy()
def contour(mask,color):
 c,_=cv2.findContours(mask.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(overlay,c,-1,color,1)
for i,(m,c) in enumerate(zip(geo['native_masks'],[(0,255,0),(255,255,0),(0,255,255)])):
 contour(m>0,c)
 yy1,xx1=np.nonzero(m);cv2.putText(overlay,'N%d'%(i+1),(int(xx1.mean()),int(yy1.mean())),cv2.FONT_HERSHEY_SIMPLEX,.35,c,1)
contour(geo['extra_masks'][0],(0,165,255));contour(extra,(0,0,255));contour(foreign,(255,0,255))
overlay[core]=(255,255,255)
cv2.putText(overlay,'E2:96 / core18',(int(xx.min())-12,int(yy.max())+16),cv2.FONT_HERSHEY_SIMPLEX,.38,(0,0,255),1)
cv2.imwrite(str(O/'mask_overlay.png'),overlay)
cv2.imwrite(str(O/'target_crop.png'),cv2.resize(overlay[330:475,150:400],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
# Independent panel for attempted union result: avoid covering target with all contour classes.
u=rgb.copy();cs,_=cv2.findContours(final.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cv2.drawContours(u,cs,-1,(255,255,255),1);u[final&foreign]=(255,0,255);u[extra]=(0,0,255)
cv2.imwrite(str(O/'counterfactual_union_crop.png'),cv2.resize(u[330:475,150:400],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
report['visualization_legend']={'N1':'green native component','N2':'cyan native component','N3':'yellow native component','E1':'orange raw-only component','E2':'red failing raw-only component','core18':'white points inside E2','foreign':'magenta separate raw source component; counterfactual union crop colors only actual overlap'}
# Seal exact source bytes for both successful diagnostic runs and pin dependencies.
for rd,script in [('source_owner_replay_r2','replay_gen60_failed_fragment_owner_20261022.py'),('extra2_counterfactual_r2','counterfactual_gen60_extra_core_20261022.py')]:
 src=W/'.runtime'/script;out=S/rd/'probe_source.py';out.write_bytes(src.read_bytes())
 assert sha(out)==json.loads((S/rd/'result.json').read_text())['helper']['sha256']
report['provenance']={'baseline_result':{'path':str(S/'source_owner_replay_r2/result.json'),'sha256':sha(S/'source_owner_replay_r2/result.json')},'counterfactual_result':{'path':str(S/'extra2_counterfactual_r2/result.json'),'sha256':sha(S/'extra2_counterfactual_r2/result.json')},'input_manifest':{'path':str(S/'failed_seed_inputs_r2/manifest.json'),'sha256':sha(S/'failed_seed_inputs_r2/manifest.json')},'sdk':cf['sdk'],'baseline_source_sha256':base['source_sha256'],'counterfactual_source_sha256':cf['source_sha256'],'baseline_command_environment':{'PYTHONPATH':str(W/'.runtime/realsense-2.56.5'),'numeric_threads':1,'python_bytecode':False},'helper':{'path':str(Path(__file__).resolve()),'sha256':sha(__file__)}}
assert base['source_sha256']==cf['source_sha256']
report['artifacts']={p.name:sha(p) for p in O.iterdir() if p.is_file()}
(O/'result.json').write_text(json.dumps(plain(report),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
print(json.dumps(plain({k:report[k] for k in ['pixel_bbox_inclusive_xy','base_point_bounds_m','support_height_percentiles','relationships']}),indent=2))
print('result_sha256',sha(O/'result.json'))
