"""Synthetic offline counterexample for spatial versus face-yaw qualification.

This does not test the enclosing source/geometry contracts or physical motion.
The only modified function exists in memory and is never installed or returned
as a qualified model. Original production source is immutable throughout.
"""
import ast, hashlib, inspect, json, math, os, sys
from pathlib import Path
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
D=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
O=D/'verification/gen37_interval_budget_review_r1'
assert Path('/media/zhuyupei/SanStand').is_mount() and 'sitecustomize' in sys.modules
assert os.environ['OFFLINE_BOUNDARY_LOG']==str(O/'boundary.jsonl')
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
release=json.loads((D/'verification/gen37_release_manifest.json').read_text())
before={p:sha(W/p) for p in release['source_sha256']}
assert before==release['source_sha256'] and len(before)==1005
sys.path.insert(0,str(W/'src/alicia_flexible_grasp_supervisor/src'))
import cv2,numpy as np
from alicia_flexible_grasp.vision import outer_rgb_rectangle as outer
cv2.setNumThreads(1)
source=inspect.getsource(outer._infer_outer_rgb_rectangle);tree=ast.parse(source);removed=0
for node in ast.walk(tree):
 if isinstance(node,ast.If) and ast.get_source_segment(source,node.test)=='proposal is not None and mask_extension > position_error+sampling':
  node.body=[ast.Pass()];removed+=1
assert removed==1
tree.body[0].name='_diagnostic_only';ast.fix_missing_locations(tree)
ns=dict(math=math,cv2=cv2,np=np,_trig_range=outer._trig_range)
exec(compile(tree,'<synthetic-diagnostic-only>','exec'),ns)

K=np.array([5000.,5000.,640.,480.]);T=np.diag([1.,-1.,-1.,1.]);T[:3,3]=[0.,0.,.52]
origin=np.array([0.,0.,.02]);tn=np.array([0.,0.,1.]);axes=np.eye(3)
phi=math.radians(10);R=np.array([[math.cos(phi),-math.sin(phi)],[math.sin(phi),math.cos(phi)]])
signs=np.array([[-1.,-1.],[1.,-1.],[1.,1.],[-1.,1.]])
true_size=np.array([.052,.035]);print_size=np.array([.044,.027])
corners=signs*true_size/2@R.T
pixels=np.column_stack((corners[:,0]*10000+640,-corners[:,1]*10000+480))
mask=np.zeros((960,1280),np.uint8);cv2.fillConvexPoly(mask,np.rint(pixels).astype(np.int32),1)
color=np.full((960,1280,3),128,np.uint8);color[mask>0]=(0,150,134)
assert np.all(cv2.cvtColor(color,cv2.COLOR_BGR2GRAY)==128)
cv2.rectangle(color,(420,345),(860,615),(0,0,0),1)
depth=np.full((960,1280),5200,np.uint16);depth[mask>0]=0
depth[345:616,420:861]=5000
scale=.0001;pixel=.0001;rms=0.
lo,hi=-print_size/2,print_size/2
kwargs=dict(proposal=(lo,hi),resolved_sides=[False]*4)
original=outer._infer_outer_rgb_rectangle(color,mask.astype(bool),depth,scale,T,K,origin,tn,axes,pixel,rms,**kwargs)
diagnostic=ns['_diagnostic_only'](color,mask.astype(bool),depth,scale,T,K,origin,tn,axes,pixel,rms,**kwargs)
proposals,search=outer._top_side_proposals(color,mask.astype(bool),depth,scale,T,K,origin,tn,axes,pixel)
numeric=None
if diagnostic['ok']:
 s=diagnostic['upper']-diagnostic['lower'];y=diagnostic['yaw_uncertainty'];e=diagnostic['edge_uncertainty']
 j=int(np.argmin(s));k=1-j;t=min(y,math.atan2(s[k],s[j]));widest=s[j]*math.cos(t)+s[k]*math.sin(t)
 numeric=dict(size_xy_m=s.tolist(),edge_m=e,yaw_rad=y,required_open_m=float(widest+2*e+.001),
  reserve_m=.002,physical_opening_m=.05,required_plus_reserve_m=float(widest+2*e+.001+.002),
  actual_true_face_yaw_rad=phi,actual_face_outside_claimed_yaw=bool(phi>y),actual_face_beyond_8deg=True)
assert before=={p:sha(W/p) for p in before}
boundary=O/'boundary.jsonl';attempts=boundary.read_text().splitlines() if boundary.exists() else [];assert not attempts
def serial(v):return v.tolist() if hasattr(v,'tolist') else v.item()
result=dict(diagnostic_only=True,qualified_model=None,motion_authorized=False,
 scope='Original outer RGB subfunction and proposals only, not upstream geometry/source contracts or full task chain',
 original_guard_result=original,in_memory_counterfactual_result=diagnostic,
 original_hough_search=search,original_hough_proposal_count=len(proposals),numeric=numeric,
 source_count=1005,source_unchanged=True,external_boundary_attempts=attempts,
 procedural_input=dict(K=K.tolist(),T=T.tolist(),top_origin=origin.tolist(),scale=scale,
  image_size=[1280,960],true_top_size_m=true_size.tolist(),true_yaw_rad=phi,
  internal_print_size_m=print_size.tolist(),print_pixel_corners=[[420,345],[860,615]],
  gray_background=128,object_bgr_equal_gray=[0,150,134],depth_inside_print_raw=5000,
  depth_other_object_raw=0,depth_outside_object_raw=5200),script_sha256=sha(__file__))
(O/'synthetic_internal_print_counterexample.json').write_text(json.dumps(result,indent=2,ensure_ascii=False,default=serial)+'\n')
print(json.dumps(dict(original=original['ok'],original_reason=original.get('reason'),
 diagnostic=diagnostic['ok'],diagnostic_reason=diagnostic.get('reason'),numeric=numeric,search=search),default=serial))
