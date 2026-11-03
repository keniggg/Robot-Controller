"""Independent fixed proposals only; original evaluator decides every verdict."""
from pathlib import Path
import argparse,json,time,hashlib,inspect
import alicia_grasp_modes.arrival_dense_identity as dense_module
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
from dataclasses import asdict
import cv2,numpy as np
from alicia_grasp_modes.arrival_dense_identity import freeze_reference,propose_homography,evaluate_appearance
from alicia_grasp_modes.arrival_orb_proposal import propose_orb_homography
from alicia_grasp_modes.tabletop import _project

PRE=1791860176945733070
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
F=N/'verification/gen14_arrival_independent'

def plain(v):
 if isinstance(v,np.ndarray):return v.tolist()
 if isinstance(v,np.generic):return v.item()
 if isinstance(v,dict):return {k:plain(x) for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [plain(x) for x in v]
 return v

def strict_orb(ref,rgb,mask):
 orb=cv2.ORB_create(nfeatures=1000,scaleFactor=1.2,nlevels=8,edgeThreshold=15,firstLevel=0,WTA_K=2,scoreType=cv2.ORB_HARRIS_SCORE,patchSize=31,fastThreshold=20)
 stats=[]
 def detect(im,m):
  points,desc=orb.detectAndCompute(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),cv2.erode((m>0).astype(np.uint8)*255,np.ones((7,7),np.uint8)))
  dist=cv2.distanceTransform((m>0).astype(np.uint8),cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
  ids=[i for i,k in enumerate(points) if dist[int(round(k.pt[1])),int(round(k.pt[0]))]>=np.sqrt(2.)*.5*k.size+1]
  stats.append(dict(original=len(points),full_patch_inside=len(ids)))
  return [points[i] for i in ids],None if desc is None else desc[ids]
 kr,dr=detect(ref.bgr,ref.mask);kc,dc=detect(rgb,mask)
 if dr is None or dc is None or min(len(dr),len(dc))<2:return None,dict(detection=stats,reason='insufficient_full_patch_descriptors')
 m=cv2.BFMatcher(cv2.NORM_HAMMING)
 f={p[0].queryIdx:p[0] for p in m.knnMatch(dr,dc,k=2) if len(p)==2 and p[0].distance<.75*p[1].distance}
 r={p[0].queryIdx:p[0] for p in m.knnMatch(dc,dr,k=2) if len(p)==2 and p[0].distance<.75*p[1].distance}
 pairs=[v for i,v in f.items() if v.trainIdx in r and r[v.trainIdx].trainIdx==i]
 stats=dict(detection=stats,mutual=len(pairs))
 if len(pairs)<4:return None,stats
 a=np.array([kr[p.queryIdx].pt for p in pairs],np.float32);b=np.array([kc[p.trainIdx].pt for p in pairs],np.float32)
 cv2.setRNGSeed(1729);H,inside=cv2.findHomography(a,b,cv2.RANSAC,3.,maxIters=2000,confidence=.995)
 stats.update(points_reference=a,points_current=b,inliers=None if inside is None else inside.ravel())
 return H,stats

def quad(mask):
 contours,_=cv2.findContours((mask>0).astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
 c=max(contours,key=cv2.contourArea);q=cv2.approxPolyDP(c,.025*cv2.arcLength(c,True),True).reshape(-1,2)
 if len(q)!=4 or not cv2.isContourConvex(q):return None
 center=q.mean(axis=0);q=q[np.argsort(np.arctan2(q[:,1]-center[1],q[:,0]-center[0]))]
 return np.roll(q,-int(np.argmin(q.sum(axis=1))),axis=0).astype(np.float32)

def plane_homography(k,previous,current,normal,offset):
 fx,fy,cx,cy=k
 L=np.array([[cx,-fx,0.],[cy,0.,-fy],[1.,0.,0.]])
 relative=np.linalg.inv(current)@previous
 return L@(relative[:3,:3]-np.outer(relative[:3,3],normal)/offset)@np.linalg.inv(L)

def main():
 p=argparse.ArgumentParser();p.add_argument('--mask-dir',required=True);p.add_argument('--output',required=True);args=p.parse_args()
 out=Path(args.output);out.mkdir(parents=True,exist_ok=False);md=Path(args.mask_dir);cv2.setNumThreads(1)
 fixture=json.loads((F/'fixture.json').read_text());cam=fixture['camera'];k=[cam[x]for x in ('fx','fy','cx','cy')]
 raw=np.load(F/f'{PRE}_source.npz');valid=rectification_rgb_valid_mask(cam,raw['depth'].shape)
 if valid is None:valid=np.ones(raw['depth'].shape,bool)
 ref=freeze_reference(raw['rgb'],raw['recorded_mask'],valid,PRE)
 points=_project(raw['depth'],k);sel=(cv2.erode((ref.mask>0).astype(np.uint8),np.ones((7,7),np.uint8))>0)&np.isfinite(raw['depth'])&(raw['depth']>.03)&(raw['depth']<2)
 xyz=points[sel].astype(float);original_count=len(xyz)
 for _ in range(3):
  center=np.mean(xyz,axis=0);_,_,V=np.linalg.svd(xyz-center,full_matrices=False);normal=V[-1];offset=-float(normal@center)
  good=np.abs(xyz@normal+offset)<=.007
  if good.all():break
  xyz=xyz[good]
 q0=quad(ref.mask);rows=[]
 # Diagnostics alone continue past the first mask rejection to reveal later
 # unmodified held-out proof. Every reported diagnostic verdict stays false.
 diagnostic_src=inspect.getsource(evaluate_appearance).replace('def evaluate_appearance(', 'def diagnostic_downstream(').replace("return finish(False, 'WHOLE_MASK_INCONSISTENT')", "metrics['diagnostic_original_whole_mask_failed'] = True")
 namespace=dict(dense_module.__dict__);exec(diagnostic_src,namespace)
 probe=namespace['diagnostic_downstream']
 for fr in fixture['frames']:
  ns=fr['source_stamp_ns']
  if ns<1791860212000000000:continue
  candidates=list(md.glob(f'{ns}_rejected_proposal_0_mask.png'))
  if len(candidates)!=1:raise ValueError((ns,[str(x) for x in candidates]))
  mask=cv2.imread(str(candidates[0]),cv2.IMREAD_GRAYSCALE);frame=np.load(fr['file']);rgb=frame['rgb'];v=rectification_rgb_valid_mask(cam,mask.shape)
  if v is None:v=np.ones(mask.shape,bool)
  proposals=[]
  prop,evidence=propose_orb_homography(ref,rgb,mask,v,ns,deadline_monotonic=time.monotonic()+5)
  proposals.append(('original_orb',None if prop is None else prop.matrix,asdict(evidence)))
  H,stats=strict_orb(ref,rgb,mask);proposals.append(('full_patch_inside_orb',H,stats))
  q1=quad(mask);proposals.append(('fixed_complete_outline_quad',None if q0 is None or q1 is None else cv2.getPerspectiveTransform(q0,q1),dict(reference=q0,current=q1)))
  H=plane_homography(k,raw['T_base_camera'],frame['T_base_camera'],normal,offset)
  proposals.append(('frozen_measured_surface_tf',H,dict(normal=normal,offset=offset,original_points=original_count,plane_inlier_points=len(xyz),residual_max=float(np.max(np.abs(xyz@normal+offset))))))
  results=[]
  for label,H,stats in proposals:
   started=time.perf_counter();prop=None if H is None else propose_homography(ref,ns,H)
   result=evaluate_appearance(ref,rgb,mask,v,ns,prop,deadline_monotonic=time.monotonic()+5)
   conditional=None
   if result.code=='WHOLE_MASK_INCONSISTENT':
    value=asdict(probe(ref,rgb,mask,v,ns,prop,deadline_monotonic=time.monotonic()+5))
    value.update(appearance_match=False,diagnostic_only=True,original_rejected_code=result.code,downstream_code=value['code'])
    value['code']='DIAGNOSTIC_ONLY_ORIGINAL_WHOLE_MASK_REJECTED'
    conditional=value
   results.append(dict(proposal_method=label,proposal_evidence=stats,H=H,evidence=asdict(result),conditional_dense_probe=conditional,wall_sec=time.perf_counter()-started))
  rows.append(dict(source_ns=ns,mask_file=str(candidates[0]),mask_sha256=hashlib.sha256((mask>0).astype(np.uint8).tobytes()).hexdigest(),proposals=results))
  print(ns,[(x['proposal_method'],x['evidence']['code'],x['evidence']['metrics'].get('forward_mask_iou'),x['evidence']['metrics'].get('holdout_ncc'))for x in results],flush=True)
 (out/'result.json').write_text(json.dumps(plain(dict(scope='Fixed proposal diagnostics; independent original dense evaluator, no selection by its score and no authority.',reference_source_ns=PRE,plane_frozen_before_current_image=True,rows=rows,no_ros_actions=True,production_changed=False)),indent=2)+'\n')
if __name__=='__main__':main()
