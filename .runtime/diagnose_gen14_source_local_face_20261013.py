"""One deterministic source-local measured-plane RGB face proposal; no authority."""
import sys,json,time,inspect,hashlib
from pathlib import Path
from dataclasses import asdict
import numpy as np,cv2
sys.path.insert(0,str(Path(__file__).parent))
from diagnose_gen14_fixed_proposals_20261013 import F,PRE,plain
from alicia_grasp_modes.tabletop import _project_pixels,Config
from alicia_grasp_modes.measured_mask_seeds import local_surface_normals
from alicia_grasp_modes.rgb_instance import _grabcut_lock
import alicia_grasp_modes.arrival_dense_identity as dense
from alicia_grasp_modes.arrival_orb_proposal import propose_orb_homography
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

def extract(frame,whole,k,plane,tolerance):
 # Source-local only: no reference/current counterpart and no held-out score.
 x,y,w,h=cv2.boundingRect((whole>0).astype('uint8')); x0=max(0,x-6);x1=min(whole.shape[1],x+w+6);y0=max(0,y-6);y1=min(whole.shape[0],y+h+6);sl=np.s_[y0:y1,x0:x1]
 depth=frame['depth'][sl];mask=whole[sl]>0;yy,xx=np.indices(depth.shape);points=_project_pixels(depth,xx+x0,yy+y0,k);base=points@frame['T_base_camera'][:3,:3].T+frame['T_base_camera'][:3,3];height=base@plane[:3]+plane[3];valid=np.isfinite(depth)&(depth>=.03)&(depth<=2.)
 foreground=mask&valid&(height>=.004)&(height<=.12);top=float(np.percentile(height[foreground],90))
 normals,reliable=local_surface_normals(points,depth,valid);ncam=frame['T_base_camera'][:3,:3].T@plane[:3];cos=np.abs(normals@ncam)
 core=cv2.erode(foreground.astype('uint8'),np.ones((3,3),'uint8'))>0
 seed=core&reliable&(cos>=np.sqrt(.5))
 if len(sys.argv)>3 and sys.argv[3]=='fit_plane':
  fit=base[seed].astype(float);original_count=len(fit)
  for _ in range(3):
   centre=fit.mean(axis=0);_,_,axes=np.linalg.svd(fit-centre,full_matrices=False);surface_normal=axes[-1];distance=np.abs((fit-centre)@surface_normal);retained=distance<=tolerance
   if retained.all():break
   fit=fit[retained]
  if len(fit)<.85*original_count:raise ValueError('measured_face_plane_not_unique')
  surface_distance=np.abs((base-centre)@surface_normal)
  topmeasured=foreground&(surface_distance<=tolerance)
  hardtop=seed&(surface_distance<=tolerance)
  hardother=foreground&(surface_distance>tolerance)
 else:
  topmeasured=foreground&(abs(height-top)<=tolerance)
  hardtop=seed&(abs(height-top)<=tolerance)
  hardother=foreground&((height<top-tolerance)|(height>top+tolerance))
 labels=np.full(depth.shape,cv2.GC_BGD,np.uint8);labels[mask]=cv2.GC_PR_FGD;labels[hardtop]=cv2.GC_FGD;labels[hardother]=cv2.GC_BGD
 if hardtop.sum()<25:raise ValueError('insufficient_actual_planar_core')
 with _grabcut_lock:
  cv2.setRNGSeed(23);cv2.grabCut(np.ascontiguousarray(frame['rgb'][sl]),labels,None,np.zeros((1,65)),np.zeros((1,65)),3,cv2.GC_INIT_WITH_MASK)
 fg=np.isin(labels,[cv2.GC_FGD,cv2.GC_PR_FGD]).astype('uint8');n,components=cv2.connectedComponents(fg,8);owners=[i for i in range(1,n)if np.count_nonzero((components==i)&hardtop)>=.85*hardtop.sum()]
 if len(owners)!=1:raise ValueError('face_unique_connected_core_required')
 chosen=components==owners[0]
 if np.any(chosen&hardother)or np.any(chosen&~mask):raise ValueError('face_current_owner_conflict')
 full=np.zeros(whole.shape,'uint8');full[sl]=chosen
 pts=base[chosen&topmeasured];center=pts.mean(0);_,s,vh=np.linalg.svd(pts-center,full_matrices=False);rms=float(np.sqrt(np.mean(((pts-center)@vh[-1])**2)));angle=float(np.degrees(np.arccos(np.clip(abs(vh[-1]@plane[:3]),-1,1))))
 return full,dict(upper_height_m=top,actual_depth_unchanged=True,raw_missing_depth_filled=False,raw_top_measured_pixels=int(topmeasured.sum()),hard_top_core_pixels=int(hardtop.sum()),hard_non_top_pixels=int(hardother.sum()),face_pixels=int(full.sum()),face_valid_depth_pixels=int((chosen&valid).sum()),face_actual_planar_points=int(len(pts)),fit_rms_m=rms,fit_support_angle_deg=angle,whole_mask_sha256=hashlib.sha256((whole>0).astype('uint8').tobytes()).hexdigest(),face_mask_sha256=hashlib.sha256(full.tobytes()).hexdigest())

def features(rgb,mask):
 orb=cv2.ORB_create(nfeatures=1000,scaleFactor=1.2,nlevels=8,edgeThreshold=15,firstLevel=0,WTA_K=2,scoreType=cv2.ORB_HARRIS_SCORE,patchSize=31,fastThreshold=20)
 return orb.detectAndCompute(cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY),cv2.erode((mask>0).astype('uint8')*255,np.ones((7,7),'uint8')))

def main():
 out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1);f=json.load(open(F/'fixture.json'));cam=f['camera'];k=[cam[z]for z in ('fx','fy','cx','cy')];seed=json.load(open(F/'original_geometry_reconstruction_v2.json'))['seed'];plane=np.array(seed['reference_plane_base']);tolerance=json.load(open(F/'view_source/source.json'))['arguments']['depth_uncertainty_m'];tolerance = float(sys.argv[2]) if len(sys.argv)>2 else tolerance;pre=np.load(F/f'{PRE}_source.npz');mask,prestats=extract(pre,pre['recorded_mask'],k,plane,tolerance);valid=rectification_rgb_valid_mask(cam,mask.shape);ref=dense.freeze_reference(pre['rgb'],mask,valid,PRE);kp0,de0=features(pre['rgb'],mask)
 cv2.imwrite(str(out/'reference_face.png'),mask*255);np.savez_compressed(out/'reference_face.npz',mask=mask)
 strictsrc=inspect.getsource(dense.evaluate_appearance).replace('def evaluate_appearance(', 'def force_dense(').replace('if colour <= policy.max_colour_distance:', 'if False:  # Diagnostic demands full dense face proof even if colour passes')
 space=dict(dense.__dict__);exec(strictsrc,space);rows=[]
 for row in f['frames']:
  ns=row['source_stamp_ns']
  if ns<1791860212000000000:continue
  frame=np.load(row['file']);whole=cv2.imread(str(F/f'{ns}_rejected_proposal_0_mask.png'),0);cur,stats=extract(frame,whole,k,plane,tolerance);kp1,de1=features(frame['rgb'],cur);matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
  forward={p[0].queryIdx:p[0]for p in matcher.knnMatch(de0,de1,k=2)if len(p)==2 and p[0].distance<.75*p[1].distance};reverse={p[0].queryIdx:p[0]for p in matcher.knnMatch(de1,de0,k=2)if len(p)==2 and p[0].distance<.75*p[1].distance};pairs=[v for i,v in forward.items()if v.trainIdx in reverse and reverse[v.trainIdx].trainIdx==i];hs=[]
  if len(pairs)>=4:
   a=np.array([kp0[p.queryIdx].pt for p in pairs],np.float32);b=np.array([kp1[p.trainIdx].pt for p in pairs],np.float32)
   cv2.setRNGSeed(1729);H,inside=cv2.findHomography(a,b,cv2.RANSAC,3.,maxIters=2000,confidence=.995);hs.append(('projective',H))
   cv2.setRNGSeed(1729);aff,inside=cv2.estimateAffine2D(a,b,method=cv2.RANSAC,ransacReprojThreshold=3.,maxIters=2000,confidence=.995,refineIters=10);hs.append(('affine',None if aff is None else np.vstack([aff,[0,0,1]])))
  results=[]
  for method,H in hs:
   prop=None if H is None else dense.propose_homography(ref,ns,H);ev=asdict(space['force_dense'](ref,frame['rgb'],cur,valid,ns,prop,deadline_monotonic=time.monotonic()+2));results.append(dict(method=method,H=H,evidence=ev,whole_object_identity_proven=False,qualification_allowed=False))
  cv2.imwrite(str(out/f'{ns}_face.png'),cur*255);rows.append(dict(source_ns=ns,face=stats,mutual_matches=len(pairs),results=results));print(ns,stats['face_pixels'],[(x['method'],x['evidence']['code'],x['evidence']['metrics'].get('holdout_ncc'),x['evidence']['metrics'].get('good_test_cells'))for x in results],flush=True)
 (out/'result.json').write_text(json.dumps(plain(dict(scope='Source-local face RGB proposal using actual measured planar core and independently measured non-top background; original dense whole-face gates explicitly required. No whole-object or qualification authority.',reference_face=prestats,original_tolerance_m=tolerance,rows=rows)),indent=2)+'\n')
if __name__=='__main__':main()
