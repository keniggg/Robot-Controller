"""Fixed physical perspective + sparse affine residual; diagnostic only."""
import sys,json,time,hashlib
from pathlib import Path
from dataclasses import fields,asdict
import numpy as np,cv2
W=Path(__file__).resolve().parent.parent;STAGE=W/'.runtime/gen14_arrival_repair_20261013'
sys.path.insert(0,str(STAGE/'src/alicia_grasp_modes/src'))
from alicia_grasp_modes.observation_measured_face import FrozenMeasuredFaceSource,freeze_measured_face,extract_measured_face,Config
from alicia_grasp_modes.arrival_dense_identity import propose_homography,evaluate_appearance
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
F=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification/gen14_arrival_independent');G=F/'full_arrival_current_geometry'
sha=lambda x:hashlib.sha256(np.ascontiguousarray(x).tobytes()).hexdigest()
def features(rgb,mask):
 o=cv2.ORB_create(nfeatures=1000,scaleFactor=1.2,nlevels=8,edgeThreshold=15,firstLevel=0,WTA_K=2,scoreType=cv2.ORB_HARRIS_SCORE,patchSize=31,fastThreshold=20)
 return o.detectAndCompute(cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY),cv2.erode((mask>0).astype('uint8')*255,np.ones((7,7),'uint8')))
def projection(k):
 fx,fy,cx,cy=k;return np.array([[cx,-fx,0.],[cy,0.,-fy],[1.,0.,0.]])
def physical_homography(frozen,curT,k):
 relative=np.linalg.solve(curT,frozen.source.transform);nref=frozen.source.transform[:3,:3].T@frozen.normal;d=float(frozen.normal@(frozen.source.transform[:3,3]-frozen.centre))
 H=projection(k)@(relative[:3,:3]-np.outer(relative[:3,3],nref)/d)@np.linalg.inv(projection(frozen.source.intrinsics));return H/H[2,2]
def main():
 out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1);f=json.loads((F/'fixture.json').read_text());cam=f['camera'];config=Config(**{x.name:f['unknown_parameters'][x.name]for x in fields(Config)if x.name in f['unknown_parameters']});k=[cam[z]for z in ('fx','fy','cx','cy')];plane=np.array(json.loads((F/'original_geometry_reconstruction_v2.json').read_text())['seed']['reference_plane_base']);ns=1791860176945733070;pre=np.load(F/f'{ns}_source.npz');valid=rectification_rgb_valid_mask(cam,pre['depth'].shape);source=FrozenMeasuredFaceSource(pre['raw_depth'],'16UC1',cam['depth_scale'],k,pre['T_base_camera'],plane,config,ns,'diagnostic-only',sha(pre['rgb']),sha(pre['recorded_mask']));frozen=freeze_measured_face(source,pre['rgb'],pre['recorded_mask'],valid,check_current=lambda:None,deadline_monotonic=time.monotonic()+5);kr,dr=features(pre['rgb'],frozen.reference.mask);rows=[]
 for path in sorted(G.glob('*_geometry.json')):
  g=json.loads(path.read_text());ns=g['source_ns'];data=np.load(g['source_fixture']['file']);whole=cv2.imread(str(G/f'{ns}_proposal_0_mask.png'),0);start=time.monotonic();row=dict(source_ns=ns,raw_npz_sha256=g['source_fixture']['sha256'],whole_mask_sha256=sha(whole),results=[])
  try:
   mask,points,normal,centre,audit=extract_measured_face(data['rgb'],whole,valid,data['depth'],k,data['T_base_camera'],plane,config,lambda:None);row['face']=audit;kc,dc=features(data['rgb'],mask);matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
   forward={p[0].queryIdx:p[0]for p in matcher.knnMatch(dr,dc,k=2)if len(p)==2 and p[0].distance<.75*p[1].distance};reverse={p[0].queryIdx:p[0]for p in matcher.knnMatch(dc,dr,k=2)if len(p)==2 and p[0].distance<.75*p[1].distance};pairs=[v for i,v in forward.items()if v.trainIdx in reverse and reverse[v.trainIdx].trainIdx==i];a=np.asarray([kr[p.queryIdx].pt for p in pairs],np.float32);b=np.asarray([kc[p.trainIdx].pt for p in pairs],np.float32);row.update(reference_points=a.tolist(),current_points=b.tolist(),mutual_matches=len(pairs));physical=physical_homography(frozen,data['T_base_camera'],k);row['physical_homography']=physical.tolist()
   if len(pairs)<4:raise ValueError('insufficient_sparse_proposal')
   for name,base in [('plain_affine',np.eye(3)),('physical_perspective_affine_residual',physical)]:
    predicted=cv2.perspectiveTransform(a[:,None,:],base)[:,0,:];cv2.setRNGSeed(1729);aff,inside=cv2.estimateAffine2D(predicted,b,method=cv2.RANSAC,ransacReprojThreshold=3.,maxIters=2000,confidence=.995,refineIters=10)
    if aff is None:row['results'].append(dict(method=name,error='no_affine'));continue
    H=np.vstack([aff,[0.,0.,1.]])@base;prop=propose_homography(frozen.reference,ns,H);evidence=evaluate_appearance(frozen.reference,data['rgb'],mask,valid,ns,prop,deadline_monotonic=time.monotonic()+5,require_dense=True);projected=cv2.perspectiveTransform(a[:,None,:],H)[:,0,:];row['results'].append(dict(method=name,H=H.tolist(),inliers=int(inside.sum()),sparse_residuals_px=np.linalg.norm(projected-b,axis=1).tolist(),inlier_mask=inside.flatten().tolist(),evidence=asdict(evidence)))
  except Exception as exc:row['error']=str(exc)
  row['elapsed_sec']=time.monotonic()-start;rows.append(row);print(ns,[(r['method'],r.get('evidence',{}).get('code'),r.get('evidence',{}).get('metrics',{}).get('good_test_cells'))for r in row['results']],row.get('error'),flush=True)
 result=dict(scope='Diagnostic two independently fixed proposal algorithms evaluated on all57 immutable full-face masks. Never select by dense score; no qualification, no source clock or transport authority. Physical perspective from actual source fitted plane/TF/K, residual affine fit only original sparse ORB matches.',reference=json.loads(frozen.audit_json),rows=rows,production_changed=False,source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in [STAGE/'src/alicia_grasp_modes/src/alicia_grasp_modes/observation_measured_face.py',STAGE/'src/alicia_grasp_modes/src/alicia_grasp_modes/arrival_dense_identity.py']})
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
if __name__=='__main__':main()
