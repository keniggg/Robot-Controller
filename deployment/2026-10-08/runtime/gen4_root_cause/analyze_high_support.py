"""Offline same-texture local support comparison; no bag reads or ROS connection."""
import hashlib, importlib.util, json, pathlib, sys
import cv2
import numpy as np
ROOT=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002/gen4_root_cause')
SESSION=ROOT/'high_posture_20261003T081809Z'
OUT=SESSION/'support_plane_review';OUT.mkdir(exist_ok=True)
cv2.setNumThreads(1)
source=json.loads((SESSION/'fresh_plan/snapshot.json').read_text())
params=json.loads((SESSION/'camera_parameters.json').read_text())
events=[json.loads(x) for x in (SESSION/'events.jsonl').read_text().splitlines()]
frames={x['label']:x for x in events if x.get('event')=='stationary_snapshot'}
assert any(x.get('event')=='sequence_complete' for x in events)
ns0=frames['baseline']['source_stamp_ns']
plane=np.array(source['plane_base']);plane/=np.linalg.norm(plane[:3]);pts=np.array(source['target_bbox_volume_corners_base']);foot=pts-(pts@plane[:3]+plane[3])[:,None]*plane[:3];anchor=foot.mean(axis=0)
path=ROOT/'analyze_rgbd_correspondence_spatialcv.py';spec=importlib.util.spec_from_file_location('support_rgbd',str(path));a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
a.OUT=OUT;a.meta=dict(anchor_position_base=anchor.tolist(),reference_plane_base=plane.tolist())
ii=params['runtime_profile']['color_intrinsics']
a.intr.width=ii['width'];a.intr.height=ii['height'];a.intr.fx=ii['fx'];a.intr.fy=ii['fy'];a.intr.ppx=ii['cx'];a.intr.ppy=ii['cy'];a.intr.coeffs=ii['coeffs']
def load(ns):
 with np.load(str(SESSION/f'{ns}_rgbd_tf.npz')) as z:
  f={k:z[k] for k in z.files}
 assert int(f['source_stamp_ns'])==ns
 assert np.array_equal(f['K'],np.array([[params['fx'],0,params['cx']],[0,params['fy'],params['cy']],[0,0,1.]]))
 return f
f0=load(ns0)
a.load=lambda ns:f0 if ns==ns0 else load(ns)
raw_match=a.match
center=a.projection(a.mappts(np.linalg.inv(f0['T_base_optical']),anchor[None]),f0['K'])[0]
def local_match(f,g):
 m=raw_match(f,g)
 d=np.linalg.norm(m['uv0']-center,axis=1);keep=(d>=75)&(d<=180)
 for key in ('uv0','uv1','z0','z1','mad0','mad1'):m[key]=m[key][keep]
 m['counts']['pre_local_scope_valid_rgbd']=m['counts']['valid_rgbd_correspondences']
 m['counts']['valid_rgbd_correspondences']=int(keep.sum())
 return m
a.match=local_match

def normalize(p):
 p=p/np.linalg.norm(p[:3]);return p if p[:3]@plane[:3]>0 else -p

def evaluate(p,reference=plane):
 delta=normalize(p)-reference
 shifts=(foot@delta[:3]+delta[3])*1000
 return dict(anchor_shift_mm=float((anchor@delta[:3]+delta[3])*1000),max_abs_footprint_shift_mm=float(np.max(abs(shifts))),footprint_signed_shift_mm=shifts.tolist())

def predicted(Fd,T):
 C=Fd@np.asarray(T)@np.linalg.inv(f0['T_base_optical'])
 result=evaluate(np.linalg.solve(C.T,plane))
 result['plane_base']=normalize(np.linalg.solve(C.T,plane)).tolist()
 result['rotation_disagreement_deg']=a.angle(C)
 return result

def fit_local(P):
 keep=np.ones(len(P),bool)
 for _ in range(4):
  c=P[keep].mean(0);_,_,v=np.linalg.svd(P[keep]-c,full_matrices=False);n=v[-1];d=-n@c
  # Diagnostic-only local robust fit. The data have already passed the same
  # physical-feature and 5x5 inverse-depth patch tests. No global table fit.
  resid=P@n+d;med=np.median(resid);mad=np.median(abs(resid-med))
  keep=abs(resid-med)<=max(.002,3*1.4826*mad)
  if keep.sum()<12:raise ValueError('insufficient local plane inliers')
 return np.r_[n,d],dict(points=len(P),inliers=int(keep.sum()),residual_rmse_mm=float(np.sqrt(np.mean(resid[keep]**2))*1000),point_span_m=np.ptp(P[keep],axis=0).tolist())

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]));from gen4_single_axis_plan import SerialUrdfFk,NAMES
xml=(SESSION/'robot_description.urdf').read_text();assert hashlib.sha256(xml.encode()).hexdigest()==source['urdf_sha256']
fk=SerialUrdfFk(xml,NAMES[:6]);X=np.array(source['T_tool_camera'])

def frame_binding(label):
 frame=frames[label];f=a.load(frame['source_stamp_ns']);lo,hi=frame['q_source_before'],frame['q_source_after'];ns=frame['source_stamp_ns']
 assert lo[0]<=ns<=hi[0]
 errors=[float(np.max(abs(fk(np.array(q[:6]))@X-f['T_base_ros_camera']))) for _,q in (lo,hi)]
 return dict(label=label,publication_header_stamp_ns=ns,bracketing_actual_stamps_ns=[lo[0],hi[0]],bracketing_actual_q_rad=[lo[1],hi[1]],fk_tf_max_abs_error_brackets=errors,stable_span_deg=frame['span_deg'],npz_sha256=hashlib.sha256((SESSION/f'{ns}_rgbd_tf.npz').read_bytes()).hexdigest())

labels=['domain_07_j5','high_j5_negative3','high_j5_negative3_return','high_j2_positive3','high_j2_positive3_return','return_14_j2','return_late','domain_01_j5','domain_03_j5','domain_05_j5','domain_06_j5']
report=dict(method='Independent RGB matches of the baseline local tabletop support annulus; fixed original plane/target footprint evaluated under measured visual relative transforms and the recorded per-image TF.',
clock_limitation='source_stamp_ns in NPZ is ROS host-publication header, not a calibrated exposure timestamp. TF is evaluated exactly at that header. These static snapshots and matching do not independently exclude long device/USB/processing queues.',
source_plane_stamp_ns=source['source_stamp_ns'],source_plane_base=plane.tolist(),fixed_footprint_base=foot.tolist(),anchor_base=anchor.tolist(),baseline=frame_binding('baseline'),local_source_annulus_px=[75,180],local_source_center_uv=center.tolist(),
model=dict(urdf_sha256=source['urdf_sha256'],T_tool_camera=X.tolist(),K=f0['K'].tolist(),depth_scale=float(f0['depth_scale']),color_intrinsics=ii),
resource_limits=dict(opencv_threads=cv2.getNumThreads(),openblas_threads='environment1',no_bag_read=True),
limitations=['Conditional on static physical scene and correct RGB matches. No single robot joint or calibration cause inferred.','Original reference plane is never refit to make an endpoint agree. Endpoint plane fits use only independently matched local physical support points.','RGB-D rigid spatial holdouts and forward/reverse single-depth PnP are independent checks of geometric fit, not proof of exposure timing.','Source-local 75..180px annulus and identical target footprint are fixed before endpoint analysis. No global free table fit.','The diagnostic local SVD trim threshold2mm is only a report estimator; it changes no production plane/grasp/consistency thresholds.'],results=[])
if sys.argv[1:]:
 labels=sys.argv[1:]
 previous=json.loads((OUT/'fixed_local_support_report.json').read_text())
 report['results']=[r for r in previous['results'] if r['label'] not in labels]
for label in labels:
 for model in ('production_pinhole','sdk_inverse_brown'):
  try:
   ns=frames[label]['source_stamp_ns'];r=a.analyze_pair(ns0,ns,model);g=a.load(ns);r['label']=label;r['frame_binding']=frame_binding(label)
   r['mask_scope']+='; fit/evaluation restricted to baseline source support annulus75..180px.'
   r['fixed_plane_prediction']=predicted(g['T_base_optical'],r['T_destination_from_source_rgbd'])
   r['fixed_plane_prediction_folds']=[predicted(g['T_base_optical'],fold['T']) for fold in r['folds']]
   r['fixed_plane_prediction_pnp_forward']=predicted(g['T_base_optical'],r['pnp_forward']['T'])
   r['fixed_plane_prediction_pnp_reverse']=predicted(g['T_base_optical'],np.linalg.inv(r['pnp_reverse']['T']))
   corr=np.load(str(OUT/f'{ns0}_{ns}_{model}_spatialcv_correspondences.npz'))
   p0,stats0=fit_local(corr['P_source']);p1,stats1=fit_local(corr['Q_destination']);p0=normalize(np.linalg.solve(f0['T_base_optical'].T,p0));p1=normalize(np.linalg.solve(g['T_base_optical'].T,p1))
   r['matched_local_planes']=dict(source_plane_base=p0.tolist(),destination_plane_base=p1.tolist(),source_fit=stats0,destination_fit=stats1,source_vs_frozen=evaluate(p0),destination_vs_frozen=evaluate(p1),destination_vs_source=evaluate(p1,p0))
   report['results'].append(r)
   print(json.dumps(dict(label=label,model=model,n=r['counts']['valid_rgbd_correspondences'],holdout=r['heldout'],tf_mm=r['tf_heldout_residual_m']['rmse']*1000,visual_mm=r['heldout_residual_m']['rmse']*1000,plane_mm=r['fixed_plane_prediction']['anchor_shift_mm'],foot_mm=r['fixed_plane_prediction']['max_abs_footprint_shift_mm'],rotation_deg=r['tf_rotation_disagreement_deg'],local_delta_mm=r['matched_local_planes']['destination_vs_source']['anchor_shift_mm'])),flush=True)
  except Exception as exc:
   report['results'].append(dict(label=label,model=model,error=repr(exc)));print(label,model,repr(exc),flush=True)
 (OUT/'fixed_local_support_report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
(OUT/'analysis_source.py').write_bytes(pathlib.Path(__file__).read_bytes())
