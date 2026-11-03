"""Offline comparison of source-paired depth to ChArUco plane; no motion or fit deployment."""
import sys,json,importlib.util
from pathlib import Path
import numpy as np,rosbag
from cv_bridge import CvBridge
spec=importlib.util.spec_from_file_location('cm','src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py');cm=importlib.util.module_from_spec(spec);spec.loader.exec_module(cm)
O=np.eye(4);O[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
for arg in sys.argv[1:]:
 p=Path(arg);geos=[(f,json.loads(f.read_text())) for f in sorted(p.glob('*_geometry.json'))];wanted={d['source_stamp_ns'] for _,d in geos};depth={}
 with rosbag.Bag(str(p/'evidence.bag')) as b:
  for _,m,_ in b.read_messages(topics=['/supervisor/camera/depth/image_raw']):
   if m.header.stamp.to_nsec() in wanted:depth[m.header.stamp.to_nsec()]=CvBridge().imgmsg_to_cv2(m,'passthrough')
 rows=[]
 for f,d in geos:
  stamp=d['source_stamp_ns']
  if stamp not in depth:rows.append(dict(geometry=f.name,status='exact_source_depth_missing'));continue
  model=cm.RuntimeColorModel(d['quality']['camera_model']);params=d['camera_parameters'];D=depth[stamp]
  if D.shape != (model.intrinsics.height,model.intrinsics.width):raise RuntimeError('depth size differs from color geometry')
  runtime=params['runtime_profile'];geometry=runtime['geometry_intrinsics'];color=runtime['color_intrinsics']
  if geometry != color:raise RuntimeError('depth geometry differs from color rays; requires explicit extrinsic mapping')
  B=O.T@np.asarray(d['T_camera_board']);ys,xs=np.mgrid[12:D.shape[0]-12:4,12:D.shape[1]-12:4];uv=np.column_stack([xs.ravel(),ys.ravel()]);z=D[ys.ravel(),xs.ravel()]*params['depth_scale'];rays=np.column_stack([model.normalized_points(uv).reshape(-1,2),np.ones(len(uv))]);P=rays*z[:,None];Q=(P-B[:3,3])@B[:3,:3]
  keep=(z>.03)&(z<1.)&(Q[:,0]>.02)&(Q[:,0]<.16)&(Q[:,1]>.02)&(Q[:,1]<.115)&(abs(Q[:,2])<.04)
  if np.count_nonzero(keep)<100:raise RuntimeError('not enough interior board depth')
  n=B[:3,2];expected=(n@B[:3,3])/(rays[keep]@n);dz=z[keep]-expected;signed=Q[keep,2]
  A=np.asarray(d['T_base_tool'])@np.asarray(d['T_tool_camera'])@O
  measured_base=P[keep]@A[:3,:3].T+A[:3,3];pnp_base=(rays[keep]*expected[:,None])@A[:3,:3].T+A[:3,3]
  rows.append(dict(geometry=f.name,status='measured',source_stamp_ns=stamp,points=int(sum(keep)),depth_scale_m=params['depth_scale'],camera_model_fingerprint=model.fingerprint,median_depth_minus_pnp_mm=float(np.median(dz)*1000),depth_minus_pnp_p05_p95_mm=(np.percentile(dz,[5,95])*1000).tolist(),median_signed_normal_error_mm=float(np.median(signed)*1000),normal_rms_mm=float(np.sqrt(np.mean(signed*signed))*1000),median_base_z_depth_minus_pnp_mm=float(np.median(measured_base[:,2]-pnp_base[:,2])*1000),accepted_positions_rad=d['accepted_positions_rad']))
 result=dict(kind='source_paired_depth_vs_rgb_pnp_diagnostic',session=str(p),scope='Uses current SDK rays, no geometric fit deployed. Board physical size not independently measured. Depth and RGB are independent sensing modalities but share assumed camera geometry; neither is external truth.',samples=rows)
 (p/'depth_vs_pnp_diagnostic.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False))
