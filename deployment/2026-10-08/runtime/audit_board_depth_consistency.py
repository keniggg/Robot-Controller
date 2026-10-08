"""Compare recorded depth rays to independent PnP board planes; no ROS I/O."""
import pathlib,json,importlib.util
import numpy as np,rosbag,cv2
from cv_bridge import CvBridge
spec=importlib.util.spec_from_file_location('board_model','src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py');model=importlib.util.module_from_spec(spec);spec.loader.exec_module(model)
root=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/handeye_auto_20260927T102700Z')
current=pathlib.Path(json.load(open('.runtime/current_handeye_task_pose_validation.json'))['session'])
folders=[root/'capture_002',root/'capture_016',root/'capture_030']+sorted(p for p in current.rglob('capture_*') if p.is_dir())
O=np.eye(4);O[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
out=[]
for p in folders:
 d=json.loads((p/'capture.json').read_text())
 if not d['accepted']:continue
 f=d['samples'][d['selected_frame_index']];stamp=f['source_stamp_ns'];B=O.T@np.array(f['T_camera_board']);cm=model.RuntimeColorModel(f['quality']['camera_model']);D=None
 with rosbag.Bag(str(p/'sources.bag')) as b:
  for _,m,_ in b.read_messages(topics=['/supervisor/camera/depth/image_raw']):
   if m.header.stamp.to_nsec()==stamp:D=CvBridge().imgmsg_to_cv2(m,'passthrough');break
 if D is None:raise RuntimeError('same source depth missing')
 ys,xs=np.mgrid[12:468:4,12:628:4];uv=np.column_stack([xs.ravel(),ys.ravel()]);z=D[ys.ravel(),xs.ravel()]*d['camera_parameters_before']['depth_scale']
 rays=np.column_stack([cm.normalized_points(uv).reshape(-1,2),np.ones(len(uv))]);P=rays*z[:,None];Q=(P-B[:3,3])@B[:3,:3]
 keep=(z>.03)&(z<1.)&(Q[:,0]>.02)&(Q[:,0]<.16)&(Q[:,1]>.02)&(Q[:,1]<.115)&(abs(Q[:,2])<.04)
 if np.count_nonzero(keep)<100:raise RuntimeError('insufficient interior board depth')
 n=B[:3,2];offset=-n@B[:3,3];expected=-offset/(rays[keep]@n);signed=Q[keep,2];dz=z[keep]-expected
 r=dict(capture=str(p),source_stamp_ns=stamp,charuco_corners=f['quality']['charuco_count'],reprojection_rms_px=f['quality']['reprojection_rms_px'],median_depth_m=float(np.median(z[keep])),points=int(sum(keep)),median_depth_minus_pnp_m=float(np.median(dz)),depth_minus_pnp_p05_p95_m=np.percentile(dz,[5,95]).tolist(),median_depth_over_pnp=float(np.median(z[keep]/expected)),median_signed_normal_error_m=float(np.median(signed)),normal_rms_m=float(np.sqrt(np.mean(signed*signed))),q_rad=f['accepted_joint_positions'][:6])
 out.append(r);print(json.dumps(r,ensure_ascii=False),flush=True)
path=current/'depth_vs_pnp_diagnostic.json';path.write_text(json.dumps(dict(scope='diagnostic_only; printed_board_metric_scale_not_independently_measured',assumed_board_square_m=.015,samples=out),ensure_ascii=False,indent=2)+'\n')
