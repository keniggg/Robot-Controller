"""Read-only coordinate-chain and parameter sensitivities; not calibration."""
from pathlib import Path
import hashlib,json,math
import numpy as np
import rosbag
from tf.transformations import quaternion_matrix
from alicia_grasp_modes import tabletop
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
OUT=ROOT/'ros_log/full_latest_restart_20261010T002347Z/gen13_handoff_analysis/support_plane_replay'
report=json.loads((OUT/'report.json').read_text());frames=report['frames'];cfg=tabletop.Config(**report['config'])
WORK=Path(__file__).resolve().parents[1]
modelpath=WORK/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
model=SerialUrdfFk(modelpath.read_text(),['Joint%d'%i for i in range(1,7)])
mount=next(t for t in report['static_tf'] if t['parent']=='tool0' and t['child']=='camera_link')
mountT=quaternion_matrix(mount['quaternion']);mountT[:3,3]=mount['xyz']
accepted=[]
with rosbag.Bag(str(Path(report['record'])/'keyframes.bag')) as b:
    for t,m,r in b.read_messages(topics=['/alicia_d/accepted_joint_states']):
        q=dict(zip(m.name,m.position));accepted.append((m.header.stamp.to_nsec(),np.asarray([q['Joint%d'%i] for i in range(1,7)])))
anchor=np.asarray(report['reference']['anchor_base']);reference=np.asarray(report['reference']['plane_base'])

def fitted(points):
    center=points.mean(axis=0);w,v=np.linalg.eigh((points-center).T@(points-center));n=v[:,0]
    if n[0]<0:n=-n
    return np.r_[n,-n@center]

def fixed_support_planes(frame):
    with np.load(frame['source_file'],allow_pickle=False) as z:
        depth=z['depth_raw'].astype(np.float32)*float(z['depth_scale']);K=z['intrinsics'];T=z['T_base_camera']
    vv,uu=np.indices(depth.shape);center=np.asarray(frame['ring_center_px']);scale=min(depth.shape[1]/640.,depth.shape[0]/480.)
    radius=np.hypot(uu-center[0],vv-center[1]);mask=(radius>=70*scale)&(radius<=frame['ring_outer_radius_px'])&(depth>=cfg.depth_min_m)&(depth<=cfg.depth_max_m)
    rays=tabletop._project_pixels(np.ones_like(depth[mask]),uu[mask],vv[mask],K)
    points=rays*depth[mask,None];p=np.asarray(frame['plane_camera']);inside=abs(points@p[:3]+p[3])<=cfg.support_threshold_m
    rays=rays[inside];values=depth[mask][inside]
    planes={}
    for label,d in [('baseline',values),('depth_plus_1mm',values+.001),('depth_minus_1mm',values-.001),('scale_plus_0p1pct',values*1.001),('scale_minus_0p1pct',values*.999)]:
        plane=tabletop._normalized_plane(np.linalg.solve(T.T,fitted(rays*d[:,None])))
        if plane[:3]@reference[:3]<0:plane=-plane
        planes[label]=plane
    return planes

refplanes=fixed_support_planes(frames[0]);refT=np.asarray(frames[0]['T_base_camera']);rows=[]
for f in frames:
    ns=f['source_stamp_ns'];before=max((r for r in accepted if r[0]<=ns),key=lambda r:r[0]);after=min((r for r in accepted if r[0]>=ns),key=lambda r:r[0]);ratio=(ns-before[0])/max(1,after[0]-before[0]);q=before[1]+ratio*(after[1]-before[1]);T=np.asarray(f['T_base_camera']);fk=model(q)@mountT
    rotate=fk[:3,:3].T@T[:3,:3]
    planes=fixed_support_planes(f)
    differential=lambda name:float(np.r_[anchor,1.]@(planes[name]-refplanes[name]))
    depth_gain=(differential('depth_plus_1mm')-differential('depth_minus_1mm'))/.002
    scale_gain=(differential('scale_plus_0p1pct')-differential('scale_minus_0p1pct'))/.002
    current=np.asarray(f['plane_base']);mount_translation_gain=reference[:3]@refT[:3,:3]-current[:3]@T[:3,:3]
    row=dict(source_stamp_ns=ns,accepted_before_ns=before[0],accepted_after_ns=after[0],source_interpolated_q_rad=q.tolist(),
        fk_exact_tf_translation_difference_mm=float(np.linalg.norm(fk[:3,3]-T[:3,3]))*1000,
        fk_exact_tf_rotation_difference_deg=float(np.rad2deg(np.arccos(np.clip((np.trace(rotate)-1)/2,-1,1)))),
        sensitivity=dict(depth_constant_bias_to_local_delta_gain=depth_gain,
            depth_scale_error_to_local_delta_m_per_unit_scale=scale_gain,
            camera_fixed_translation_to_local_delta_gradient=mount_translation_gain.tolist(),
            camera_fixed_translation_gradient_norm=float(np.linalg.norm(mount_translation_gain)),
            nominal_anchor_delta_mm=f['reference_delta']['anchor_signed_delta_mm']))
    rows.append(row)
result=dict(source_report=str(OUT/'report.json'),source_report_sha256=hashlib.sha256((OUT/'report.json').read_bytes()).hexdigest(),
    urdf_path=str(modelpath),urdf_sha256=hashlib.sha256(modelpath.read_bytes()).hexdigest(),static_mount=mount,
    rows=rows,method='Central finite differences with original measured inlier membership fixed; same perturbation applied to initial and current frame. No corrected geometry is admitted.',
    limitations=['FK is from the expanded metric model and linearly interpolated accepted source samples; its agreement with exact recorded TF checks software consistency only.',
        'The earlier coordinate_sensitivity.json used the default uncalibrated URDF; its FK disagreement is a diagnostic model mismatch, not a runtime fault. This artifact supersedes only that FK comparison.',
        'Sensitivity gradients are not measured calibration errors and do not establish a unique physical cause.',
        'No threshold, calibration, geometry, ROS state or device changes.'])
path=OUT/'coordinate_sensitivity_expanded_metric.json'
with path.open('x') as file:json.dump(result,file,ensure_ascii=False,indent=2);file.write('\n')
print(json.dumps(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),rows=rows)))
