"""Compare sealed calibration capture JSON joint domains; no bag or ROS I/O."""
from pathlib import Path
import hashlib,json
import numpy as np
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
DECL=ROOT/'ros_log/grasp_repair_resume_20260928T120606Z/expanded_metric_training_once/split_declaration.json'
OUT=ROOT/'ros_log/full_latest_restart_20261010T002347Z/gen13_handoff_analysis/support_plane_replay'
d=json.loads(DECL.read_text());c=json.loads((OUT/'coordinate_sensitivity_expanded_metric.json').read_text());current=json.loads((OUT/'bias_observability.json').read_text())
fk=SerialUrdfFk(Path(c['urdf_path']).read_text(),['Joint%d'%i for i in range(1,7)])
mt=c['static_mount'];X=quaternion_matrix(mt['quaternion']);X[:3,3]=mt['xyz']
inputs={}
for source in d['all_original_sources']:
    path=Path(source['capture']);raw=path.read_bytes()
    assert hashlib.sha256(raw).hexdigest()==source['capture_sha256']
    capture=json.loads(raw)
    sample=next(s for s in capture['samples'] if s['source_stamp_ns']==source['source_stamp_ns'])
    q=dict(zip(sample['accepted_joint_names'],sample['accepted_joint_positions']))
    q=np.asarray([q['Joint%d'%i] for i in range(1,7)])
    inputs[str(path)]=dict(path=str(path),source_stamp_ns=sample['source_stamp_ns'],sha256=source['capture_sha256'],q=q)

def rotangle(a,b):return float(np.rad2deg(np.arccos(np.clip((np.trace(a.T@b)-1)/2,-1,1))))
groups={};allrows=[]
for name,key in [('training45','training_captures'),('independent8','heldout_independent_captures')]:
    rows=[inputs[p] for p in d[key]];q=np.asarray([r['q'] for r in rows]);cams=np.asarray([fk(v)@X for v in q]);deg=np.rad2deg(q)
    groups[name]=dict(count=len(rows),joint_min_deg=deg.min(axis=0).tolist(),joint_max_deg=deg.max(axis=0).tolist(),joint_span_deg=np.ptp(deg,axis=0).tolist(),
        optical_axis_downward_cosine_range=[float((-cams[:,2,0]).min()),float((-cams[:,2,0]).max())],
        source_captures=[{k:v for k,v in r.items() if k!='q'} for r in rows])
    for view in current['viewpoints'][:3]:
        cq=np.asarray(view['q_rad']);cam=fk(cq)@X;diff=np.rad2deg(abs(q-cq));nearest=int(np.argmin(np.linalg.norm(diff,axis=1)));cdeg=np.rad2deg(cq)
        outs=np.maximum(deg.min(axis=0)-cdeg,0)+np.maximum(cdeg-deg.max(axis=0),0)
        angles=[rotangle(cam[:3,:3],t[:3,:3]) for t in cams]
        axes=[float(np.rad2deg(np.arccos(np.clip(cam[:3,0]@t[:3,0],-1,1)))) for t in cams]
        allrows.append(dict(group=name,source_stamp_ns=view['source_stamp_ns'],joint_deg=cdeg.tolist(),
            outside_joint_interval_deg=outs.tolist(),nearest_capture_joint_l2=rows[nearest]['path'],
            nearest_joint_l2_deg=float(np.linalg.norm(diff[nearest])),nearest_joint_max_abs_deg=float(diff[nearest].max()),
            minimum_camera_rotation_distance_deg=min(angles),minimum_optical_axis_distance_deg=min(axes),
            nearest_camera_rotation_capture=rows[int(np.argmin(angles))]['path']))
report=dict(declaration=str(DECL),declaration_sha256=hashlib.sha256(DECL.read_bytes()).hexdigest(),model_sha256=c['urdf_sha256'],
    groups=groups,current_comparisons=allrows,
    method='Read exact declared selected source samples from small capture JSON files and verify their recorded SHA256; recompute all camera poses with the same current expanded-metric URDF/mount for a comparable pose-domain diagnostic.',
    limitations=['Training45 is fit data, not independent accuracy validation.','Axis-aligned joint intervals do not prove coverage of a six-dimensional joint pose or camera pose.','Only the specifically documented original independent8 and training45 are compared; later prospective validation datasets are not scanned.','Comparable FK camera pose domains do not independently verify physical camera orientation accuracy.'],
    bags_scanned=False,production_changes=False,robot_commands=False)
p=OUT/'historical_pose_domain.json'
with p.open('x') as file:json.dump(report,file,indent=2,ensure_ascii=False);file.write('\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),groups={k:{a:b for a,b in v.items() if a!='source_captures'} for k,v in groups.items()},comparisons=allrows)))
