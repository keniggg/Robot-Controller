"""Bind saved runtime K to an exact frozen-audit projection, offline only."""
import hashlib,json
from pathlib import Path
import numpy as np
from tf.transformations import quaternion_matrix
from hover_camera_intrinsics_20260927 import OUT, Projection, provenance
assert Path('/media/zhuyupei/SanStand').is_mount()
record=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20260927T071553.378661Z_f557b777f3ee4ad48b4e05890fdaac34')
audit_path=record/'keyframes_audits/ba37e0297d84de8e212d6062459e48a7e60118ced2df49dd81b8a33281d66fb4.json'
a=json.loads(audit_path.read_text());frames=json.loads((OUT/'rgbd_frame_metadata.json').read_text())
r=next(x for x in frames['static'] if x['parent']=='tool0' and x['child']=='camera_link')
toolcam=quaternion_matrix(r['xyzw']);toolcam[:3,3]=r['xyz'];O=np.eye(4);O[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
l=a['lineage'][0];stage=l['execution_sequence']['stages'][0];T=quaternion_matrix(stage['quaternion_xyzw']);T[:3,3]=stage['position_m'];T=T@toolcam@O
point=T[:3,:3].T@(np.array(a['target_cloud_center_base'])-T[:3,3]);metric=l['latest_soft_evidence']['visibility_metrics'][0]
expected=np.array([metric['u'],metric['v']]);actual=Projection().project(point)[0]
old=np.array([438.60040283203125*point[0]/point[2]+317.7792663574219,438.4169616699219*point[1]/point[2]+245.35714721679688])
res={'scope':'runtime_intrinsics_snapshot_crosschecked_against_same_record_frozen_audit_not_new_calibration','runtime':provenance(),
     'record_id':record.name,'plan_id':a['plan_id'],'audit_path':str(audit_path),'audit_sha256':hashlib.sha256(audit_path.read_bytes()).hexdigest(),
     'audit_snapshot_stamp_ns':a['snapshot_stamp_ns'],'frozen_raw_depth_stamp_ns':frames['frozen']['images']['depth']['stamp_ns'],
     'audit_projection_source':'lineage[0].latest_soft_evidence.visibility_metrics[0]; reconstructed using its execution_sequence pregrasp, target_cloud_center_base, and bag static handeye',
     'expected_uv_px':expected.tolist(),'active_sdk_pinhole_uv_px':actual.tolist(),'active_sdk_uv_error_px':(actual-expected).tolist(),
     'stale_yaml_uv_px':old.tolist(),'stale_yaml_uv_error_px':(old-expected).tolist(),
     'audit_depth_m':metric['depth_m'],'reconstructed_depth_m':float(point[2]),
     'depth_scale_evidence':'saved runtime namespace equals active profile SDK depth sensor scale; units are meters per uint16 unit; no per-frame CameraInfo in bag',
     'raw_frozen_depth_sha256':hashlib.sha256(np.load(OUT/'frozen_depth.npy').tobytes()).hexdigest(),
     'graspnet_input_depth_sha256':a['graspnet_input']['depth_sha256'],
     'graspnet_input_is_processed_context_depth_not_full_raw':'remote_grasp6d_node.py _build_graspnet_input uses build_graspnet_input_context result.depth_raw; these two hashes are not expected to match'}
assert max(abs(actual-expected))<1e-9
(OUT/'intrinsics_audit_crosscheck.json').write_text(json.dumps(res,indent=2))
frames['camera_intrinsics_evidence']=provenance();(OUT/'rgbd_frame_metadata.json').write_text(json.dumps(frames,indent=2))
print(json.dumps({k:v for k,v in res.items() if k!='runtime'},indent=2))
