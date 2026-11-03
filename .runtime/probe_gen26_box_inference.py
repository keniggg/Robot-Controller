import json
from pathlib import Path
import numpy as np
from alicia_flexible_grasp.vision.regular_box_inference import qualify_regular_box_top
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
P=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen26_box_inference_design')
f=np.load(P/'source_1792033654741969108.npz')
s=json.loads((P/'source_manifest.json').read_text());g=s['source_geometry']
T=f['T_base_camera'];O=np.eye(4);O[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];T=T@O
geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g.get('support_inlier_ratio',1.),g['object_points_base_m'],'instance_mask')
audit=json.loads(Path(s['audit']).read_text())
identity=TargetTrackIdentity.from_stream(audit['generation'],audit['target_epoch'])
r=qualify_regular_box_top(f['depth_raw'],f['object_mask'],f['intrinsics'],float(f['depth_scale_m']),T,geometry,identity,int(f['source_stamp_ns']),color_bgr=f['color'])
print(r.ok,r.code)
print(json.dumps(dict(r.audit),indent=2))
if r.ok:
 print('model',r.model.model_sha256,'size',r.model.size_xyz_m.tolist(),'edge_uncertainty',r.model.empirical_edge_uncertainty_m,'yaw',r.model.empirical_yaw_uncertainty_rad)
 (P/'qualified_initial_model_v1.json').write_text(json.dumps(r.model.to_evidence_dict(),indent=2)+'\n')
