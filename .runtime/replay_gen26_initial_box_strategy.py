"""Immutable initial-frame inference/proposal evidence, no ROS/device calls."""
from pathlib import Path
import hashlib,json
import numpy as np
from alicia_flexible_grasp.vision.regular_box_inference import (
    qualify_regular_box_top,enclose_inferred_box_geometry,inferred_box_enclosure_vertices)
from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry,evaluate_inferred_box_contact
from alicia_flexible_grasp.grasp.tabletop_geometry_candidates import TabletopGeometryConfig,generate_tabletop_proposals

W=Path(__file__).resolve().parents[1]
P=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen26_box_inference_design')
OUT=P/'initial_strategy_replay'
assert Path('/media/zhuyupei/SanStand').is_mount()
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
sources=[W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp'/s for s in (
    'vision/regular_box_inference.py','grasp/gripper_geometry.py','grasp/tabletop_geometry_candidates.py')]
source_hashes={str(p):sha(p) for p in sources}
manifest=json.loads((P/'source_manifest.json').read_text());g=manifest['source_geometry']
audit=json.loads(Path(manifest['audit']).read_text())
row=next(row for row in manifest['rows'] if row['source_stamp_ns']==1792033654741969108)
assert sha(row['npz'])==row['sha256']
frame=np.load(row['npz']);O=np.eye(4);O[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
T=frame['T_base_camera']@O
geometry=GeometryEstimate(True,'','',g['obb_center_base_m'],g['R_base_obb'],g['obb_size_xyz_m'],g['support_normal_base'],g['support_offset_m'],g.get('support_inlier_ratio',1.),g['object_points_base_m'],'instance_mask')
identity=TargetTrackIdentity.from_stream(audit['generation'],audit['target_epoch'])
kw=dict(depth_raw=frame['depth_raw'],object_mask=frame['object_mask'],camera_intrinsics=frame['intrinsics'],
        depth_scale_m=float(frame['depth_scale_m']),T_base_camera=T,geometry=geometry,
        source_identity=identity,source_stamp_ns=int(frame['source_stamp_ns']))
without_color=qualify_regular_box_top(**kw)
qualified=qualify_regular_box_top(**kw,color_bgr=frame['color'])
assert qualified.ok,(qualified.code,qualified.audit)
model=qualified.model
enclosure,enclosure_audit=enclose_inferred_box_geometry(geometry,model)
gripper=GripperGeometry(.050,.002,np.array([.0434,.0286,.060]),np.array([.1175,.155,.0774]),.003)
normal=model.support_normal_base
contacts=[]
for index in (0,1):
    contact=evaluate_inferred_box_contact(model,model.center_base,model.axes_base[:,index],-normal,gripper,
        support_normal_base=normal,opening_fit_clearance_each_side_m=.002)
    contacts.append(dict(axis_index=index,ok=contact.ok,code=contact.code,
        inferred_width_m=contact.inferred_width_m,width_uncertainty_m=contact.width_uncertainty_m,
        contact_height_bounds_m=contact.contact_height_bounds_m,targeted_view=contact.targeted_view))
proposals=generate_tabletop_proposals(enclosure.object_points_base,enclosure.center_base,enclosure.axes_base,
    enclosure.size_xyz_m,-model.support_offset_m*normal,normal,TabletopGeometryConfig(),finger_geometry=gripper,inferred_box=model)
report=dict(schema_version=1,source_manifest=str(P/'source_manifest.json'),source_manifest_sha256=sha(P/'source_manifest.json'),
    source_frame=row['npz'],source_frame_sha256=row['sha256'],source_audit=manifest['audit'],source_audit_sha256=sha(manifest['audit']),
    coordinate_conversion=dict(original='base_from_ROS_camera_link',used='base_from_OpenCV_optical',
       right_multiply=O.tolist(),result_sha256=hashlib.sha256(np.ascontiguousarray(T).tobytes()).hexdigest()),
    original_geometry_contains_actual_measured_points=True,source_point_count=len(geometry.object_points_base),
    depth_only=dict(ok=without_color.ok,code=without_color.code,audit=dict(without_color.audit)),
    rgb_and_depth=dict(ok=qualified.ok,code=qualified.code,audit=dict(qualified.audit)),
    inferred_model=model.to_evidence_dict(),collision_enclosure=enclosure_audit,
    collision_vertices=inferred_box_enclosure_vertices(model).tolist(),contacts=contacts,
    proposals=dict(ok=proposals.ok,code=proposals.failure_code,count=len(proposals.proposals),
        rejections=proposals.surface_evidence_rejections),source_hashes=source_hashes,
    source_unchanged=all(sha(p)==digest for p,digest in source_hashes.items()),
    no_ros_calls=True,no_hardware_access=True,no_temperature_reads=True,
    conclusion='Complete RGB top with central metric depth qualifies an explicit inferred box; this recorded frame still requests targeted precision observation because the full unresolved mask-edge strip plus yaw uncertainty consumes the physical 50 mm aperture budget.',
    limitations=['This is offline geometry/proposal replay, not motion or physical grasp success.',
      'Empirical uncertainty budgets are not certified calibration error bounds.',
      'The camera-link TF is preserved unchanged in the original NPZ; the explicit optical conversion above matches production deprojection.'])
assert report['source_unchanged']
OUT.mkdir(exist_ok=False)
p=OUT/'findings.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
receipt=dict(findings=str(p),sha256=sha(p),runner=str(Path(__file__).resolve()),runner_sha256=sha(__file__),source_hashes=source_hashes)
(OUT/'receipt.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(receipt=receipt,contacts=[{k:v for k,v in c.items() if k!='targeted_view'} for c in contacts],proposal_code=proposals.failure_code),ensure_ascii=False))
