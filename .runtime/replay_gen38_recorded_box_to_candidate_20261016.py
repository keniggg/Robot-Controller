"""Exact closed-record pixels through box and contact-family admission only.

No historical fused surface is reconstructed from the collision union. This
does not replay IK, the missing old continuous curve, or a physical grasp.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
from alicia_flexible_grasp.vision.regular_box_inference import (
    qualify_regular_box_top, enclose_inferred_box_geometry, inferred_box_enclosure_vertices)
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, evaluate_inferred_box_contact)

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
D = E/'ros_log/full_latest_gen36_restart_20261017T012037Z'
G = E/'grasp_records/20261017T032113.476278Z_a84cc89fc53e4c2b82680749d78b9b9f'
I = D/'verification/gen37_actual_attempt1_source_images_r1'
ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument('--output', type=Path, required=True)
a = ap.parse_args()
assert Path('/media/zhuyupei/SanStand').is_mount() and 'sitecustomize' in sys.modules
a.output.resolve().relative_to(D/'verification')
a.output.mkdir(exist_ok=False)
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
read = lambda p: json.loads(Path(p).read_text())
def plain(v):
    if isinstance(v, np.ndarray): return v.tolist()
    if isinstance(v, np.generic): return v.item()
    raise TypeError(type(v).__name__)
sources = [W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp'/p for p in (
    'vision/regular_box_inference.py', 'vision/cuboid_rgb_interval.py',
    'vision/cuboid_silhouette.py', 'grasp/gripper_geometry.py')]
before = {str(p): sha(p) for p in sources}
manifest = read(G/'manifest.json')
assert manifest['state'] == 'complete' and manifest['recorder_closed'] and manifest['result'] == 'failure'
path = G/'keyframes_audits/0e5ebbd3c562cfab2617b338045f554ffc0920a23626fbc2a49cc474cc9d798d.json'
assert sha(path) == path.stem
audit = read(path)
q = audit['remote_diagnostics']['tabletop_geometry']['regular_box_inference']
stamp = audit['snapshot_stamp_ns']
assert stamp == q['source_stamp_ns'] == 1792207340733134031
extraction = read(I/'receipt.json')
pixels = {}
for role, suffix in (('color', '.png'), ('mask', '.png'), ('depth', '.npy')):
    p = I/(str(stamp)+'_'+role+suffix)
    assert sha(p) == extraction['output_sha256'][p.name]
    pixels[role] = np.load(p, allow_pickle=False) if role == 'depth' else cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
    assert hashlib.sha256(pixels[role].tobytes()).hexdigest() == q['retained_pixel_sha256'][role+'_sha256']
camera = read(D/'gen37_runtime/gen37_grasp_scene_r2/camera_configuration.json')
K = np.asarray([camera[k] for k in ('fx', 'fy', 'cx', 'cy')])
g = audit['replay_geometry']
points = np.asarray(g['object_points_base_m'])
assert hashlib.sha256(points.tobytes()).hexdigest() == g['object_points_sha256']
geometry = GeometryEstimate(True, '', '', np.asarray(g['obb_center_base_m']),
    np.asarray(g['R_base_obb']), np.asarray(g['obb_size_xyz_m']),
    np.asarray(g['support_normal_base']), g['support_offset_m'],
    g['support_inlier_ratio'], points, g['source_mode'])
T = np.asarray(audit['snapshot_transform']['T_base_optical'])
identity = TargetTrackIdentity(**q['source_target_identity'])
result = qualify_regular_box_top(pixels['depth'], pixels['mask'], K,
    camera['depth_scale'], T, geometry, identity, stamp, geometry.source_mode,
    color_bgr=pixels['color'])
report = dict(kind='gen38_exact_record_box_and_contact_family_replay',
    scope='box_qualification_and_pure_contact_admission_not_whole_task',
    source_stamp_ns=stamp, record=str(G), original_result='failure',
    historical_failure=q['code'], actual_production_code=result.code,
    box_qualified=result.ok, audit=dict(result.audit),
    original_record_reclassified=False, contact_or_motion_authorized=False,
    physical_grasp_verified=False, ros_initialized=False,
    temperature_values_read=False, motion_commands_sent=False,
    inputs_sha256={str(p): sha(p) for p in (path, G/'manifest.json', I/'receipt.json')},
    source_sha256=before)
if result.ok:
    model = result.model
    gripper = GripperGeometry(.05, .002, np.array([.0434, .0286, .06]),
        np.array([.1175, .155, .0774]), .003)
    reports = []
    for axis in (0, 1):
        contact = evaluate_inferred_box_contact(model, model.center_base,
            model.axes_base[:, axis], -model.support_normal_base, gripper,
            opening_fit_clearance_each_side_m=.0005)
        reports.append(dict(axis=axis, ok=contact.ok, code=contact.code,
            inferred_width_m=contact.inferred_width_m,
            width_uncertainty_m=contact.width_uncertainty_m,
            targeted_view=dict(contact.targeted_view)))
    scene, enclosure = enclose_inferred_box_geometry(geometry, model)
    assert np.array_equal(scene.object_points_base, points)
    report.update(model=model.to_evidence_dict(), principal_contact_admission=reports,
        accepted_principal_contact_count=sum(row['ok'] for row in reports),
        collision_enclosure=enclosure,
        inferred_collision_vertices=inferred_box_enclosure_vertices(model),
        accepted_historical_measurements_preserved=True,
        historical_fused_surface_not_reconstructed=True)
after = {str(p): sha(p) for p in sources}
assert after == before
report.update(source_unchanged=True,
    limitations=['This result alone does not establish candidate IK, full path feasibility, '
        'task delivery, fresh actual following or physical grip.'])
(a.output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, default=plain)+'\n')
print(json.dumps(dict(output=str(a.output), box_qualified=result.ok,
    code=result.code, accepted_contact_count=report.get('accepted_principal_contact_count'))))
