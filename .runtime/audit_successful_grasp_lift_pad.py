"""Closed-bag initial lift contact diagnosis; no ROS master or command API."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag

from alicia_flexible_grasp.robot.observation_path_guard import FrozenObservationScene, SerialUrdfFk
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
from alicia_flexible_grasp.robot.lift_path_continuity import _intersection_vertices, _intersection_in_pad
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, _stage_boxes, _box_corners, _signed_polygon_margin,
    ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M as PAIR,
    ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M as PATCH,
    ANALYTICAL_FINGER_BOX_PADDING_XYZ_M as PADDING,
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('archive', type=Path)
    p.add_argument('--urdf', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--user-grip-statement', default=None)
    a = p.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    if not mount.is_mount() or mount not in a.output.resolve().parents:
        raise RuntimeError('mounted SanStand output required')
    raw = (a.archive/'runner.log').read_text().split('linear 6D lift planning failed: ', 1)[1]
    failure = json.JSONDecoder().raw_decode(raw)[0]
    evidence = failure['derivation']
    fk = SerialUrdfFk(a.urdf.read_text(), ARM_NAMES)
    if fk.model_sha256 != evidence['model_sha256']:
        raise ValueError('recorded model differs from replay model')
    found = None
    with rosbag.Bag(str(a.archive/'keyframes.bag')) as bag:
        for topic, plan, stamp in bag.read_messages(topics=[
                '/grasp_6d/plan_enriched', '/grasp_6d/preview_plan_enriched']):
            if str(plan.plan_id) == evidence['plan_id']:
                scene = FrozenObservationScene.from_contact_plan(plan)
                if (scene.geometry_digest != evidence['object_geometry_sha256']
                        or scene.digest != evidence['scene_sha256']):
                    raise ValueError('recorded lift/plan scene identity mismatch')
                found = dict(topic=topic, receipt_ns=stamp.to_nsec(),
                             contact_evidence=json.loads(plan.contact_geometry_evidence))
                break
    if found is None:
        raise ValueError('recorded exact plan not found')
    # These are the original frozen 50 mm analytical hardware dimensions.
    gripper = GripperGeometry(.05, .002, np.array([.0434, .0286, .06]),
                             np.array([.1175, .155, .0774]), .003)
    opening = evidence['accepted_opening_m']
    T = fk(evidence['measured_start_positions_rad'])
    local_center = T[:3, :3].T@(scene.center-T[:3, 3])
    local_rotation = T[:3, :3].T@scene.rotation
    bodies = []
    for name, center, size in _stage_boxes(np.eye(4), gripper, opening, 'y', 'z'):
        if 'finger' not in name:
            continue
        vertices = _intersection_vertices(center, np.eye(3), size,
                                           local_center, local_rotation, scene.size)
        result = dict(name=name, original_predicate_pass=bool(_intersection_in_pad(
            T, scene.center, scene.rotation, scene.size, name, gripper, opening)),
            intersection_numerically_resolved=vertices is not None)
        if vertices is not None:
            projection = vertices[:, [0, 2]]-PAIR[[0, 2]]
            margins = np.array([_signed_polygon_margin(q, PATCH) for q in projection])
            side = 1. if name == 'left_finger' else -1.
            penetration = side*(vertices[:, 1]-PAIR[1])-.5*opening
            corners = _box_corners(T[:3, :3]@center+T[:3, 3], T[:3, :3], size)
            result.update(vertices_tool_m=vertices.tolist(), projected_xz_relative_to_pair_m=projection.tolist(),
                pad_signed_margin_m=margins.tolist(), minimum_pad_margin_m=float(margins.min()),
                penetration_range_m=[float(penetration.min()), float(penetration.max())],
                penetration_allowed_range_m=[-.5*PADDING[1], .5*gripper.finger_size_xyz_m[1]],
                minimum_support_clearance_m=float(np.min(corners@scene.normal+scene.offset)))
        bodies.append(result)
    report = dict(schema=1, scope='exact_initial_geometry_only_closed_bag',
        plan_id=evidence['plan_id'], scene_sha256=scene.digest,
        object_geometry_sha256=scene.geometry_digest, model_sha256=fk.model_sha256,
        accepted_opening_m=opening, accepted_stamp_ns=evidence['accepted_stamp_ns'],
        measured_joints_rad=evidence['measured_start_positions_rad'],
        T_base_tool_measured=T.tolist(), target_tool_center_m=local_center.tolist(),
        target_tool_rotation=local_rotation.tolist(), target_size_m=scene.size.tolist(),
        finger_pad_polygon_relative_xz_m=PATCH.tolist(), pad_bodies=bodies,
        matched_plan=found, sources={'runner_log':str(a.archive/'runner.log'),
            'runner_sha256':sha(a.archive/'runner.log'), 'bag':str(a.archive/'keyframes.bag'),
            'urdf':str(a.urdf), 'script_sha256':sha(Path(__file__))},
        user_observation={'statement':a.user_grip_statement,
            'source':('user confirmation for this exact attempt' if a.user_grip_statement else 'no user statement supplied'),
            'independent_of_automatic_geometry':True},
        attachment_confirmed_by_automatic_evidence=False, lift_executed=False,
        motion_authorized=False, production_changed=False,
        limitations=['A padded rectangular finger envelope is a conservative broad phase, not the exact tapered STL solid.',
            'A negative full-envelope intersection margin does not prove real material collision outside a pad.',
            'This replay does not certify an alternate mesh predicate or any complete lift path.',
            'World and hypothetical carried occupancy are unchanged; user confirmation is not fabricated attachment evidence.'])
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output/'initial_pad_audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'output':str(a.output), 'matched_geometry_sha256':scene.geometry_digest,
        'bodies':[{k:v for k,v in x.items() if k in ('name','original_predicate_pass','intersection_numerically_resolved',
                    'minimum_pad_margin_m','penetration_range_m','minimum_support_clearance_m')} for x in bodies]}, indent=2))


if __name__ == '__main__':
    main()
