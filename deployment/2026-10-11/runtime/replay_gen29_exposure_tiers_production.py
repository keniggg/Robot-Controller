"""Bounded offline proposal comparison; no ROS operations or production edits.

The counterfactual changes only the optimizer's requested exposure. Actual
endpoint acceptance still calls every production gate with original noise.
"""
import argparse
import hashlib
import inspect
import itertools
import json
from pathlib import Path
import time

import numpy as np
from alicia_grasp_modes import reachable_observation_views as views
from alicia_grasp_modes import observation_arrival_projection as projection
from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ahash(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()


p = argparse.ArgumentParser()
p.add_argument('--manifest', required=True)
p.add_argument('--output', required=True)
args = p.parse_args()
meta_path = Path(args.manifest)
meta = json.loads(meta_path.read_text())
frame = next(row for row in meta['frames'] if row['source_stamp_ns'] == 1791549916246385812)
out = Path(args.output)
out.mkdir(parents=True, exist_ok=True)
assert not (out/'result.json').exists()
audit_path = Path(meta['record'])/'keyframes_audits/8b168b430bcec6b8fac454723d8f4ff176df9d98032009a3c614dc11137c6bdf.json'
audit = json.loads(audit_path.read_text())
selected = audit['selected']
source_audit = selected['observation_source_audit']
search = selected['observation_view']['proposal_search']
raw_path = Path(frame['npz'])
raw = np.load(raw_path)
camera = frame['camera']
status = frame['ready_status']
if isinstance(status, list):
    assert len(status) == 1
    status = status[0]
if 'message' in status:
    status = status['message']
stamp = audit['snapshot_stamp_ns']
assert status['state'] == 'ready' and status['source_stamp_ns'] == stamp
assert ahash(raw['depth_raw']) == source_audit['raw_depth_sha256']
assert ahash(raw['raw_mask']) == source_audit['raw_mask_sha256']
assert ahash(raw['tool_from_camera']) == source_audit['tool_camera_sha256']
g = audit['replay_geometry']
depth, mask, T = raw['depth_raw'], raw['raw_mask'], raw['T_base_camera']
ys, xs = np.nonzero((depth > 0) & (mask == 0))
z = depth[ys, xs].astype(float)*camera['depth_scale']
optical_points = np.column_stack(((xs-camera['cx'])*z/camera['fx'],
    (ys-camera['cy'])*z/camera['fy'], z))
C = np.eye(4)
C[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
optical = T@C
points = optical_points@optical[:3, :3].T+optical[:3, 3]
normal, offset = np.asarray(g['support_normal_base']), g['support_offset_m']
support = points[np.abs(points@normal+offset) <= .002]
assert len(support) == source_audit['support_point_count']
center, R, size = map(np.asarray, (g['obb_center_base_m'], g['R_base_obb'], g['obb_size_xyz_m']))
corners = center+(np.asarray(list(itertools.product((-.5, .5), repeat=3)))*size)@R.T
assert ahash(corners) == source_audit['conservative_target_corners_sha256']
selection = dict(mode='unknown', strategy='observe_return',
    generation=status['generation'], stamp_ns=status['selection_stamp_ns'])
contract = projection.capture_arrival_projection(status, selection, stamp, camera)
assert contract['audit'] == source_audit['arrival_projection']
source = dict(binding=selected['observation_source_binding'],
    camera=source_audit['camera_parameters'], points_base=support,
    corners_base=corners, center_base=center, tool_from_camera=raw['tool_from_camera'],
    geometry=dict(support_normal_base=normal, support_offset_m=offset,
        R_base_obb=R, obb_size_xyz_m=size), view_limits=source_audit['view_limits'],
    arrival_projection=contract)
model_path = Path('.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf')
fk = SerialUrdfFk(model_path.read_text(), ['Joint%d' % i for i in range(1, 7)])
assert fk.model_sha256 == search['model_sha256']
common = dict(source=source, current_joint_positions=search['current_joint_positions'], fk=fk,
    gripper=GripperGeometry(.05, .002, np.array([.0434, .0286, .06]), np.array([.1175, .155, .0774]), .003),
    opening_width_m=.05, gaps=audit['observe_return']['measured_missing_sides'],
    depth_uncertainty_m=.003, side_depth_uncertainty_m=search['optical_side_uncertainty_m'],
    joint_error_bounds_rad=search['joint_error_bounds_rad'], max_directions=2,
    max_iterations=96, max_seconds=2., proposal_image_reserve_px=4.)
paths = [Path(views.__file__), Path(projection.__file__), model_path]
before = {str(path): sha(path) for path in paths}

from types import SimpleNamespace as NS
from copy import deepcopy
from test_remote_modes import module as remote_module, base
from alicia_grasp_modes.silhouette_observation_order import transform_fingerprint
from alicia_grasp_modes.observation_view_utility import predicted_information_utility

rows, report = views.generate_reachable_observation_views(**common)
assert report['status'] == 'complete' and len(rows) == 4, report
# Actual production scheduling with sealed source inputs. Only transport/time
# ownership is fixture state; this does not invoke strict MoveIt or grant a plan.
node = remote_module.ModeAwareRemoteGrasp6DNode.__new__(remote_module.ModeAwareRemoteGrasp6DNode)
node._unknown_observation_planning_active = lambda: True
node._joint_space_observation_active = lambda: True
node._diagnostic_observation_planning_active = lambda: False
node._independent_observation_planning_active = lambda: False
source = dict(source, base_from_tool=T@np.linalg.inv(raw['tool_from_camera']))
source['audit'] = dict(source_audit, frozen_tool_reference_sha256=transform_fingerprint(source['base_from_tool']))
binding = source['binding']
ticket = NS(request_id=binding['request_id'], generation=binding['generation'], target_epoch=binding['target_epoch'])
prepared = NS(ticket=ticket, snapshot=NS(stamp_ns=stamp,
    target_identity=NS(track_id=binding['target_track_id'])))
node._require_unknown_observation_prepared = lambda p: None if p is prepared else (_ for _ in ()).throw(ValueError('foreign prepared'))
node._require_diagnostic_observation_view_source = lambda p: source if p is prepared else (_ for _ in ()).throw(ValueError('foreign source'))
node._observation_endpoint_search_hints = lambda *_a: (_ for _ in ()).throw(AssertionError('advisory hints before first FK'))
node._active_unknown_observation_check_runtime = dict(prepared=prepared)
variants=[]
for proposal in rows:
    checks=proposal['endpoint_checks']; transform=np.asarray(proposal['T_base_tool0'])
    pose=base.make_pose_stamped('base_link', transform[:3,3], base.quaternion_from_matrix(transform),
        stamp=base.rospy.Time(stamp//10**9,stamp%10**9))
    gain=checks['missing_side_view_gain']
    benefit=predicted_information_utility(gain['predicted_new_side_gain_m'], gain['depth_uncertainty_m'],
        max(checks['working_volume']['target_corner_depths_m']), min(camera['fx'],camera['fy']))
    evidence=dict(information_utility=benefit, proposal=deepcopy(proposal))
    variants.append(dict(sequence=NS(pregrasp=pose,observation_view_audit=deepcopy(evidence)),
        observation_view=evidence, observation_envelope=NS(**checks['observation_envelope']), search_family='joint_space_fk'))
def summarise(v):
    a=v['observation_view']; p=a['proposal']; m=a['check_scheduling_metrics']
    return dict(tier=p['exposure_target_noise_multiplier'], gap=p['direction_gap_index'],
        missing_side=p['endpoint_checks']['missing_side_view_gain']['missing_side_sign'],
        gain_m=p['endpoint_checks']['missing_side_view_gain']['predicted_new_side_gain_m'],
        support_status=m['measured_support_schedule']['status'],
        information_geometry_rank=m['information_geometry_rank'],
        utility=a['information_utility']['utility'], joint_max_delta_rad=p['joint_max_delta_rad'])
rank_comparisons=[]
for label, inputs in [('baseline_only', [v for v in variants if v['observation_view']['proposal']['exposure_target_noise_multiplier']==1.]),
                       ('both_tiers', variants)]:
    node._observation_information_comparison=dict(ticket=ticket,deadline=None)
    ordered=node._order_unknown_observation_checks(inputs,defer_endpoint_hints=True)
    iterator=node._order_observation_checks(inputs)
    first=next(iterator); iterator.close()
    rank_comparisons.append(dict(label=label, order=[summarise(v) for v in ordered], first_fk=summarise(first)))
assert rank_comparisons[1]['first_fk']['tier']==2.
assert rank_comparisons[1]['first_fk']['missing_side']==-1
assert all(row['support_status']=='sufficient' for row in rank_comparisons[1]['order'])
after={str(path):sha(path) for path in paths};assert before==after
result=dict(scope='PRODUCTION_GENERATOR_AND_ORIGINAL_MODE_ORDER_OFFLINE_NO_STRICT_CALL',
    audit_path=str(audit_path),audit_sha256=sha(audit_path),source_manifest=str(meta_path),
    source_manifest_sha256=sha(meta_path),source_npz_sha256=sha(raw_path),
    support_hash_exact=ahash(support)==source_audit['support_points_sha256'],
    arrival_projection_audit_exact=True, source_stamp_ns=stamp,
    production_before=before,production_after=after,script_sha256=sha(__file__),
    candidates=rows,report=report,rank_comparisons=rank_comparisons,
    limitations=['Only transport/current ticket ownership mocked for ordering; actual FK, all endpoint gates, measured-support scheduler and mode first-FK ordering are production.',
        'Reconstructed frozen tool reference can differ by floating point roundoff from original audit hash.',
        'No strict MoveIt, full path/following, physical acquisition or common contact height is established.'])
(out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(count=len(rows),elapsed_sec=report['elapsed_sec'],rank_comparisons=rank_comparisons)),flush=True)
