"""Sealed gen25 audit and exported exact-TF projection comparison; no ROS IO."""
import hashlib
import itertools
import json
import math
from pathlib import Path
import re
import numpy as np
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask, rectification_contract

D = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_ros_20261009T003407Z/gen25_handoff_analysis')
M = json.loads((D / 'source_images/manifest.json').read_text())
A = json.loads((D / 'arrival_and_terminal.json').read_text())
R = Path(M['record']) / 'keyframes_audits'
AUDITS = ['4121ad85d2a1860e8ffc8ef41295a523d44c1489b642c717430ef7a6f53fb1fc', 'f285d9f5e25bcb113c3770ebe3f7b47c2f1e82de4d171a075e7629d6664e0d3e', 'e319974f04dabbffbf5bbaaa9556193b6c2978a352679318fb1f2dd41620ac24']
ACTUAL = [1791544640948935270, 1791544654717986583, 1791544674989701986]
camera = M['camera']
valid = rectification_rgb_valid_mask(camera, (camera['height'], camera['width']))
contract = rectification_contract(camera)
fp = np.array(M['anchor_status']['reference_support_footprint_base'], dtype=np.float64)
plane = np.array(M['anchor_status']['reference_support_plane_base'], dtype=np.float64)
height = M['anchor_status']['geometry_metrics']['median_height_m']
prism = np.vstack((fp, fp + height * plane[:3] / np.linalg.norm(plane[:3])))

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def array_sha(value):
    return hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest()

def project(points, transform, k):
    T = np.array(transform)
    xyz = (points - T[:3, 3]) @ T[:3, :3]
    uv = np.column_stack((k['cx'] - k['fx'] * xyz[:, 1] / xyz[:, 0], k['cy'] - k['fy'] * xyz[:, 2] / xyz[:, 0]))
    margin = min(float(uv[:, 0].min()), float(uv[:, 1].min()), float(k['width'] - 1 - uv[:, 0].max()), float(k['height'] - 1 - uv[:, 1].max()))
    bounds = [int(math.floor(uv[:, 0].min())), int(math.floor(uv[:, 1].min())), int(math.ceil(uv[:, 0].max())) + 1, int(math.ceil(uv[:, 1].max())) + 1]
    x0, y0, x1, y1 = bounds
    clipped = [max(0, x0), max(0, y0), min(k['width'], x1), min(k['height'], y1)]
    invalid = None if valid is None else int(np.count_nonzero(~valid[clipped[1]:clipped[3], clipped[0]:clipped[2]]))
    return dict(uv=uv.tolist(), uv_min=uv.min(axis=0).tolist(), uv_max=uv.max(axis=0).tolist(), image_margin_px=margin, min_depth_m=float(xyz[:, 0].min()), max_depth_m=float(xyz[:, 0].max()), projected_bbox_xyxy=bounds, bbox_outside_canvas=bool(x0 < 0 or y0 < 0 or x1 > k['width'] or y1 > k['height']), invalid_rgb_pixels_in_clipped_bbox=invalid)

rows = []
for idx, (aid, source_ns) in enumerate(zip(AUDITS, ACTUAL), 1):
    path = R / (aid + '.json')
    audit = json.loads(path.read_text())
    selected = audit['selected']
    geom = audit['replay_geometry']
    obb = np.asarray(geom['obb_center_base_m']) + (np.asarray(list(itertools.product((-.5, .5), repeat=3))) * np.asarray(geom['obb_size_xyz_m'])) @ np.asarray(geom['R_base_obb']).T
    source = selected['observation_source_audit']
    proposal = selected['observation_view']['proposal']
    trajectory = selected['observation_orientation_search']['variant_results'][0]['following_support']['unknown_observation_view']['target_trajectory']
    npz_path = D / ('source_images/view%d_arrival_%d.npz' % (idx, source_ns))
    data = np.load(str(npz_path), allow_pickle=False)
    T_actual = data['T_base_camera']
    Ts = dict(nominal_proposal=proposal['T_base_camera_link'], strict_goal=trajectory['endpoint_transform_base_camera'], measured_exact_source=T_actual.tolist())
    goal_q = json.loads(re.search(r'observation_goal_joint_positions=(\[[^\]]+\])', selected['moveit']['reason']).group(1))
    settle = A['views'][idx-1]['arrival_request']['settle']
    actual_q = settle['accepted_positions_rad']
    strict_T = np.asarray(Ts['strict_goal'])
    angular = math.degrees(math.acos(float(np.clip((np.trace(strict_T[:3, :3].T @ T_actual[:3, :3])-1)/2, -1., 1.))))
    mount = np.linalg.inv(np.asarray(proposal['T_base_tool0'])) @ np.asarray(proposal['T_base_camera_link'])
    rows.append(dict(view_index=idx, request_id=audit['request_id'], plan_id=audit['plan_id'], source_stamp_ns=audit['snapshot_stamp_ns'], audit_path=str(path), audit_file_sha256=sha(path), source_audit=source, source_obb=geom, source_obb_corners_sha256=array_sha(obb), source_obb_sha_matches_recorded=array_sha(obb)==source['conservative_target_corners_sha256'], measured_source_stamp_ns=source_ns, measured_source_kind='third qualified source' if idx < 3 else 'seventh exact online rejected source; first rejected source RGB-D was not saved', measured_npz_path=str(npz_path), measured_npz_sha256=sha(npz_path), camera_transforms=Ts, tool_from_camera_reconstructed_from_recorded_proposal=mount.tolist(), mount_reconstruction_is_not_bitexact_hash_authority=True, strict_goal_joint_positions_rad=goal_q, measured_settle=settle, measured_settle_minus_strict_goal_rad=(np.asarray(actual_q)-goal_q).tolist(), measured_exact_camera_pose_error_vs_strict=dict(position_m=float(np.linalg.norm(T_actual[:3,3]-strict_T[:3,3])), orientation_deg=angular), original_strict_path_view={k:trajectory[k] for k in ('minimum_sampled_image_edge_distance_px','required_image_margin_px','robot_description_sha256','sample_count','certifies_continuous_visibility','trajectory_sha256')}, predicted_information_gain=selected['observation_view']['observe_return_missing_side_gain'], previous_registered_gain=audit['observe_return'].get('previous_gain'), projection={name:dict(planner_snapshot_obb=project(obb,T,source['camera_parameters']), frozen_arrival_footprint_prism=project(prism,T,source['camera_parameters'])) for name,T in Ts.items()}))
    # Dense object points are provenance inputs but unnecessary in this compact report.
    rows[-1]['source_obb']={k:v for k,v in geom.items() if k!='object_points_base_m'}

model=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade/.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf')
out=dict(record=M['record'], record_manifest_sha256=A['manifest_sha256'], sealed_manifest=A['sealed_manifest'], read_only=True, no_ros_calls=True, input_files={str(D/'source_images/manifest.json'):sha(D/'source_images/manifest.json'),str(D/'arrival_and_terminal.json'):sha(D/'arrival_and_terminal.json')}, camera_parameters={k:camera[k] for k in ('width','height','fx','fy','cx','cy','depth_scale','intrinsics_source')}, rectification_contract=contract, rgb_valid_mask_sha256=array_sha(valid), frozen_prism=dict(anchor_source_stamp_ns=M['anchor_source_stamp_ns'], footprint_base=fp.tolist(), footprint_sha256=array_sha(fp), support_plane_base=plane.tolist(), support_plane_sha256=array_sha(plane), measured_height_m=height, corners_base=prism.tolist(), source='Original acquired anchor/status; reused unchanged by arrival spatial support, no fitted or expanded replacement'), model_file=dict(path=str(model), exists=model.exists(), file_sha256=sha(model) if model.exists() else None, recorded_robot_description_sha256=rows[0]['original_strict_path_view']['robot_description_sha256']), views=rows, limitations=['Pure projection of saved audit geometry and exact same-source TF; not a new motion, collision or full-path certification.', 'Arrival settle q and exact RGB-D camera TF are separately timestamped; view 3 feedback varied by at most one Joint2 encoder count during its window.', 'Planner snapshot OBB and immutable acquired footprint prism are different measured geometry contracts. Their margins must not be compared as if the same target set.', 'The required RGB region is union of full first16 seed crop and prism plus4px guard. This report projects prism bbox only; exact required-region/343invalid evidence is in rgb_required_region_trace/result.json.', 'Strict path image-edge margin is measured against the canvas, not proof that the immutable arrival required-region contains only observed RGB.'])
dest=D/'planned_actual_projection.json'
dest.write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(dict(path=str(dest),sha256=sha(dest),views=[dict(view_index=x['view_index'],obb_hash_matches=x['source_obb_sha_matches_recorded'],margins={k:{g:round(vv['image_margin_px'],6) for g,vv in v.items()} for k,v in x['projection'].items()},endpoint_error=x['measured_exact_camera_pose_error_vs_strict']) for x in rows]),indent=2))
