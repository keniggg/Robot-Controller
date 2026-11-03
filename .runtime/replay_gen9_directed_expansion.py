#!/usr/bin/env python3
"""Frozen-file endpoint replay; no ROS node, parameter read, service or action."""
import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

import numpy as np
import rosbag
import rospy
from tf.transformations import quaternion_matrix

ROOT = Path(__file__).resolve().parents[1]
for path in ('src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0, str(ROOT / path))
from alicia_grasp_modes.observe_return_visual import (
    directed_missing_side_rotations, missing_side_view_gain,
    bounded_existing_camera_rolls)
from alicia_grasp_modes.observation_policy import observation_distance_preferences
from alicia_flexible_grasp.grasp.gripper_geometry import (
    GripperGeometry, evaluate_open_gripper_observation_envelope)
from alicia_flexible_grasp.grasp.observation_working_volume import (
    observation_obb_corners, validate_planned_observation_working_volume)

RECORD = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/'
              '20261006T062011.775107Z_5dbdfcdf7c6541878f7d7920b816955f')
INITIAL = RECORD / 'keyframes_audits/40024ab112e2b3a33683a76091116b7672e7908c255508f841ec4409d70f2b1d.json'
GAPS = RECORD / 'keyframes_audits/cbcf591d9f3f5384d5c5ae86038f59f4ad552e092b2a5d56b4eeb040eb63b04c.json'
SOURCES = [
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observation_policy.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/gripper_geometry.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/grasp/observation_working_volume.py',
    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_view_limits.py',
]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    frozen = out / 'sources'
    frozen.mkdir()
    hashes = {}
    for relative in SOURCES + ['.runtime/replay_gen9_directed_expansion.py']:
        path = ROOT / relative
        payload = path.read_bytes()
        hashes[relative] = digest(payload)
        (frozen / path.name).write_bytes(payload)
    manifest = json.loads((RECORD / 'manifest.json').read_text())
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
    initial = json.loads(INITIAL.read_text())
    carrier = json.loads(GAPS.read_text())
    assert digest(INITIAL.read_bytes()) == INITIAL.stem
    assert digest(GAPS.read_bytes()) == GAPS.stem
    shutil.copyfile(INITIAL, out / 'initial_audit.json')
    shutil.copyfile(GAPS, out / 'gap_carrier_audit.json')
    stamp = initial['snapshot_stamp_ns']
    assert carrier['observe_return']['initial_surface_stamp_ns'] == stamp
    gaps = carrier['observe_return']['measured_missing_sides']
    assert all(g['source_view_stamps_ns'] == [stamp] for g in gaps)
    assert all(g['target_epoch'] == initial['target_epoch'] for g in gaps)
    profiles = initial['remote_diagnostics']['tabletop_geometry']['adaptive_stage_profiles']
    uncertainty = max(p['depth_uncertainty_m'] for p in profiles)
    assert uncertainty == .003
    geometry = initial['replay_geometry']
    center = np.array(geometry['obb_center_base_m'])
    assert all(np.allclose(g['target_center_base_m'], center, atol=1e-12, rtol=0) for g in gaps)
    base_camera = np.array(initial['snapshot_transform']['T_base_camera_link'])
    records, static_tf = [], None
    # The recorder did not record CameraInfo. Preserve its metadata's actual
    # parameter-read time semantics rather than inventing image association.
    with rosbag.Bag(str(RECORD / 'keyframes.bag'), 'r') as bag:
        for topic, msg, receipt in bag.read_messages(
                topics=['/grasp_record/camera_runtime_metadata', '/tf_static'],
                end_time=rospy.Time.from_sec(stamp / 1e9 + 1.)):
            if topic == '/tf_static':
                for tr in msg.transforms:
                    if tr.header.frame_id == 'tool0' and tr.child_frame_id == 'camera_link':
                        value = dict(translation_m=[getattr(tr.transform.translation, k) for k in 'xyz'],
                            quaternion_xyzw=[getattr(tr.transform.rotation, k) for k in 'xyzw'],
                            stamp_ns=tr.header.stamp.to_nsec(), parent='tool0', child='camera_link')
                        if static_tf is not None:
                            assert value == static_tf
                        static_tf = value
            else:
                metadata = json.loads(msg.data)
                p = metadata['parameters']
                records.append(dict(receipt_ns=receipt.to_nsec(),
                    original_payload_sha256=digest(msg.data.encode()),
                    parameter_read_interval=metadata['parameter_read_interval'],
                    image_time_association=metadata['image_time_association'],
                    geometry_activation_stamp_ns=p['runtime_profile']['geometry_activation_stamp_ns'],
                    rectification_digest=p['runtime_profile']['rectification']['digest'],
                    intrinsics={k:p[k] for k in ('fx','fy','cx','cy','width','height','distortion_model')},
                    intrinsics_source=p['intrinsics_source']))
    assert static_tf is not None and len(records) >= 2
    assert len({json.dumps(r['intrinsics'],sort_keys=True) for r in records}) == 1
    intrinsics = records[0]['intrinsics']
    k = [intrinsics[key] for key in ('fx','fy','cx','cy')]
    tool_camera = quaternion_matrix(static_tf['quaternion_xyzw'])
    tool_camera[:3,3] = static_tf['translation_m']
    save(out / 'recorded_camera_and_tf.json', dict(metadata=records, tf_static=static_tf,
        T_tool0_camera_link=tool_camera.tolist(), camera_info_recorded=False,
        image_time_association_proven=False))
    corners = observation_obb_corners(center, geometry['R_base_obb'], geometry['obb_size_xyz_m'])
    normal = np.array(geometry['support_normal_base'])
    gripper = GripperGeometry(.05,.002,[.0434,.0286,.0600],[.1175,.1550,.0774],.003)
    common = (gaps, base_camera[:3,:3], normal, uncertainty, .26)
    old = directed_missing_side_rotations(*common)
    expanded = directed_missing_side_rotations(*common, allow_geometry_directed_expansion=True)
    assert not old and expanded
    rolls = bounded_existing_camera_rolls((0.,15.,-15.,30.,-30.,45.,-45.,60.,-60.,75.,-75.,90.,-90.,180.))
    rows = []
    for direction_index, (rotation, proposal) in enumerate(expanded):
        for roll in rolls:
            rad = np.deg2rad(roll)
            camera_rotation = rotation @ np.array([[1.,0.,0.],
                [0.,np.cos(rad),-np.sin(rad)],[0.,np.sin(rad),np.cos(rad)]])
            distances = observation_distance_preferences(corners,center,camera_rotation,
                base_camera[:3,3],k,intrinsics['width'],intrinsics['height'])
            for distance in distances:
                camera = np.eye(4)
                camera[:3,:3] = camera_rotation
                camera[:3,3] = center - distance*camera_rotation[:,0]
                tool = camera @ np.linalg.inv(tool_camera)
                gain = missing_side_view_gain(gaps,camera[:3,3],uncertainty)
                envelope = evaluate_open_gripper_observation_envelope(gripper=gripper,
                    T_base_tool0=tool,opening_width_m=.05,support_normal_base=normal,
                    support_offset_m=geometry['support_offset_m'],obb_center_base=center,
                    R_base_obb=geometry['R_base_obb'],obb_size_xyz_m=geometry['obb_size_xyz_m'])
                try:
                    volume = validate_planned_observation_working_volume(corners,camera,k,
                        intrinsics['width'],intrinsics['height'],snapshot_stamp_ns=stamp)
                except ValueError as exc:
                    volume = dict(ok=False,reason=str(exc))
                incidence = np.arccos(np.clip(camera_rotation[:,0] @ -normal,-1.,1.))
                projected_side = float(geometry['obb_size_xyz_m'][2] * np.sin(incidence))
                translation = float(np.linalg.norm(tool[:3,3]-(base_camera@np.linalg.inv(tool_camera))[:3,3]))
                checks = dict(missing_side_gain=bool(gain['ok']),cad=bool(envelope.ok),
                    working_volume=bool(volume['ok']),side_evidence=projected_side>=uncertainty,
                    translation_baseline=translation>=.015)
                rows.append(dict(direction_index=direction_index,proposal=proposal,
                    camera_roll_offset_deg=roll,camera_target_distance_m=distance,
                    T_base_camera_link=camera.tolist(),T_base_tool0=tool.tolist(),
                    gain=gain,observation_envelope=asdict(envelope),working_volume=volume,
                    projected_side_evidence_m=projected_side,translation_baseline_m=translation,
                    checks=checks,eligible_for_path_check=all(checks.values()),
                    motion_authorized=False))
    accepted = [r for r in rows if r['eligible_for_path_check']]
    by_direction = []
    for i, (_, metadata) in enumerate(expanded):
        selected = [r for r in rows if r['direction_index']==i]
        passed = [r for r in selected if r['eligible_for_path_check']]
        by_direction.append(dict(index=i,angle_deg=metadata['camera_rotation_offset_deg'],
            minimum_required_rotation_deg=metadata['minimum_required_rotation_deg'],
            predicted_nominal_gain_m=metadata['predicted_new_side_gain_m'],
            evaluated=len(selected),eligible_for_path_check=len(passed),
            passed_counts={name:sum(r['checks'][name] for r in selected) for name in selected[0]['checks']},
            qualifying_clearance_range_m=([min(r['observation_envelope']['minimum_support_clearance_m']for r in passed),
                max(r['observation_envelope']['minimum_support_clearance_m']for r in passed)]if passed else None)))
    result = dict(schema='frozen_gen9_directed_observation_endpoint_replay_v1',
        created_at_utc=datetime.now(timezone.utc).isoformat(),source_sha256=hashes,
        initial_audit_path=str(INITIAL),initial_audit_sha256=INITIAL.stem,
        gap_carrier_audit_path=str(GAPS),gap_carrier_audit_sha256=GAPS.stem,
        initial_snapshot_stamp_ns=stamp,gap_carrier_snapshot_stamp_ns=carrier['snapshot_stamp_ns'],
        gap_source_is_initial_snapshot=True,recorded_bag_path=str(RECORD/'keyframes.bag'),
        bag_sha256_from_sealed_manifest=next(x['sha256']for x in manifest['files']if x['path']=='keyframes.bag'),
        bag_hash_recomputed=False,bag_closed_before_read=True,
        depth_uncertainty_m=uncertainty,support_clearance_m=.003,opening_width_m=.05,
        camera_limits=dict(minimum_depth_m=.07,maximum_depth_m=.50,roi_margin_px=36.),
        original_directed_count=len(old),expanded_directed_count=len(expanded),
        existing_camera_roll_count=len(rolls),evaluated_candidate_count=len(rows),
        eligible_for_path_check_count=len(accepted),direction_summaries=by_direction,
        checks=dict(moveit_called=False,hardware_or_live_ros_called=False,
            full_arm_collision_checked=False,trajectory_checked=False,tracking_checked=False,
            measured_information_gain_validated=False,real_machine_feasibility_proven=False),
        limitation='Frozen endpoint geometry only. Passing candidates still require full-arm IK/path/tracking '
            'checks and image-bound arrival/registration evidence. Intrinsics are recorded parameter metadata '
            'with their own read times, not CameraInfo bound to the initial image.',
        candidates=rows)
    for relative, expected in hashes.items():
        assert digest((ROOT/relative).read_bytes()) == expected, 'source changed during replay: '+relative
    save(out/'result.json',result)
    output_hashes={str(f.relative_to(out)):digest(f.read_bytes())for f in sorted(out.rglob('*'))if f.is_file()}
    save(out/'manifest.json',dict(files=output_hashes,local_only=True))
    print(json.dumps(dict(output=str(out),result_sha256=output_hashes['result.json'],
        source_sha256=hashes,evaluated=len(rows),eligible_for_path_check=len(accepted),
        direction_summaries=by_direction),ensure_ascii=False))


if __name__ == '__main__':
    main()
