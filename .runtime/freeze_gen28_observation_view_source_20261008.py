#!/usr/bin/env python3
"""Recover one sealed raw observation source for offline path qualification.

No ROS node, live topic, service, parameter write or hardware operation occurs.
Recovered Python object identities only adapt recorded evidence to the pure
measured_view_source routine; they do not recreate historical live authority.
"""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import rosbag
import rospy
import tf2_py
import yaml
from cv_bridge import CvBridge
from tf.transformations import quaternion_matrix

from alicia_grasp_modes.silhouette_observation_order import transform_fingerprint
from alicia_grasp_modes.silhouette_observation_view_source import measured_view_source, array_fingerprint
from alicia_flexible_grasp.robot.observation_view_limits import (
    DEFAULT_OBSERVATION_VIEW_PARAMETERS, effective_observation_view_limits)
from alicia_flexible_grasp.robot.observation_view_guard import validate_observation_support_projection

ROOT = Path(__file__).resolve().parents[1]
ARCHIVE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
AUDIT = ARCHIVE / 'ros_log/user_grasp_20261008T090751Z/source_snapshot_review/gen28_observation_frozen_audit.json'
AUDIT_SHA = '554fffc15e60d2cbda26b11c4d1cdc8319e3e91441f4180cebb196e5542acb5f'
RECORD = ARCHIVE / 'grasp_records/20261008T092818.564023Z_decf934e904a4ca1916b775e621829ce'
NS_EXACT = 1791451738727244377


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def tf_matrix(message):
    t, q = message.transform.translation, message.transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    matrix = quaternion_matrix(quaternion)
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    output = args.output.resolve()
    assert ARCHIVE.resolve() in output.parents and not output.exists()
    assert digest(AUDIT) == AUDIT_SHA
    audit = json.loads(AUDIT.read_text())
    assert audit['snapshot_stamp_ns'] == NS_EXACT
    manifest = json.loads((RECORD / 'manifest.json').read_text())
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
    bag_manifest = next(f for f in manifest['files'] if f['path'] == 'keyframes.bag')
    bag_path = RECORD / 'keyframes.bag'
    assert bag_path.stat().st_size == bag_manifest['size_bytes']
    image_topics = dict(color='/supervisor/camera/color/image_raw',
                        depth='/supervisor/camera/depth/image_raw',
                        mask='/perception/object_mask', object='/perception/object')
    reverse = {v: k for k, v in image_topics.items()}
    exact, metadata, static, dynamic, statuses = {}, [], [], [], []
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, msg, received in bag.read_messages(topics=list(reverse) + [
                '/grasp_record/camera_runtime_metadata', '/tf_static', '/tf',
                '/perception/unknown/detector_status']):
            if topic in reverse:
                if msg.header.stamp.to_nsec() == NS_EXACT:
                    key = reverse[topic]
                    assert key not in exact, 'duplicate exact raw input ' + key
                    exact[key] = msg
            elif topic == '/grasp_record/camera_runtime_metadata':
                value = json.loads(msg.data)
                if value.get('status') == 'available':
                    metadata.append(value)
            elif topic == '/tf_static':
                static.extend(msg.transforms)
            elif topic == '/tf':
                dynamic.extend(msg.transforms)
            else:
                value = json.loads(msg.data)
                if value.get('source_stamp_ns') == NS_EXACT:
                    statuses.append(value)
    assert set(exact) == set(image_topics), 'missing exact recorded RGB-D/mask/object'
    assert metadata and statuses
    keys = ('width', 'height', 'fx', 'fy', 'cx', 'cy', 'depth_scale', 'frame_id')
    camera = {k: metadata[0]['parameters'][k] for k in keys}
    assert all({k: m['parameters'][k] for k in keys} == camera for m in metadata)
    assert camera['frame_id'] == 'camera_link'
    assert any(s['detail'] == 'tracked' and s['generation'] == 28 for s in statuses)
    bridge = CvBridge()
    raw = np.asarray(bridge.imgmsg_to_cv2(exact['depth'], 'passthrough'))
    color = np.asarray(bridge.imgmsg_to_cv2(exact['color'], 'bgr8'))
    mask = np.asarray(bridge.imgmsg_to_cv2(exact['mask'], 'mono8'))
    assert raw.dtype == np.uint16 and mask.dtype == np.uint8
    buffer = tf2_py.BufferCore(rospy.Duration(300.))
    for transform in static:
        buffer.set_transform_static(transform, 'sealed_recording')
    for transform in sorted(dynamic, key=lambda t: t.header.stamp.to_nsec()):
        buffer.set_transform(transform, 'sealed_recording')
    stamp = rospy.Time(NS_EXACT // 10**9, NS_EXACT % 10**9)
    tool_camera = tf_matrix(buffer.lookup_transform_core('tool0', 'camera_link', stamp))
    bag_base_camera = tf_matrix(buffer.lookup_transform_core('base_link', 'camera_link', stamp))
    frozen_base_camera = np.array(audit['snapshot_transform']['T_base_camera_link'], dtype=float)
    frozen_base_optical = np.array(audit['snapshot_transform']['T_base_optical'], dtype=float)
    assert np.allclose(bag_base_camera, frozen_base_camera, atol=1e-12, rtol=0.)
    transform_payload = {k: audit['snapshot_transform'][k] for k in (
        'snapshot_stamp_ns', 'snapshot_source_frame', 'canonical_candidate_frame',
        'T_base_optical', 'T_base_camera_link')}
    encoded = json.dumps(transform_payload, ensure_ascii=True, separators=(',', ':'), sort_keys=True).encode('utf-8')
    assert hashlib.sha256(encoded).hexdigest() == audit['snapshot_transform']['transform_sha256']
    geo = audit['replay_geometry']
    assert geo['available'] and geo['source_mode'] == 'instance_mask'
    geometry = NS(ok=True, support_normal_base=np.array(geo['support_normal_base']),
                  support_offset_m=geo['support_offset_m'], center_base=np.array(geo['obb_center_base_m']),
                  axes_base=np.array(geo['R_base_obb']), size_xyz_m=np.array(geo['obb_size_xyz_m']))
    identity = NS(epoch=audit['target_epoch'], track_id=audit['observe_return']['target_track_id'])
    sample_stamps = audit['pipeline_metrics']['snapshot_evidence']['source_stamp_ns']
    snapshot = NS(sample_stamp_ns=tuple(sample_stamps), stamp_ns=NS_EXACT,
                  stamp_sec=audit['snapshot_stamp_sec'], target_identity=identity,
                  target_epoch=audit['target_epoch'], frame_id='camera_link', color_bgr=color)
    ticket = NS(payload=(snapshot, None), snapshot_stamp_sec=snapshot.stamp_sec,
                target_epoch=snapshot.target_epoch, request_id=audit['request_id'], generation=audit['generation'])
    sample = NS(stamp_ns=NS_EXACT, target_identity=identity, target_epoch=snapshot.target_epoch,
                frame_id='camera_link', object_msg=exact['object'], color_bgr=color,
                depth_raw=raw, object_mask=mask)
    prepared = NS(snapshot=snapshot, ticket=ticket, geometry=geometry,
                  pose_estimator=NS(_T_base_optical=frozen_base_optical))
    source = measured_view_source(sample, prepared, NS(**camera), tool_camera)
    config_path = ROOT / 'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml'
    configured = yaml.safe_load(config_path.read_text())['grasp_6d']['remote']
    requested = {k: configured.get(k, v) for k, v in DEFAULT_OBSERVATION_VIEW_PARAMETERS.items()}
    effective = effective_observation_view_limits(requested)
    source['view_limits'] = dict(requested_visibility_limits_snapshot=requested,
                                effective_visibility_limits=effective)
    source['audit']['view_limits'] = source['view_limits']
    reference_hashes = sorted(set(e['observation_orientation_search']['scheduling']['reference']['transform_sha256']
        for e in audit['stable_evaluations'] if e.get('observation_orientation_search', {}).get('scheduling', {}).get('reference')))
    reference_hash_matches = reference_hashes == [source['audit']['frozen_tool_reference_sha256']]
    try:
        reference_support = validate_observation_support_projection(frozen_base_camera,
            source['points_base'], source['center_base'], [camera[k] for k in ('fx', 'fy', 'cx', 'cy')],
            camera['width'], camera['height'], target_corners_base=source['corners_base'], **effective)
    except ValueError as exc:
        reference_support = dict(getattr(exc, 'audit', {}), ok=False, reason=str(exc))
    geometry_json = {k: v for k, v in geo.items() if k != 'object_points_base_m'}
    model_path = ROOT / '.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
    metadata_json = {k: v for k, v in source.items() if k not in {'points_base', 'corners_base'}}
    metadata_json.update(schema=1, purpose='offline_frozen_view_and_path_geometry_replay',
        geometry=geometry_json, execution_authorized=False,
        source_provenance=dict(audit_path=str(AUDIT), audit_sha256=AUDIT_SHA,
            bag_path=str(bag_path), bag_manifest_sha256=bag_manifest['sha256'],
            bag_manifest_sha256_rehashed_this_run=False, bag_size_verified=True,
            manifest_sha256=digest(RECORD / 'manifest.json'),
            raw_color_sha256=array_fingerprint(color), exact_image_topics=image_topics,
            source_status_count=len(statuses), camera_metadata_equal_projection_count=len(metadata),
            camera_metadata_scope='recorded parameter content; not exposure-time hardware association',
            config_path=str(config_path), config_sha256=digest(config_path),
            source_code_path=str(Path(__file__)), source_code_sha256=digest(__file__),
            model_path=str(model_path), model_sha256=digest(model_path)),
        replay_checks=dict(snapshot_transform_sha256_matches=True,
            frozen_tool_reference_sha256_matches_original_scheduler=reference_hash_matches,
            original_scheduler_reference_sha256=reference_hashes,
            reconstructed_tool_reference_sha256=source['audit']['frozen_tool_reference_sha256'],
            bag_tf_max_abs_difference=float(np.max(np.abs(bag_base_camera-frozen_base_camera))),
            reference_endpoint_support_projection=reference_support),
        limitations=[
            'Original audit does not persist raw-depth/mask/support/corner digests; output array hashes bind reconstructed sealed raw source, not an unavailable original in-memory hash.',
            'Reconstructed tool-reference fingerprint equality with the original scheduler is recorded separately; a mismatch forbids claiming bit-identical original planning arrays or reusing their authority.',
            'Visibility limits are from saved source configuration (36 px, 70-500 mm); the request did not record a complete parameter snapshot. Current execution must capture and bind its real parameter values.',
            'Geometry is the exact audit OBB/support used for request 41; raw support is reconstructed from its final source only, not fused depth or synthesized plane points.',
            'Generation 3 is the recorded internal planning generation; grasp selection generation is 28. No old request or authority is reused.',
            'No complete successful trajectory or contemporaneous stationary controller desired-state bridge is certified by these inputs.',
            'Projection predicts existing measured support only; arrival measurement, freshness, tracking, collision and grasp checks remain required.'
        ], grasp_mode_selection=audit['grasp_mode_selection'])
    output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(output / 'source.npz', points_base=source['points_base'],
        corners_base=source['corners_base'], tool_from_camera=np.array(source['tool_from_camera']),
        center_base=np.array(source['center_base']), base_from_tool=np.array(source['base_from_tool']),
        base_from_camera=frozen_base_camera, base_from_optical=frozen_base_optical,
        raw_depth=raw, raw_mask=mask, color_bgr=color,
        object_points_base=np.array(geo['object_points_base_m']))
    metadata_json['source_npz_sha256'] = digest(output / 'source.npz')
    (output / 'source.json').write_text(json.dumps(metadata_json, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(output=str(output), point_count=len(source['points_base']),
        source_sha256=digest(output/'source.json'), source_npz_sha256=metadata_json['source_npz_sha256'],
        view_limits=source['view_limits'], reference_support=reference_support,
        original_reference_hash_verified=reference_hash_matches), ensure_ascii=False))


if __name__ == '__main__':
    main()
