"""Compare locked-plane representations on sealed gen27 RGB-D and exact TF.

Offline segmentation evidence only: no ROS node, service, target reset or motion.
This does not reproduce asynchronous node scheduling or authorize execution.
"""
import argparse
from dataclasses import asdict, fields
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import rosbag
import rospy
import tf2_py
from cv_bridge import CvBridge


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--evidence', required=True, type=Path)
    args = parser.parse_args()
    evidence = args.evidence.resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert Path('/media/zhuyupei/SanStand') in evidence.parents
    work = Path(__file__).resolve().parents[1]
    old_path = evidence / 'before_tabletop.py'
    new_path = work / 'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py'
    baseline = load('gen27_baseline_tabletop', old_path)
    candidate = load('gen27_candidate_tabletop', new_path)
    producer = load('gen27_producer_transform', evidence / 'before_unknown_tabletop_perception.py')
    record = evidence.parent / 'active_record.json'
    record_dir = Path(json.loads(record.read_text())['record_dir'])
    assert json.loads((record_dir / 'recorder_outcome.json').read_text())['bag_closed']
    bag_path = record_dir / 'keyframes.bag'
    params = json.loads((evidence / 'runtime_parameters.json').read_text())
    config_values = params['parameters']['/unknown_perception'] or {}
    cfg = baseline.Config(**{f.name: config_values[f.name]
                           for f in fields(baseline.Config) if f.name in config_values})
    cv2.setNumThreads(1)
    statuses, depths, metadata, dynamic, static = [], {}, [], [], []
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, message, _ in bag.read_messages(topics=[
                '/perception/unknown/detector_status',
                '/supervisor/camera/depth/image_raw', '/grasp_record/camera_runtime_metadata',
                '/tf', '/tf_static']):
            if topic.endswith('detector_status'):
                value = json.loads(message.data)
                if value.get('generation') == 27:
                    statuses.append(value)
            elif topic.endswith('depth/image_raw'):
                depths[message.header.stamp.to_nsec()] = message
            elif topic.endswith('camera_runtime_metadata'):
                metadata.append(json.loads(message.data))
            elif topic == '/tf_static':
                static.extend(message.transforms)
            else:
                dynamic.extend(message.transforms)
    projection_keys = ['fx', 'fy', 'cx', 'cy', 'depth_scale', 'frame_id',
                       'align_depth_to_color', 'width', 'height']
    cameras = [m['parameters'] for m in metadata if m.get('status') == 'available']
    assert cameras
    camera = cameras[0]
    assert all(all(c[k] == camera[k] for k in projection_keys) for c in cameras)
    assert all(params['parameters']['/camera'][k] == camera[k] for k in projection_keys)
    intrinsics = [camera[k] for k in ['fx', 'fy', 'cx', 'cy']]
    buffer = tf2_py.BufferCore(rospy.Duration(120.))
    for transform in static:
        buffer.set_transform_static(transform, 'sealed_bag')
    for transform in sorted(dynamic, key=lambda t: t.header.stamp.to_nsec()):
        buffer.set_transform(transform, 'sealed_bag')
    acquired = next(s for s in statuses if s.get('detail') == 'acquired')
    failure = next(s for s in statuses if s.get('detail') ==
                   'perception_failure: support_gap_reference_plane_changed')
    plane = np.asarray(acquired['reference_support_plane_base'])
    anchor_ns = acquired['source_stamp_ns']
    failed_ns = failure['source_stamp_ns']
    bridge = CvBridge()

    def inputs(ns):
        message = depths[ns]
        raw = bridge.imgmsg_to_cv2(message, 'passthrough')
        depth = np.asarray(raw, np.float32)
        if message.encoding in ('16UC1', 'mono16'):
            depth = depth * float(camera['depth_scale'])
        else:
            assert message.encoding == '32FC1'
        exact = buffer.lookup_transform_core('base_link', camera['frame_id'], message.header.stamp)
        transform = producer.transform_matrix(exact)
        return depth, transform

    depth, transform = inputs(anchor_ns)
    initial = baseline.segment(depth, intrinsics, transform, cfg)
    anchor = min(initial.candidates, key=lambda c: np.linalg.norm(
        c.position_base - np.asarray(acquired['anchor_position_base'])))
    assert np.array_equal(initial.plane_base, plane), 'Original acquisition plane not reproduced'
    assert np.allclose(anchor.position_base, acquired['anchor_position_base'], rtol=0, atol=1e-12)
    rows = []
    for status in statuses:
        ns = status.get('source_stamp_ns')
        if ns is None or not anchor_ns < ns <= failed_ns + 1_000_000_000:
            continue
        row = dict(source_stamp_ns=ns, recorded_detail=status['detail'])
        if ns not in depths:
            row['missing_exact_depth'] = True
            rows.append(row)
            continue
        depth, transform = inputs(ns)
        results = {}
        for name, module in [('baseline', baseline), ('candidate', candidate)]:
            try:
                result = module.segment_with_support_fallback(depth, intrinsics, transform, cfg,
                    anchor=anchor, last=anchor, reference_plane_base=plane)
                results[name] = result
                row[name] = dict(plane_equal_to_frozen=np.array_equal(result.plane_base, plane),
                    max_coefficient_difference=float(np.max(abs(result.plane_base-plane))),
                    support_source=result.metrics.get('support_source'),
                    support_current_audit_available=result.metrics.get('support_current_audit_available'),
                    support_current_audit_reason=result.metrics.get('support_current_audit_reason'),
                    candidates=len(result.candidates))
            except ValueError as error:
                row[name] = dict(error=str(error))
        if len(results) == 2:
            a, b = results['baseline'], results['candidate']
            row['segmentation_unchanged'] = bool(len(a.candidates) == len(b.candidates) and all(
                np.array_equal(x.mask, y.mask)
                and np.array_equal(x.position_base, y.position_base)
                and np.array_equal(x.position_camera, y.position_camera)
                and np.array_equal(x.extent_m, y.extent_m)
                and np.array_equal(x.support_footprint_base, y.support_footprint_base)
                and x.metrics == y.metrics
                and x.quality == y.quality for x, y in zip(a.candidates, b.candidates)))
        rows.append(row)
    failed = next(r for r in rows if r['source_stamp_ns'] == failed_ns)
    assert failed['baseline']['plane_equal_to_frozen'] is False
    assert failed['candidate']['plane_equal_to_frozen'] is True
    assert failed['candidate']['support_current_audit_available'] is False
    assert failed['segmentation_unchanged']
    assert all(r.get('segmentation_unchanged', True) for r in rows)
    report = dict(scope=__doc__, bag=str(bag_path), generation=27,
        config=asdict(cfg), config_provenance=params['scope'],
        recorded_camera_projection_snapshots_checked=len(cameras),
        original_acquisition_plane_and_position_reproduced=True,
        source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in [old_path, new_path, Path(__file__)]},
        frozen_plane=plane.tolist(), frozen_normal_norm=float(np.linalg.norm(plane[:3])),
        failure_source_stamp_ns=failed_ns, first_loss=failure['first_loss'], rows=rows,
        physical_grasp_success=False, live_motion_authorized=False,
        conclusion='False plane-identity mismatch reproduced and repaired; unavailable current support remains unavailable; candidate masks, positions, extents and qualities unchanged')
    (evidence / 'real_frame_replay.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(output=str(evidence / 'real_frame_replay.json'), failure=failed,
                         rows=len(rows), conclusion=report['conclusion']), indent=2))


if __name__ == '__main__':
    main()
