"""Exact archived gen10 RGB-D through full producer, with offline ROS transport.

Uses original source ns, native pixels, archived exact TF and accepted feedback.
The clock/feedback set represents recorded decision completion; this is not a
reconstruction of CPU scheduling or callback arrival within that decision.
"""
from copy import deepcopy
from dataclasses import fields
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from unittest.mock import Mock

import numpy as np
import cv2
from _pytest.monkeypatch import MonkeyPatch

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK / 'src/alicia_grasp_modes/tests'))
import test_unknown_stream as fixture

EVIDENCE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/'
                'generation10_first_loss_exact_20261004T080900Z/report.json')
PARAMETERS = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/'
                  'unknown_first_loss_pair_20261004T075530Z_120061/metadata.json')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def replay(producer):
    producer_bytes = producer.read_bytes()
    spec = importlib.util.spec_from_file_location('gen10_replay_producer', str(producer))
    module = importlib.util.module_from_spec(spec)
    exec(compile(producer_bytes, str(producer), 'exec'), module.__dict__)
    patch = MonkeyPatch()
    patch.setattr(fixture, 'module', module)
    evidence = json.loads(EVIDENCE.read_text())
    archive = json.loads(PARAMETERS.read_text())
    config = archive['unknown_perception_parameters']
    assert config == archive['current_unknown_perception_parameters']
    assert archive['handeye_unchanged'] and archive['robot_description_unchanged']
    node, clock, _, _, parameters = fixture.rgb_interest_node(patch)
    node.config = module.Config(**{item.name: config[item.name]
        for item in fields(module.Config) if item.name in config})
    node._rgb_interest_config = deepcopy(config)
    node.tracker, node.acquisition = module.TargetTracker(node.config), module.InitialAcquisition(node.config)
    node.max_age = float(config.get('max_frame_age_sec', .8))
    node.tf_wait = float(config.get('tf_wait_sec', .4))
    node.minimum_quality = float(config.get('minimum_geometry_quality', .5))
    statuses = {row['message']['source_stamp_ns']: row for row in evidence['statuses']}
    initial = next(iter(statuses.values()))['message']
    node.generation, node.selection_ns = initial['generation'], initial['selection_stamp_ns']
    epoch = evidence['control_reference_epochs'][0]['message']['stamp']
    node._rgb_interest_epoch = epoch['secs'] * 10**9 + epoch['nsecs']
    parameters.update({'/camera': deepcopy(evidence['latest_camera_metadata_before_loss']['message']['parameters']),
        '/grasp_mode/selection': dict(mode='unknown', strategy='observe_return',
                                     generation=node.generation, stamp_ns=node.selection_ns),
        '/handeye': deepcopy(archive['handeye_parameters']), '/unknown_perception': deepcopy(config),
        '/robot_description': (PARAMETERS.parent / 'robot_description.urdf').read_text()})
    assert hashlib.sha256(parameters['/robot_description'].encode()).hexdigest() == archive['robot_description_sha256']
    now, matrices, result_holder, ownership_audits = [0], {}, [], []
    def stamp(ns):
        return module.rospy.Time(int(ns) // 10**9, int(ns) % 10**9)
    patch.setattr(module.rospy.Time, 'now', staticmethod(lambda: stamp(now[0])))
    patch.setattr(module.rospy, 'logwarn', Mock())
    patch.setattr(module, 'transform_matrix', lambda args: matrices[args[0]].copy())
    original_segment = module.segment_with_support_fallback
    def segment(*args, **kwargs):
        result = original_segment(*args, **kwargs)
        result_holder[:] = [result]
        return result
    patch.setattr(module, 'segment_with_support_fallback', segment)
    original_ownership = node._locked_rgb_fragment_ownership
    def ownership(mask, old_mask, current_mask):
        fragment, old, current = mask != 0, old_mask != 0, current_mask != 0
        core = cv2.erode(fragment.astype(np.uint8), np.ones((3, 3), np.uint8)) != 0
        joint = old & current
        def xy(pixels):
            yy, xx = np.nonzero(pixels)
            return list(map(list, zip(xx.tolist(), yy.tolist())))
        old_distance = cv2.distanceTransform((~old).astype(np.uint8), cv2.DIST_C, 3)
        current_distance = cv2.distanceTransform((~current).astype(np.uint8), cv2.DIST_C, 3)
        item = dict(fragment_pixels=int(fragment.sum()), core_pixels=int(core.sum()),
            reference_rgb_pixels=int(old.sum()), current_rgb_pixels=int(current.sum()),
            reference_rgb_bbox=list(cv2.boundingRect(old_mask)),
            current_rgb_bbox=list(cv2.boundingRect(current_mask)),
            reference_current_rgb_iou=float((old & current).sum() / (old | current).sum()),
            fragment_outside_old_xy=xy(fragment & ~old),
            fragment_outside_current_xy=xy(fragment & ~current),
            core_outside_joint_xy=xy(core & ~joint),
            joint_retained_pixels=int((fragment & joint).sum()),
            joint_retained_fraction=float((fragment & joint).sum() / fragment.sum()),
            maximum_distance_outside_old_px=float(old_distance[fragment].max()),
            maximum_distance_outside_current_px=float(current_distance[fragment].max()),
            fragment_outside_old_one_pixel_band=int((fragment & (old_distance > 1.)).sum()),
            fragment_outside_current_one_pixel_band=int((fragment & (current_distance > 1.)).sum()),
            reference_rgb_mask_sha256=hashlib.sha256(old_mask.tobytes()).hexdigest(),
            current_rgb_mask_sha256=hashlib.sha256(current_mask.tobytes()).hexdigest())
        ownership_audits.append(item)
        try:
            value = original_ownership(mask, old_mask, current_mask)
            item['result'] = value
            return value
        except ValueError as exc:
            item['error'] = str(exc)
            raise
    patch.setattr(node, '_locked_rgb_fragment_ownership', ownership)
    records = []
    try:
        for raw_row in evidence['raw_frames']:
            source = raw_row['source_stamp_ns']
            if source not in statuses:
                continue
            recorded = statuses[source]['message']
            assert sha(raw_row['file']) == raw_row['sha256']
            with np.load(raw_row['file'], allow_pickle=False) as raw:
                rgb, depth = raw['color_bgr'].copy(), raw['depth_raw'].copy()
                matrices['base_link'], matrices['tool0'] = raw['T_base_camera'].copy(), raw['T_tool_camera'].copy()
                recorded_mask = raw['published_mask'].copy()
            for key, array in [('color_bgr', rgb), ('depth_raw', depth)]:
                assert hashlib.sha256(array.tobytes()).hexdigest() == raw_row['images'][key]['array_sha256']
            now[0] = recorded.get('frame_timing', {}).get('status_build_ros_ns', statuses[source]['bag_arrival_ns'])
            clock[0] = 10. + (now[0] - evidence['raw_frames'][0]['source_stamp_ns']) * 1e-9
            node._rgb_interest_feedback.clear()
            node._rgb_interest_feedback.extend((row['source_stamp_ns'], np.array(row['position'][:6]))
                for row in evidence['accepted_feedback'] if row['bag_arrival_ns'] <= now[0])
            source_messages = (node.bridge.cv2_to_imgmsg(rgb, encoding='bgr8'),
                               node.bridge.cv2_to_imgmsg(depth, encoding='16UC1'))
            for message in source_messages:
                message.header.stamp, message.header.frame_id = stamp(source), node.camera_frame
            anchor, last, reference = node.tracker.anchor, node.tracker.last, node._locked_rgb_hold_reference
            last_good = node.last_good_source_stamp_ns
            result_holder.clear()
            ownership_audits.clear()
            node.latest = source_messages
            node.process(None)
            emitted = json.loads(node.pubs['detector_status'].publish.call_args[0][0].data)
            detected = bool(node.pubs['object'].publish.call_args[0][0].detected)
            mask = node.bridge.imgmsg_to_cv2(node.pubs['object_mask'].publish.call_args[0][0])
            result = result_holder[0] if result_holder else None
            candidates = []
            for candidate in result.candidates if result is not None else ():
                item = dict(bbox=list(candidate.bbox), extent_m=candidate.extent_m.tolist(),
                    measured_pixel_count=int(candidate.metrics['valid_points']),
                    position_base=candidate.position_base.tolist(),
                    mask_sha256=hashlib.sha256(candidate.mask.tobytes()).hexdigest())
                if anchor is not None:
                    item['extent_ratios_xy'] = (candidate.extent_m[:2] / np.maximum(anchor.extent_m[:2], .003)).tolist()
                candidates.append(item)
            records.append(dict(source_stamp_ns=source, decision_clock_ns=now[0],
                recorded_detail=recorded['detail'], emitted_detail=emitted['detail'],
                detected=detected, target_lost=node.tracker.lost,
                anchor_unchanged=node.tracker.anchor is anchor, last_unchanged=node.tracker.last is last,
                reference_unchanged=node._locked_rgb_hold_reference is reference,
                last_good_before=last_good, last_good_after=node.last_good_source_stamp_ns,
                published_mask_matches_recorded=bool(np.array_equal(mask, recorded_mask)),
                empty_mask=not bool(np.any(mask)), candidates=candidates,
                ownership_audits=deepcopy(ownership_audits),
                status=emitted, recorded_status=recorded,
                source_context=node._rgb_interest_source_context(stamp(source))))
    finally:
        patch.undo()
    return dict(schema=1, scope=__doc__, producer=str(producer),
        producer_sha256=hashlib.sha256(producer_bytes).hexdigest(), producer_sha256_after=sha(producer),
        evidence_file=str(EVIDENCE), evidence_sha256=sha(EVIDENCE),
        parameter_file=str(PARAMETERS), parameter_sha256=sha(PARAMETERS),
        script_sha256=sha(__file__), rows=records)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--producer', type=Path,
        default=WORK / 'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = replay(args.producer)
    with args.output.open('x') as stream:
        stream.write(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(output=str(args.output), sha256=sha(args.output), producer_sha256=result['producer_sha256'],
        rows=[{key: row[key] for key in ('source_stamp_ns', 'recorded_detail', 'emitted_detail',
            'detected', 'target_lost', 'published_mask_matches_recorded')} for row in result['rows']]), indent=2))
