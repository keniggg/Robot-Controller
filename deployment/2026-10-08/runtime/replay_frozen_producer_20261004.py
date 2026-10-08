"""Archived full producer replay, fake ROS transport only; never publishes live."""
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import types
from unittest.mock import Mock

import cv2
import numpy as np
from _pytest.monkeypatch import MonkeyPatch

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK / 'src/alicia_grasp_modes/tests'))
from test_unknown_stream import module, rgb_interest_node
from alicia_grasp_modes.rgb_observation_region import RgbObservationTracker

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
SOURCE = ROOT / 'rgb_observation_bootstrap_20261004T034110Z/result.json'
cv2.setNumThreads(1)


def stamp(ns):
    return module.rospy.Time(int(ns)//1000000000, int(ns)%1000000000)


def run(inherit):
    original = json.loads(SOURCE.read_text())
    mp = MonkeyPatch()
    n, clock, _, _, parameters = rgb_interest_node(mp)
    context = original['rows'][0]['source_context']
    n.generation = context['selection']['generation']
    n.selection_ns = context['selection']['stamp_ns']
    n._rgb_interest_epoch = context['epoch_ns']
    n._rgb_interest_config = deepcopy(context['perception'])
    parameters['/camera'] = deepcopy(context['camera'])
    if 'runtime_profile_geometry' in parameters['/camera']:
        parameters['/camera']['runtime_profile'] = parameters['/camera'].pop('runtime_profile_geometry')
    assert module.camera_geometry_snapshot(parameters['/camera']) == context['camera']
    parameters['/grasp_mode/selection'] = deepcopy(context['selection'])
    parameters['/handeye'] = deepcopy(context['handeye'])
    parameters['/unknown_perception'] = deepcopy(context['perception'])
    n._rgb_interest_source_context = lambda when: deepcopy(context)
    now = [0]
    mp.setattr(module.rospy.Time, 'now', staticmethod(lambda: stamp(now[0])))
    mp.setattr(module.rospy, 'logwarn', Mock())
    current_transform = [None]
    mp.setattr(module, 'transform_matrix', lambda args: current_transform[0].copy()
               if args[0] == 'base_link' else np.asarray(context['tool_camera']).copy())
    history = []
    replayed = []
    proposals_seen = []
    original_propose = module.propose_rgb_observation_regions
    def proposal(*args, **kwargs):
        value = original_propose(*args, **kwargs)
        proposals_seen.append(value)
        return value
    mp.setattr(module, 'propose_rgb_observation_regions', proposal)
    original_unavailable = n.unavailable
    def unavailable(*args, **kwargs):
        before = deepcopy(n._rgb_observation_interest)
        original_unavailable(*args, **kwargs)
        history.append(dict(reason=args[0], preserve_requested=kwargs.get('preserve_rgb_interest', False),
            ready_before=before.get('observation_interest_ready'),
            ready_after=n._rgb_observation_interest.get('observation_interest_ready'),
            source_before=before.get('source_stamp_ns'),
            source_after=n._rgb_observation_interest.get('source_stamp_ns')))
    n.unavailable = unavailable
    freeze_index = None
    admitted_histories = {}
    inherited_stamps = []
    for index, row in enumerate(original['rows']):
        source_path = Path(row['source_file'])
        assert hashlib.sha256(source_path.read_bytes()).hexdigest() == row['source_file_sha256']
        with np.load(str(source_path)) as raw:
            rgb, depth = raw['color_bgr'].copy(), raw['depth_raw'].copy()
            current_transform[0] = raw['T_base_camera'].copy()
        assert hashlib.sha256(rgb.tobytes()).hexdigest() == row['source_rgb_sha256']
        assert hashlib.sha256(depth.tobytes()).hexdigest() == row['source_depth_sha256']
        ns = row['source_stamp_ns']
        now[0] = ns + int(round(row['source_age_at_end_sec']*1e9))
        clock[0] = (now[0]-original['rows'][0]['source_stamp_ns'])*1e-9+10.
        n._rgb_interest_feedback.clear()
        n._rgb_interest_feedback.extend((int(pair[0]), np.asarray(pair[1]))
            for pair in (row['accepted_before'], row['accepted_after']))
        messages = (n.bridge.cv2_to_imgmsg(rgb, encoding='bgr8'),
            n.bridge.cv2_to_imgmsg(depth, encoding='16UC1' if depth.dtype == np.uint16 else '32FC1'))
        for message in messages:
            message.header.stamp = stamp(ns)
            message.header.frame_id = n.camera_frame
        n.latest = messages
        proposals_seen.clear()
        n.process(None)
        status = json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
        interest = n._rgb_observation_interest
        admitted_histories[ns] = dict(proposals=proposals_seen[-1] if proposals_seen else None,
            track=deepcopy(interest.get('rgb_track', {})), q=row['q_source'])
        replayed.append(dict(index=index, source_stamp_ns=ns, detail=status['detail'],
            ready=interest['observation_interest_ready'], interest_reason=interest['reason'],
            handoff=deepcopy(n._rgb_handoff_status),
            published_ready=status['observation_interest']['observation_interest_ready'],
            preserve=history[-1], detected=bool(n.pubs['object'].publish.call_args[0][0].detected)))
        if freeze_index is None and interest['observation_interest_ready']:
            initial_stamps = tuple(interest['rgb_track']['source_stamps_ns'])
            request = dict(version=1, request_id='offline-archive-replay',
                source_id=interest['source_id'], selection=deepcopy(context['selection']),
                source_stamp_ns=str(ns), deadline_ns=str(now[0]+100_000_000_000), max_observations=3,
                **{key:interest[key] for key in ('region_mask_sha256','source_rgb_sha256',
                    'source_depth_sha256','observation_set_sha256')})
            frozen_regions = tuple(n._rgb_reference_cache[-1]['regions'])
            n._rgb_reference_request_cb(module.String(data=json.dumps(request)))
            n._service_rgb_reference_request()
            ack = json.loads(n.pubs['rgb_reference_status'].publish.call_args[0][0].data)
            assert ack['status'] == 'frozen', ack
            freeze_index = index
            replayed[-1]['freeze_ack'] = ack
            replayed[-1]['initial_three_stamps'] = initial_stamps
            if inherit:
                trackers = [RgbObservationTracker() for reg in frozen_regions]
                for original_ns in initial_stamps:
                    old = admitted_histories[original_ns]
                    proposals = old['proposals']
                    for tr, anchor in zip(trackers, frozen_regions):
                        scores = []
                        for ri, candidate in enumerate(proposals.candidates):
                            inter = np.count_nonzero(candidate.mask & anchor.mask)
                            iou = inter/max(1,candidate.area_px+anchor.area_px-inter)
                            if iou >= .70:
                                scores.append((ri,iou))
                        assert len(scores) == 1, ('inherited_history_not_unique', original_ns,scores)
                        region_index = scores[0][0]
                        track = tr.observe(replace(proposals, selected_index=region_index,
                            reason='offline_exact_original_initial_history'), source_id=interest['source_id'],
                            generation=n.generation, stamp_ns=original_ns,joints_rad=old['q'])
                assert all(len(tr._stamps)==3 for tr in trackers)
                n._rgb_set_handoff_trackers = trackers
                n._rgb_handoff_scope = (ack['reference_id'],0)
                inherited_stamps = list(initial_stamps)
    mp.undo()
    return dict(freeze_index=freeze_index, inherit_confirmed_history=inherit,
        inherited_real_source_stamps=inherited_stamps, rows=replayed,
        fresh_ready=sum(x['ready'] for x in replayed[freeze_index+1:]) if freeze_index is not None else 0,
        ready_publication_mismatches=sum(x['ready'] != x['published_ready'] for x in replayed),
        note='Inheritance is an offline state experiment only; production source is unchanged. '
             'Fresh raw RGB matching/support/TF/joints/source/deadline/publishing execute original producer.')


if __name__ == '__main__':
    report = dict(source=str(SOURCE), source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        producer_sha256=hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest(),
        ros_io=False, live_publications=False, robot_commands_sent=False,
        baseline=run(False), counterfactual_inherit_initial_history=run(True))
    output = Path(sys.argv[1]) if len(sys.argv)>1 else None
    if output:
        if not Path('/media/zhuyupei/SanStand').is_mount() or ROOT not in output.parents:
            raise RuntimeError('external archive required')
        output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({name:dict(freeze_index=report[name]['freeze_index'],
        fresh_ready=report[name]['fresh_ready'],ready_publication_mismatches=report[name]['ready_publication_mismatches'],
        rows=[{k:r[k] for k in ('index','detail','ready','interest_reason','preserve')} for r in report[name]['rows']])
        for name in ('baseline','counterfactual_inherit_initial_history')},indent=2))
