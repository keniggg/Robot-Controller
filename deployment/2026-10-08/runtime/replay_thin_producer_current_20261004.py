"""Replay archived raw frames through the producer; transport is mocked only."""
from collections import Counter
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import Mock

import cv2
import numpy as np
from _pytest.monkeypatch import MonkeyPatch

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK/'src/alicia_grasp_modes/tests'))
from test_unknown_stream import module, rgb_interest_node

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
cv2.setNumThreads(1)

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def stamp(ns):
    return module.rospy.Time(int(ns)//1_000_000_000, int(ns)%1_000_000_000)

def replay(source, hold_enabled):
    original = json.loads(source.read_text())
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
    for key in ('handeye', 'unknown_perception', 'grasp_mode/selection'):
        parameters['/'+key] = deepcopy(context[{'handeye':'handeye',
            'unknown_perception':'perception', 'grasp_mode/selection':'selection'}[key]])
    n._rgb_interest_source_context = lambda when: deepcopy(context)
    now = [0]
    mp.setattr(module.rospy.Time, 'now', staticmethod(lambda: stamp(now[0])))
    mp.setattr(module.rospy, 'logwarn', Mock())
    current_transform = [None]
    mp.setattr(module, 'transform_matrix', lambda args: current_transform[0].copy()
        if args[0] == 'base_link' else np.asarray(context['tool_camera']).copy())
    if not hold_enabled:
        n._same_station_zero_candidate_rgb_hold = lambda *args: False
    rows = []
    try:
        for index, row in enumerate(original['rows']):
            if row.get('source_context') != context:
                raise ValueError('archived_context_changed')
            path = Path(row['source_file'])
            assert sha(path) == row['source_file_sha256']
            with np.load(str(path)) as raw:
                rgb, depth = raw['color_bgr'].copy(), raw['depth_raw'].copy()
                current_transform[0] = raw['T_base_camera'].copy()
            assert hashlib.sha256(rgb.tobytes()).hexdigest() == row['source_rgb_sha256']
            assert hashlib.sha256(depth.tobytes()).hexdigest() == row['source_depth_sha256']
            ns = row['source_stamp_ns']
            now[0] = ns + int(round(row['source_age_at_end_sec']*1e9))
            clock[0] = (now[0]-original['rows'][0]['source_stamp_ns'])*1e-9+10.
            # Record contains exact accepted brackets, not inferred actuator state.
            n._rgb_interest_feedback.clear()
            n._rgb_interest_feedback.extend((int(pair[0]), np.asarray(pair[1]))
                for pair in (row['accepted_before'], row['accepted_after']))
            messages = (n.bridge.cv2_to_imgmsg(rgb, encoding='bgr8'),
                n.bridge.cv2_to_imgmsg(depth, encoding='16UC1' if depth.dtype == np.uint16 else '32FC1'))
            for message in messages:
                message.header.stamp = stamp(ns)
                message.header.frame_id = n.camera_frame
            n.latest = messages
            n.process(None)
            status = json.loads(n.pubs['detector_status'].publish.call_args[0][0].data)
            interest = n._rgb_observation_interest
            rows.append(dict(index=index, source_stamp_ns=ns, state=status['state'],
                detail=status['detail'], target_locked=status['target_locked'],
                target_lost=status['target_lost'], rgb_hold=status.get('rgb_identity_hold'),
                interest_ready=interest.get('observation_interest_ready'),
                interest_reason=interest.get('reason'),
                detected=bool(n.pubs['object'].publish.call_args[0][0].detected)))
    finally:
        mp.undo()
    return dict(hold_enabled=hold_enabled, rows=rows,
        details=dict(Counter(row['detail'] for row in rows)),
        detected_frames=sum(row['detected'] for row in rows),
        held_frames=sum(row['detail']=='measured_depth_temporarily_unavailable_rgb_identity_held' for row in rows),
        last_target_lost=rows[-1]['target_lost'])

if __name__ == '__main__':
    source = Path(sys.argv[1]).resolve()
    output = Path(sys.argv[2]).resolve()
    if not Path('/media/zhuyupei/SanStand').is_mount() or ROOT not in output.parents:
        raise RuntimeError('external archive required; no fallback')
    report = dict(source=str(source), source_sha256=sha(source),
        script_sha256=sha(__file__), producer_sha256=sha(module.__file__),
        seed_sha256=sha(WORK/'src/alicia_grasp_modes/src/alicia_grasp_modes/measured_mask_seeds.py'),
        ros_io=False, live_publications=False, robot_commands_sent=False,
        limitation='Archived sampling omits intervening live frames; not an exact replay of first live loss.',
        without_hold=replay(source, False), with_hold=replay(source, True))
    output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({key:{name:value for name,value in report[key].items() if name!='rows'}
        for key in ('without_hold','with_hold')}, indent=2))
