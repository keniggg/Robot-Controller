"""Offline reproduction of a stationary frame spanning intentional contact motion."""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
from _pytest.monkeypatch import MonkeyPatch
from alicia_flexible_grasp_supervisor.msg import GraspState

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK / 'src/alicia_grasp_modes/tests'))
from test_observation_arrival_station import completed
from test_observation_arrival_frame_completion_guards import pending_source
from test_locked_rgb_hold import latest_status, module


def reproduce(moved):
    patch = MonkeyPatch()
    try:
        node, clock, _, joint, params, marker = completed(patch)
        node._station_compute_state_cb(GraspState(state='MOVE_PREGRASP', active=True))
        pair, reference = pending_source(node, clock, joint)
        original = node.buffer.lookup_transform
        events = []
        fired = False

        def lookup(*args, **kwargs):
            nonlocal fired
            value = original(*args, **kwargs)
            token = getattr(node, '_arrival_frame_completion', None)
            if token is not None and not fired:
                fired = True
                before = reference.get('observation_arrival_retry') is marker
                q = marker['arrival_q'].copy()
                if moved:
                    q[0] += .005
                joint(clock[0] - .001, q)
                events.append(dict(token_already_created=True,
                    marker_before_callback=before,
                    marker_after_callback=reference.get('observation_arrival_retry') is marker,
                    joint_delta_rad=float(np.max(abs(q-marker['arrival_q']))),
                    marker_invalidated_reason=marker.get('invalidated_reason'),
                    pose_invalidated=reference['pose_invalidated']))
            return value

        patch.setattr(node.buffer, 'lookup_transform', lookup)
        node.process(None)
        status = latest_status(node)
        assert fired
        if moved:
            assert node.tracker.lost
            assert status['detail'] == 'perception_failure: station_frame_completion_qualification_failed'
            assert 'arrival_station_invalidated_reason' not in status['frame_timing']
            assert events[0]['marker_invalidated_reason'] == 'movement_after_qualified_arrival'
        else:
            assert not node.tracker.lost and status['state'] == 'ready'
        return dict(movement_injected=moved, task_state='MOVE_PREGRASP',
            source_stamp_ns=pair[0].header.stamp.to_nsec(), callback_events=events,
            state=status['state'], detail=status['detail'], target_lost=status['target_lost'],
            frame_timing=status.get('frame_timing'), first_loss=status.get('first_loss'),
            object_detected=node.pubs['object'].publish.call_args[0][0].detected)
    finally:
        patch.undo()


def main():
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount()
    out = mount / '实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen14_analysis'
    out.mkdir(exist_ok=True)
    report = dict(scope='Offline existing production callback/process reproduction with synthetic camera pixels and accepted joint messages, no ROS graph.',
        control=reproduce(False), race=reproduce(True),
        source_sha256={name: hashlib.sha256((WORK/name).read_bytes()).hexdigest() for name in (
            'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
            'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py')},
        finding='The normal frame completion binds the fixed arrival marker before TF. A subsequent accepted joint callback removes that marker on >.004 rad movement, irrespective of task phase. The qualification function sees no marker and returns None without arrival_station_invalidated_reason; the still-bound token converts this into global target loss.',
        limits=['Synthetic sequence proves the code branch, not the exact callback ordering in the recording.',
                'MOVE_PREGRASP state alone is not a sufficient identity/motion authorization certificate. A future correction must bind the actual contact plan, track, epoch and approved motion lifecycle; do not suppress actual unexplained movement or relax freshness.',
                'No source or live configuration changes and no device/ROS operations.'])
    path = out / 'contact_motion_station_race_reproduction.json'
    with path.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps(dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        control=report['control']['state'], race=report['race']['detail']), ensure_ascii=False))


if __name__ == '__main__':
    main()
