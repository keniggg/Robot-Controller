"""Offline scheduling cost only; synthetic source, not a gen21 raw replay."""
import hashlib
import json
from itertools import product
from pathlib import Path
import sys
import time

import numpy as np
from alicia_grasp_modes.observation_support_schedule import ObservationSupportSchedule
from alicia_flexible_grasp.robot.observation_view_guard import (
    ObservationViewError, validate_observation_support_projection)


def main(output):
    output = Path(output)
    if output.exists():
        raise ValueError('refusing to overwrite benchmark evidence')
    rng = np.random.RandomState(31)
    uv = rng.uniform([80., 60.], [560., 420.], (74098, 2))
    points = np.column_stack((np.full(len(uv), .3),
        .3/400.*(320.-uv[:, 0]), .3/400.*(240.-uv[:, 1])))
    center = np.array([.28, 0., 0.])
    corners = center + np.array(list(product((-.01, .01), repeat=3)))
    limits = dict(roi_margin_px=36., minimum_depth_m=.07, maximum_depth_m=.5)
    source = dict(points_base=points, center_base=center, corners_base=corners,
        tool_from_camera=np.eye(4), camera=dict(fx=400., fy=400., cx=320., cy=240., width=640, height=480),
        view_limits=dict(effective_visibility_limits=limits))
    poses = []
    for distance, roll in product(np.linspace(.09, .38, 5), np.linspace(-np.pi, np.pi, 35, endpoint=False)):
        c, s = np.cos(roll), np.sin(roll)
        pose = np.eye(4); pose[:3, :3] = [[1., 0., 0.], [0., c, -s], [0., s, c]]
        pose[0, 3] = center[0] - distance
        poses.append(pose)
    trials = []
    for _ in range(3):
        start, cpu = time.perf_counter(), time.process_time()
        schedule = ObservationSupportSchedule(source)
        predicted = [schedule.evaluate(pose) for pose in poses]
        witness = dict(wall_sec=time.perf_counter()-start, cpu_sec=time.process_time()-cpu)
        start, cpu = time.perf_counter(), time.process_time()
        cached = [schedule.evaluate(pose) for pose in poses]
        reuse = dict(wall_sec=time.perf_counter()-start, cpu_sec=time.process_time()-cpu)
        assert predicted == cached
        start, cpu = time.perf_counter(), time.process_time()
        full = []
        for pose in poses:
            try:
                full.append(validate_observation_support_projection(pose, points, center, [400., 400., 320., 240.],
                    640, 480, target_corners_base=corners, **limits))
            except ObservationViewError as exc:
                full.append(exc.audit)
        full_cost = dict(wall_sec=time.perf_counter()-start, cpu_sec=time.process_time()-cpu)
        assert all(f['ok'] for row, f in zip(predicted, full) if row['status'] == 'sufficient')
        trials.append(dict(witness=witness, cached=reuse, full=full_cost,
            sufficient=sum(row['status'] == 'sufficient' for row in predicted),
            unknown=sum(row['status'] == 'unknown' for row in predicted),
            full_pass=sum(row['ok'] for row in full)))
    files = [Path(__file__), Path('src/alicia_grasp_modes/src/alicia_grasp_modes/observation_support_schedule.py'),
        Path('src/alicia_grasp_modes/src/alicia_grasp_modes/silhouette_observation_planning.py')]
    report = dict(source='synthetic 74098 measured-point fixture, 175 full camera poses',
        is_real_failed_generation21_replay=False, actual_generation21_raw_support_unavailable=True,
        measured_thresholds_unchanged=True, planning_or_execution_performed=False,
        trials=trials, file_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(trials=trials, report=str(output))))


if __name__ == '__main__':
    main(sys.argv[1])
