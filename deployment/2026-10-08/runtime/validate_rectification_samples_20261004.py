#!/usr/bin/env python3
"""Archived real RGB-D preservation checks; no ROS or device access."""
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

WORK = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
CAPTURE = MOUNT / '实验记录/Robot-Controller/ros_log/camera_stage_capture_20261004T124933Z'
sys.path[:0] = [str(WORK / '.runtime/realsense-2.56.5'),
                str(WORK / 'src/alicia_flexible_grasp_supervisor/src')]
import pyrealsense2 as sdk
from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    if not MOUNT.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    output = CAPTURE / ('rectification_validation_' + time.strftime('%Y%m%dT%H%M%SZ', time.gmtime()))
    output.mkdir()
    stages = CAPTURE / 'stages'
    summary = json.loads((stages / 'summary.json').read_text())
    for row in summary['files']:
        p = stages / row['name']
        assert p.stat().st_size == row['bytes'] and sha(p) == row['sha256'], p
    source = WORK / 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_rectification.py'
    frozen_sha = sha(source)
    info = json.loads((stages / 'sample_01.json').read_text())['aligned_depth_intrinsics']
    intrinsic = dict(info)
    intrinsic['cx'], intrinsic['cy'] = intrinsic.pop('ppx'), intrinsic.pop('ppy')
    intrinsic['model'] = intrinsic['model'].split('.')[-1]
    begin = time.perf_counter()
    rectifier = PairedPinholeRectification(intrinsic, sdk)
    build_seconds = time.perf_counter() - begin
    rows, timings = [], []
    for index in range(1, 13):
        name = 'sample_%02d' % index
        metadata = json.loads((stages / (name + '.json')).read_text())
        assert metadata['complete'] and metadata['aligned_depth_intrinsics'] == info
        with np.load(stages / (name + '.npz'), allow_pickle=False) as arrays:
            rgb, raw = arrays['final_rgb'], arrays['final_depth']
            color, depth, origins = rectifier.apply(rgb, raw, return_source_indices=True)
            valid = depth > 0
            positive_origins = origins[valid]
            assert np.count_nonzero(valid) == np.count_nonzero(raw)
            assert np.array_equal(np.sort(positive_origins), np.flatnonzero(raw))
            assert np.array_equal(depth[valid], raw.ravel()[positive_origins])
            assert np.all(origins[~valid] == -1)
            assert np.all(color[~rectifier.rgb_valid] == 0)
            # Output padding can coexist with measured depth. Neither may erase
            # the other: all original measured samples must survive unchanged.
            rows.append(dict(index=index, raw_positive=int(np.count_nonzero(raw)),
                output_positive=int(valid.sum()), values_and_origins_exact=True,
                real_depth_under_unobserved_color=int(np.count_nonzero(valid & ~rectifier.rgb_valid)),
                input_sha256=sha(stages / (name + '.npz'))))
            for repeat in range(5):
                begin = time.perf_counter()
                rectifier.apply(rgb, raw)
                timings.append((time.perf_counter() - begin) * 1000)
            if index in (1, 12):
                assert cv2.imwrite(str(output / (name + '_rectified.png')), color)
    assert sha(source) == frozen_sha, 'source changed during verification'
    report = dict(passed=True, offline_only=True, robot_commands_sent=False,
        sdk_version=sdk.__version__, sdk_sha256=sha(sdk.__file__),
        source_sha256=frozen_sha, script_sha256=sha(__file__), contract=rectifier.contract,
        construction_sec=build_seconds, rows=rows,
        apply_time_ms=dict(zip(('min', 'median', 'p95', 'max'), np.percentile(timings, [0, 50, 95, 100]).tolist())),
        provenance='Exact archived post-filter RGB/Z16 pairs; no renewed target identity or motion authority.',
        limitation='Proves preservation and mapping on recorded data, not live calibration accuracy or grasp success.',
        png_files={p.name:sha(p) for p in output.glob('*.png')})
    (output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    (output / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(output=str(output), passed=True, source_sha256=frozen_sha,
                         apply_time_ms=report['apply_time_ms'], construction_sec=build_seconds)), flush=True)


if __name__ == '__main__':
    main()
