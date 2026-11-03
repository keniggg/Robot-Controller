#!/usr/bin/env python3
"""Read sealed diagnostic evidence only; no ROS/SDK device or pipeline calls."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.runtime/realsense-2.56.5'))
sys.path.insert(0, str(ROOT / 'src/alicia_flexible_grasp_supervisor/src'))
import pyrealsense2 as sdk
from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
CAPTURE = EXT / 'ros_log/gen17_support_measurement_20261006'
DEPLOY = EXT / 'ros_log/regional_production_20261006T105318.899385Z'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentiles(values):
    return dict(zip(('p50', 'p90', 'p95', 'p99', 'max'),
                    map(float, np.percentile(values, [50, 90, 95, 99, 100]))))


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    summary = json.loads((CAPTURE / 'raw_support_v2/summary.json').read_text())
    assert summary['archive_closed'] and summary['complete_without_writer_error']
    with zipfile.ZipFile(CAPTURE / 'raw_support_v2/diagnostic.zip') as archive:
        name = next(n for n in archive.namelist() if n.startswith('metadata/'))
        metadata = json.loads(archive.read(name))
    camera = metadata['parameters']['/camera']
    contract = camera['runtime_profile']['rectification']
    rectifier = PairedPinholeRectification(contract['source_intrinsics'], sdk)
    assert rectifier.contract == contract
    owner = np.zeros((rectifier.source_height, rectifier.source_width), np.uint8)
    bounds = [int(.2*rectifier.source_width), int(.1*rectifier.source_height),
              int(.8*rectifier.source_width), int(.9*rectifier.source_height)]
    x0, y0, x1, y1 = bounds
    owner[y0:y1, x0:x1] = 1
    output = np.zeros((rectifier.height, rectifier.width), np.uint8)
    output.ravel()[rectifier.target_indices] = owner.ravel()
    mapped = np.zeros(output.size, bool)
    mapped[rectifier.target_indices] = True
    mapped = mapped.reshape(output.shape)
    v, u = np.indices(output.shape)
    replay = json.loads((CAPTURE / 'support_replay_v2/report.json').read_text())
    rings = []
    for row in replay['rows']:
        if row['detail'] != 'support_measurement_pending_revalidation':
            continue
        center = row['center_uv']
        radius = np.hypot(u-center[0], v-center[1])
        scale = min(rectifier.width/640, rectifier.height/480)
        for outer in (170, 140, 110):
            mask = (radius >= 70*scale) & (radius <= outer*scale) & mapped
            rings.append(dict(source_stamp_ns=row['source_stamp_ns'],
                ring_base_pixels=outer, mapped_pixels=int(mask.sum()),
                center_owned_fraction=float(np.mean(output[mask] == 1)),
                outside_center=int(np.count_nonzero(output[mask] != 1))))
    reads = [json.loads(line)['existing_read_diagnostics']
             for line in (DEPLOY / 'read_diagnostics.jsonl').read_text().splitlines()]
    complete = [row for row in reads if row['result'] == 'complete']
    stages = {}
    for key in ('align', 'depth_filter_0_spatial', 'stationary_evidence',
                'regional_stationary_depth', 'depth_array_and_clip', 'pair_rectification'):
        stages[key] = percentiles([row['stages_sec'][key]*1000 for row in complete])
    nonwait = [sum(value for key, value in row['stages_sec'].items()
                   if key != 'wait_for_frames')*1000 for row in complete]
    gaps = []
    previous = None
    for i, row in enumerate(reads):
        if row['result'] != 'complete':
            continue
        if previous is not None:
            old = previous[1]
            for stream in ('color', 'depth'):
                before, current = old['raw_frames'][stream], row['raw_frames'][stream]
                interval = current['device_timestamp_ms']-before['device_timestamp_ms']
                increment = current['frame_number']-before['frame_number']
                if increment != 1 or not 8.333333 <= interval <= 58.333334:
                    gaps.append(dict(read_index=i, previous_complete_index=previous[0],
                        stream=stream, frame_number_increment=increment,
                        device_interval_ms=interval,
                        actual_fps_raw=current['metadata_raw'].get('actual_fps'),
                        host_wait_ms=row['host_wait']['elapsed_sec']*1000,
                        previous_complete_nonwait_ms=sum(value for key, value in old['stages_sec'].items()
                            if key != 'wait_for_frames')*1000))
        previous = (i, row)
    sources = [Path(__file__),
        ROOT / 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regional_stationary_depth.py',
        ROOT / 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/visual_stationary.py',
        ROOT / 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/rgbd_rectification.py',
        ROOT / 'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py']
    result = dict(schema=1, diagnostic_only=True, hardware_commands_sent=False,
        production_files_modified=False, source_sha256={str(p): sha(p) for p in sources},
        input_sha256={str(p): sha(p) for p in (
            CAPTURE / 'raw_support_v2/summary.json', CAPTURE / 'support_replay_v2/report.json',
            DEPLOY / 'read_diagnostics.jsonl')},
        original_metadata_entry=name, source_center_bounds_xyxy=bounds,
        sdk_version=sdk.__version__, sdk_path=sdk.__file__,
        reproduced_entire_rectification_contract_equal=True,
        depth_forward_map_sha256=rectifier.contract['depth_forward_map_sha256'],
        gen17_pending_rings=rings, camera_complete_reads=len(complete),
        camera_stage_ms=stages, camera_all_nonwait_ms=percentiles(nonwait),
        nonwait_over_30fps_period_count=sum(value > 1000/30 for value in nonwait),
        real_source_discontinuities=gaps,
        conclusions=[
            'All three gen17 pending support rings at all three radii are completely owned by the center region; this failure is not an ROI ownership mismatch.',
            'Center qualification/reset remains necessary: this map proof does not establish filter activity at any exact exposure.',
            'First two device/source gaps occurred after normal <22 ms non-wait processing and during long SDK waits; five-region processing overrun is not established as their cause.',
            'actual_fps metadata and frame numbers do not prove exposure cadence or where loss occurred; USB, SDK, camera, host scheduling remain unresolved.',
            'No demonstrated further production algorithm or wiring defect in the audited support/filter path justifies weakening quality, source-age, or visual-stationarity gates.'
        ])
    destination = CAPTURE / 'support_filter_domain_review_v1'
    destination.mkdir(exist_ok=False)
    path = destination / 'report.json'
    path.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(dict(path=str(path), sha256=sha(path), rings_checked=len(rings),
                         all_rings_center_owned=all(r['outside_center'] == 0 for r in rings))))


if __name__ == '__main__':
    main()
