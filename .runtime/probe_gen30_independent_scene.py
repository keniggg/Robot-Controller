"""Offline synthetic scene probe; records only on the mounted experiment disk."""
from pathlib import Path
import hashlib
import json
import os
import sys
import time

work = Path(__file__).resolve().parents[1]
for relative in ('src/alicia_grasp_modes/tests', 'src/alicia_grasp_modes/src',
                 'src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0, str(work / relative))
import cv2
import numpy as np
from test_gen30_current_instance_independent import current_box_scene, run_current_refiner

mount = Path('/media/zhuyupei/SanStand')
root = mount / '实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
assert len(sys.argv) == 2 and '/' not in sys.argv[1]
out = root / sys.argv[1]
assert mount.is_mount() and not os.statvfs(mount).f_flag & os.ST_RDONLY
assert not out.exists()
paths = [
    'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/measured_mask_seeds.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py',
    'src/alicia_grasp_modes/tests/test_gen30_current_instance_independent.py',
    '.runtime/probe_gen30_independent_scene.py',
]
def hashes():
    return {p: hashlib.sha256((work / p).read_bytes()).hexdigest() for p in paths}
before = hashes()
cv2.setNumThreads(1)
rows = []
for colour in ((35, 35, 35), (80, 80, 80), (55, 100, 185), (25, 70, 180)):
    for missing in (False, True):
        scene = current_box_scene(side_colour=colour, missing_side=missing)
        saved = {k: v.copy() for k, v in scene.items() if isinstance(v, np.ndarray)}
        started = time.monotonic()
        row = dict(side_colour=colour, missing_side=missing)
        try:
            result = run_current_refiner(scene)
            mask = result.mask > 0
            row.update(outcome='returned', whole_iou=float(np.count_nonzero(mask & scene['truth'])/
                np.count_nonzero(mask | scene['truth'])),
                side_coverage=float(np.mean(mask[scene['side']])),
                outside_truth_pixels=int(np.count_nonzero(mask & ~scene['truth'])),
                metrics=result.metrics)
        except ValueError as exc:
            row.update(outcome='rejected', reason=str(exc))
        row['elapsed_sec'] = time.monotonic() - started
        row['arrays_unchanged'] = all(np.array_equal(scene[k], v) for k, v in saved.items())
        assert row['arrays_unchanged']
        rows.append(row)
receipt = dict(source_sha256=before, source_unchanged=before == hashes(), rows=rows,
    no_ros_calls=True, no_hardware_access=True, production_modified=False,
    physical_grasp_verified=False,
    limit='Synthetic ray-cast box, actual public refiner only; no production producer/session timing.')
assert receipt['source_unchanged']
out.mkdir()
(out / 'receipt.json').write_text(json.dumps(receipt, indent=2, allow_nan=False) + '\n')
print(json.dumps(dict(output=str(out / 'receipt.json'), rows=[{k:v for k,v in r.items() if k != 'metrics'} for r in rows])))
