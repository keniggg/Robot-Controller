"""Read original RGB refit candidates only; no alternate fit or changed gates."""
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np

W = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(W/'src/alicia_flexible_grasp_supervisor/src'))
from alicia_flexible_grasp.vision import cuboid_rgb_interval as c

M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
G = E/'grasp_records/20261018T023244.888815Z_d5686c81c6144223b5adb93e60ca7cee'
I = N/'verification/gen43_actual_source_inspection_r1'
OUT = N/'verification/gen43_existing_refit_candidates_r1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    assert M.is_mount() and not OUT.exists()
    assert OUT.parent.stat().st_dev == M.stat().st_dev
    policy = json.loads((E/'maintenance/storage_control.json').read_text())
    archive = json.loads((E/'grasp_records/archive_config.json').read_text())
    assert policy['archive_policy'] == 'local_only'
    assert archive['upload_enabled'] is False and archive['delete_enabled'] is False
    inputs = {str(Path(c.__file__).resolve()): sha(c.__file__),
              str(Path(__file__).resolve()): sha(__file__)}
    rows = []
    for request, filename, stamp in (
            (1, 'a4c568310c48ede8623c3d3347babb9b6a2d52de29af2292c3196b9354f9b8d2.json', 1792290768221260547),
            (18, 'b7d9a682a3f359f3963ecbf9e6fd95ecffa3768c8a55e81176f35da8b3451945.json', 1792290823025146722)):
        audit_path = G/'keyframes_audits'/filename
        report = json.loads(audit_path.read_text())
        a = report['remote_diagnostics']['tabletop_geometry']['regular_box_inference']['cuboid_rgb_interval']
        source = I/('request'+str(request))/str(stamp)
        for path in (audit_path, source/'metadata.json', source/'color_bgr.npy', source/'mask_uint8.npy'):
            inputs[str(path)] = sha(path)
        color, mask = np.load(source/'color_bgr.npy'), np.load(source/'mask_uint8.npy') > 0
        edges = cv2.Canny(cv2.cvtColor(color, cv2.COLOR_BGR2GRAY), 50., 100.)
        ey, ex = np.where(edges)
        edge_pixels = np.column_stack((ex, ey))
        camera = json.loads((source/'metadata.json').read_text())['intrinsics']
        K = np.array([camera[name] for name in ('fx', 'fy', 'cx', 'cy')])
        T = np.array(report['snapshot_transform']['T_base_optical'])
        A = np.array(a['seed_axes_base'])
        lo, hi = np.array(a['seed_lower']), np.array(a['seed_upper'])
        h, hu = a['seed_nominal_height_m'], np.ptp(a['original_common_height_interval_m'])/2.
        d = report['replay_geometry']['support_offset_m']
        captured = []

        def trace(frame, event, _arg):
            if frame.f_code is c._refit_cuboid.__code__:
                if event == 'return':
                    captured.extend(frame.f_locals['rows'])
                return trace
            return None

        sys.settrace(trace)
        try:
            fitted = c._refit_cuboid(lo, hi, h, hu, A, d, T, K, edges, edge_pixels)
        finally:
            sys.settrace(None)
        assert fitted['parameters'].tolist() == a['bounded_refit_parameters']
        assert fitted['score'] == max(a['full_silhouette_rgb_distances_px']+a['corner_rgb_distances_px'])
        inspected = []
        for index, candidate in enumerate(captured):
            polygon = np.zeros(mask.shape, np.uint8)
            cv2.fillConvexPoly(polygon, np.rint(candidate['model']['hull_pixels']).astype(np.int32), 1)
            interior = cv2.erode(polygon, np.ones((3, 3), np.uint8)) > 0
            denominator = int(interior.sum())
            numerator = int(np.count_nonzero(interior & mask))
            coverage = float(numerator)/max(1, denominator)
            inspected.append(dict(iteration_index=index, model=c._plain(candidate['model']),
                axes=c._plain(candidate['axes']), lower=c._plain(candidate['lower']), upper=c._plain(candidate['upper']),
                height=candidate['height'], score=candidate['score'], parameters=c._plain(candidate['parameters']),
                original_selected=candidate is fitted, silhouette_mask_coverage=coverage,
                mask_intersection_pixels=numerator, eroded_whole_hull_pixels=denominator,
                original_max_rgb_gate_passed=candidate['score'] <= math.sqrt(8.),
                original_mask_coverage_gate_passed=coverage >= .88,
                both_original_raster_and_coverage_gates_passed=(candidate['score'] <= math.sqrt(8.) and coverage >= .88)))
        rows.append(dict(request_id=request, source_stamp_ns=stamp,
            exact_original_refit_reproduced=True, initial_four_groups_unchanged=True,
            algorithm_altered=False, alternative_candidates_generated=False,
            args=dict(seed_axes_base=A.tolist(), seed_lower=lo.tolist(), seed_upper=hi.tolist(),
                height=h, height_uncertainty=hu, support_offset_m=d, T_base_optical=T.tolist(), K=K.tolist()),
            candidates=inspected, original_selected_index=next(r['iteration_index'] for r in inspected if r['original_selected']),
            existing_candidates_passing_both=[r['iteration_index'] for r in inspected if r['both_original_raster_and_coverage_gates_passed']]))
    assert all(sha(path) == digest for path, digest in inputs.items())
    OUT.mkdir()
    result = dict(schema=1, kind='original_nominal_refit_candidate_trace', created_ns=time.time_ns(),
        method='Trace only original _refit_cuboid return locals; recompute unchanged full-hull coverage of each already-retained original L2 iteration.',
        source_and_inputs_sha256=inputs, requests=rows, changed_production_source=False,
        original_gates_unchanged=True, full_box_qualification_proven=False,
        ROS_called=False, hardware_contacted=False, motion_commands_sent=False, temperature_queried=False)
    output = OUT/'result.json'
    with output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    for row in rows:
        print(json.dumps(dict(request_id=row['request_id'], retained_candidates=len(row['candidates']),
            original_selected_index=row['original_selected_index'],
            existing_candidates_passing_both=row['existing_candidates_passing_both'],
            summary=[{key:r[key] for key in ('iteration_index','score','silhouette_mask_coverage','mask_intersection_pixels','eroded_whole_hull_pixels')} for r in row['candidates']]), ensure_ascii=False))
    print(json.dumps(dict(result=str(output), sha256=sha(output))))


if __name__ == '__main__':
    main()
