"""Finite same-current-source ownership diagnosis; no ROS or execution authority."""
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib
import json
import os
import sys
import time

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
N = M / '实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z'
I = N / 'verification/gen29_attempt1_shadow_frames'
O = N / 'verification/gen30_current_fragment_ownership_r1'
assert M.is_mount() and not O.exists()
assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:1'
for p in ('src/alicia_grasp_modes/src', 'src/alicia_flexible_grasp_supervisor/src'):
    sys.path.insert(0, str(W / p))
import cv2
import numpy as np
from alicia_grasp_modes import tabletop as table, rgb_instance as rgb
from alicia_grasp_modes.rgb_observation_handoff import _appearance
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance, match_observation_appearance
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def mh(a):
    return hashlib.sha256(np.ascontiguousarray(a > 0, dtype=np.uint8).tobytes()).hexdigest()


def iou(a, b):
    a, b = a > 0, b > 0
    return float(np.count_nonzero(a & b) / np.count_nonzero(a | b))


def serial(a):
    if isinstance(a, np.ndarray):
        return a.tolist()
    if isinstance(a, np.generic):
        return a.item()
    raise TypeError(type(a).__name__)


inputs = {}


def bound(path):
    inputs[str(path)] = sha(path)
    return json.loads(Path(path).read_text())


meta = bound(I / 'mask_contract_review_r2/result.json')
assert inputs[str(I / 'mask_contract_review_r2/result.json')] == '6090a42d3800e3e256cccee7012d80b734760e978064d17238970f2f2140c9de'
assert meta['source_unchanged'] and meta['input_unchanged']
baseline_path = W / '.runtime/gen30_baseline_source/manifest.json'
baseline = json.loads(baseline_path.read_text())
assert len(baseline) == 970 and all(sha(W / p) == s for p, s in baseline.items())
script_sha = sha(__file__)
cfg = table.Config(**meta['effective_config'])
cv2.setNumThreads(1)
qns = meta['qualification_template_source_ns']
qfolder = I / ('source_%d' % qns)
qmeta = bound(qfolder / 'metadata.json')
inputs[str(qfolder / 'raw_arrays.npz')] = sha(qfolder / 'raw_arrays.npz')
q = dict(np.load(qfolder / 'raw_arrays.npz', allow_pickle=False))
qvalid = rectification_rgb_valid_mask(qmeta['camera_parameters'], q['depth_raw'].shape)
histogram, contour = _appearance(q['color_bgr'], q['object_mask'])
reference = FrozenObservationAppearance(q['color_bgr'], q['object_mask'], qvalid, qns, histogram, contour)
O.mkdir()
rows = []


def gate(candidate, anchor, last):
    return dict(matches_original_geometry=bool(table._matching_candidates([candidate], anchor, last, cfg)),
        anchor_distance_m=float(np.linalg.norm(candidate.position_base - anchor.position_base)),
        prior_step_m=float(np.linalg.norm(candidate.position_base - last.position_base)),
        extent_ratio=(candidate.extent_m[:2] / np.maximum(anchor.extent_m[:2], .003)).tolist())


for previous in meta['rows'][1:]:
    ns = previous['source_stamp_ns']
    folder = I / ('source_%d' % ns)
    status = bound(folder / 'detector_status.json')['payload']
    frame_meta = bound(folder / 'metadata.json')
    inputs[str(folder / 'raw_arrays.npz')] = sha(folder / 'raw_arrays.npz')
    a = dict(np.load(folder / 'raw_arrays.npz', allow_pickle=False))
    before_arrays = {k: hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k, v in a.items()}
    color, K, T = a['color_bgr'], a['intrinsics'], a['T_base_camera_link']
    depth = a['depth_raw'].astype(np.float32) * float(a['depth_scale_m'])
    valid = rectification_rgb_valid_mask(frame_meta['camera_parameters'], depth.shape)
    anchor = NS(position_base=np.asarray(status['anchor_position_base']), extent_m=np.asarray(status['anchor_extent_m']),
        support_footprint_base=np.asarray(status['reference_support_footprint_base']),
        metrics={'median_height_m': status['reference_object_height_m']})
    last = NS(position_base=np.asarray(previous['prior_position_used']))
    locked_plane = np.asarray(status['reference_support_plane_base'])
    current_plane = np.asarray(previous['support_plane'])
    assert previous['support_source'] == 'current_ring_fit'
    spatial = rgb.freeze_observation_spatial_reference(anchor, locked_plane)
    wall, cpu = time.perf_counter(), time.process_time()
    result = table.segment_with_support_fallback(depth, K, T, cfg, anchor, last, locked_plane, prefer_current_support=True)
    segmentation_time = dict(wall_sec=time.perf_counter()-wall, cpu_sec=time.process_time()-cpu)
    np.testing.assert_array_equal(result.plane_base, current_plane)
    candidates = result.candidates
    original_matches = table._matching_candidates(candidates, anchor, last, cfg)
    assert len(candidates) == 4 and len(original_matches) == 1
    main = next(index for index, candidate in enumerate(candidates) if candidate is original_matches[0][1])
    assert mh(candidates[main].mask) == previous['native_candidates'][0]['native_normalized_mask_sha256']
    paths, outputs = {}, {}
    for mode in ('original_independent_calls', 'same_source_frame_refinement'):
        wall, cpu = time.perf_counter(), time.process_time()
        owner = rgb._FrameRefinement(depth, K, T, spatial, cfg) if mode != 'original_independent_calls' else None
        details, produced = [], []
        for index, candidate in enumerate(candidates):
            cw, cc = time.perf_counter(), time.process_time()
            detail = dict(candidate_index=index, native_pixels=int(np.count_nonzero(candidate.mask)),
                native_mask_sha256=mh(candidate.mask), native_geometry=gate(candidate, anchor, last))
            try:
                refined = (owner.refine(color, candidate, rgb_valid=valid) if owner is not None else
                    rgb.refine_candidate(color, depth, K, T, candidate, cfg, rgb_valid=valid,
                        observation_arrival_retry=True, observation_spatial_reference=spatial))
                detail.update(outcome='returned', mask_pixels=int(np.count_nonzero(refined.mask)),
                    mask_sha256=mh(refined.mask), refined_geometry=gate(refined, anchor, last),
                    metrics=refined.metrics)
                produced.append(refined)
            except ValueError as exc:
                detail.update(outcome='rejected', error=str(exc))
                produced.append(None)
            detail.update(wall_sec=time.perf_counter()-cw, cpu_sec=time.process_time()-cc)
            details.append(detail)
        paths[mode] = dict(wall_sec=time.perf_counter()-wall, cpu_sec=time.process_time()-cpu, candidates=details)
        outputs[mode] = produced
    original, shared = paths.values()
    for first, second in zip(original['candidates'], shared['candidates']):
        assert (first['outcome'], first.get('error'), first.get('mask_sha256')) == (second['outcome'], second.get('error'), second.get('mask_sha256'))
    produced = outputs['same_source_frame_refinement']
    assert mh(produced[main].mask) == previous['expected_normalized_mask_sha256']
    # Recreate each ORIGINAL first crop/core independently; none is unioned or promoted.
    owner = rgb._FrameRefinement(depth, K, T, spatial, cfg)
    pc = T.T @ current_plane
    heights = np.einsum('ijk,k->ij', owner.points, pc[:3]) + pc[3]
    margin = max(8, int(round(16 * min(depth.shape[1]/640., depth.shape[0]/480.))))
    cores, core_rows = [], []
    for index, c in enumerate(candidates):
        yy, xx = np.nonzero(c.mask)
        sl = np.s_[max(0, int(yy.min())-margin):min(depth.shape[0], int(yy.max())+margin+1),
                   max(0, int(xx.min())-margin):min(depth.shape[1], int(xx.max())+margin+1)]
        core = np.zeros(depth.shape, bool)
        try:
            small, _, evidence = owner.confidence_labels(c.mask[sl] > 0, owner.points[sl], depth[sl], heights[sl], pc[:3], cfg, bgr=color[sl])
            core[sl] = small
            core_rows.append(dict(candidate_index=index, core_pixels=int(core.sum()), evidence=evidence))
        except ValueError as exc:
            core_rows.append(dict(candidate_index=index, error=str(exc), core_pixels=0))
        cores.append(core)
    pairwise, identities = [], []
    for index, c in enumerate(produced):
        if c is None:
            continue
        context = dict(reference=reference, reference_role='qualified_station_identity_template',
            source_ns=ns, same_station_mask_iou_minimum=.85, check_current=lambda: None)
        appearance = match_observation_appearance(color, c.mask, histogram, contour,
            rgb_valid=valid, appearance_context=context)
        identities.append(dict(candidate_index=index, appearance=appearance,
            owns_core_fraction=[float(np.count_nonzero((c.mask > 0) & core)/np.count_nonzero(core)) if np.any(core) else None for core in cores]))
        assert cv2.imwrite(str(O / ('%d_candidate_%d_refined.png' % (ns, index))), c.mask)
        for second in range(index):
            if produced[second] is not None:
                pairwise.append(dict(first=second, second=index, iou=iou(c.mask, produced[second].mask)))
    raw_union = np.logical_or.reduce([c.mask > 0 for c in candidates])
    # Connectivity alone is reported, never used as an RGB seed or output.
    components = table._depth_components(raw_union.astype(np.uint8), depth, cfg.min_object_points, cfg.depth_edge_m, connectivity=8)
    row = dict(source_stamp_ns=ns, original_matching_index=main, current_plane=current_plane,
        source_support_audit=result.metrics, segmentation_time=segmentation_time, paths=paths,
        same_source_cache_identical_to_original=True, original_main_mask_exact=True,
        native_cores=core_rows, identities=identities, pairwise=pairwise,
        all_native_eight_connected_components=len(components),
        new_source_deadlines_replayed=False, current_template_used_only_for_original_identity_check=True)
    rows.append(row)
    assert before_arrays == {k: hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k, v in a.items()}
    print(json.dumps(dict(source_stamp_ns=ns, main_index=main,
        per_candidate=[{k:v for k,v in x.items() if k in ('candidate_index','outcome','error','mask_pixels','native_geometry','refined_geometry')} for x in shared['candidates']],
        identities=identities, pairwise=pairwise,
        original_wall=original['wall_sec'], new_wall=shared['wall_sec']), default=serial), flush=True)

assert all(sha(W / p) == s for p, s in baseline.items())
assert all(sha(p) == s for p, s in inputs.items()) and sha(__file__) == script_sha
report = dict(schema=1, rows=rows, source_sha256=baseline, source_count=970,
    source_unchanged=True, input_sha256=inputs, input_unchanged=True, script_sha256=script_sha,
    classification='offline_finite_original_current_fragment_ownership_diagnosis',
    no_ros_calls=True, no_hardware_access=True, production_modified=False, execution_authority=False,
    physical_grasp_verified=False, new_restart_runs_only_gen29=True,
    limitations=['Three recorded sources with documented prior-position proxies, not a full asynchronous producer replay.',
        'Nonmatching native fragments are diagnostic hypotheses only and are never admitted as original matches.',
        'The appearance callback is offline and confers no current-source or identity-deadline authority.',
        'Recorded spatial reference proposes a crop only; no old image, mask or depth becomes current foreground.',
        'CPU/wall measurements are single finite passes under current host load, not controlled runtime benchmarks.'])
(O / 'result.json').write_text(json.dumps(report, indent=2, default=serial, allow_nan=False) + '\n')
print(json.dumps(dict(result=str(O / 'result.json'), sha256=sha(O / 'result.json'))), flush=True)
