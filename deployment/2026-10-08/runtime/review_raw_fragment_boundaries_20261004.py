"""Static same-image segment/outline audit; never establishes a tracked target."""
from copy import deepcopy
from dataclasses import replace
import argparse
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from cv_bridge import CvBridge

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('raw_hold_replay', HERE/'review_raw_fragment_hold_20261004.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)
module = replay.module


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    archive = Path('/media/zhuyupei/SanStand')
    if not archive.is_mount() or archive/'实验记录/Robot-Controller' not in args.output.resolve().parents:
        raise RuntimeError('SanStand report output required')
    cv2.setNumThreads(1)
    rows, frames = replay.load_archive(args.source)
    results, prior = [], None
    for index, (row, (rgb, native, transform)) in enumerate(zip(rows, frames)):
        config = module.Config(**row['config'])
        intrinsics = [row['camera'][name] for name in ('fx', 'fy', 'cx', 'cy')]
        depth = native.astype(np.float32)*row['camera']['depth_scale']
        initial = module.segment_with_support_fallback(depth, intrinsics, transform, config)
        selected, reason = module.TargetTracker(config).choose(initial, depth.shape)
        refined, refinement_error = None, None
        if selected is not None:
            try:
                refined = module.refine_candidate(rgb, depth, intrinsics, transform, selected, config)
            except ValueError as exc:
                refinement_error = str(exc)
        result = dict(index=index, source_stamp_ns=row['source_stamp_ns'],
            initial_candidate_count=len(initial.candidates), selection_reason=reason,
            selected_refined_points=None if refined is None else refined.metrics['valid_points'],
            refinement_error=refinement_error,
            initial_candidates=[dict(bbox=list(candidate.bbox), pixels=candidate.metrics['valid_points'])
                                for candidate in initial.candidates])
        if prior is not None:
            locked = module.segment_with_support_fallback(depth, intrinsics, transform, config,
                anchor=prior['raw'], last=prior['raw'], reference_plane_base=prior['plane'])
            if len(locked.candidates) >= 2:
                reference = prior['refined']
                x, y, width, height = cv2.boundingRect(reference.mask)
                policy = replace(module.CenterSearchPolicy(center_xy=(.5, .5)),
                    center_xy=((x+.5*width)/rgb.shape[1], (y+.5*height)/rgb.shape[0]))
                proposals = module.propose_rgb_observation_regions(rgb, policy=policy)
                ref_hist, ref_contour = module._appearance(prior['rgb'], reference.mask)
                gates = module.HandoffPolicy()
                scored = []
                for proposal_index, proposal in enumerate(proposals.candidates):
                    mask = module._mask(proposal.mask, rgb.shape[:2])
                    iou = np.count_nonzero(mask & reference.mask)/np.count_nonzero(mask | reference.mask)
                    hist, contour = module._appearance(rgb, mask)
                    color = float(np.sqrt(max(0., 1.-np.sum(np.sqrt(hist*ref_hist)))))
                    shape = float(cv2.matchShapes(ref_contour, contour, cv2.CONTOURS_MATCH_I1, 0.))
                    if iou >= .85 and color <= gates.max_colour_distance and shape <= gates.max_contour_distance:
                        score = .65*color/gates.max_colour_distance+.35*shape/gates.max_contour_distance
                        scored.append((score, proposal_index, iou, color, shape))
                scored.sort()
                unique = bool(scored and (len(scored) == 1 or scored[1][0]-scored[0][0] > gates.ambiguity_margin))
                matching_mask = proposals.candidates[scored[0][1]].mask if unique else None
                bridge = CvBridge()
                depth_message = bridge.cv2_to_imgmsg(native, encoding='16UC1')
                dummy = SimpleNamespace(config=config, bridge=bridge,
                    tracker=SimpleNamespace(anchor=prior['raw'], last=prior['raw']))
                candidates = []
                seen = np.zeros(depth.shape, bool)
                for candidate in locked.candidates:
                    mask = candidate.mask != 0
                    outside_old = mask & (reference.mask == 0)
                    outside_current = None if matching_mask is None else mask & (matching_mask == 0)
                    ownership = None
                    try:
                        fragment = module.UnknownTabletopPerception._locked_rgb_shrinking_fragment(
                            dummy, candidate, dict(image_shape=depth.shape, mask=reference.mask),
                            (None, depth_message), row['camera'])
                        classification = 'metric_path' if fragment is None else 'fragment_geometry_valid'
                        if fragment is not None and matching_mask is not None:
                            ownership = module.UnknownTabletopPerception._locked_rgb_fragment_ownership(
                                dummy, fragment['mask'], reference.mask, matching_mask)
                            classification = 'valid_owned_fragment'
                    except (ValueError, TypeError) as exc:
                        classification = str(exc)
                    count = int(np.count_nonzero(mask))
                    candidates.append(dict(bbox=list(candidate.bbox), points=count,
                        ratios_xy=(candidate.extent_m[:2]/np.maximum(prior['raw'].extent_m[:2], .003)).tolist(),
                        fragment_helper_result=classification,
                        production_ownership_evidence=ownership,
                        outside_reference_pixels=int(np.count_nonzero(outside_old)),
                        outside_reference_fraction=float(np.count_nonzero(outside_old)/count),
                        outside_current_pixels=None if outside_current is None else int(np.count_nonzero(outside_current)),
                        overlap_previous_fragment_pixels=int(np.count_nonzero(mask & seen))))
                    seen |= mask
                age = (row['source_stamp_ns']-rows[prior['index']]['source_stamp_ns'])*1e-9+row['source_age_at_end_sec']
                result['multi_fragment_static_audit'] = dict(reference_index=prior['index'],
                    reference_age_seconds=age, within_original_point8_seconds=age <= .8,
                    reference_outline_pixels=int(np.count_nonzero(reference.mask)),
                    current_unique_rgb_match=unique,
                    best_rgb_match=None if not scored else dict(iou=scored[0][2], color=scored[0][3], shape=scored[0][4]),
                    candidates=candidates,
                    all_members_pass_fragment_geometry_and_current_ownership=(unique and all(
                        candidate['fragment_helper_result'] == 'valid_owned_fragment'
                        and candidate['overlap_previous_fragment_pixels'] == 0 for candidate in candidates)))
        results.append(result)
        if refined is not None and refined.metrics['valid_points'] >= 120 and selected.quality >= .5:
            prior = dict(index=index, raw=deepcopy(selected), refined=deepcopy(refined),
                         plane=initial.plane_base.copy(), rgb=rgb)
    report = dict(source=str(args.source), source_sha256=replay.sha(args.source),
        script_sha256=replay.sha(__file__), producer_sha256=replay.sha(module.__file__),
        raw_hashes_verified=True, ros_io=False, robot_commands_sent=False,
        interpretation=['Static reference frames have sufficient per-frame measured points but are not confirmed three-frame target locks.',
            'The original production segmentation, RGB refinement, proposal, appearance and fragment helper are invoked without relaxing thresholds.',
            'No result here is a generation-9 exact first-loss replay or motion authorization.'], rows=results)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps([dict(index=row['index'], **row['multi_fragment_static_audit'])
                     for row in results if 'multi_fragment_static_audit' in row], indent=2))


if __name__ == '__main__':
    main()
