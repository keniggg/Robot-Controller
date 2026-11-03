#!/usr/bin/env python3
"""Offline camera-stage evidence. No ROS, device opening, or geometry authority."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

WORK = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WORK / '.runtime/realsense-2.56.5'))
sys.path.insert(0, str(WORK / 'src/alicia_grasp_modes/src'))
import pyrealsense2 as rs
from alicia_grasp_modes import tabletop
from alicia_grasp_modes.rgb_observation_handoff import HandoffPolicy, _appearance
from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy, propose_rgb_observation_regions

REFERENCE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/rgb_acquisition_gateway_20261004T114259.948391Z_1541ec9e/result.json')
STAGES = ('raw_depth', 'aligned_depth', 'post_spatial_depth', 'pre_stationary_depth',
          'post_stationary_depth', 'final_depth')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array_sha(array):
    return hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest()


def quantiles(values):
    return dict(zip(('min', 'p10', 'p50', 'p90', 'max'),
                    np.percentile(values, [0, 10, 50, 90, 100]).tolist())) if len(values) else None


def intrinsics(info):
    out = rs.intrinsics()
    for name in ('width', 'height', 'fx', 'fy', 'ppx', 'ppy'):
        setattr(out, name, info[name])
    out.model = getattr(rs.distortion, info['model'].split('.')[-1])
    out.coeffs = info['coeffs']
    return out


def rays(info):
    model = intrinsics(info)
    vv, uu = np.indices((info['height'], info['width']))
    return np.asarray([rs.rs2_deproject_pixel_to_point(model, [float(u), float(v)], 1.)
                       for u, v in zip(uu.ravel(), vv.ravel())]).reshape(*uu.shape, 3)


def project(points, model):
    return np.asarray([rs.rs2_project_point_to_pixel(model, point.tolist()) for point in points])


def delta(before, after, selected, scale):
    valid = selected & (before > 0) & (after > 0)
    return dict(lost_positive_to_zero=int(np.count_nonzero(selected & (before > 0) & (after == 0))),
                filled_zero_to_positive=int(np.count_nonzero(selected & (before == 0) & (after > 0))),
                changed_positive=int(np.count_nonzero(valid & (before != after))),
                signed_change_mm=quantiles((after[valid].astype(float)-before[valid])*scale*1000),
                byte_identical=bool(np.array_equal(before[selected], after[selected])))


def plane_fit(points, mask, config):
    try:
        plane, ratio, residual, count = tabletop._fit_plane(points[mask], config)
        return plane, dict(plane_optical=plane.tolist(), inlier_ratio=ratio,
                           residual_mm=residual*1000, sampled_points=count,
                           available_points=int(mask.sum()))
    except ValueError as exc:
        return None, dict(error=str(exc), available_points=int(mask.sum()))


def projected_native_footprint(mask, depth_z, color_rays, native_model, rotation, translation):
    """Map color ROI rays to native pixels at explicit hypothetical color Z.

    This only locates native samples including zeros; it never creates measured
    points. Working-range endpoints are discrete sensitivity samples, not a
    proof of coverage over every intermediate depth.
    """
    color_points = color_rays[mask]*np.asarray(depth_z).reshape(-1, 1)
    native_points = (color_points-translation) @ rotation
    uv = project(native_points, native_model)
    pixels = np.rint(uv).astype(int)
    inside = ((pixels[:, 0] >= 0) & (pixels[:, 0] < mask.shape[1]) &
              (pixels[:, 1] >= 0) & (pixels[:, 1] < mask.shape[0]))
    footprint = np.zeros(mask.shape, bool)
    footprint[pixels[inside, 1], pixels[inside, 0]] = True
    return footprint, uv


def raw_point_raster(raw, scale, native_info, color_model, rotation, translation, selected):
    """SDK numerical corner transfer for nearby positive raw samples only.

    Mirrors scalar align.cpp rectangle rasterization. Saved actual align output
    remains ground truth; any implementation difference is reported separately.
    """
    vv, uu = np.nonzero((raw > 0) & selected)
    z = raw[vv, uu]*scale
    corners = []
    for offset in (-.5, .5):
        optical = np.c_[(uu+offset-native_info['ppx'])/native_info['fx'],
                        (vv+offset-native_info['ppy'])/native_info['fy'], np.ones(len(uu))]*z[:, None]
        projected = project(optical @ rotation.T + translation, color_model)
        corners.append((projected+.5).astype(int))
    raster = np.zeros(raw.shape, raw.dtype)
    for row, col, first, last in zip(vv, uu, *corners):
        if first[0] < 0 or first[1] < 0 or last[0] >= raw.shape[1] or last[1] >= raw.shape[0]:
            continue
        view = raster[first[1]:last[1]+1, first[0]:last[0]+1]
        value = raw[row, col]
        view[:] = np.where(view, np.minimum(view, value), value)
    return raster


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert rs.__version__ == '2.56.5'
    cv2.setNumThreads(1)
    args.output.mkdir(parents=True, exist_ok=True)
    stages = args.capture/'stages'
    summary = json.loads((stages/'summary.json').read_text())
    capture_result = json.loads((args.capture/'result.json').read_text())
    assert capture_result['observer_sha256'] == sha(WORK/'.runtime/camera_stage_observer.py')
    for item in summary['files']:
        path = stages/item['name']
        assert path.stat().st_size == item['bytes'] and sha(path) == item['sha256']
    original = json.loads(REFERENCE.read_text())
    source = original['frames']['initial']
    assert sha(source['source_file']) == source['source_file_sha256']
    with np.load(source['source_file'], allow_pickle=False) as old:
        original_hist, original_contour = _appearance(old['color_bgr'], old['region_mask'].astype(np.uint8))
    policy, search, config = HandoffPolicy(), CenterSearchPolicy(center_xy=(.5, .5)), tabletop.Config()
    report = dict(schema=1, scope='same_frame_camera_stage_offline_diagnosis',
        robot_commands_sent=False, ros_io=False, sensor_options_changed=False,
        metric_geometry_authority=False, original_identity_confirmed=False,
        script_sha256=sha(__file__), observer_sha256=sha(WORK/'.runtime/camera_stage_observer.py'),
        source_capture=str(args.capture), capture_result_sha256=sha(args.capture/'result.json'),
        source_manifest_sha256=sha(stages/'summary.json'), all_source_file_and_array_hashes_verified=True,
        original_reference=dict(path=str(REFERENCE), sha256=sha(REFERENCE),
            source_stamp_ns=source['source_stamp_ns'], source_file_sha256=source['source_file_sha256']),
        search_policy=asdict(search), appearance_policy=asdict(policy), plane_config=asdict(config),
        capture_summary=summary, sdk_version=rs.__version__, rows=[],
        limitations=[
            'No same-source actual SDK feedback/TF was captured. A preserved command is not a measured pose. Original identity is only an appearance hypothesis, never a restored episode or geometry authority.',
            'Reference association uses default generic centered RGB proposals and the original immutable RGB histogram/contour; no hand-selected target colour, physical size or mask is used.',
            'Pre-align and aligned grids differ. Raw zeros are measured only in an SDK-mapped footprint with explicit depth sensitivity; direct same-index RGB ROI counting is prohibited.',
            'Native-footprint depth hypotheses are discrete sensitivity samples, not a continuous full-range coverage proof. This capture has matching 640x480 native/color grids; the helper is not generalized to unequal dimensions.',
            'Support-relative height is camera-local measured plane distance, not independently known physical thickness. No missing depth is filled.',
            'SDK alignment retains native Z16 values; color-ray times aligned Z follows the SDK aligned-depth interpretation, not an independently exact color-axis depth. The direct positive-raw forward transform reports color Z separately.',
            'Scalar SDK corner rasterization is a numerical comparison with actual SDK output, not an assertion that the optimized align implementation is identical.',
            'Independent device/sensor exposure and stereo confidence were not captured; their causal contribution cannot be isolated.',
            'Observer source and regression tests show identity-preserving returns and owned copies. Tap overhead is real; no simultaneous unobserved sensor run exists to prove zero timing effect.',
        ])
    first_info = json.loads((stages/'sample_01.json').read_text())
    native_info = first_info['sdk_frames']['depth']['intrinsics']
    color_info = first_info['sdk_frames']['color']['intrinsics']
    native_model, color_model = intrinsics(native_info), intrinsics(color_info)
    native_rays, color_rays = rays(native_info), rays(color_info)
    assert not any(native_info['coeffs'])
    rotation = np.array(first_info['depth_to_color']['rotation_column_major']).reshape(3, 3, order='F')
    translation = np.array(first_info['depth_to_color']['translation_m'])
    vv, uu = np.indices(native_rays.shape[:2])
    radius = np.hypot(uu-native_rays.shape[1]/2, vv-native_rays.shape[0]/2)
    report['native_intrinsics'], report['color_intrinsics'] = native_info, color_info
    report['depth_to_color'] = first_info['depth_to_color']
    for index in range(1, 13):
        info = json.loads((stages/f'sample_{index:02}.json').read_text())
        assert info['complete'] and info['sdk_frames']['depth']['intrinsics'] == native_info
        assert info['sdk_frames']['color']['intrinsics'] == color_info
        assert info['depth_to_color'] == first_info['depth_to_color']
        for stream in ('depth', 'color'):
            recorded = info['production_read_diagnostics']['raw_frames'][stream]
            assert recorded['frame_number'] == info['sdk_frames'][stream]['frame_number']
            assert recorded['device_timestamp_ms'] == info['sdk_frames'][stream]['timestamp_ms']
        with np.load(stages/f'sample_{index:02}.npz', allow_pickle=False) as archive:
            data = {key:archive[key] for key in archive.files}
        for name, array in data.items():
            assert array_sha(array) == info['arrays'][name]['sha256']
        scale = info['depth_scale_m']
        rgb = data['raw_rgb']
        if index in (1, 12):
            assert cv2.imwrite(str(args.output/f'sample_{index:02}_raw_rgb.png'), rgb)
        proposals = propose_rgb_observation_regions(rgb, policy=search)
        alternatives = []
        for number, candidate in enumerate(proposals.candidates):
            hist, contour = _appearance(rgb, candidate.mask.astype(np.uint8))
            colour = float(np.sqrt(max(0., 1.-np.sum(np.sqrt(hist*original_hist)))))
            shape = float(cv2.matchShapes(original_contour, contour, cv2.CONTOURS_MATCH_I1, 0.))
            alternatives.append(dict(index=number, bbox_xywh=list(candidate.bbox_xywh),
                pixels=candidate.area_px, colour_distance=colour, contour_distance=shape,
                eligible=colour <= policy.max_colour_distance and shape <= policy.max_contour_distance,
                score=.65*colour/policy.max_colour_distance+.35*shape/policy.max_contour_distance))
        eligible = sorted((a for a in alternatives if a['eligible']), key=lambda a:a['score'])
        unique = bool(eligible and (len(eligible) == 1 or eligible[1]['score']-eligible[0]['score'] > policy.ambiguity_margin))
        row = dict(index=index, depth_frame=info['sdk_frames']['depth'], color_frame=info['sdk_frames']['color'],
            pairing=info['production_read_diagnostics']['pair_timing'],
            observer_overhead_ms=info['capture_overhead_sec']*1000,
            read_duration_ms=(info['read_completed_monotonic']-info['read_started_monotonic'])*1000,
            stationary=info['runtime_profile']['stationary_temporal'],
            stationary_rgb=info['production_read_diagnostics']['stationary_rgb_evidence'],
            raw_rgb_identical_to_final=bool(np.array_equal(rgb, data['final_rgb'])),
            global_positive={name:int(np.count_nonzero(data[name])) for name in STAGES},
            global_deltas={before+'__'+after:delta(data[before], data[after], np.ones(rgb.shape[:2], bool), scale)
                           for before, after in zip(STAGES[1:-1], STAGES[2:])},
            proposal_reason=proposals.reason, alternatives=alternatives, unique_appearance_hypothesis=unique)
        if not unique:
            report['rows'].append(row)
            continue
        mask = proposals.candidates[eligible[0]['index']].mask.astype(bool)
        row['hypothesis'] = dict(**eligible[0], mask_sha256=array_sha(mask))
        raw, aligned = data['raw_depth'], data['aligned_depth']
        z = aligned.astype(float)*scale
        valid = (z >= config.depth_min_m) & (z <= config.depth_max_m)
        points = color_rays*z[:, :, None]
        plane, plane_audit = plane_fit(points, valid & (radius >= 70) & (radius <= 170), config)
        row['measured_camera_support_ring_70_170'] = plane_audit
        x, y, width, height = eligible[0]['bbox_xywh']
        local_radius = np.hypot(uu-(x+(width-1)/2), vv-(y+(height-1)/2))
        inner = float(np.hypot(width, height))
        outer = min(170., 3*inner)
        local_plane, local_audit = plane_fit(points, valid & (local_radius >= inner) & (local_radius <= outer), config)
        row['measured_camera_local_support'] = dict(inner_radius_px=inner, outer_radius_px=outer, **local_audit)
        row['color_roi_stages'] = {}
        for name in STAGES[1:]:
            depth = data[name].astype(float)*scale
            selected = mask & (depth >= config.depth_min_m) & (depth <= config.depth_max_m)
            measured = color_rays[selected]*depth[selected, None]
            heights = measured@plane[:3]+plane[3] if plane is not None else np.array([])
            local_heights = measured@local_plane[:3]+local_plane[3] if local_plane is not None else np.array([])
            row['color_roi_stages'][name] = dict(pixels=int(mask.sum()), valid=int(selected.sum()),
                zero=int(np.count_nonzero(mask & (data[name] == 0))),
                optical_z_mm=quantiles(depth[selected]*1000),
                height_to_ring_plane_mm=quantiles(heights*1000),
                height_to_local_plane_mm=quantiles(local_heights*1000),
                at_least_4mm_to_local_plane=int(np.count_nonzero(local_heights >= .004)))
        row['color_roi_deltas'] = {before+'__'+after:delta(data[before], data[after], mask, scale)
                                  for before, after in zip(STAGES[1:-1], STAGES[2:])}
        row['native_footprint_depth_sensitivity'] = []
        nominal = float(np.median(z[mask & valid]))
        row['nominal_footprint_assumed_color_z_m'] = nominal
        footprints = []
        for assumed in sorted(set((.03, .06, .12, nominal, .5, 1., 2.))):
            footprint, _ = projected_native_footprint(mask, np.repeat(assumed, int(mask.sum())), color_rays,
                                                      native_model, rotation, translation)
            footprints.append(footprint)
            row['native_footprint_depth_sensitivity'].append(dict(assumed_color_z_m=assumed,
                pixels=int(footprint.sum()), raw_positive=int(np.count_nonzero(raw[footprint])),
                raw_zero=int(np.count_nonzero(raw[footprint] == 0)),
                raw_positive_fraction=float(np.count_nonzero(raw[footprint])/footprint.sum()),
                raw_optical_z_mm=quantiles(raw[footprint & (raw > 0)]*scale*1000)))
        raw_positive = raw > 0
        native_points = native_rays[raw_positive]*(raw[raw_positive]*scale)[:, None]
        current_pixels = project(native_points@rotation.T+translation, color_model)
        pixel_indices = np.rint(current_pixels).astype(int)
        contained = ((pixel_indices[:, 0] >= 0) & (pixel_indices[:, 0] < mask.shape[1]) &
                     (pixel_indices[:, 1] >= 0) & (pixel_indices[:, 1] < mask.shape[0]))
        hits = contained.copy()
        hits[contained] &= mask[pixel_indices[contained, 1], pixel_indices[contained, 0]]
        raw_in_mask = np.zeros(mask.shape, bool)
        raw_in_mask[raw_positive] = hits
        native_color_points = native_points@rotation.T+translation
        raw_roi_height = native_color_points[hits]@local_plane[:3]+local_plane[3] if local_plane is not None else np.array([])
        row['positive_raw_measurements_projecting_into_rgb_outline'] = dict(count=int(hits.sum()),
            raw_optical_z_mm=quantiles(native_points[hits, 2]*1000),
            color_optical_z_mm=quantiles(native_color_points[hits, 2]*1000),
            height_to_local_plane_mm=quantiles(raw_roi_height*1000),
            at_least_4mm_to_local_plane=int(np.count_nonzero(raw_roi_height >= .004)))
        # Include a broad SDK-mapped footprint margin so corner rasterization
        # covers all neighbors which can contribute to the RGB outline.
        vicinity = cv2.dilate(np.logical_or.reduce(footprints).astype(np.uint8), np.ones((11, 11), np.uint8)).astype(bool)
        scalar = raw_point_raster(raw, scale, native_info, color_model, rotation, translation, vicinity)
        row['alignment_sdk_corner_reconstruction'] = dict(
            rgb_outline_pixels=int(mask.sum()), raw_corner_coverage=int(np.count_nonzero(scalar[mask])),
            actual_aligned_positive=int(np.count_nonzero(aligned[mask])),
            actual_zero_despite_raw_corner_coverage=int(np.count_nonzero(mask & (scalar > 0) & (aligned == 0))),
            actual_positive_without_scalar_corner_coverage=int(np.count_nonzero(mask & (scalar == 0) & (aligned > 0))),
            both_positive_different_depth=int(np.count_nonzero(mask & (scalar > 0) & (aligned > 0) & (scalar != aligned))),
            actual_zero_and_no_raw_corner_coverage=int(np.count_nonzero(mask & (scalar == 0) & (aligned == 0))))
        if index in (1, 12):
            overlay = rgb.copy()
            cv2.drawContours(overlay, cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], -1, (0, 255, 0), 1)
            assert cv2.imwrite(str(args.output/f'sample_{index:02}_appearance_hypothesis.png'), overlay)
        report['rows'].append(row)
        print(json.dumps(dict(index=index, outline_pixels=int(mask.sum()), aligned_valid=int(np.count_nonzero(aligned[mask])),
                              scalar=row['alignment_sdk_corner_reconstruction']), ensure_ascii=False), flush=True)
    report['png_files'] = [dict(name=path.name, sha256=sha(path)) for path in sorted(args.output.glob('*.png'))]
    matched = [row for row in report['rows'] if row['unique_appearance_hypothesis']]
    nominal_footprints = [next(v for v in row['native_footprint_depth_sensitivity']
                              if v['assumed_color_z_m'] == row['nominal_footprint_assumed_color_z_m'])
                         for row in matched]
    report['summary'] = dict(
        appearance_hypotheses=len(matched), identity_verified=False, same_source_actual_pose_available=False,
        raw_positive_fraction_at_nominal_color_depth=quantiles(np.array([v['raw_positive_fraction'] for v in nominal_footprints])),
        actual_aligned_positive_fraction=quantiles(np.array([row['color_roi_stages']['aligned_depth']['valid']/row['hypothesis']['pixels'] for row in matched])),
        rgb_unchanged_all=all(row['raw_rgb_identical_to_final'] for row in report['rows']),
        temporal_inactive_all=all(not row['stationary']['active'] for row in report['rows']),
        post_spatial_to_final_identical_all=all(row['global_deltas'][a+'__'+b]['byte_identical']
            for row in report['rows'] for a,b in zip(STAGES[2:-1],STAGES[3:])),
        post_alignment_lost_positive_to_zero=sum(delta['lost_positive_to_zero'] for row in report['rows'] for delta in row['global_deltas'].values()),
        post_alignment_filled_zero_to_positive=sum(delta['filled_zero_to_positive'] for row in report['rows'] for delta in row['global_deltas'].values()),
        actual_alignment_zero_with_scalar_sdk_raw_corner_coverage=sum(row['alignment_sdk_corner_reconstruction']['actual_zero_despite_raw_corner_coverage'] for row in matched),
        interpretation='Large missing-depth regions already exist in raw SDK matcher output. This capture does not support blaming spatial/temporal/clip for creating holes. Spatial filtering does change measured values near a height threshold. Optimized alignment differs slightly from scalar SDK reconstruction, and raw-vs-aligned grids are never treated as identical.',
        software_repair_limit='This evidence does not establish that the holes are irreparable by sensor/matcher settings, illumination or view. Those causes were not independently varied or measured. Existing holes cannot be called measured geometry by filling or relaxing the 4 mm/60 point rules.')
    destination = args.output/'camera_stage_analysis.json'
    destination.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(report=str(destination), sha256=sha(destination)), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
