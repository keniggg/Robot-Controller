#!/usr/bin/env python3
"""Offline provenance/statistics only; no ROS imports or device access."""
import hashlib
import json
import re
from pathlib import Path

import numpy as np


WORK = Path(__file__).resolve().parent.parent
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
LOG = EXT / 'ros_log'
PAIR = LOG / 'unknown_first_loss_pair_20261004T114247Z_168778/report.json'
BOOT = LOG / 'rgb_observation_bootstrap_20261004T114905Z/result.json'
ROI = WORK / '.runtime/gen13_native_depth_diagnostic_20261004.json'
CAMERA_LOG = LOG / ('ros_live_20260922_session/'
                    'abc820c4-bf8a-11f1-b6e0-91e7974b0c29/supervisor_camera-8.log')
GATEWAYS = [LOG / ('rgb_acquisition_gateway_' + suffix) for suffix in (
    '20261004T114259.948391Z_1541ec9e',
    '20261004T114306.354414Z_97bf470d',
    '20261004T114315.448775Z_90ae50c0')]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def array_stats(path):
    with np.load(path, allow_pickle=False) as z:
        d = z['depth_raw']
        scale = float(z['depth_scale'])
        assert d.dtype == np.uint16 and d.shape == (480, 640)
        valid = d != 0
        result = dict(source_file=str(path), source_sha256=sha(path),
                      source_stamp_ns=int(z['source_stamp_ns']),
                      depth_dtype=str(d.dtype), shape=list(d.shape),
                      depth_scale_m=scale, valid_pixels=int(valid.sum()),
                      valid_fraction=float(valid.mean()),
                      positive_depth_mm_percentiles=np.percentile(
                          d[valid].astype(float) * scale * 1000., [0, 10, 50, 90, 100]).tolist())
        if 'region_mask' in z:
            mask = z['region_mask'].astype(bool)
            gray = z['color_bgr'].astype(float).mean(axis=2)
            result['saved_rgb_region'] = dict(
                pixels=int(mask.sum()), valid_depth_pixels=int((valid & mask).sum()),
                valid_fraction=float(valid[mask].mean()),
                rgb_channel_mean_percentiles=np.percentile(gray[mask], [0, 10, 50, 90, 100]).tolist(),
                note='RGB brightness alone does not measure stereo exposure or texture quality')
        return result


def camera_log_summary():
    # Bounded tail only, then exact local-PDT minute windows corresponding to
    # the gen13 motions and four later stationary observations.
    length = 2500000
    size = CAMERA_LOG.stat().st_size
    offset = max(0, size - length)
    with CAMERA_LOG.open('rb') as stream:
        stream.seek(offset)
        data = stream.read(length).decode('utf-8', 'replace')
    rows = []
    for line in data.splitlines():
        match = re.search(r'2026-10-04 (04:(?:42|43|44|49):[0-9:.,]+)', line)
        if not match or 'Camera ' not in line:
            continue
        for label in ('Camera RGB-D pair unavailable: ', 'Camera backend read stages: '):
            if label not in line:
                continue
            value = json.loads(line.split(label, 1)[1])
            frames = value.get('raw_frames', {})
            rows.append(dict(local_pdt_time=match.group(1), kind=label[:-2],
                             result=value.get('result'),
                             pair_timing=value.get('pair_timing'),
                             stationary_rgb_evidence=value.get('stationary_rgb_evidence'),
                             stages_sec=value.get('stages_sec'),
                             sdk_source={k:{a:b for a,b in v.items() if a in
                                 ('frame_number','device_timestamp_ms','metadata_raw')}
                                         for k,v in frames.items()}))
    return dict(path=str(CAMERA_LOG), tail_offset=offset, tail_bytes=min(size, length),
                complete_log_scanned=False, warnings_throttled=True, rows=rows)


def main():
    pair = json.loads(PAIR.read_text())
    roi = json.loads(ROI.read_text())
    frames = []
    for directory in GATEWAYS:
        for phase in ('initial','postplan','proof','submission'):
            row = array_stats(directory / (phase + '_source.npz'))
            row.update(gateway=directory.name, phase=phase)
            frames.append(row)
    bootstrap = json.loads(BOOT.read_text())
    later = []
    for row in bootstrap['rows']:
        path = Path(row['source_file'])
        measured = array_stats(path)
        analysis = next(x for x in roi['rows'] if x['source_stamp_ns'] == measured['source_stamp_ns'])
        outline = analysis['scopes']['rgb_outline']
        measured['offline_associated_rgb_outline'] = {
            k:outline[k] for k in ('pixels','valid_native_depth','native_zero_depth',
                                   'height_to_current_plane_mm','raw_ge_4mm_inside_roi')}
        measured['q_source'] = row['q_source']
        later.append(measured)
    report = dict(
        scope='offline camera-chain provenance and saved published-depth diagnostics',
        ros_io=False, device_access=False, robot_commands_sent=False,
        source_reports=[dict(path=str(p), sha256=sha(p)) for p in (PAIR, BOOT, ROI)],
        code_sha256={str(p.relative_to(WORK)):sha(p) for p in [
            WORK/'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py',
            WORK/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/realsense_manager.py',
            WORK/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/depth_preset.py']},
        native_semantics='native uint16 bytes of published 16UC1 image, not pre-align sensor depth',
        processing_order=['sensor matcher / high_accuracy preset', 'SDK depth-to-color alignment',
                          'spatial filter: iterations=2 alpha=.5 delta=20 raw units',
                          'conditional stationary temporal: alpha=.4 delta=100 persistence=0',
                          'clip depth outside .03..2 m to zero', '16UC1 ROS publication',
                          'passthrough copy / NPZ serialization'],
        observed_camera_profile=pair['camera_parameters']['runtime_profile'],
        final_stationary_temporal=pair['current_camera_parameters']['runtime_profile']['stationary_temporal'],
        exposure_evidence=dict(
            baseline_autoexposure=True, baseline_manual_exposure_setting=33000,
            baseline_depth_gain=16,
            scope='saved restore baseline, not measured per-frame/current effective exposure',
            actual_exposure_gain_per_frame_recorded=False),
        gateway_frames=frames, subsequent_stationary_frames=later,
        bounded_camera_log=camera_log_summary(),
        confirmed=[
            'Holes already exist in published depth before tabletop geometry segmentation.',
            'The subsequent four frames have identical accepted q; available depth is not adequate to establish the original target geometry.',
            'Generic temporal and hole filling filters are disabled; stationary temporal is separate, has persistence zero, and cannot restore a current depth hole.',
            'The SDK spatial and temporal algorithms average valid depth; they do not normally zero in-range depths around this object (roughly .24 m).',
            'Only the explicit range clip in this host chain intentionally zeroes valid out-of-range samples.',
            'Rejected timestamp-skew pairs are discarded before alignment; rejection resets stationary history and never publishes a stale replacement pair.',
            'Alignment retains the target/color distortion model; this pipeline does not rectify RGB when color_projection_correction is disabled.'],
        hypotheses_not_proven=[
            'High Accuracy matcher confidence policy can reduce fill; its contribution here lacks a same-scene unfiltered/preset comparison.',
            'Weak or ambiguous stereo texture, glare, exposure, or alignment can contribute; no pre-align depth or left/right stereo images were archived.',
            'Stationary-filter resets can reduce smoothing availability but do not explain creation of zero pixels from valid .24m depth.',
            'Pure-pinhole downstream geometry ignores nonzero color distortion; independent support sensitivity analysis is needed before attributing millimetre residuals.'],
        missing_evidence=['same-source pre-align depth', 'same-source post-align/pre-filter depth',
                          'same-source post-spatial/pre-temporal depth', 'per-frame temporal active/rejection state',
                          'per-frame actual exposure/gain/auto-exposure state',
                          'left/right stereo images and sensor advanced-settings readback',
                          'same-pose source-identical A/B evidence for matcher changes'],
        vendor_primary_sources=[
            dict(url='https://dev.realsenseai.com/docs/d400-series-visual-presets/',
                 supports='High Accuracy trades fill factor for matcher confidence'),
            dict(url='https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/',
                 supports='D400 exposure, gain, texture and preset influence stereo depth; D415/D435 examples are not D405-specific measured evidence'),
            dict(url='https://github.com/realsenseai/librealsense/blob/v2.56.5/doc/post-processing-filters.md',
                 supports='Spatial holes-fill default zero; temporal persistence zero disables history filling; disparity-domain recommendation'),
            dict(url='https://raw.githubusercontent.com/realsenseai/librealsense/v2.56.5/src/proc/temporal-filter.h',
                 supports='Current positive depths are retained or averaged; zero current samples only replaced by configured persistence'),
            dict(url='https://raw.githubusercontent.com/realsenseai/librealsense/v2.56.5/src/proc/spatial-filter.h',
                 supports='Configured depth-domain smoothing averages nearby depths and does not erase valid .24m samples'),
            dict(url='https://raw.githubusercontent.com/realsenseai/librealsense/v2.56.5/src/proc/align.cpp',
                 supports='Alignment initializes output to zero, skips invalid source, splats mapped depths, retains target intrinsics including distortion')],
        limitations=['No hardware/reconfiguration/ROS operation was performed.',
                     'SDK/software-device filter replay was not performed; algorithm inference uses vendor v2.56.5 source.',
                     'No causal claim identifies the sensor matcher versus alignment without earlier pipeline images.'])
    out = WORK / '.runtime/gen13_camera_chain_diagnostic_20261004.json'
    report['script_sha256'] = sha(Path(__file__).resolve())
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(path=str(out), sha256=sha(out), gateway_frames=len(frames),
                         later_frames=len(later), log_rows=len(report['bounded_camera_log']['rows']))))


if __name__ == '__main__':
    main()
