#!/usr/bin/env python3
"""Offline exact-state LK replay and spatial-filter diagnostics from sealed tap.

No ROS/device calls or changes to production parameters. Gaussian and synthetic
translation comparisons use the actual fixed anchor and original seed points;
they do not confer stationary permission, support quality or motion authority.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import zipfile

import cv2
import numpy as np

W = Path(__file__).resolve().parents[1]
VISUAL_PATH = W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/visual_stationary.py'
VISUAL_MODULE = 'alicia_flexible_grasp.vision.visual_stationary'
VISUAL_LOADED_SHA = hashlib.sha256(VISUAL_PATH.read_bytes()).hexdigest()
_spec = importlib.util.spec_from_file_location('offline_exact_visual_window', VISUAL_PATH)
_visual = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_visual)
if hashlib.sha256(VISUAL_PATH.read_bytes()).hexdigest() != VISUAL_LOADED_SHA:
    raise ValueError('visual_source_changed_during_import')
V = _visual.VisualStationaryWindow
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
DOMAIN = 'raw_sdk_color_before_pair_rectification'
MAX_CAPTURE_BYTES, MAX_SNAPSHOT_BYTES = 64*1024*1024, 8*1024*1024
SIGMAS = (None, .6, .8, 1.)


def require(value, reason):
    if not value:
        raise ValueError(reason)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def gray_for(full, bounds):
    x0, y0, x1, y1 = bounds
    crop = np.ascontiguousarray(full[y0:y1, x0:x1])
    h, w = crop.shape[:2]
    ratio = min(1., V.MAX_SIDE/float(max(h, w)))
    small = cv2.resize(crop, (max(1, round(w*ratio)), max(1, round(h*ratio))),
                       interpolation=cv2.INTER_AREA) if ratio < 1 else crop
    return crop, cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), np.float32([w/small.shape[1], h/small.shape[0]])


def source_tuple(source, host):
    require(isinstance(source, dict), 'source_missing')
    number, stamp, fps = (source.get(key) for key in ('frame_number', 'device_timestamp_ms', 'stream_fps'))
    domain = str(source.get('timestamp_domain', '')).rsplit('.', 1)[-1]
    require(type(number) is int and number >= 0 and domain in ('hardware_clock', 'global_time'), 'source_identity_invalid')
    require(all(type(v) in (int, float) and math.isfinite(v) for v in (stamp, fps, host)), 'source_clock_invalid')
    require(stamp >= 0 and 1 <= fps <= 240 and host >= 0, 'source_clock_out_of_range')
    return number, float(stamp), domain, float(fps), float(host)


def validate_snapshot(arrays, meta):
    require(meta.get('schema') == 1 and meta.get('diagnostic_only') is True
            and meta.get('motion_authorized') is False and meta.get('pixels_changed') is False
            and meta.get('coordinate_domain') == DOMAIN
            and meta.get('publication_outcome_unknown') is True
            and meta.get('enclosing_read_outcome_unknown') is True, 'snapshot_scope_invalid')
    implementation = meta.get('source_implementation', {}).get(VISUAL_MODULE, {})
    require(implementation.get('path') == str(VISUAL_PATH.resolve())
            and implementation.get('sha256') == VISUAL_LOADED_SHA
            and sha(VISUAL_PATH.read_bytes()) == VISUAL_LOADED_SHA, 'visual_implementation_not_exact')
    raw, anchor_raw, anchor, points = (arrays[k] for k in ('raw_color', 'anchor_raw_color', 'anchor_gray', 'seed_points'))
    require(raw.dtype == anchor_raw.dtype == np.uint8 and raw.shape == anchor_raw.shape
            and raw.ndim == 3 and raw.shape[2] == 3 and 48 <= min(raw.shape[:2])
            and max(raw.shape[:2]) <= 8192, 'raw_rgb_geometry_invalid')
    require(anchor.dtype == np.uint8 and anchor.ndim == 2 and points.dtype == np.float32
            and points.ndim == 3 and points.shape[1:] == (1, 2)
            and V.MIN_POINTS <= len(points) <= V.MAX_POINTS and np.isfinite(points).all(), 'actual_anchor_or_points_invalid')
    bounds = meta['bounds_xyxy']
    require(isinstance(bounds, list) and len(bounds) == 4 and all(type(x) is int for x in bounds), 'crop_bounds_invalid')
    x0, y0, x1, y1 = bounds
    require(0 <= x0 < x1 <= raw.shape[1] and 0 <= y0 < y1 <= raw.shape[0], 'crop_outside_raw_image')
    h, w = raw.shape[:2]
    expected = [int(.2*w), int(.1*h), int(.8*w), int(.9*h)]
    if meta.get('region') == 'nested_center':
        l, t, r, b = expected; width, height = (r-l)//2, (b-t)//2
        l, t = l+(r-l-width)//2, t+(b-t-height)//2
        expected = [l, t, l+width, t+height]
    else:
        require(meta.get('region') == 'center', 'unsupported_visual_region')
    require(bounds == expected, 'region_bounds_not_actual_contract')
    before = meta['pre_observe']
    require(before.get('anchor_bound_verified') is True, 'unbound_actual_anchor')
    crop, current, scale = gray_for(raw, bounds)
    _, rebuilt_anchor, _ = gray_for(anchor_raw, bounds)
    require(np.array_equal(rebuilt_anchor, anchor), 'actual_anchor_rgb_does_not_reproduce_gray')
    require(list(raw.shape) == meta['raw_color_shape']
            and list(anchor_raw.shape) == before['anchor_raw_color_shape']
            and list(crop.shape) == meta['observed_crop_shape']
            and list(anchor.shape) == before['anchor_gray_shape']
            and len(points) == before['seed_points_count'], 'snapshot_shape_binding_changed')
    require(np.array_equal(np.asarray(meta['scale_full_resolution_xy'], np.float32), scale), 'snapshot_scale_changed')
    require(before['anchor_filter_host_monotonic'] == before['anchor_host_monotonic']
            and before['anchor_source']['device_timestamp_ms'] == before['anchor_device_timestamp_ms'], 'anchor_source_clock_changed')
    anchor_source = source_tuple(before['anchor_source'], before['anchor_filter_host_monotonic'])
    current_source = source_tuple(meta['source'], meta['filter_host_monotonic'])
    previous = before['previous_source_tuple']
    require(previous == before['previous'] and isinstance(previous, list) and len(previous) == 6
            and type(previous[0]) is int and len(previous[5]) == 3, 'pre_observe_previous_binding_invalid')
    previous_source = source_tuple(dict(frame_number=previous[0], device_timestamp_ms=previous[1],
        timestamp_domain=previous[2], stream_fps=previous[3]), previous[4])
    require(list(crop.shape) == previous[5], 'pre_observe_shape_changed')
    require(anchor_source[2:4] == current_source[2:4] == previous_source[2:4]
            and anchor_source[0] <= previous[0] and current_source[0] == previous[0]+1
            and anchor_source[1] <= previous[1] < current_source[1]
            and anchor_source[4] <= previous[4] < current_source[4], 'source_lineage_invalid')
    fps = current_source[3]
    require(.25/fps <= (current_source[1]-previous[1])/1000. <= 1.75/fps
            and 0 < current_source[4]-previous[4] <= max(.1, 2.5/fps), 'source_discontinuity_not_lk_comparison')
    count = before['continuous_frame_count']
    require(type(count) is int and count >= 1 and count == before['continuous_frames'], 'pre_observe_count_invalid')
    require(previous_source[0]-anchor_source[0]+1 == count, 'anchor_frame_count_lineage_invalid')
    require(meta.get('category') in ('stationary', 'visual_motion')
            and meta.get('original_return', {}).get('reason') == meta['category'], 'unsupported_original_return')
    return crop, current, scale


def lk(anchor, current, points, scale):
    kw = dict(winSize=(15, 15), maxLevel=2,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 15, .01))
    forward, status, error = cv2.calcOpticalFlowPyrLK(anchor, current, points, None, **kw)
    require(forward is not None and status is not None and np.isfinite(forward).all(), 'forward_tracking_failed')
    backward, reverse, reverse_error = cv2.calcOpticalFlowPyrLK(current, anchor, forward, None, **kw)
    require(backward is not None and reverse is not None and np.isfinite(backward).all(), 'backward_tracking_failed')
    fb = np.linalg.norm((backward-points).reshape(-1, 2)*scale, axis=1)
    xy = forward.reshape(-1, 2)
    valid = ((status.ravel() == 1) & (reverse.ravel() == 1) & (fb <= V.MAX_FORWARD_BACKWARD_PX)
             & (xy[:, 0] >= 0) & (xy[:, 0] < current.shape[1])
             & (xy[:, 1] >= 0) & (xy[:, 1] < current.shape[0]))
    vectors = (forward-points).reshape(-1, 2)*scale
    motion = np.linalg.norm(vectors, axis=1)
    coverage = V._coverage(forward[valid], current.shape)
    eligible = int(valid.sum()) >= max(V.MIN_POINTS, math.ceil(.8*len(points))) and coverage >= 3
    metrics = dict(tracked_points=int(valid.sum()), covered_quadrants=coverage,
        tracks_eligible=bool(eligible), motion_p90_px=float(np.percentile(motion[valid], 90)) if eligible else None,
        motion_median_px=float(np.median(motion[valid])) if eligible else None)
    metrics['p90_motion_gate_passed'] = bool(eligible and metrics['motion_p90_px'] <= V.MAX_MOTION_PX)
    return dict(metrics=metrics, forward=forward, backward=backward, status=status.ravel(),
        reverse=reverse.ravel(), error=error.ravel(), reverse_error=reverse_error.ravel(),
        fb=fb, valid=valid, vectors=vectors, motion=motion)


def finite_or_none(value):
    return float(value) if math.isfinite(float(value)) else None


def direction_statistics(vectors):
    lengths = np.linalg.norm(vectors, axis=1)
    meaningful = lengths > 1e-9
    vectors, lengths = vectors[meaningful], lengths[meaningful]
    if not len(vectors):
        return dict(count=0, unit_vector_resultant=None, mean_direction_xy=None,
                    fraction_within_30deg_of_mean=None)
    units = vectors/lengths[:, None]
    mean = np.mean(units, axis=0); resultant = float(np.linalg.norm(mean))
    direction = mean/resultant if resultant > 1e-9 else None
    return dict(count=len(vectors), unit_vector_resultant=resultant,
        mean_direction_xy=None if direction is None else direction.tolist(),
        fraction_within_30deg_of_mean=None if direction is None else
            float(np.mean(np.dot(units, direction) >= math.cos(math.pi/6))))


def point_report(anchor, current, points, tracked, scale, bounds):
    rows=[]; x0, y0, _, _ = bounds
    h, w = anchor.shape
    # OpenCV resize maps pixel centers as (u+.5)*scale-.5, distinct from
    # simple displacement scaling. Offsets restore the actual full RGB frame.
    def raw_uv(uv): return ((np.asarray(uv)+.5)*scale-.5+np.array([x0,y0])).tolist()
    for i, point in enumerate(points.reshape(-1, 2)):
        xy = tracked['forward'].reshape(-1, 2)[i]
        a = cv2.getRectSubPix(anchor, (15, 15), tuple(map(float, point))).astype(float)
        b = cv2.getRectSubPix(current, (15, 15), tuple(map(float, xy))).astype(float)
        residual = b-a
        border_xy = [min(point[0], w-1-point[0])*scale[0], min(point[1], h-1-point[1])*scale[1]]
        rows.append(dict(index=i, valid=bool(tracked['valid'][i]),
            forward_status=int(tracked['status'][i]), backward_status=int(tracked['reverse'][i]),
            seed_gray_uv=point.tolist(), current_gray_uv=xy.tolist(),
            seed_raw_uv=raw_uv(point), current_raw_uv=raw_uv(xy),
            seed_quadrant=int(point[0]>=w/2)+2*int(point[1]>=h/2),
            current_quadrant=int(xy[0]>=w/2)+2*int(xy[1]>=h/2),
            motion_full_input_xy=tracked['vectors'][i].tolist(),
            motion_full_input_px=float(tracked['motion'][i]),
            above_original_threshold=bool(tracked['valid'][i] and tracked['motion'][i]>V.MAX_MOTION_PX),
            fb_full_input_px=float(tracked['fb'][i]),
            lk_l1_error=finite_or_none(tracked['error'][i]) if tracked['status'][i] else None,
            reverse_lk_l1_error=finite_or_none(tracked['reverse_error'][i]) if tracked['reverse'][i] else None,
            distance_to_crop_boundary_full_px=float(min(border_xy)),
            patch_touches_boundary=bool(min(point[0],point[1],w-1-point[0],h-1-point[1])<7
                or min(xy[0],xy[1],w-1-xy[0],h-1-xy[1])<7),
            patch_mean_delta=float(residual.mean()), patch_abs_residual=float(np.abs(residual).mean()),
            patch_zero_mean_rms=float(np.sqrt(np.mean((residual-residual.mean())**2))),
            anchor_patch_std=float(a.std())))
    return rows


def gaussian_comparisons(arrays, meta, current, scale):
    anchor, points = arrays['anchor_gray'], arrays['seed_points']
    def compare(a, b):
        rows=[]
        for sigma in SIGMAS:
            aa=a if sigma is None else cv2.GaussianBlur(a, (3,3), sigmaX=sigma, sigmaY=sigma, borderType=cv2.BORDER_REFLECT_101)
            bb=b if sigma is None else cv2.GaussianBlur(b, (3,3), sigmaX=sigma, sigmaY=sigma, borderType=cv2.BORDER_REFLECT_101)
            result=lk(aa, bb, points, scale)
            rows.append(dict(sigma=sigma, **result['metrics']))
        return rows
    actual=compare(anchor, current)
    synthetic=[]
    # Full raw reference warp precedes the ORIGINAL crop/resize/gray path.
    # These are synthetic controls, not physical observations or source frames.
    for magnitude in (.3,.5,.8):
        for direction in ((1.,0.),(0.,1.),(2**-.5,2**-.5)):
            dx,dy=(magnitude*v for v in direction)
            warp=np.float32([[1.,0.,dx],[0.,1.,dy]])
            rgb=arrays['anchor_raw_color']
            moved=cv2.warpAffine(rgb,warp,(rgb.shape[1],rgb.shape[0]),
                flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT_101)
            _, gray, _=gray_for(moved,meta['bounds_xyxy'])
            comparisons=compare(anchor,gray)
            synthetic.append(dict(requested_translation_full_px=[dx,dy],magnitude_full_px=magnitude,
                interpolation='OpenCV INTER_LINEAR; fractional interpolation table may quantize warp',
                comparisons=comparisons,
                any_variant_motion_gate_passed=any(row['p90_motion_gate_passed'] for row in comparisons)))
    return dict(kind='spatial_noise_diagnostic_only', kernel=[3,3], anchor_reseeded=False,
        original_seed_points_preserved=True, original_motion_p90_threshold_px=V.MAX_MOTION_PX,
        actual_pair=actual, synthetic_full_image_translations=synthetic,
        all_synthetic_motion_detected_by_original_p90=all(
            row['tracks_eligible'] and row['motion_p90_px'] > V.MAX_MOTION_PX
            for case in synthetic for row in case['comparisons']),
        limitation='Synthetic resampling controls cannot establish real stationarity; filter comparisons are not production approval.')


def exact_replay(arrays, meta, *, include_gaussian=True):
    crop,current,scale=validate_snapshot(arrays,meta)
    anchor,points=arrays['anchor_gray'],arrays['seed_points'];before=meta['pre_observe']
    node=V();node._anchor=anchor.copy();node._points=points.copy()
    previous=before['previous_source_tuple'];node._previous=tuple(previous[:5])+(tuple(previous[5]),)
    node._anchor_stamp=before['anchor_device_timestamp_ms'];node._anchor_host=before['anchor_host_monotonic']
    node._count=before['continuous_frame_count']
    report=node.observe(crop,meta['source'],meta['filter_host_monotonic'])
    tracked=lk(anchor,current,points,scale)
    rows=point_report(anchor,current,points,tracked,scale,meta['bounds_xyxy'])
    above=tracked['valid'] & (tracked['motion']>V.MAX_MOTION_PX)
    result=dict(exact_return_equal=report==meta['original_return'],replayed_return=report,
        original_return=meta['original_return'],anchor_raw_reproduces_gray=True,
        source=meta['source'],anchor_source=before['anchor_source'],region=meta['region'],category=meta['category'],
        scale=scale.tolist(),recomputed_lk_metrics=tracked['metrics'],
        valid_motion_direction=direction_statistics(tracked['vectors'][tracked['valid']]),
        above_threshold_motion_direction=direction_statistics(tracked['vectors'][above]),points=rows,
        scope='Actual saved pre-state/source/host replay; no inference about enclosing RGB-D read or publication success.')
    if include_gaussian:result['gaussian_diagnostic']=gaussian_comparisons(arrays,meta,current,scale)
    return result


def sealed_entries(capture):
    capture=Path(capture).resolve();summary_path=capture/'summary.json'
    require(not summary_path.is_symlink(),'summary_symlink_not_allowed')
    require(summary_path.stat().st_size<=64*1024,'summary_byte_bound_exceeded')
    summary=json.loads(summary_path.read_text())
    require(summary.get('schema')==1 and summary.get('sealed') is True and not summary.get('errors')
        and summary.get('diagnostic_only') is True and summary.get('pixels_changed') is False
        and summary.get('motion_authorized') is False and summary.get('coordinate_domain')==DOMAIN,
        'capture_not_valid_sealed_diagnostic')
    require(all(summary.get(k) is True for k in
        ('original_process_restored','original_observe_restored','original_reset_restored'))
        and all(summary.get(k)==0 for k in ('queued_unwritten','inflight_copies','queue_bytes')),
        'capture_not_restored_or_drained')
    entries=summary.get('files')
    require(isinstance(entries,list) and 0<len(entries)<=32,'capture_file_count_invalid')
    names=set();total=0
    for entry in entries:
        name=entry['name'];unresolved=capture/name;path=unresolved.resolve()
        require(isinstance(name,str) and name==Path(name).name and name.endswith('.npz')
            and path.parent==capture and not unresolved.is_symlink()
            and name not in names,'snapshot_path_or_duplicate_invalid')
        names.add(name)
        require(type(entry['bytes']) is int and 0<entry['bytes']<=MAX_SNAPSHOT_BYTES
            and path.stat().st_size==entry['bytes'],'snapshot_byte_binding_invalid')
        total+=entry['bytes'];require(total<=MAX_CAPTURE_BYTES,'capture_byte_bound_exceeded')
    return summary,entries


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(argv);capture=a.capture.resolve();output=a.output.resolve()
    require(Path('/media/zhuyupei/SanStand').is_mount(),'SanStand_unavailable_no_fallback')
    require(EXT in capture.parents and EXT in output.parents and not output.exists(),'external_new_paths_required')
    summary,entries=sealed_entries(capture)
    source_before=sha(VISUAL_PATH.read_bytes());summary_before=sha((capture/'summary.json').read_bytes())
    previous_threads=cv2.getNumThreads();cv2.setNumThreads(1)
    report=dict(schema=1,opencv_version=cv2.__version__,opencv_build_sha256=sha(cv2.getBuildInformation().encode()),
        capture_opencv_build_not_recorded=True,diagnostic_only=True,motion_authorized=False,
        capture=str(capture),summary_sha256=summary_before,
        source_sha256={str(p.relative_to(W)):sha(p.read_bytes()) for p in (Path(__file__).resolve(),VISUAL_PATH)},rows=[])
    try:
        for entry in entries:
            path=capture/entry['name'];require(sha(path.read_bytes())==entry['sha256'],'snapshot_hash_changed')
            with zipfile.ZipFile(path) as archive:
                require(sum(info.file_size for info in archive.infolist())<=MAX_SNAPSHOT_BYTES,'snapshot_expansion_bound_exceeded')
            with np.load(path,allow_pickle=False) as arrays:
                require(arrays['metadata_json'].dtype==np.uint8 and arrays['metadata_json'].nbytes<=16*1024,'metadata_invalid')
                metadata=json.loads(arrays['metadata_json'].tobytes())
                require(metadata['source']==entry['source'] and metadata['category']==entry['category'].split('/')[-1]
                    and entry['category']==metadata['region']+'/'+metadata['category']
                    and metadata['source_implementation']==summary['source_implementation'],'sealed_source_binding_changed')
                row=exact_replay(arrays,metadata)
            require(sha(path.read_bytes())==entry['sha256'],'snapshot_changed_during_replay')
            row.update(file=entry['name'],file_sha256=entry['sha256']);report['rows'].append(row)
    finally:
        cv2.setNumThreads(previous_threads)
    require(source_before==sha(VISUAL_PATH.read_bytes()) and summary_before==sha((capture/'summary.json').read_bytes()),'source_or_summary_changed_during_replay')
    report['all_original_returns_exactly_reproduced']=all(row['exact_return_equal'] for row in report['rows'])
    report['limitations']=['Exact replay requires recorded original report equality; runtime OpenCV build was not recorded by the capture.',
        'Gaussian and synthetic controls are diagnostic only; no new image or source identity is issued.',
        'No conclusion about physical vibration, support-plane quality or successful grasp follows from these comparisons alone.']
    output.mkdir(exist_ok=False)
    with (output/'report.json').open('x') as f:json.dump(report,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(dict(report=str(output/'report.json'),rows=len(report['rows']),all_exact=report['all_original_returns_exactly_reproduced'])))
    return 0 if report['all_original_returns_exactly_reproduced'] else 2


if __name__=='__main__':
    raise SystemExit(main())
