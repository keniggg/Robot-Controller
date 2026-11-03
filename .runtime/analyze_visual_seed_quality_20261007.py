#!/usr/bin/env python3
"""Offline fixed seed-quality sensitivity on actual captured anchor/current pairs.

Four-quadrant quotas, LK, .25 px threshold and track eligibility stay original.
Quality candidates are fixed before inspecting motion. No ROS, device access,
production edits, point removal based on motion, or reconstructed time window.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import cv2
import numpy as np

W=Path(__file__).resolve().parents[1]
REPLAY_PATH=W/'.runtime/replay_raw_visual_snapshots_20261006.py'
REPLAY_SHA='45cdaeb5efe533db742feca16db6373caa890d7224b18b77fb3bce6f7a2d0e2e'
if hashlib.sha256(REPLAY_PATH.read_bytes()).hexdigest()!=REPLAY_SHA:
    raise ValueError('frozen_replayer_changed')
spec=importlib.util.spec_from_file_location('frozen_actual_visual_replayer',REPLAY_PATH)
r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
QUALITIES=(.04,.08,.12)
DIRECTIONS=((1.,0.),(0.,1.),(2**-.5,2**-.5))
MAGNITUDES=(.3,.5,.8)


def seed(gray,quality):
    r.require(quality in (.02,)+QUALITIES,'unregistered_quality_candidate')
    points=[];h,w=gray.shape
    for row in range(2):
        for col in range(2):
            y0,y1=row*h//2,(row+1)*h//2;x0,x1=col*w//2,(col+1)*w//2
            found=cv2.goodFeaturesToTrack(gray[y0:y1,x0:x1],16,quality,8,blockSize=5)
            if found is not None:
                found+=np.float32([x0,y0]);points.append(found)
    points=np.concatenate(points) if points else np.empty((0,1,2),np.float32)
    coverage=r.V._coverage(points,gray.shape)
    return points,dict(seed_points_count=len(points),covered_quadrants=coverage,
        seed_eligible=bool(len(points)>=r.V.MIN_POINTS and coverage>=3),
        reason='seed_eligible' if len(points)>=r.V.MIN_POINTS and coverage>=3 else 'insufficient_texture')


def candidate(arrays,meta,gray,scale,quality):
    points,summary=seed(arrays['anchor_gray'],quality)
    out=dict(quality_level=quality,**summary,actual_pair=None,synthetic_controls=[])
    original=arrays['seed_points']
    out['same_points_as_original']=bool(np.array_equal(points,original))
    old={tuple(v) for v in original.reshape(-1,2)}
    out['points_not_in_original_seed']=sum(tuple(v) not in old for v in points.reshape(-1,2))
    if not summary['seed_eligible']:
        return out
    out['actual_pair']=r.lk(arrays['anchor_gray'],gray,points,scale)['metrics']
    out['actual_pair_reason']=('insufficient_tracks' if not out['actual_pair']['tracks_eligible']
        else 'pair_motion_gate_pass' if out['actual_pair']['p90_motion_gate_passed'] else 'visual_motion')
    raw=arrays['anchor_raw_color']
    for magnitude in MAGNITUDES:
        for direction in DIRECTIONS:
            dx,dy=(magnitude*x for x in direction)
            shifted=cv2.warpAffine(raw,np.float32([[1.,0.,dx],[0.,1.,dy]]),(raw.shape[1],raw.shape[0]),
                flags=cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT_101)
            _,current,_=r.gray_for(shifted,meta['bounds_xyxy'])
            metrics=r.lk(arrays['anchor_gray'],current,points,scale)['metrics']
            out['synthetic_controls'].append(dict(requested_translation_full_px=[dx,dy],
                magnitude_full_px=magnitude,**metrics))
    out['all_synthetic_motion_detected']=all(x['tracks_eligible'] and x['motion_p90_px']>.25
        for x in out['synthetic_controls'])
    return out


def analyze(arrays,meta):
    _,gray,scale=r.validate_snapshot(arrays,meta)
    original_points,original_seed=seed(arrays['anchor_gray'],.02)
    r.require(np.array_equal(original_points,arrays['seed_points']),'original_seed_not_exactly_reconstructed')
    baseline=r.lk(arrays['anchor_gray'],gray,arrays['seed_points'],scale)['metrics']
    return dict(region=meta['region'],category=meta['category'],source=meta['source'],
        original_seed_exact=True,original_seed=original_seed,original_pair=baseline,
        candidates=[candidate(arrays,meta,gray,scale,q) for q in QUALITIES])


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture',required=True,type=Path);parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args(argv);capture=args.capture.resolve();output=args.output.resolve()
    r.require(Path('/media/zhuyupei/SanStand').is_mount(),'SanStand_unavailable_no_fallback')
    r.require(r.EXT in capture.parents and r.EXT in output.parents and not output.exists(),'external_new_paths_required')
    summary,entries=r.sealed_entries(capture)
    report=dict(schema=1,diagnostic_only=True,motion_authorized=False,production_modified=False,
        capture=str(capture),capture_summary_sha256=r.sha((capture/'summary.json').read_bytes()),
        source_sha256={str(p.relative_to(W)):r.sha(p.read_bytes()) for p in (Path(__file__).resolve(),REPLAY_PATH,r.VISUAL_PATH)},
        fixed_quality_levels=list(QUALITIES),opencv_version=cv2.__version__,rows=[],
        limitations=['Preselected fixed quality thresholds; no points are removed based on observed movement.',
            'Original anchor/current pair sensitivity only; the intermediate frames and a fresh temporal window are not reconstructed.',
            'Four categories are sampled with per-category caps and are not an unbiased runtime failure-rate estimate.',
            'Passing this pair-level counterfactual is not proof of physical stationarity or support quality.',
            'Synthetic translations use OpenCV INTER_LINEAR resampling of the actual anchor full RGB.'])
    old_threads=cv2.getNumThreads();cv2.setNumThreads(1)
    try:
        for entry in entries:
            path=capture/entry['name'];r.require(r.sha(path.read_bytes())==entry['sha256'],'snapshot_hash_changed')
            with np.load(path,allow_pickle=False) as arrays:
                meta=json.loads(arrays['metadata_json'].tobytes())
                r.require(meta['source']==entry['source'] and meta['source_implementation']==summary['source_implementation'],
                    'sealed_source_binding_changed')
                row=analyze(arrays,meta)
            r.require(r.sha(path.read_bytes())==entry['sha256'],'snapshot_changed_during_analysis')
            row.update(file=entry['name'],file_sha256=entry['sha256']);report['rows'].append(row)
    finally:
        cv2.setNumThreads(old_threads)
    r.require(all(r.sha((W/name).read_bytes())==digest for name,digest in report['source_sha256'].items()),'analysis_source_changed')
    r.require(r.sha((capture/'summary.json').read_bytes())==report['capture_summary_sha256'],'capture_summary_changed')
    output.mkdir(exist_ok=False)
    with (output/'report.json').open('x') as f:json.dump(report,f,ensure_ascii=False,allow_nan=False,indent=2);f.write('\n')
    print(json.dumps(dict(report=str(output/'report.json'),rows=len(report['rows']),sha256=r.sha((output/'report.json').read_bytes()))))


if __name__=='__main__':main()
