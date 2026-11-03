"""Offline fixed-input support-plane fit comparison, no ROS or execution authority.

Input NPZ arrays must be raw row-major ring points, shape (N,3), with names
`ring_170`, `ring_140`, `ring_110` (or pass --key for another exact array).
The original same evenly spaced <=1800 samples and all acceptance thresholds
are kept. More trials are diagnostic only and do not modify production code.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from alicia_grasp_modes.tabletop import Config, _fit_plane


def evaluate(points, plane, threshold):
    signed = np.einsum('ij,j->i', points, plane[:3]) + plane[3]
    keep = abs(signed) <= threshold
    count = int(np.count_nonzero(keep))
    return keep, dict(count=count, total=len(points), ratio=count / len(points),
        residual_m=float(np.sqrt(np.mean(signed[keep] ** 2))) if count else None)


def refine(points, plane, threshold, iterations):
    current, metrics = evaluate(points, plane, threshold)
    best_plane, best_metrics = plane.copy(), metrics
    history = [dict(stage='ransac', plane=plane.tolist(), **metrics)]
    for step in range(iterations):
        if np.count_nonzero(current) < 3:
            break
        center = np.mean(points[current], axis=0)
        _, _, vh = np.linalg.svd(points[current] - center, full_matrices=False)
        plane = np.r_[vh[-1], -float(vh[-1] @ center)]
        next_keep, metrics = evaluate(points, plane, threshold)
        history.append(dict(stage='svd_%d' % (step + 1), plane=plane.tolist(), **metrics))
        if (metrics['count'] > best_metrics['count'] or
                (metrics['count'] == best_metrics['count'] and
                 metrics['residual_m'] < best_metrics['residual_m'])):
            best_plane, best_metrics = plane.copy(), metrics
        if np.array_equal(current, next_keep):
            break
        current = next_keep
    if best_plane[3] < 0:
        best_plane = -best_plane
    return best_plane, best_metrics, history


def fit(points, threshold, trials, seed, iterations=2):
    started = time.monotonic()
    rng = np.random.RandomState(seed)
    best_count, best = 0, None
    count_updates = []
    for trial in range(trials):
        indices = rng.randint(len(points), size=3)
        if len(set(indices.tolist())) < 3:
            continue
        a, b, c = points[indices]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        keep = abs(np.einsum('ij,j->i', points - a, normal)) <= threshold
        count = int(np.count_nonzero(keep))
        if count > best_count:
            best_count = count
            best = np.r_[normal, -float(normal @ a)]
            count_updates.append([trial, count])
    if best is None:
        return dict(seed=seed, trials=trials, error='no nondegenerate triplets')
    plane, metrics, history = refine(points, best, threshold, iterations)
    return dict(seed=seed, trials=trials, plane=plane.tolist(), **metrics,
        refinements=history, count_updates=count_updates,
        elapsed_sec=time.monotonic() - started)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--key', action='append')
    parser.add_argument('--config-json', type=Path)
    parser.add_argument('--trials', type=int, default=8192)
    args = parser.parse_args()
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount() and mount in args.output.resolve().parents
    config = Config(**json.loads(args.config_json.read_text())) if args.config_json else Config()
    raw = np.load(args.input, allow_pickle=False)
    keys = args.key or [key for key in raw.files if key.startswith('ring_')
        and raw[key].ndim == 2 and raw[key].shape[1] == 3]
    assert keys, raw.files
    rows = []
    for key in keys:
        full = np.asarray(raw[key], dtype=float)
        assert full.ndim == 2 and full.shape[1] == 3 and np.all(np.isfinite(full))
        sample = full[np.linspace(0, len(full)-1,
            min(len(full), config.support_max_samples), dtype=int)]
        row = dict(key=key, full_points=len(full), sampled_points=len(sample),
            original_gate=dict(support_threshold_m=config.support_threshold_m,
                support_min_ratio=config.support_min_ratio,
                support_min_points=config.support_min_points))
        try:
            plane, ratio, residual, count = _fit_plane(full, config)
            row['production'] = dict(ok=True, plane=plane.tolist(), ratio=ratio,
                residual_m=residual, sample_count=count)
        except ValueError as exc:
            row['production'] = dict(ok=False, reason=str(exc))
        row['fits'] = []
        for trials, seed, iterations in [(128, 17, 2), (args.trials, 17, 2),
                (args.trials, 103, 2), (args.trials, 307, 16)]:
            result = fit(sample, config.support_threshold_m, trials, seed, iterations)
            if 'plane' in result:
                _, result['full_raw_evaluation'] = evaluate(full,
                    np.asarray(result['plane']), config.support_threshold_m)
                result['original_fit_quality_passed'] = bool(
                    result['count'] >= config.support_min_points and
                    result['ratio'] >= config.support_min_ratio and
                    result['residual_m'] <= config.support_threshold_m)
            row['fits'].append(result)
        # Whole-set least squares is an independent starting model, with no
        # reference-plane filtering or synthesized inlier points.
        center = sample.mean(axis=0)
        _, _, vh = np.linalg.svd(sample - center, full_matrices=False)
        plane, metrics, history = refine(sample,
            np.r_[vh[-1], -float(vh[-1] @ center)], config.support_threshold_m, 16)
        row['independent_all_sample_svd'] = dict(plane=plane.tolist(), **metrics,
            refinements=history)
        rows.append(row)
        print(json.dumps(dict(key=key, production=row['production'],
            ratios=[dict(trials=x['trials'], seed=x['seed'], ratio=x.get('ratio'),
                full_ratio=x.get('full_raw_evaluation', {}).get('ratio'),
                elapsed_sec=x.get('elapsed_sec')) for x in row['fits']]), ensure_ascii=False), flush=True)
    report = dict(scope=__doc__, input_path=str(args.input),
        input_sha256=hashlib.sha256(args.input.read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        config=asdict(config), rows=rows, motion_authorized=False,
        old_plane_used_for_selection=False, production_changed=False,
        conclusion_scope='Same sampled points only; finite fits do not prove global optimum, and fit quality alone does not qualify spatial/reference gates.')
    args.output.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
