"""Offline pixel-displacement diagnostic; does not qualify robot motion."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

W = Path(__file__).resolve().parents[1]
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
helper = W / '.runtime/extract_gen49_arrival_visibility_20261018.py'
assert hashlib.sha256(helper.read_bytes()).hexdigest() == '2b724378134ad3986368ab5b10da3883a5a544e4b90b844eb2b09b36ae3f9cb2'
spec = importlib.util.spec_from_file_location('gen49_offline_boundary', helper)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
cv2, np = base.cv2, base.np


def measure(a, b, roi):
    mask = np.zeros(a.shape, dtype=np.uint8)
    x0, y0, x1, y1 = roi
    mask[y0:y1, x0:x1] = 255
    points = cv2.goodFeaturesToTrack(a, 500, 0.02, 8, mask=mask)
    if points is None:
        return {'valid': False, 'reason': 'no_features'}
    kwargs = dict(winSize=(21, 21), maxLevel=3,
                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    forward, status1, _ = cv2.calcOpticalFlowPyrLK(a, b, points, None, **kwargs)
    backward, status2, _ = cv2.calcOpticalFlowPyrLK(b, a, forward, None, **kwargs)
    p = points.reshape(-1, 2)
    q = forward.reshape(-1, 2)
    fb = np.linalg.norm(backward.reshape(-1, 2) - p, axis=1)
    keep = (status1.ravel() == 1) & (status2.ravel() == 1) & (fb <= 0.5)
    keep &= np.isfinite(q).all(axis=1)
    if not keep.any():
        return {'valid': False, 'reason': 'no_consistent_tracks', 'detected': len(points)}
    displacement = q[keep] - p[keep]
    norms = np.linalg.norm(displacement, axis=1)
    result = dict(valid=True, roi_xyxy=roi, detected=int(len(points)),
                  tracked_consistent=int(keep.sum()),
                  median_dx_dy_px=np.median(displacement, axis=0).tolist(),
                  displacement_norm_p50_p90_p99_px=np.percentile(norms, [50, 90, 99]).tolist(),
                  forward_backward_max_px=float(fb[keep].max()))
    if keep.sum() >= 3:
        affine, inliers = cv2.estimateAffinePartial2D(p[keep], q[keep], method=cv2.RANSAC,
                                                     ransacReprojThreshold=0.75)
        result['affine_partial_2x3'] = affine.tolist() if affine is not None else None
        result['affine_inlier_count'] = int(inliers.sum()) if inliers is not None else 0
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--images', required=True, help='JSON list of path/source_stamp_ns rows')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    source = Path(args.images).resolve()
    out = Path(args.output).resolve()
    source.relative_to(V)
    out.relative_to(V)
    assert Path('/media/zhuyupei/SanStand').is_mount() and not out.exists()
    base.O = out
    out.mkdir()
    metadata = json.loads(source.read_text())
    if isinstance(metadata, dict):
        metadata = metadata['rows']
    assert isinstance(metadata, list) and len(metadata) >= 2
    rows = sorted(metadata, key=lambda r: r['source_stamp_ns'])
    assert len(set(r['source_stamp_ns'] for r in rows)) == len(rows)
    images = []
    inputs = []
    for row in rows:
        path = Path(row.get('path', row.get('png_path'))).resolve()
        path.relative_to(V)
        if row.get('png_sha256'):
            assert base.sha(path) == row['png_sha256']
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        assert gray is not None and gray.shape == (489, 653)
        images.append(gray)
        inputs.append(dict(path=str(path), source_stamp_ns=row['source_stamp_ns'],
                           sha256=base.sha(path)))
    pairs = []
    for i in range(1, len(images)):
        pairs.append(dict(from_ns=rows[i-1]['source_stamp_ns'], to_ns=rows[i]['source_stamp_ns'],
            delta_sec=(rows[i]['source_stamp_ns']-rows[i-1]['source_stamp_ns'])/1e9,
            tabletop=measure(images[i-1], images[i], [80, 30, 600, 300]),
            box=measure(images[i-1], images[i], [420, 330, 560, 450])))
    result = dict(kind='G49_recorded_RGB_pixel_displacement_diagnostic',
        input_index=str(source), input_index_sha256=base.sha(source), inputs=inputs,
        helper_sha256=base.sha(__file__), pairs=pairs,
        first_to_last=dict(tabletop=measure(images[0], images[-1], [80, 30, 600, 300]),
                           box=measure(images[0], images[-1], [420, 330, 560, 450])),
        method='Shi-Tomasi and bidirectional pyramidal LK; diagnostic regions chosen on inspected original images',
        limits=['Pixel shifts do not prove physical camera pose, depth accuracy or absence of vibration.',
                'Recorded RGB is sparse; missing captures and motion between samples are not reconstructed.',
                'Diagnostic feature filters are not robot perception or motion gates.'],
        external_boundary_attempts=base.ATTEMPTS, ROS_initialized=False, hardware_contacted=False,
        production_modified=False, original_images_modified=False)
    assert not base.ATTEMPTS
    base.save(out/'result.json', result)
    print(json.dumps(dict(result=str(out/'result.json'), sha256=base.sha(out/'result.json'),
                          pairs=pairs, first_to_last=result['first_to_last']), ensure_ascii=False))


if __name__ == '__main__':
    main()
