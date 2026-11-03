"""Offline fixed feature proposals; all original appearance gates remain decisive."""
import hashlib
import json
import time
from dataclasses import asdict
from pathlib import Path

import cv2
import numpy as np

from alicia_grasp_modes.arrival_dense_identity import freeze_reference, propose_homography, evaluate_appearance
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
F = N / 'verification/gen14_arrival_independent'
OUT = N / 'verification/gen14_small_support_features_r2'
PRE = 1791860176945733070


def plain(value):
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    if isinstance(value, dict): return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [plain(v) for v in value]
    return value


def features(image, mask, method):
    if method == 'sift':
        detector = cv2.SIFT_create(nfeatures=1000)
    else:
        detector = cv2.ORB_create(nfeatures=1000, scaleFactor=1.2, nlevels=8,
            edgeThreshold=15, firstLevel=0, WTA_K=2, scoreType=cv2.ORB_HARRIS_SCORE,
            patchSize=int(method), fastThreshold=20)
    keys, desc = detector.detectAndCompute(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
        cv2.erode((mask > 0).astype(np.uint8) * 255, np.ones((7, 7), np.uint8)))
    dist = cv2.distanceTransform((mask > 0).astype(np.uint8), cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    # SIFT support is conservatively bounded by its four-bin descriptor window.
    multiplier = 3.0 if method == 'sift' else 0.5
    ids = [i for i, k in enumerate(keys)
           if dist[round(k.pt[1]), round(k.pt[0])] >= np.sqrt(2.) * multiplier * k.size + 1]
    return [keys[i] for i in ids], None if desc is None else desc[ids], dict(total=len(keys), contained=len(ids))


def proposal(a, b, method):
    ka, da, ca = a; kb, db, cb = b
    audit = dict(reference=ca, current=cb)
    if da is None or db is None or min(len(da), len(db)) < 2: return None, audit
    matcher = cv2.BFMatcher(cv2.NORM_L2 if method == 'sift' else cv2.NORM_HAMMING)
    def ratios(x, y):
        return {p[0].queryIdx: p[0] for p in matcher.knnMatch(x, y, k=2)
                if len(p) == 2 and p[0].distance < .75 * p[1].distance}
    fw, bw = ratios(da, db), ratios(db, da)
    pairs = [m for i, m in fw.items() if m.trainIdx in bw and bw[m.trainIdx].trainIdx == i]
    audit['mutual_matches'] = len(pairs)
    if len(pairs) < 4: return None, audit
    src = np.array([ka[m.queryIdx].pt for m in pairs], np.float32)
    dst = np.array([kb[m.trainIdx].pt for m in pairs], np.float32)
    cv2.setRNGSeed(1729)
    h, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 3., maxIters=2000, confidence=.995)
    audit.update(reference_points=src, current_points=dst, inliers=inliers)
    return h, audit


def red_fraction(rgb, mask):
    px = rgb[mask > 0].astype(float)
    # Diagnostic colour count, neither semantic segmentation nor identity proof.
    return float(np.mean((px[:, 2] > 1.3 * px[:, 1]) & (px[:, 2] > 1.3 * px[:, 0])))


def main():
    OUT.mkdir(exist_ok=False); cv2.setNumThreads(1)
    fixture = json.loads((F / 'fixture.json').read_text()); cam = fixture['camera']
    raw = np.load(F / f'{PRE}_source.npz')
    valid = rectification_rgb_valid_mask(cam, raw['depth'].shape)
    if valid is None: valid = np.ones(raw['depth'].shape, bool)
    reference = freeze_reference(raw['rgb'], raw['recorded_mask'], valid, PRE)
    methods = ['15', '21']
    cached = {m: features(raw['rgb'], raw['recorded_mask'], m) for m in methods}
    rows = []
    for item in fixture['frames']:
        ns = item['source_stamp_ns']
        if ns < 1791860212000000000: continue
        frame = np.load(item['file'])
        mask = cv2.imread(str(F / f'{ns}_rejected_proposal_0_mask.png'), cv2.IMREAD_GRAYSCALE)
        tests = []
        for method in methods:
            start = time.perf_counter()
            h, audit = proposal(cached[method], features(frame['rgb'], mask, method), method)
            prop = None if h is None else propose_homography(reference, ns, h)
            verdict = evaluate_appearance(reference, frame['rgb'], mask, valid, ns, prop,
                deadline_monotonic=time.monotonic() + 5.)
            tests.append(dict(method=method, proposal=audit, H=h, original_verdict=asdict(verdict),
                elapsed_sec=time.perf_counter() - start))
        rows.append(dict(source_ns=ns, red_dominant_fraction=red_fraction(frame['rgb'], mask), tests=tests))
        print(ns, [(r['method'], r['proposal'].get('mutual_matches'), r['original_verdict']['code']) for r in tests], flush=True)
    result = dict(scope='Fixed proposal diagnostic only; no pose or qualification claim. Generous diagnostic local clock, not source-time replay.',
        original_evaluator_unchanged=True, reference_digest=reference.digest, opencv_version=cv2.__version__,
        unavailable_methods=['SIFT: unavailable in actual PYTHONNOUSERSITE ROS environment'],
        reference_red_dominant_fraction=red_fraction(raw['rgb'], raw['recorded_mask']),
        reference_pixel_source_ns=PRE, rows=rows, motion_commands_sent=False,
        original_fixture_sha256=hashlib.sha256((F / 'fixture.json').read_bytes()).hexdigest())
    (OUT / 'result.json').write_text(json.dumps(plain(result), indent=2) + '\n')


if __name__ == '__main__': main()
