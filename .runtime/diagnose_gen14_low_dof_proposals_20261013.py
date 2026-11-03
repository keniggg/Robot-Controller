"""Diagnostic affine/similarity proposals. Conditional evidence never authorizes."""
import importlib.util
import inspect
import json
import time
from dataclasses import asdict
from pathlib import Path
import cv2
import numpy as np
import alicia_grasp_modes.arrival_dense_identity as dense

SPEC = importlib.util.spec_from_file_location('small_feature_diagnostic', Path(__file__).with_name('diagnose_gen14_small_support_features_20261013.py'))
helper = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(helper)
OUT = helper.N / 'verification/gen14_low_dof_proposals_r1'


def original_matches(rgb, mask):
    orb = cv2.ORB_create(nfeatures=1000, scaleFactor=1.2, nlevels=8, edgeThreshold=15,
        firstLevel=0, WTA_K=2, scoreType=cv2.ORB_HARRIS_SCORE, patchSize=31, fastThreshold=20)
    points, desc = orb.detectAndCompute(cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY),
        cv2.erode((mask > 0).astype(np.uint8) * 255, np.ones((7, 7), np.uint8)))
    return points, desc, dict(total=len(points), support_containment_claimed=False)


def main():
    OUT.mkdir(); cv2.setNumThreads(1)
    fixture = json.loads((helper.F / 'fixture.json').read_text()); cam = fixture['camera']
    raw = np.load(helper.F / f'{helper.PRE}_source.npz')
    valid = helper.rectification_rgb_valid_mask(cam, raw['depth'].shape)
    if valid is None: valid = np.ones(raw['depth'].shape, bool)
    ref = dense.freeze_reference(raw['rgb'], raw['recorded_mask'], valid, helper.PRE)
    reference = original_matches(raw['rgb'], raw['recorded_mask'])
    # Inspect subsequent original gates only; never report this bypass as pass.
    src = inspect.getsource(dense.evaluate_appearance).replace('def evaluate_appearance(', 'def inspect_after_mask(').replace("return finish(False, 'WHOLE_MASK_INCONSISTENT')", "metrics['original_mask_gate_failed'] = True")
    space = dict(vars(dense)); exec(src, space); downstream = space['inspect_after_mask']
    rows = []
    for f in fixture['frames']:
        ns = f['source_stamp_ns']
        if ns < 1791860212000000000: continue
        frame = np.load(f['file']); rgb = frame['rgb']
        mask = cv2.imread(str(helper.F / f'{ns}_rejected_proposal_0_mask.png'), cv2.IMREAD_GRAYSCALE)
        h, evidence = helper.proposal(reference, original_matches(rgb, mask), '31')
        proposals = [('original_projective', h)]
        if 'reference_points' in evidence:
            a, b = evidence['reference_points'], evidence['current_points']
            for name, estimate in [('affine', cv2.estimateAffine2D), ('similarity', cv2.estimateAffinePartial2D)]:
                cv2.setRNGSeed(1729)
                m, _ = estimate(a, b, method=cv2.RANSAC, ransacReprojThreshold=3., maxIters=2000, confidence=.995, refineIters=10)
                proposals.append((name, None if m is None else np.vstack([m, [0., 0., 1.]])))
        tests = []
        for name, h in proposals:
            prop = None if h is None else dense.propose_homography(ref, ns, h)
            result = asdict(dense.evaluate_appearance(ref, rgb, mask, valid, ns, prop, deadline_monotonic=time.monotonic()+5))
            condition = None
            if result['code'] == 'WHOLE_MASK_INCONSISTENT':
                condition = asdict(downstream(ref, rgb, mask, valid, ns, prop, deadline_monotonic=time.monotonic()+5))
                condition.update(appearance_match=False, diagnostic_only=True, actual_rejection='WHOLE_MASK_INCONSISTENT', downstream_code=condition['code'], code='CONDITIONAL_DIAGNOSTIC_NOT_ACCEPTANCE')
            tests.append(dict(method=name, H=h, original_verdict=result, conditional=condition))
        rows.append(dict(source_ns=ns, matches=evidence, tests=tests))
        print(ns, [(t['method'], t['original_verdict']['code'], None if t['conditional'] is None else t['conditional']['downstream_code']) for t in tests], flush=True)
    (OUT/'result.json').write_text(json.dumps(helper.plain(dict(reference_digest=ref.digest, rows=rows, no_ros_actions=True,
        scope='Fixed lower-DOF proposal diagnostic; original evaluator unchanged. Conditional bypass is diagnostic false, never acceptance or qualification.')) ,indent=2)+'\n')


if __name__ == '__main__': main()
