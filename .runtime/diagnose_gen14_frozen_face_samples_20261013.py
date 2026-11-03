"""Frozen measured-height face diagnostic; never whole-object qualification."""
import importlib.util
import json
import time
from dataclasses import asdict
from pathlib import Path
import cv2
import numpy as np
from alicia_grasp_modes.tabletop import _project
from alicia_grasp_modes.arrival_dense_identity import freeze_reference, propose_homography, evaluate_appearance
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask

SPEC = importlib.util.spec_from_file_location('gen14_original_features', Path(__file__).with_name('diagnose_gen14_low_dof_proposals_20261013.py'))
helper = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(helper)
F = helper.helper.F
OUT = helper.helper.N/'verification/gen14_frozen_face_samples_r1'


def main():
    OUT.mkdir(); cv2.setNumThreads(1)
    fixture = json.loads((F/'fixture.json').read_text()); camera = fixture['camera']
    seed = json.loads((F/'original_geometry_reconstruction_v2.json').read_text())['seed']
    source = json.loads((F/'view_source/source.json').read_text())
    normal = np.array(seed['reference_plane_base'])
    height = seed['premotion_refined_metrics']['upper_seed_height_m']
    tolerance = source['arguments']['depth_uncertainty_m']
    k = [camera[x] for x in ('fx', 'fy', 'cx', 'cy')]
    raw = np.load(F/f'{helper.helper.PRE}_source.npz')
    valid = rectification_rgb_valid_mask(camera, raw['depth'].shape)
    if valid is None: valid = np.ones(raw['depth'].shape, bool)

    def face(frame, mask):
        xyz = _project(frame['depth'], k)
        base = xyz @ frame['T_base_camera'][:3,:3].T + frame['T_base_camera'][:3,3]
        measured_height = base @ normal[:3] + normal[3]
        good = np.isfinite(xyz).all(axis=2) & np.isfinite(frame['depth']) & (frame['depth'] > .03) & (frame['depth'] < 2.)
        selected = (mask > 0) & good & (np.abs(measured_height - height) <= tolerance)
        return selected.astype(np.uint8), dict(original_pixels=int(np.count_nonzero(mask)),
            original_valid_depth=int(np.count_nonzero((mask>0)&good)),
            selected_actual_depth_pixels=int(selected.sum()),
            no_holes_filled=True, original_depth_unchanged=True)

    premask, pre_stats = face(raw, raw['recorded_mask'])
    reference = freeze_reference(raw['rgb'], premask, valid, helper.helper.PRE)
    reference_features = helper.original_matches(raw['rgb'], premask)
    np.savez_compressed(OUT/'reference_face.npz', mask=premask, normal=normal, height=height, tolerance=tolerance)
    rows = []
    for item in fixture['frames']:
        ns = item['source_stamp_ns']
        if ns < 1791860212000000000: continue
        frame = np.load(item['file'])
        whole = cv2.imread(str(F/f'{ns}_rejected_proposal_0_mask.png'), cv2.IMREAD_GRAYSCALE)
        mask, stats = face(frame, whole)
        h, proposal_stats = helper.helper.proposal(reference_features, helper.original_matches(frame['rgb'], mask), '31')
        proposals = [('projective', h)]
        if 'reference_points' in proposal_stats:
            cv2.setRNGSeed(1729)
            aff, _ = cv2.estimateAffine2D(proposal_stats['reference_points'], proposal_stats['current_points'],
                method=cv2.RANSAC, ransacReprojThreshold=3., maxIters=2000, confidence=.995, refineIters=10)
            proposals.append(('affine', None if aff is None else np.vstack([aff, [0.,0.,1.]])))
        verdicts = []
        for name, h in proposals:
            prop = None if h is None else propose_homography(reference, ns, h)
            result = evaluate_appearance(reference, frame['rgb'], mask, valid, ns, prop,
                deadline_monotonic=time.monotonic()+5.)
            verdicts.append(dict(method=name, face_evaluator=asdict(result), H=h))
        rows.append(dict(source_ns=ns, geometry_samples=stats, proposal=proposal_stats, verdicts=verdicts,
            whole_object_identity_proven=False, qualification_allowed=False))
        print(ns,stats['selected_actual_depth_pixels'],[(v['method'],v['face_evaluator']['code'],v['face_evaluator']['metrics'].get('colour_distance')) for v in verdicts],flush=True)
    report = dict(scope='Shared measured-height samples only. No guarantee of whole-face support, surface normal, spatial uniqueness or whole-object identity; original evaluator face colour pass alone cannot authorize.',
        frozen_premotion_plane=normal, frozen_premotion_upper_seed_height_m=height, frozen_original_uncertainty_m=tolerance,
        reference_samples=pre_stats, reference_digest=reference.digest, rows=rows, no_ros_actions=True,
        production_changed=False, unknown_depth_filled=False, geometry_authority=False, motion_authorized=False)
    (OUT/'result.json').write_text(json.dumps(helper.helper.plain(report),indent=2)+'\n')


if __name__ == '__main__': main()
