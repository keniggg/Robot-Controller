"""Offline fixed train-only registration experiment, never an acceptance artifact."""
import sys, json, time, hashlib, inspect
from pathlib import Path
from dataclasses import fields, asdict
import cv2
import numpy as np
from scipy.optimize import least_squares

W = Path(__file__).resolve().parent.parent
C = W / '.runtime/gen14_arrival_repair_20261013'
sys.path.insert(0, str(C / 'src/alicia_grasp_modes/src'))
from alicia_grasp_modes import observation_measured_face as face
from alicia_grasp_modes import arrival_dense_identity as dense
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
F = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification/gen14_arrival_independent')
PRE = 1791860176945733070


def sha(a):
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def refine(reference, image, mask, H):
    # PRE-only selection, even checkerboard texture cells, with a fixed two-pixel
    # cell boundary exclusion. No current identity score selects these samples.
    x0, y0, width, height = reference.bounds
    yy, xx = np.nonzero((cv2.erode(reference.mask, np.ones((7, 7), np.uint8)) > 0) & reference.valid)
    cells = np.c_[(xx-x0)//8, (yy-y0)//8]
    take = np.array([tuple(c) in set(reference.texture_cells) for c in cells])
    take &= cells.sum(axis=1) % 2 == 0
    take &= ((xx-x0) % 8 >= 2) & ((xx-x0) % 8 < 6) & ((yy-y0) % 8 >= 2) & ((yy-y0) % 8 < 6)
    xx, yy = xx[take], yy[take]
    if len(xx) < 100:
        raise ValueError('insufficient_fixed_train_samples')
    source = np.c_[xx, yy, np.ones(len(xx))]
    normalized = np.c_[(xx-(x0+width/2))/width, (yy-(y0+height/2))/height, np.ones(len(xx))]
    predicted = source @ H.T
    predicted = predicted[:, :2] / predicted[:, 2, None]
    x = reference.gray[yy, xx]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).astype(float)
    consumed = np.zeros(gray.shape, bool)
    calls = [0]

    def residual(p):
        calls[0] += 1
        uv = predicted + normalized @ p[:6].reshape(2, 3).T
        bx, by = np.floor(uv).astype(int).T
        if np.any(bx < 0) or np.any(by < 0) or np.any(bx+1 >= gray.shape[1]) or np.any(by+1 >= gray.shape[0]):
            raise ValueError('train_warp_left_image')
        # Track ALL pixel supports, including rejected trust-region proposals
        # and finite-difference evaluations. Final holdout cannot consume these.
        for dy in (0, 1):
            for dx in (0, 1):
                consumed[by+dy, bx+dx] = True
        fx, fy = (uv-np.c_[bx, by]).T
        y = ((1-fx)*(1-fy)*gray[by, bx] + fx*(1-fy)*gray[by, bx+1]
             + (1-fx)*fy*gray[by+1, bx] + fx*fy*gray[by+1, bx+1])
        return y - (p[6]*x+p[7])

    p0 = np.array([0., 0., 0., 0., 0., 0., 1., 0.])
    result = least_squares(residual, p0, method='trf', jac='2-point',
                           bounds=([-3.]*6+[-np.inf]*2, [3.]*6+[np.inf]*2),
                           loss='soft_l1', f_scale=10., max_nfev=12,
                           ftol=1e-5, xtol=1e-5, gtol=1e-5)
    # Equivalent affine increment using the SAME fixed PRE normalization.
    normalizer = np.array([[1/width, 0., -(x0+width/2)/width],
                           [0., 1/height, -(y0+height/2)/height], [0., 0., 1.]])
    assert np.allclose(H[2], [0., 0., 1.])
    refined = H.copy()
    refined[:2] += result.x[:6].reshape(2, 3) @ normalizer
    return refined, consumed, dict(training_samples=len(xx), residual_calls=calls[0],
        optimizer_nfev=result.nfev, optimizer_status=int(result.status),
        optimizer_parameters=result.x.tolist(), optimizer_final_cost=float(result.cost),
        all_current_interpolation_support_pixels=int(consumed.sum()),
        current_support_sha256=sha(consumed), preselected_training_only=True,
        final_holdout_used_for_search_or_selection=False)


def disjoint_evaluate(consumed, *args, **kwargs):
    # Diagnostic copy, adding only a stricter independence exclusion. Keep
    # original reference cell denominator and ALL original thresholds.
    source = inspect.getsource(dense.evaluate_appearance)
    needle = 'current_flat_indices = iy * mask.shape[1] + ix'
    assert source.count(needle) == 1
    source = source.replace(needle, needle + '\n        contaminated = optimizer_consumed[iy, ix]\n        metrics["optimizer_contaminated_holdout_pairs_removed"] = int(np.count_nonzero(test & contaminated))\n        test &= ~contaminated')
    namespace = dict(dense.__dict__, optimizer_consumed=consumed)
    exec(compile(source, '<diagnostic-stricter-independence-only>', 'exec'), namespace)
    return namespace['evaluate_appearance'](*args, **kwargs)


def main():
    out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    production = [Path(face.__file__), Path(dense.__file__)]
    before = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in production}
    fixture = json.loads((F/'fixture.json').read_text()); cam = fixture['camera']
    config = face.Config(**{x.name: fixture['unknown_parameters'][x.name] for x in fields(face.Config) if x.name in fixture['unknown_parameters']})
    k = [cam[x] for x in ('fx', 'fy', 'cx', 'cy')]
    plane = np.array(json.loads((F/'original_geometry_reconstruction_v2.json').read_text())['seed']['reference_plane_base'])
    pre = np.load(F/f'{PRE}_source.npz')
    valid = rectification_rgb_valid_mask(cam, pre['depth'].shape)
    original = face.FrozenMeasuredFaceSource(pre['raw_depth'], '16UC1', cam['depth_scale'], k,
        pre['T_base_camera'], plane, config, PRE, 'diagnostic-only', sha(pre['rgb']), sha(pre['recorded_mask']))
    frozen = face.freeze_measured_face(original, pre['rgb'], pre['recorded_mask'], valid,
        check_current=lambda: None, deadline_monotonic=time.monotonic()+5)
    rows = []
    geometry = json.loads((F/'full_arrival_current_geometry/result.json').read_text())
    for g in sorted(geometry['rows'], key=lambda r: r['source_ns']):
        ns = g['source_ns']; data = np.load(g['source_fixture']['file'])
        mask = cv2.imread(g['proposals'][0]['mask_path'], 0)
        row = dict(source_ns=ns, whole_mask_sha256=sha(mask), source_npz_sha256=g['source_fixture']['sha256'])
        begin = time.monotonic()
        try:
            measured, _, _, _, audit = face.extract_measured_face(data['rgb'], mask, valid, data['depth'], k,
                data['T_base_camera'], plane, config, lambda: None)
            initial = face._affine_proposal(frozen.reference, data['rgb'], measured, ns, lambda: None)
            H, consumed, optimization = refine(frozen.reference, data['rgb'], measured, np.asarray(initial.matrix))
            proposal = dense.propose_homography(frozen.reference, ns, H)
            # Every evaluation below is after the fixed train-only optimizer.
            args = (frozen.reference, data['rgb'], measured, valid, ns)
            options = dict(deadline_monotonic=time.monotonic()+5, require_dense=True)
            baseline = dense.evaluate_appearance(*args, initial, **options)
            diagnostic_only = dense.evaluate_appearance(*args, proposal, **options)
            independent = disjoint_evaluate(consumed, *args, proposal, **options)
            row.update(face=audit, optimization=optimization, baseline=asdict(baseline),
                refinement_without_history_exclusion_NOT_independent=asdict(diagnostic_only),
                refinement_with_history_exclusion=asdict(independent), initial_H=np.asarray(initial.matrix).tolist(), refined_H=H.tolist())
            print(ns, baseline.code, diagnostic_only.code, independent.code,
                  independent.metrics.get('optimizer_contaminated_holdout_pairs_removed'), flush=True)
        except Exception as error:
            row['error'] = repr(error); print(ns, repr(error), flush=True)
        row['elapsed_sec'] = time.monotonic()-begin; rows.append(row)
    report = dict(scope='Offline fixed train-only affine refinement diagnostic, not source-clock replay, not qualification or deployable production',
        original_gates_unchanged=True, current_optimizer_history_excluded_from_final_holdout=True,
        no_holdout_optimizer_selection=True, rows=rows, source_sha256=before,
        source_unchanged=all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == h for p, h in before.items()),
        no_ros_master=True, motion_commands_sent=False, script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
