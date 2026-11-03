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


from alicia_grasp_modes.arrival_train_only_proposal import refine_train_only_proposal

def main():
    out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    import alicia_grasp_modes.arrival_train_only_proposal as training
    production = [Path(face.__file__), Path(dense.__file__), Path(training.__file__)]
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
            variants = {}
            for method in ('analytic', '2-point'):
                tick = time.monotonic()
                proposal, audit_training = refine_train_only_proposal(frozen.reference,
                    data['rgb'], measured, valid, ns, initial, check=lambda: None, jacobian=method)
                verdict = dense.evaluate_appearance(frozen.reference, data['rgb'], measured, valid,
                    ns, proposal, deadline_monotonic=time.monotonic()+5, require_dense=True)
                variants[method] = dict(optimization=audit_training, verdict=asdict(verdict),
                    H=np.asarray(proposal.matrix).tolist(), elapsed_sec=time.monotonic()-tick)
            delta = np.max(np.abs(np.asarray(variants['analytic']['H'])-np.asarray(variants['2-point']['H'])))
            row.update(face=audit, variants=variants, max_abs_H_delta=float(delta),
                pass_equal=variants['analytic']['verdict']['appearance_match']==variants['2-point']['verdict']['appearance_match'])
            print(ns, variants['analytic']['verdict']['code'], variants['2-point']['verdict']['code'], delta, flush=True)
        except Exception as error:
            row['error'] = repr(error); print(ns, repr(error), flush=True)
        row['elapsed_sec'] = time.monotonic()-begin; rows.append(row)
    report = dict(scope='Offline fixed analytic versus numerical Jacobian registration equivalence, same 12-evaluation objective, not source-clock or qualification replay',
        original_gates_unchanged=True, counts={m:sum(r.get('variants',{}).get(m,{}).get('verdict',{}).get('appearance_match',False) for r in rows)for m in ('analytic','2-point')}, all_verdicts_equal=all(r.get('pass_equal',False) for r in rows), current_optimizer_history_excluded_from_final_holdout=True,
        no_holdout_optimizer_selection=True, rows=rows, source_sha256=before,
        source_unchanged=all(hashlib.sha256(Path(p).read_bytes()).hexdigest() == h for p, h in before.items()),
        no_ros_master=True, motion_commands_sent=False, script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/'result.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
