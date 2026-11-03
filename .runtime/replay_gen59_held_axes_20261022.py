#!/usr/bin/env python3
"""Pure, recorded GEN59 held-endpoint arithmetic; never a motion/path proof."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import sys
import time

sys.dont_write_bytecode = True
W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
R = N.parents[1] / 'grasp_records/20261022T044426.800138Z_c2cd1e6e1a8d4226ab4af1eaaa6646ff'
C = W / '.runtime/gen59_observation_pose_candidate_r3'
AUDIT = R / 'keyframes_audits/27ac190088e2c5fb1bbacbd1ead32f26f62f7322218682bdc26ca7edc554f822.json'
CONTEXT = N / 'gen57_resume_r3/actual_attempt_r1_geometry/historical_parameter_reconstruction_r1/attempt_2/context_parameters.json'
RELEASE = N / 'verification/gen59_release_manifest.json'


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--counterfactual', action='store_true')
    args = parser.parse_args()
    out = args.output.resolve()
    if not str(out).startswith(str(N) + '/') or not Path('/media/zhuyupei/SanStand').is_mount():
        raise RuntimeError('mounted external output required')
    out.mkdir(parents=True, exist_ok=False)
    # Python 3.8 platform otherwise spawns `uname -p` during SciPy import.
    # The real kernel tuple suffices here; processor name is not used by FK.
    platform._uname_cache = platform.uname_result(*os.uname(), '')
    blocked = []
    def guard(event, values):
        deny = (event.startswith('socket.') or event in (
            'subprocess.Popen', 'os.system', 'os.fork', 'os.posix_spawn', 'os.exec'))
        if event == 'open' and isinstance(values[0], (str, bytes, os.PathLike)):
            p = Path(values[0]).resolve()
            mode, flags = values[1:3]
            writing = (isinstance(mode, str) and any(v in mode for v in 'wax+')) or (
                isinstance(flags, int) and bool(flags & (os.O_WRONLY | os.O_RDWR | os.O_CREAT | os.O_TRUNC | os.O_APPEND)))
            deny |= writing and not (p == out or out in p.parents)
            deny |= str(p).startswith('/dev/') and str(p) != '/dev/null'
        if deny:
            blocked.append(dict(event=event, arguments=str(values)[:200]))
            raise RuntimeError('offline audit denied ' + event)
    sys.addaudithook(guard)
    release = json.loads(RELEASE.read_text())
    pins = release['source_sha256']
    # Real TF numerical helpers are imported; no rospy initialization or mocks.
    sys.path[:0] = [str(C / 'src/alicia_flexible_grasp_supervisor/src'),
                    '/opt/ros/noetic/lib/python3/dist-packages',
                    '/usr/lib/python3/dist-packages']
    import numpy as np
    import scipy
    from scipy.optimize import minimize
    from alicia_flexible_grasp.robot import held_pregrasp_selection as selection
    from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint, HeldAxesPredictionError
    from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
    from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk, ARM_NAMES, SDK_QUANTUM_RAD as Q
    from alicia_flexible_grasp.robot.endpoint_correction import sdk_counts
    sources = {}
    for module in list(sys.modules.values()):
        path = getattr(module, '__file__', None)
        if path and str(path).startswith(str(C) + '/'):
            p = Path(path); rel = str(p.relative_to(C)); actual = sha(p)
            if pins.get(rel) != actual:
                raise RuntimeError('source mismatch: ' + rel)
            sources[rel] = actual
    context = json.loads(CONTEXT.read_text())
    fk = SerialUrdfFk(context['/robot_description'], ARM_NAMES)
    audit = json.loads(AUDIT.read_text())
    rows = []
    started = time.monotonic()
    for index in (0, 4, 10, 17):
        origin = audit['lineage'][index]
        precheck = origin['contact_sequence_precheck']
        recorded = precheck['held_pregrasp_selection']
        assert precheck['model_sha256'] == fk.model_sha256
        rows.append(row := dict(lineage_index=index, track_id=origin['tracking']['track_id'],
            plan_id=precheck['plan_id'], recorded=precheck, evidence_replays=[]))
        evs = [recorded['original_failure']['evidence']] + [
            t['rejection_evidence'] for t in recorded['trials']]
        for ev in evs:
            sdk = (np.array(ev['initial_sdk_counts']) - 2048) * Q
            actual = np.array(ev['virtual_start_positions_rad'])
            try:
                _, new = predict_held_axes_endpoint(fk, sdk, actual, ev['fixed_goal_pose'],
                    position_tolerance_m=ev['position_tolerance_m'],
                    orientation_tolerance_rad=ev['orientation_tolerance_rad'],
                    allow_already_converged=True)
            except HeldAxesPredictionError as exc:
                new = exc.evidence
            fields = ('position_error_m', 'orientation_error_rad', 'predicted_position_error_m',
                      'predicted_orientation_error_rad')
            diffs = {k: abs(ev[k] - new[k]) for k in fields}
            row['evidence_replays'].append(dict(code=new['code'], evidence=new,
                absolute_differences=diffs, exact_delta=new['proposed_delta_counts'] == ev['proposed_delta_counts'],
                numeric_match=all(v <= 1e-12 for v in diffs.values())))
        try:
            _, new = selection.select_held_pregrasp_endpoint(fk, precheck['source_sample'],
                recorded['fixed_pregrasp_goal'], recorded['fixed_approach_goal'],
                recorded['original_nominal_command_rad'], position_tolerance_m=.010,
                orientation_tolerance_rad=math.radians(5))
            row['whole_selector'] = dict(success=True, report=new)
        except HeldAxesPredictionError as exc:
            row['whole_selector'] = dict(success=False, code=str(exc), evidence=exc.evidence)
        if args.counterfactual:
            # Diagnostic bounded proposals, SAME goals/held axes/count box/hard gates.
            # This never replaces production's search or certifies a feasible path.
            offset = np.array(recorded['sdk_minus_encoder_rad'])
            nominal = np.array(recorded['original_nominal_command_rad'])
            pg, ag = map(pose_matrix, (recorded['fixed_pregrasp_goal'], recorded['fixed_approach_goal']))
            free = np.array([0, 2, 3, 5])
            low = np.maximum.reduce((nominal-.25, fk.limits[:, 0], fk.limits[:, 0]-offset,
                                     np.full(6, -math.pi)-offset))
            high = np.minimum.reduce((nominal+.25, fk.limits[:, 1], fk.limits[:, 1]-offset,
                                      np.full(6, (4095-2048)*Q)-offset))
            def residual(q, target):
                if np.any(q < fk.limits[:, 0]) or np.any(q > fk.limits[:, 1]):
                    return 1e3, math.pi
                t = fk(q)
                return float(np.linalg.norm(t[:3,3]-target[:3,3])), math.acos(float(np.clip((np.trace(t[:3,:3].T@target[:3,:3])-1)/2,-1,1)))
            def after(x):
                q = x[:6].copy(); q[free] += x[6:]; return q
            def objective(x):
                return (residual(after(x), ag)[0]/.006)**2
            def constraint(x):
                p,a = residual(x[:6],pg)
                _,a0 = residual(x[:6],ag)
                _,a1 = residual(after(x),ag)
                return np.array([1-p/.010, 1-a/math.radians(5), 1-a0/math.radians(5), 1-a1/math.radians(5)])
            row['counterfactuals'] = []
            for trial in recorded['trials']:
                x = np.array(trial['solver_endpoint']); x[6:] *= Q
                solved = minimize(objective, x, method='SLSQP',
                    bounds=list(zip(low,high))+[(-32*Q,32*Q)]*4,
                    constraints=[dict(type='ineq',fun=constraint)],
                    options=dict(maxiter=150, ftol=1e-12))
                command = (sdk_counts(solved.x[:6]+offset)-2048)*Q
                item = dict(solver_success=bool(solved.success), message=str(solved.message),
                    iterations=int(solved.nit), endpoint=solved.x.tolist(),
                    continuous_residual_m=residual(after(solved.x),ag)[0],
                    normalized_hard_constraints=constraint(solved.x).tolist(),
                    quantized_command_rad=command.tolist())
                try:
                    _, a = predict_held_axes_endpoint(fk, command, command-offset,
                        recorded['fixed_pregrasp_goal'],position_tolerance_m=.010,
                        orientation_tolerance_rad=math.radians(5),allow_already_converged=True)
                    _, b = predict_held_axes_endpoint(fk,a['predicted_command_positions_rad'],
                        a['predicted_positions_rad'],recorded['fixed_approach_goal'],
                        position_tolerance_m=.006,orientation_tolerance_rad=math.radians(5),
                        allow_already_converged=True)
                    item.update(conditional_endpoint_qualified=True, pregrasp=a, approach=b)
                except HeldAxesPredictionError as exc:
                    item.update(conditional_endpoint_qualified=False, code=str(exc), evidence=exc.evidence)
                row['counterfactuals'].append(item)
        print(json.dumps(dict(lineage_index=index, baseline_selector_success=row['whole_selector']['success'],
            replay_match=all(v['numeric_match'] and v['exact_delta'] for v in row['evidence_replays']),
            counterfactual_qualified=[v['conditional_endpoint_qualified'] for v in row.get('counterfactuals',[])])),flush=True)
    result = dict(kind='offline_recorded_held_axes_review', execution_authorized=False,
        no_ros_node_initialized=True, no_motion=True, no_network=True, source_root=str(C),
        input_hashes={str(p):sha(p) for p in (AUDIT,CONTEXT,RELEASE)},
        imported_source_sha256=sources, model_sha256=fk.model_sha256,
        environment=dict(python=sys.version,numpy=np.__version__,scipy=scipy.__version__),
        elapsed_sec=time.monotonic()-started, rows=rows, blocked_events=blocked,
        limits=['Pure endpoint arithmetic only; no live freshness, original deadline, path, collision or actuator response proof.',
                'Optional optimizer counterfactual failure does not prove global or local-box infeasibility.',
                'A conditional endpoint witness is not an execution authorization.'])
    (out/'result.json').write_text(json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+'\n')
    for rel,digest in sources.items():
        assert sha(C/rel)==digest
    print(json.dumps(dict(result=str(out/'result.json'),sha256=sha(out/'result.json'))),flush=True)


if __name__ == '__main__':
    main()
