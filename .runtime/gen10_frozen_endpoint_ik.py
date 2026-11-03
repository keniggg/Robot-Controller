#!/usr/bin/env python3
"""Read sealed observation poses and test bounded numerical IK, without ROS."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
AUDIT = EXT / 'grasp_records/20261009T031019.431802Z_cf2238ea63164fd2a86f610925c32457/keyframes_audits/18452e4dea5750147f6da04b6122b9f605e6b39e98ef10c966855962b624f3d3.json'
MODEL = EXT / 'ros_log/user_grasp_20261008T090751Z/source_snapshot_review/gen28_fk_ik_100ms.model.urdf'
rotation = getattr(Rotation, 'from_matrix', None) or Rotation.from_dcm


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=90.)
    parser.add_argument('--max-nfev', type=int, default=80)
    parser.add_argument('--seeds', type=int, default=1)
    parser.add_argument('--indices', type=int, nargs='*')
    args = parser.parse_args()
    if not Path('/media/zhuyupei/SanStand').is_mount() or EXT not in args.output.resolve().parents:
        raise ValueError('output must be on mounted SanStand')
    audit = json.loads(AUDIT.read_text()); diagnostic = audit['observation_failure_diagnostics']
    search = diagnostic['joint_space_fk_search']['audit']
    xml = MODEL.read_text(); model_sha = hashlib.sha256(xml.encode()).hexdigest()
    assert model_sha == search['model_sha256']
    fk = SerialUrdfFk(xml, ['Joint%d' % i for i in range(1, 7)])
    q0 = np.asarray(search['current_joint_positions']); limits = fk.limits.copy()
    bounds = (limits[:, 0]+1e-10, limits[:, 1]-1e-10)
    rows = sorted(diagnostic['observation_candidates'],
                  key=lambda r: r['observation_view']['check_scheduling_index'])
    indices = args.indices if args.indices is not None else list(range(len(rows)))
    rng = np.random.RandomState(20261009)
    seeds = [q0, np.mean(limits, axis=1)]
    while len(seeds) < args.seeds:
        seeds.append(rng.uniform(*bounds))
    started = time.monotonic()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as output:
        def emit(row):
            output.write(json.dumps(row, allow_nan=False)+'\n'); output.flush()
        emit(dict(kind='scope',ros_calls=False,execution_authority=False,
            source_audit_sha256=hashlib.sha256(AUDIT.read_bytes()).hexdigest(),
            model_sha256=model_sha,model_file=str(MODEL),
            model_scope='archived XML hash matches gen10 audit exactly',
            seed_scope='frozen gen10 measured q; not current manually moved position',
            q0=q0.tolist(),joint_limits=limits.tolist(),seconds=args.seconds,
            max_nfev=args.max_nfev,seeds=args.seeds))
        completed = []
        try:
            for rank in indices:
                row = rows[rank]; target = np.asarray(row['base_from_tool'])
                def residual(q):
                    if time.monotonic()-started > args.seconds:
                        raise TimeoutError('bounded offline numerical search complete')
                    actual = fk(q)
                    return np.r_[5.*(actual[:3,3]-target[:3,3]),
                        rotation(actual[:3,:3] @ target[:3,:3].T).as_rotvec()]
                best = None
                for seed_index, seed in enumerate(seeds[:args.seeds]):
                    result = least_squares(residual, np.clip(seed, *bounds), bounds=bounds,
                        max_nfev=args.max_nfev, ftol=1e-9, xtol=1e-9, gtol=1e-9)
                    err = residual(result.x)
                    current = dict(kind='fit', rank=rank,candidate_index=row['candidate_index'],
                        seed_index=seed_index,q=result.x.tolist(),nfev=result.nfev,
                        position_error_m=float(np.linalg.norm(err[:3])/5.),
                        rotation_error_rad=float(np.linalg.norm(err[3:])),cost=float(np.linalg.norm(err)),
                        active_bounds=result.active_mask.tolist(),solver_success=bool(result.success),
                        exact_pose_match=bool(np.linalg.norm(err[:3])/5.<1e-5 and np.linalg.norm(err[3:])<1e-4),
                        roll=row['camera_roll_offset_deg'],tilt=row['camera_tilt_offset_deg'],
                        distance=row['camera_target_distance_m'],T_base_tool0=fk(result.x).tolist(),
                        goal_base_from_tool=target.tolist(),whole_arm_collision_checked=False,
                        following_support_checked=False,complete_path_checked=False)
                    emit(current)
                    if best is None or current['cost'] < best['cost']: best = current
                    if current['exact_pose_match']: break
                completed.append(best)
                if rank % 20 == 0 or best['exact_pose_match']:
                    print(json.dumps({k:best[k] for k in ('rank','exact_pose_match','position_error_m','rotation_error_rad','active_bounds')}),flush=True)
            emit(dict(kind='completed',tested=len(completed),exact_count=sum(r['exact_pose_match'] for r in completed),
                elapsed_sec=time.monotonic()-started,best_rank=min(completed,key=lambda r:r['cost'])['rank']))
        except TimeoutError:
            emit(dict(kind='bounded_stop',tested=len(completed),exact_count=sum(r['exact_pose_match'] for r in completed),
                elapsed_sec=time.monotonic()-started))


if __name__ == '__main__': main()
