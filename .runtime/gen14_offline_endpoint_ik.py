"""Independent bounded numerical IK on recorded goals; no ROS or collision claim."""
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk

NEW = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_no_skin_20261011T000625Z')
INPUT = NEW / 'gen14_analysis/req16_read_only_ik_probe_inputs.json'
OUT = NEW / 'gen14_analysis/offline_endpoint_ik.json'
data = json.loads(INPUT.read_text())
names = tuple('Joint%d' % i for i in range(1, 7))
seed_by_name = dict(zip(data['recorded_joint_seed']['name'], data['recorded_joint_seed']['position']))
seed = np.array([seed_by_name[n] for n in names])
fk = SerialUrdfFk(Path(data['model_path']).read_text(), names)
assert fk.model_sha256 == data['model_sha256']
lower, upper = fk.limits[:, 0] + 1e-9, fk.limits[:, 1] - 1e-9
initial = fk(seed)
began = time.monotonic()
results = []
seen = set()
rows = []
for row in data['selected_probes'] + data['all_220_in_original_first_path_dispatch_order']:
    if row['candidate_index'] not in seen:
        rows.append(row)
        seen.add(row['candidate_index'])


class BudgetExpired(Exception):
    pass


def save(status):
    report = dict(
        source_inputs=str(INPUT), source_inputs_sha256=hashlib.sha256(INPUT.read_bytes()).hexdigest(),
        model_sha256=fk.model_sha256, joint_names=names, joint_limits_rad=fk.limits.tolist(),
        recorded_seed_rad=seed.tolist(), seed_tool_transform=initial.tolist(),
        method='scipy bounded least_squares, position/0.01m and SO3 rotvec/0.10rad',
        maximum_function_evaluations_per_solve=120, status=status,
        elapsed_sec=time.monotonic() - began, exact_endpoint_numerical_tolerance=dict(position_m=1e-4, angle_rad=1e-3),
        results=results, passed_count=sum(r['numerical_endpoint_pass'] for r in results),
        limitations=['Only numerical endpoint IK; collision, following allowance, path, arrival measurement and current task authority not certified.',
                     'No solution found under bounded local optimization does not prove global infeasibility.',
                     'No ROS, live parameters, motion, or production source changes.'])
    OUT.write_text(json.dumps(report, indent=2) + '\n')


try:
    for row in rows:
        target = np.asarray(row['target_base_from_tool'])
        start = time.monotonic()
        def residual(q):
            if time.monotonic() - began > 65.:
                raise BudgetExpired()
            actual = fk(q)
            return np.r_[(actual[:3, 3] - target[:3, 3]) / .01,
                         Rotation.from_dcm(target[:3, :3].T @ actual[:3, :3]).as_rotvec() / .10]
        optimized = least_squares(residual, seed, bounds=(lower, upper), max_nfev=120,
                                  xtol=1e-10, ftol=1e-10, gtol=1e-10)
        actual = fk(optimized.x)
        position = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
        angle = float(np.linalg.norm(Rotation.from_dcm(target[:3, :3].T @ actual[:3, :3]).as_rotvec()))
        result = dict(candidate_index=row['candidate_index'], scheduling_index=row['scheduling_index'],
                      distance_m=row['distance_m'], tilt_deg=row['tilt_deg'], roll_deg=row['roll_deg'],
                      originally_checked=row['original_strict_result']['checked'],
                      numerical_endpoint_pass=position <= 1e-4 and angle <= 1e-3,
                      position_error_m=position, orientation_error_rad=angle,
                      optimizer_success=bool(optimized.success), nfev=int(optimized.nfev),
                      elapsed_sec=time.monotonic() - start, joint_positions_rad=optimized.x.tolist(),
                      minimum_joint_limit_margin_rad=float(np.min(np.minimum(optimized.x-lower, upper-optimized.x))),
                      active_bounds=optimized.active_mask.tolist())
        results.append(result)
        if result['numerical_endpoint_pass'] or len(results) % 20 == 0:
            save('running')
            print(json.dumps(result), flush=True)
    save('complete')
except BudgetExpired:
    save('bounded_time_exhausted')
print('REPORT', str(OUT), hashlib.sha256(OUT.read_bytes()).hexdigest(), 'count', len(results),
      'passed', sum(r['numerical_endpoint_pass'] for r in results), flush=True)
