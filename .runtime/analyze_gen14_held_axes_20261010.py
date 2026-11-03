"""Offline sealed GEN14 replay. No ROS node, publisher, service or parameter access."""
import sys
from pathlib import Path
WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
sys.path[:0] = ['/opt/ros/noetic/lib/python3/dist-packages', str(WORK/'devel/lib/python3/dist-packages'), str(WORK/'src/alicia_flexible_grasp_supervisor/src')]
import hashlib
import json
import math
from itertools import product
import numpy as np
import rosbag
import rospy
from scipy.optimize import minimize
from geometry_msgs.msg import Pose
from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk, ARM_NAMES, SDK_QUANTUM_RAD as Q, stationary_following_error
from alicia_flexible_grasp.robot.pregrasp_feedforward import PregraspFeedforward

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = EXT/'ros_log/full_latest_restart_20261010T002347Z'
REC = EXT/'grasp_records/20261010T060144.544180Z_94750fef3cae477792d13cdcb9d4ade0'
OUT = NEW/'gen14_analysis'
OUT.mkdir(exist_ok=True)
log = NEW/'support_offset_policy_deployment/consumers.log'
failure = None
for line in log.read_text().splitlines():
    if '[Grasp] FAILED CONTACT_ENDPOINT_COMPENSATION_FAILED:' in line:
        failure = json.loads(line[line.index('{'):])
assert failure['plan_id'] == '212cd5c13c08337a1a24a79b'
model = WORK/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
fk = SerialUrdfFk(model.read_text(), ARM_NAMES)
assert fk.model_sha256 == failure['model_sha256']
limits = {'/alicia_d/sdk_command': failure['sdk_stamp_ns'], '/alicia_d/accepted_joint_states': failure['accepted_stamp_ns']}
rows = {x: [] for x in limits}
sources = {x: [] for x in limits}
with rosbag.Bag(str(REC/'keyframes.bag')) as bag:
    for topic, msg, receipt in bag.read_messages(topics=list(limits), start_time=rospy.Time.from_sec(1791612245.1), end_time=rospy.Time.from_sec(1791612246.3)):
        if msg.header.stamp.to_nsec() <= limits[topic]:
            rows[topic].append(msg)
            sources[topic].append(dict(stamp_ns=msg.header.stamp.to_nsec(), receipt_ns=receipt.to_nsec(), frame_id=msg.header.frame_id, names=list(msg.name), positions=list(msg.position)))
assert all(rows[t][-1].header.stamp.to_nsec() == ns for t, ns in limits.items())
sample = stationary_following_error(fk, rows['/alicia_d/sdk_command'], rows['/alicia_d/accepted_joint_states'], now_sec=1791612246.08, epoch_ns=failure['epoch_ns'])
goal = Pose()
for obj, names, values in [(goal.position, 'xyz', failure['fixed_goal_pose'][:3]), (goal.orientation, 'xyzw', failure['fixed_goal_pose'][3:])]:
    for key, value in zip(names, values): setattr(obj, key, value)
ff = PregraspFeedforward(fk, sample, goal, position_tolerance_m=.006, orientation_tolerance_rad=math.radians(5))
try:
    ff.propose_held_axes()
    replay_error = None
except ValueError as exc:
    replay_error = str(exc)
assert replay_error == failure['code']
c = ff.contract
q = np.array(sample['accepted_positions_rad'])
sdk = np.array(sample['sdk_positions_rad'])
guide, _ = c._remaining_budget_guide(q)
initial_error = c.residual(q)
guide_error = c.residual(q + guide*Q)
print('initial', initial_error, 'guide', guide.tolist(), 'guide_error', guide_error, flush=True)
free = np.array([0, 2, 3, 5])
def pred(x):
    result = q.copy()
    result[free] += np.asarray(x)*Q
    return result
def objective(x): return c.residual(pred(x))[0]**2
constraint = {'type': 'ineq', 'fun': lambda x: math.radians(5)-c.residual(pred(x))[1]}
starts = [np.zeros(4), guide[free]] + list(np.random.RandomState(14).uniform(-32, 32, (12, 4)))
searches, lattice = [], []
visited = set()
for i, start in enumerate(starts):
    result = minimize(objective, start, method='SLSQP', bounds=[(-32,32)]*4, constraints=[constraint], options={'maxiter':120, 'ftol':1e-14})
    position, angle = c.residual(pred(result.x))
    searches.append(dict(index=i, seed_counts=start.tolist(), result_counts=result.x.tolist(), position_error_m=position, orientation_error_rad=angle, success=bool(result.success), iterations=int(result.nit), function_evaluations=int(result.nfev), message=str(result.message)))
    for point in product(*[sorted(set(max(-32,min(32,v)) for v in (math.floor(x)-1, math.floor(x), math.ceil(x), math.ceil(x)+1))) for x in result.x]):
        if point in visited: continue
        visited.add(point)
        delta = np.zeros(6, dtype=int)
        delta[free] = point
        target = np.array(c.initial_counts)+delta
        command = (target-2048)*Q
        predicted = command-(sdk-q)
        try:
            fk(command)
            p, a = c.residual(predicted)
        except ValueError:
            continue
        valid = bool(np.all((target>=0)&(target<=4095)) and max(abs(command-predicted))<=.035 and p<=.006 and a<=math.radians(5) and p<initial_error[0]-1e-6 and np.any(delta))
        lattice.append(dict(delta_counts=delta.tolist(), position_error_m=p, orientation_error_rad=a, feasible=valid))
    print('seed', i, position, angle, result.success, flush=True)
best = min(lattice, key=lambda x:x['position_error_m'])
report = dict(scope='sealed_bag_offline_only', record=str(REC), failure=failure, replay_code=replay_error, exact_recorded_source_stamps=True, replay_evaluation_clock_sec=1791612246.08, replay_clock_note='Chosen inside recorded call interval, not asserted as exact original sample clock. FK/guide values do not depend on this clock.', model_file=str(model), model_sha256=fk.model_sha256, source_messages=sources, following_sample=sample, original_guide=dict(delta_counts=guide.tolist(), initial_position_error_m=initial_error[0], initial_orientation_error_rad=initial_error[1], predicted_position_error_m=guide_error[0], predicted_orientation_error_rad=guide_error[1], following_offset_max_rad=float(max(abs(sdk-q))), failed_position_check=bool(guide_error[0]>.006), failed_orientation_check=bool(guide_error[1]>math.radians(5)), failed_following_check=bool(max(abs(sdk-q))>.035), failed_progress_check=bool(guide_error[0]>=initial_error[0]-1e-6)), constrained_search=dict(method='14 fixed deterministic bounded SLSQP starts plus adjacent integer lattice; fixed Joint2/Joint5; original 32-count/6mm/5deg gates', maximum_iterations_per_seed=120, continuous=searches, quantized_evaluated=len(lattice), best_quantized=best, feasible_quantized=[x for x in lattice if x['feasible']], global_infeasibility_proven=False, path_authorized=False, physical_response_proven=False))
report['source_sha256']={str(path.relative_to(WORK)):hashlib.sha256(path.read_bytes()).hexdigest() for path in [WORK/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot'/name for name in ['pregrasp_feedforward.py','pregrasp_compensation.py','pregrasp_gateway.py','contact_sequence_precheck.py','stationary_following.py','observation_path_guard.py']]}
(OUT/'held_axes_replay.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
print('best_quantized', best, 'feasible_count', len(report['constrained_search']['feasible_quantized']), flush=True)
