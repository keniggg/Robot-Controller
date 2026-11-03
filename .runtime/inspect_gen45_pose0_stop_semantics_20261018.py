"""Replay original held predictor on exact closed G44 pre-correction numbers."""
import hashlib
import json
import math
from pathlib import Path
import socket
import sys
import time

W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V = E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
M = V/'gen44_actual_handoff_monitor_r1'
G = E/'grasp_records/20261018T062015.472638Z_df36d0775e4d4fb4a8b3baa949267488'
O = V/'gen45_pose0_stop_semantics_review_r1'
sys.path[:0] = [str(W/'src/alicia_flexible_grasp_supervisor/src'),
               str(W/'devel/lib/python3/dist-packages')]
attempts = []
class OfflineBoundary(BaseException):
    pass
class OfflineSocket(socket.socket):
    def __new__(cls, *args, **kwargs):
        attempts.append('socket'); raise OfflineBoundary('closed numeric replay only')
socket.socket = OfflineSocket
import numpy as np
import rospy
def reject(*args, **kwargs):
    attempts.append(repr(args)); raise OfflineBoundary('ROS boundary forbidden')
for name in ('init_node', 'Publisher', 'Subscriber', 'Service', 'ServiceProxy',
             'wait_for_service', 'get_param', 'set_param', 'delete_param'):
    setattr(rospy, name, reject)
from alicia_flexible_grasp.robot.held_axes_prediction import predict_held_axes_endpoint
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SerialUrdfFk, SDK_QUANTUM_RAD as Q


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    O.mkdir(exist_ok=False)
    monitor = json.loads((M/'result.json').read_text())
    assert sha(M/'result.json') == 'aa4ea542be4d44ac0e4b5a599e44841ce2782ab88f7b7771a3fe34ff91a78709'
    pp = M/'pregrasp_proposal.json'; ap = M/'pregrasp_measured_result.json'
    for p in (pp, ap): assert sha(p) == monitor['evidence_sha256'][str(p)]
    before, after = json.loads(pp.read_text()), json.loads(ap.read_text())
    config_path = G/'perception_execution_configuration.json'
    assert sha(config_path) == monitor['evidence_sha256'][str(config_path)]
    cfg = json.loads(config_path.read_text())['parameters']['/grasp']
    model = W/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
    assert sha(model) == before['model_sha256']
    sources = ['src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/'+n+'.py'
        for n in ('held_axes_prediction', 'pregrasp_feedforward', 'pregrasp_gateway',
                  'contact_sequence_precheck', 'pregrasp_compensation')]
    pins = {name: sha(W/name) for name in sources}
    release_path = V/'gen44_release_manifest.json'; release = json.loads(release_path.read_text())
    assert all(release['source_sha256'][name] == h for name, h in pins.items())
    fk = SerialUrdfFk(model.read_text(), ARM_NAMES)
    sdk = before['command_start_positions_rad']; actual = before['measured_start_positions_rad']
    goal = before['fixed_goal_pose']
    results = {}
    for allow in (True, False):
        counts, report = predict_held_axes_endpoint(fk, sdk, actual, goal,
            position_tolerance_m=cfg['unknown_measured_endpoint_position_tolerance_m'],
            orientation_tolerance_rad=math.radians(cfg['measured_endpoint_orientation_tolerance_deg']),
            allow_already_converged=allow)
        results[str(allow)] = dict(target_counts=counts.tolist(), report=report)
    noop, step = results['True']['report'], results['False']['report']
    assert noop['code'] == 'PREGRASP_CONDITIONALLY_CONVERGED'
    assert noop['steps_predicted'] == 0 and noop['proposed_delta_counts'] == [0]*6
    assert noop['predicted_positions_rad'] == actual
    assert noop['predicted_command_positions_rad'] == sdk
    assert step['code'] == 'PREGRASP_HELD_AXES_GUIDE_QUALIFIED'
    assert step['steps_predicted'] == 1 and step['proposed_delta_counts'] == before['proposed_delta_counts']
    for key in ('position_error_m', 'orientation_error_rad', 'predicted_position_error_m',
                'predicted_orientation_error_rad'):
        assert abs(step[key]-before[key]) < 1e-13, key
    assert .006 < noop['position_error_m'] <= .010 and noop['orientation_error_rad'] <= math.radians(5)
    assert after['code'] == 'PREGRASP_NO_CARTESIAN_PROGRESS'
    assert after['position_error_m'] >= noop['position_error_m']-1e-6
    flags = ('unknown_approach_held_axes_feedforward_enabled',
        'unknown_contact_cartesian_compensation_enabled', 'contact_endpoint_precision_enabled',
        'measured_endpoint_check_enabled', 'pregrasp_cartesian_compensation_enabled',
        'unknown_pregrasp_feedforward_enabled', 'unknown_pregrasp_held_axes_feedforward_enabled')
    assert all(cfg.get(k) is True for k in flags)
    actual_after = (np.asarray(after['measured_counts'])-2048)*Q
    from alicia_flexible_grasp.robot.pregrasp_compensation import pose_matrix
    def residual(q):
        goal_matrix=pose_matrix(goal); T=fk(q)
        return float(np.linalg.norm(T[:3,3]-goal_matrix[:3,3])), math.acos(float(np.clip(
            (np.trace(goal_matrix[:3,:3].T@T[:3,:3])-1)/2,-1.,1.)))
    after_position, after_angle = residual(actual_after)
    assert abs(after_position-after['position_error_m']) < 1e-13
    assert abs(after_angle-after['orientation_error_rad']) < 1e-13
    archived_inputs = O/'original_numeric_inputs.json'
    archived_inputs.write_text(json.dumps(dict(before=before,after=after,grasp_config=cfg),indent=2)+'\n')
    report = dict(analysis_completed=True, kind='exact_G44_pure_stop_semantics_replay',
        actual_experiment_result='failure_preserved', numeric_inputs_are_exact_closed_record=True,
        timestamps_or_measurements_fabricated=False,
        response_or_gateway_handler_replayed=False, source=monitor['record'],
        plan_id=before['plan_id'], model_sha256=fk.model_sha256,
        before_position_m=noop['position_error_m'], before_orientation_rad=noop['orientation_error_rad'],
        predictor_already_converged=results['True'], execution_proposal_default=results['False'],
        actual_after_position_m=after_position, actual_after_orientation_rad=after_angle,
        actual_failure_code=after['code'], actual_failure_must_not_be_relabelled_success=True,
        complete_independent_route_flags={k:cfg[k] for k in flags},
        confirmed_policy_difference='Pure prediction accepts original10mm/5deg and proposes0motion; execution attempts optional6mm precision and may fail after a submitted step.',
        minimal_direction='For unused pose0 episode and exactly complete independent-path route only, use original fresh no-motion hard acceptance and pass to original current-CAD/fresh held/contact full-path checks; retain all submitted-response failures and pose1<=6mm.',
        alternative='Retain bounded guide computation for legacy audit compatibility but abandon any unsubmitted optional proposal when already hard-satisfied on the complete route; this preserves guide-only old assertions but is more work than early fresh handoff.',
        no_future_route_pass_or_physical_success_proven=True,
        source_sha256=pins, source_unchanged=all(sha(W/p)==h for p,h in pins.items()),
        evidence_sha256={str(p):sha(p) for p in (M/'result.json',pp,ap,config_path,model,release_path,archived_inputs)},
        helper_sha256=sha(__file__), external_boundary_attempts=attempts,
        ROS_initialized=False, hardware_contacted=False, temperature_read=False,
        production_or_old_tests_modified=False, pytest_run=False, written_ns=time.time_ns())
    assert not attempts and report['source_unchanged']
    p=O/'result.json';p.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    assert json.loads(p.read_text())==report
    (O/'helper_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(result=str(p),sha256=sha(p),noop=noop['code'],step=step['proposed_delta_counts'],
        position_m=noop['position_error_m'],actual_after_m=after_position),ensure_ascii=False))


if __name__=='__main__': main()
