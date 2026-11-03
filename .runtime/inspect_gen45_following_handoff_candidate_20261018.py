"""One original held-approach prediction after the G44 counterfactual no-op."""
from inspect_gen45_pose0_stop_semantics_20261018 import *
from alicia_flexible_grasp.robot.held_axes_prediction import HeldAxesPredictionError


def main():
    O2 = V/'gen45_following_handoff_prediction_r1'
    O2.mkdir(exist_ok=False)
    pp = M/'pregrasp_proposal.json'
    before = json.loads(pp.read_text())
    audit_path = G/'keyframes_audits/737d88b523bc7f98dd6e16ab1247ae71cc74d43f9984b9c7cf7527d173118d8e.json'
    audit = json.loads(audit_path.read_text())
    assert audit['plan_id'] == before['plan_id']
    stages = audit['selected']['execution_sequence']['stages']
    assert stages[0]['position_m']+stages[0]['quaternion_xyzw'] == before['fixed_goal_pose']
    stage = stages[1]
    assert stage['stage'] == 'approach'
    goal = stage['position_m']+stage['quaternion_xyzw']
    model = W/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf'
    assert sha(model) == before['model_sha256']
    fk = SerialUrdfFk(model.read_text(), ARM_NAMES)
    try:
        target, report = predict_held_axes_endpoint(fk, before['command_start_positions_rad'],
            before['measured_start_positions_rad'], goal, position_tolerance_m=.006,
            orientation_tolerance_rad=math.radians(5), allow_already_converged=True)
        qualified = True
    except HeldAxesPredictionError as exc:
        report = exc.evidence; qualified = False
    result = dict(analysis_completed=True, original_guide_only=True,
        counterfactual_scope='next held-approach guide from unchanged original G44 pre-correction start; not an actual future command or measurement',
        plan_id=before['plan_id'], held_endpoint_prediction_qualified=qualified,
        endpoint_prediction=report, complete_path_checked=False,
        hardware_response_verified=False, execution_authorized=False,
        original_preflight_conditional_pass=audit['selected']['contact_sequence_precheck']['success'],
        source_same_as_original_preflight=False,
        no_global_infeasibility_claim=True, no_new_budget_or_search_algorithm=True,
        evidence_sha256={str(p):sha(p) for p in (pp,audit_path,model,V/'gen45_pose0_stop_semantics_review_r1/result.json')},
        helper_sha256=sha(__file__), external_boundary_attempts=attempts,
        production_or_test_modified=False, ROS_initialized=False, hardware_contacted=False,
        temperature_read=False, pytest_run=False)
    assert not attempts
    p=O2/'result.json';p.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    assert json.loads(p.read_text())==result
    (O2/'helper_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(result=str(p),sha256=sha(p),qualified=qualified,
        code=report['code'],position_m=report.get('predicted_position_error_m'),
        delta=report.get('proposed_delta_counts')),ensure_ascii=False))


if __name__=='__main__':main()
