#!/usr/bin/env python3
"""Compare sealed precheck virtual endpoints with the same frozen aperture."""
import base64
import hashlib
import importlib.util
import json
from pathlib import Path
import time

W=Path(__file__).resolve().parent.parent
helper=W/'.runtime/probe_gen43_bounded_aperture_feasibility_20261018.py'
spec=importlib.util.spec_from_file_location('sealed_bounded_probe',helper)
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
OUT=p.N/'verification/gen43_precheck_stage0_aperture_replay_r1'
OUT.mkdir(exist_ok=False)
def sha(f):return hashlib.sha256(Path(f).read_bytes()).hexdigest()
def put(name,value):
    f=OUT/name;f.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return dict(path=str(f),sha256=sha(f))
wire=json.loads((p.INPUT/'frozen_plan.json').read_text())
raw=base64.b64decode(wire['wire_base64']);assert hashlib.sha256(raw).hexdigest()==wire['wire_sha256']
plan=p.Grasp6DPlan();plan.deserialize(raw);assert p.plan_id_matches_content(plan)
fk=p.SerialUrdfFk(p.URDF.read_text(),p.ARM_NAMES)
runtime=json.loads((p.INPUT/'runtime_compensation_report.json').read_text())
assert fk.model_sha256==runtime['model_sha256']
evaluate=p.plan_aperture_evaluator(plan,{},.04980000000000001,0)
log=p.N/'gen43_runtime/compute_gateway_deployment_r1/consumers.log'
matches=[]
for line in log.read_text().splitlines():
    if 'remote 6D contact precheck delivery: ' not in line:continue
    value=json.loads(line.split('remote 6D contact precheck delivery: ',1)[1])
    if value['plan_id']==plan.plan_id:matches.append((line,value))
assert len(matches)==3
line,delivery=matches[0]
precheck=delivery['precheck'];selection=precheck['held_pregrasp_selection'];pred=selection['predicted_pregrasp']
source=put('original_precheck.json',delivery)
cases={
 'precheck_virtual_pregrasp':pred['predicted_positions_rad'],
 'original_measured_before_correction':runtime['measured_start_positions_rad'],
}
results={label:evaluate(fk(q))for label,q in cases.items()}
observed_sdk=p.np.array(runtime['command_start_positions_rad']);planned_sdk=p.np.array(selection['selected_command_rad'])
planned_bias=p.np.array(precheck['source_sample']['sdk_positions_rad'])-p.np.array(precheck['source_sample']['accepted_positions_rad'])
actual_bias=p.np.array(runtime['sdk_minus_encoder_rad'])
report=dict(kind='gen43_original_precheck_stage0_aperture_replay',completed_ns=time.time_ns(),
    plan_id=plan.plan_id,exact_original_precheck=source,matched_precheck_reports=len(matches),
    precheck_prediction_cache_hit=precheck['prediction_cache_hit'],
    virtual_pregrasp_is_not_measured=True,actual_measured_initial_only=True,
    nominal_selected_pregrasp_command_counts=p.np.rint(planned_sdk/p.Q+2048).astype(int).tolist(),
    actual_reached_pregrasp_command_counts=p.np.rint(observed_sdk/p.Q+2048).astype(int).tolist(),
    actual_minus_prechecked_pregrasp_command_counts=p.np.rint((observed_sdk-planned_sdk)/p.Q).astype(int).tolist(),
    planning_source_sdk_minus_accepted_counts=(planned_bias/p.Q).tolist(),
    measured_pregrasp_sdk_minus_accepted_counts=(actual_bias/p.Q).tolist(),
    planning_vs_execution_joint_endpoint_difference_is_not_alone_a_bug_proof=True,
    aperture_results=results,model_sha256=fk.model_sha256,
    precheck_stage0_pose_prediction_m=pred['predicted_position_error_m'],
    precheck_stage0_pose_prediction_deg=p.math.degrees(pred['predicted_orientation_error_rad']),
    source_sha256={str(helper):sha(helper),str(p.URDF):sha(p.URDF)},
    actual_fresh_response_or_path_success_claimed=False,production_or_test_modified=False,
    network_boundary_attempts=p.boundary,ros_init_called=False,temperature_read=False,
    motion_authorized=False,helper_sha256=sha(Path(__file__)))
assert not p.boundary
ref=put('result.json',report)
put('seal.json',dict(result=ref,precheck=source))
print(json.dumps(dict(result=ref,command_difference=report['actual_minus_prechecked_pregrasp_command_counts'],
    results={k:{s:v.get(s)for s in ['ok','code','reason','required_open_width_m','aperture_margin_m',
        'actual_current_minimum_support_clearance_m','projected_contact_minimum_support_clearance_m']}for k,v in results.items()}),ensure_ascii=False))
