"""Offline original final sample; predicted steps never authorize movement."""
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import rosbag
from alicia_flexible_grasp.grasp.contact_execution import plan_contact_evaluator
from alicia_flexible_grasp.grasp.contact_height_margin import contact_height_evidence_limit
from alicia_flexible_grasp.robot.pregrasp_compensation import PregraspCompensation
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES, SDK_QUANTUM_RAD as Q, SerialUrdfFk

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_support_fixed_20261006T040304Z/contact_margin_replay')

def main():
    old=json.loads((ROOT/'original_final_compensation_report.json').read_text())
    config=json.loads((ROOT/'current_runtime_parameters.json').read_text())['parameters']
    with rosbag.Bag(str(ROOT/'original_contact_plan.bag')) as b:
        plan=next(b.read_messages())[1]
    fk=SerialUrdfFk(config['/robot_description'], ARM_NAMES)
    assert fk.model_sha256==old['model_sha256']
    measured=(np.asarray(old['measured_counts'])-2048)*Q
    sdk=(np.asarray(old['sdk_counts'])-2048)*Q
    sample=dict(measurement_kind='stationary_sdk_following_not_executed_endpoint',
        model_sha256=fk.model_sha256,epoch_ns=old['epoch_ns'],joint_names=ARM_NAMES,
        stamp_sec=old['accepted_stamp_ns']*1e-9+.05,
        sdk_stamp_ns=old['sdk_stamp_ns'],accepted_stamp_ns=old['accepted_stamp_ns'],
        stationary_window_start_ns=old['accepted_stamp_ns']-400_000_000,
        stationary_window_end_ns=old['accepted_stamp_ns'],
        accepted_positions_rad=measured.tolist(),sdk_positions_rad=sdk.tolist())
    evaluator=plan_contact_evaluator(plan,config['/grasp_6d/remote/gripper_geometry'],.04975,
        translation_evidence_limit_m=contact_height_evidence_limit(config['/grasp']))
    initial=evaluator(fk(measured))
    c=PregraspCompensation(fk,sample,old['fixed_goal_pose'],position_tolerance_m=.010,
                          endpoint_acceptance=evaluator)
    start=time.monotonic()
    report=dict(plan_id=plan.plan_id,source='original recorded SDK and measured counts',
        stationary_window_semantics='synthetic offline wrapper of already proven static sample; no ROS observation or execution authority',
        initial=initial,motion_authorized=False,physical_grasp_verified=False)
    try:
        code,step,evidence=c.evaluate(sample)
        report.update(code=code,step=None if step is None else dict(target_counts=list(step.target_counts),
            baseline_counts=list(step.baseline_counts)),evidence=evidence)
    except Exception as exc:
        report.update(error=repr(exc),evidence=c.last_evidence)
    report['evaluation_wall_sec']=time.monotonic()-start
    hypothetical=[]
    if report.get('code')=='PREGRASP_STEP_PROPOSED':
        # Explicit ideal directional-response model only. These are never
        # presented as new hardware feedback or sent to the gateway.
        for index in range(c.MAX_STEPS):
            delta=np.asarray(step.target_counts)-step.baseline_counts
            c.committed(step,sample['stamp_sec']+2.)
            measured=measured+delta*Q
            sample=dict(sample,stamp_sec=sample['stamp_sec']+4.,
                sdk_stamp_ns=sample['sdk_stamp_ns']+4_000_000_000,
                accepted_stamp_ns=sample['accepted_stamp_ns']+4_000_000_000,
                stationary_window_start_ns=sample['stationary_window_start_ns']+4_000_000_000,
                stationary_window_end_ns=sample['stationary_window_end_ns']+4_000_000_000,
                sdk_positions_rad=list(step.positions),accepted_positions_rad=measured.tolist())
            tick=time.monotonic()
            try:
                code,step,evidence=c.evaluate(sample)
                hypothetical.append(dict(code=code,evaluation_wall_sec=time.monotonic()-tick,evidence=evidence))
                if step is None:break
            except Exception as exc:
                hypothetical.append(dict(error=repr(exc),evidence=c.last_evidence));break
    report['hypothetical_ideal_response_only']=hypothetical
    report['script_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    output=ROOT/('replay_'+time.strftime('%Y%m%dT%H%M%SZ',time.gmtime())+'.json')
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),code=report.get('code'),error=report.get('error'),
        elapsed=report['evaluation_wall_sec'],hypothetical_final_code=(hypothetical[-1].get('code',hypothetical[-1].get('error')) if hypothetical else None),
        hypothetical_steps=len(hypothetical),initial_height=initial['height_registration_margin']['maximum_symmetric_normal_translation_m'],
        proposed_delta_counts=report['evidence'].get('proposed_delta_counts'),
        predicted_height=report['evidence'].get('predicted_contact',{}).get('height_registration_margin',{}).get('maximum_symmetric_normal_translation_m')),ensure_ascii=False),flush=True)

if __name__=='__main__':main()
