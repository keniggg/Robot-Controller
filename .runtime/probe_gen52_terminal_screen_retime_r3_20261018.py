"""Original validate_contact_prediction versus r3, exact sealed curve values."""
from pathlib import Path
from dataclasses import dataclass
from types import SimpleNamespace as NS
from copy import deepcopy
import ast,hashlib,json,math,os,time,xml.etree.ElementTree as ET
import numpy as np
from probe_gen52_g51_full_collision_curves_20261018 import load,plan_from_curve,digest
from review_gen51_g50_strict_records_20261018 import pose_matrix
W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
D=V/'gen52_contact_collision_review_r1'
R=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp'
REL=Path('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py')
C=W/'.runtime/gen52_contact_endpoint_candidate'/REL
B=W/'.runtime/gen52_baseline_source'/REL
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    assert os.path.ismount('/media/zhuyupei/SanStand')
    env=dict(__name__='__main__',np=np,math=math,time=time,hashlib=hashlib,ET=ET,dataclass=dataclass,
        stamp_nanoseconds=lambda t:t.to_nsec(),wire_digest=digest,deepcopy=deepcopy,pose_matrix=pose_matrix,
        pose_values=lambda p:p,JointTrajectoryPoint=NS,ARM_NAMES=tuple('Joint%d'%i for i in range(1,7)))
    geom=R/'grasp/gripper_geometry.py';guard=R/'robot/observation_path_guard.py';linear=R/'robot/contact_linear_compensation.py'
    for n in ast.parse(geom.read_text()).body:
        if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id.startswith('ANALYTICAL_') for t in n.targets):exec(compile(ast.Module(body=[n],type_ignores=[]),str(geom),'exec'),env)
    load(geom,{'_readonly_vector','_finite_number','GripperGeometry','parse_tool_axis','_validated_rotation','gripper_box_centers','_stage_boxes','_box_corners','_obb_overlap'},env)
    load(guard,{'ObservationPathError','FrozenObservationScene','_array','_axis_rotation','SerialUrdfFk','segment_coefficients','_polynomial','_polynomial_bounds','_derivative_bound','validate_observation_trajectory'},env)
    load(linear,{'compensate_cartesian_path'},env)
    load(C,{'ContactPrecheckError','predict_contact_command_path','_emit_contact_replay_failure','contact_curve_terminal_collision'},env)
    baseline=dict(env);candidate=dict(env)
    load(B,{'validate_contact_prediction'},baseline);load(C,{'validate_contact_prediction'},candidate)
    fixture=W/'.runtime/gen52_contact_endpoint_candidate/src/alicia_flexible_grasp_supervisor/tests/fixtures/gen52_g51_contact_collision_inputs.json'
    records=json.loads(fixture.read_text())['cases'];rows=[]
    for d in records:
        fk=env['SerialUrdfFk'](d['model']['xml'],d['following_sample']['joint_names'])
        scene=env['FrozenObservationScene'](**{k:np.array(v) if k in ('center','normal','rotation','size') else v for k,v in d['scene'].items()})
        g=env['GripperGeometry'](**{k:np.array(v) if isinstance(v,list) else v for k,v in d['config']['evaluated_gripper'].items()})
        s=d['following_sample'];bias=np.array(s['sdk_positions_rad'])-s['accepted_positions_rad'];trials=[]
        for variant,scope in [('baseline',baseline),('candidate_r3',candidate)]:
            counts=dict(retime=0,full_continuous=0,remaining=0,whole_arm=0);received=[]
            original=scope['validate_observation_trajectory']
            def full(*a,**k):counts['full_continuous']+=1;return original(*a,**k)
            scope['validate_observation_trajectory']=full
            def retime(curve):counts['retime']+=1;curve.joint_trajectory.header.seq=22;return curve
            planner=NS(_retime_cartesian_plan=retime,_same_joint_path_geometry=lambda a,b:[p.positions for p in a.joint_trajectory.points]==[p.positions for p in b.joint_trajectory.points])
            def remaining():counts['remaining']+=1;return 5.
            def collision(*a,**k):counts['whole_arm']+=1;raise AssertionError('no whole-arm service after collision')
            def sink(error,curve,hold,kind,stage):
                received.append(dict(curve_header_seq=curve.joint_trajectory.header.seq,curve=kind,stage=stage,terminal_time_ns=curve.joint_trajectory.points[-1].time_from_start.to_nsec(),hold_positions=hold.positions,sample=error.contact_replay_sample));return {'status':'captured','diagnostic_only':True,'execution_authorized':False}
            nominal=plan_from_curve(d['checked_curve'],bias);nominal.joint_trajectory.header.seq=0;start=time.monotonic()
            try:scope['validate_contact_prediction'](nominal,fk,s,d['plan_data']['poses_xyz_xyzw'][2],scene,g,d['opening_m'],planner,NS(),collision,remaining,failure_sink=sink)
            except scope['ContactPrecheckError'] as exc:row=dict(variant=variant,rejected=True,reason=str(exc),elapsed_sec=time.monotonic()-start,counters=counts,receipt=received)
            else:raise AssertionError('original collision was not rejected')
            scope['validate_observation_trajectory']=original
            assert row['reason'].startswith('predicted_command continuous geometry: target OBB collision') and len(received)==1 and received[0]['curve_header_seq']==22
            if variant=='candidate_r3':assert counts['retime']==1 and counts['full_continuous']==0 and received[0]['sample']['continuous_validation_executed'] is False
            else:assert counts['retime']==2 and counts['full_continuous']==2
            trials.append(row)
        rows.append(dict(plan_id=d['binding']['plan_id'],trials=trials))
    result=dict(passed=True,scope='actual_precheck_function_three_original_curves_retimed_passive_receipts',rows=rows,candidate_sha256=sha(C),fixture_sha256=sha(fixture),baseline_sha256=sha(B),source_sha256={str(p):sha(p) for p in [geom,guard,linear,Path(__file__)]},limitations=['Metadata/Duration/Pose adapters only; no ROS imports or transport. Same original FK/CAD/SAT/quintic functions and exact curve values are compiled from frozen source.','Retiming provider preserves all curve values and marks header seq22, explicitly testing passive replay contract. Actual complete MoveIt timing and preceding selector stages are outside this probe.','Guarded real-module pytest remains root responsibility; no physical candidate, motion authority or global reachability claim.'],tests_or_ROS_or_hardware_run=False,live_source_modified=False,temperature_read=False)
    p=D/'terminal_screen_original_replay_r3.json'
    with p.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(dict(path=str(p),sha256=sha(p),rows=[dict(plan=r['plan_id'],baseline_sec=r['trials'][0]['elapsed_sec'],candidate_sec=r['trials'][1]['elapsed_sec'],candidate_retime_count=r['trials'][1]['counters']['retime']) for r in rows])))
if __name__=='__main__':main()
