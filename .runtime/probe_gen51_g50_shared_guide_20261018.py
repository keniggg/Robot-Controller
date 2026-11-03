"""Original four recorded selector calls, pure AST code only; no ROS/planner."""
from pathlib import Path
from copy import deepcopy
from dataclasses import dataclass
from itertools import combinations,product
import ast,hashlib,json,math,os,time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import minimize
from review_gen51_g50_strict_records_20261018 import pose_matrix
W=Path(__file__).resolve().parents[1]
ROOT=Path('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp')
C=W/'.runtime/gen51_held_guide_candidate'
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
OUT=V/'gen51_g50_shared_guide_probe_r2'
Q=2*math.pi/4096
ARM_NAMES=tuple('Joint%d'%i for i in range(1,7))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,x):
    with p.open('x') as f:json.dump(x,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
def load_nodes(path,names,env):
    body=[n for n in ast.parse(path.read_text()).body if isinstance(n,(ast.ClassDef,ast.FunctionDef)) and n.name in names]
    assert {n.name for n in body}==set(names)
    exec(compile(ast.Module(body=body,type_ignores=[]),str(path),'exec'),env)
def environment(candidate):
    env=dict(__name__='gen51_pure_shared_guide_probe',np=np,math=math,time=time,
       hashlib=hashlib,ET=ET,ObservationPathError=RuntimeError,deepcopy=deepcopy,
       dataclass=dataclass,combinations=combinations,product=product,minimize=minimize,
       Q=Q,SDK_QUANTUM_RAD=Q,ARM_NAMES=ARM_NAMES,pose_matrix=pose_matrix)
    load_nodes(W/ROOT/'robot/observation_path_guard.py',{'_array','_axis_rotation','SerialUrdfFk'},env)
    load_nodes(W/ROOT/'robot/endpoint_correction.py',{'sdk_counts','CorrectionStep'},env)
    load_nodes(Path('/opt/ros/noetic/lib/python3/dist-packages/tf/transformations.py'),
       {'quaternion_matrix'}, dict_env := dict(numpy=np, math=math, _EPS=np.finfo(float).eps*4.0))
    env['quaternion_matrix']=dict_env['quaternion_matrix']
    path=(C if candidate else W)/ROOT/'robot/pregrasp_compensation.py'
    names={'pose_matrix','remaining_budget_guide','PregraspCompensation'}
    if candidate:names.add('_normalized_remaining_budget_guide')
    load_nodes(path,names,env)
    load_nodes(W/ROOT/'robot/held_axes_prediction.py',{'HeldAxesPredictionError','predict_held_axes_endpoint'},env)
    load_nodes(W/ROOT/'robot/held_pregrasp_selection.py',{'select_held_pregrasp_endpoint'},env)
    load_nodes(W/ROOT/'robot/pregrasp_feedforward.py',{'PregraspFeedforward'},env)
    return env

def main():
    OUT.mkdir(exist_ok=False)
    fixture=C/'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen51_g50_held_guide_inputs.json'
    data=json.loads(fixture.read_text());results=[]
    for variant in ('baseline','candidate'):
        env=environment(variant=='candidate');fk=env['SerialUrdfFk'](data['model_xml'],ARM_NAMES)
        assert fk.model_sha256==data['model_sha256']
        for case in data['cases']:
            before=deepcopy(case);began=time.monotonic();row=dict(variant=variant,plan_id=case['plan_id'])
            try:
                command,report=env['select_held_pregrasp_endpoint'](fk,case['source_sample'],
                    case['pregrasp_goal'],case['approach_goal'],case['nominal_pregrasp_command_rad'],
                    position_tolerance_m=.010,orientation_tolerance_rad=math.radians(5))
            except env['HeldAxesPredictionError'] as exc:
                row.update(qualified=False,code=str(exc),evidence=exc.evidence)
            else:
                # Explicitly synthetic future provider, never a historical
                # encoder record: test the unchanged execution method against
                # exactly the predictor's conditional pregrasp endpoint.
                predicted_pre=report['predicted_pregrasp']
                sample=deepcopy(case['source_sample'])
                sample['sdk_positions_rad']=predicted_pre['predicted_command_positions_rad']
                sample['accepted_positions_rad']=predicted_pre['predicted_positions_rad']
                contract=env['PregraspCompensation'](fk,sample,case['approach_goal'],
                    position_tolerance_m=.006,orientation_tolerance_rad=math.radians(5))
                actual=env['PregraspFeedforward'].__new__(env['PregraspFeedforward'])
                actual.contract=contract;actual.initial=deepcopy(sample)
                actual.step=actual.completed=None;actual.finished=False;actual.evidence={}
                step,execution=actual.propose_held_axes()
                prediction=report['predicted_approach']
                target=tuple(env['sdk_counts'](prediction['predicted_command_positions_rad']))
                assert tuple(step.target_counts)==target
                assert execution['proposed_delta_counts']==prediction['proposed_delta_counts']
                row.update(qualified=True,selected_command=command.tolist(),report=report,
                    execution_method_same_counts=True,execution_method_evidence=execution,
                    future_sample_is_synthetic_provider=True,full_path_checked=False)
            assert case==before
            row['elapsed_sec']=time.monotonic()-began
            write(OUT/('%s_%s.json'%(variant,case['plan_id'])),row);results.append(row)
            print(json.dumps(dict(variant=variant,plan=case['plan_id'],qualified=row['qualified'],
                code=row.get('code'),elapsed=row['elapsed_sec']),ensure_ascii=False),flush=True)
    result=dict(kind='GEN51_original_four_selector_shared_guide_probe',created_ns=time.time_ns(),
        baseline_failed=sum(not r['qualified'] for r in results if r['variant']=='baseline'),
        candidate_qualified=sum(r['qualified'] for r in results if r['variant']=='candidate'),
        candidate_execution_proposal_same_counts=sum(r.get('execution_method_same_counts',False)
             for r in results if r['variant']=='candidate'),expected_original_cases=4,
        fixture_sha256=sha(fixture),candidate_source_sha256=sha(C/ROOT/'robot/pregrasp_compensation.py'),
        original_source_sha256=sha(W/ROOT/'robot/pregrasp_compensation.py'),
        helper_sha256=sha(Path(__file__)),
        limitations=['Only endpoint selector/predictor and actual propose_held_axes method arithmetic were run.',
        'The future execution sample uses a synthetic provider; its positions are explicit conditional predictions, not historical measured feedback.',
        'No MoveIt or complete command/actual geometry path, SDK response, fresh physical arrival, task success or motion authority is asserted.'],
        ROS_initialized=False,hardware_contacted=False,temperature_read=False,production_or_tests_changed=False)
    result['passed']=result['baseline_failed']==result['candidate_qualified']==result['candidate_execution_proposal_same_counts']==4
    write(OUT/'result.json',result);print(json.dumps(result,ensure_ascii=False),flush=True)
if __name__=='__main__':main()
