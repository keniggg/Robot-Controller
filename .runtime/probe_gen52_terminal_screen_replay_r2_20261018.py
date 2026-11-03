"""Fair same-process recorded-curve proof versus terminal-first benchmark."""
from pathlib import Path
from dataclasses import dataclass
from types import SimpleNamespace as NS
import ast,gzip,hashlib,json,math,os,time,xml.etree.ElementTree as ET
import numpy as np
from probe_gen52_g51_full_collision_curves_20261018 import load,plan_from_curve,digest
W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
D=V/'gen52_contact_collision_review_r1'
R=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp'
C=W/'.runtime/gen52_contact_endpoint_candidate/src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/contact_sequence_precheck.py'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    assert os.path.ismount('/media/zhuyupei/SanStand')
    env=dict(__name__='__main__',np=np,math=math,time=time,hashlib=hashlib,ET=ET,dataclass=dataclass,
        stamp_nanoseconds=lambda t:t.to_nsec(),wire_digest=digest)
    geom=R/'grasp/gripper_geometry.py';guard=R/'robot/observation_path_guard.py'
    for n in ast.parse(geom.read_text()).body:
        if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id.startswith('ANALYTICAL_') for t in n.targets):
            exec(compile(ast.Module(body=[n],type_ignores=[]),str(geom),'exec'),env)
    load(geom,{'_readonly_vector','_finite_number','GripperGeometry','parse_tool_axis','_validated_rotation','gripper_box_centers','_stage_boxes','_box_corners','_obb_overlap'},env)
    load(guard,{'ObservationPathError','FrozenObservationScene','_array','_axis_rotation','SerialUrdfFk','segment_coefficients','_polynomial','_polynomial_bounds','_derivative_bound','validate_observation_trajectory'},env)
    load(C,{'contact_curve_terminal_collision'},env)
    fixture=C.parents[4]/'tests/fixtures/gen52_g51_contact_collision_inputs.json'
    # The supervisor directory is an explicit local source relation.
    fixture=W/'.runtime/gen52_contact_endpoint_candidate/src/alicia_flexible_grasp_supervisor/tests/fixtures/gen52_g51_contact_collision_inputs.json'
    records=json.loads(fixture.read_text())['cases'];rows=[]
    for d in records:
        fk=env['SerialUrdfFk'](d['model']['xml'],d['following_sample']['joint_names'])
        scene=env['FrozenObservationScene'](**{k:np.array(v) if k in ('center','normal','rotation','size') else v for k,v in d['scene'].items()})
        g=env['GripperGeometry'](**{k:np.array(v) if isinstance(v,list) else v for k,v in d['config']['evaluated_gripper'].items()})
        s=d['following_sample'];bias=np.array(s['sdk_positions_rad'])-s['accepted_positions_rad']
        curves=[('nominal',plan_from_curve(d['checked_curve'],bias)),('predicted_command',plan_from_curve(d['checked_curve']))]
        trials=[]
        for variant in ('original_full_pair','candidate_terminal_pair'):
            began=time.monotonic();row=dict(variant=variant,checked=[])
            for label,curve in curves:
                try:
                    if variant=='candidate_terminal_pair':
                        result=env['contact_curve_terminal_collision'](curve,fk,scene,g,d['opening_m'])
                    else:
                        hold=NS(positions=curve.joint_trajectory.points[0].positions,velocities=[0.]*6,accelerations=[0.]*6)
                        result=env['validate_observation_trajectory'](curve,scene,fk,g,d['opening_m'],hold)
                except env['ObservationPathError'] as exc:
                    row.update(rejected=True,curve=label,reason=str(exc),sample=getattr(exc,'contact_replay_sample',None));break
                else:row['checked'].append(label)
            row['elapsed_sec']=time.monotonic()-began;trials.append(row)
        assert all(t['rejected'] and t['curve']=='predicted_command' for t in trials)
        assert trials[1]['sample']['first_collision_along_curve_claimed'] is False
        rows.append(dict(plan_id=d['binding']['plan_id'],trials=trials,ratio_full_to_screen=trials[0]['elapsed_sec']/trials[1]['elapsed_sec']))
    out=dict(passed=True,scope='three_exact_same_branch_recorded_curve_rejections_only',rows=rows,
        candidate_sha256=sha(C),fixture_sha256=sha(fixture),source_sha256={str(p):sha(p) for p in [geom,guard,Path(__file__)]},
        elapsed_excludes=['model and scene hydration','original preceding selector and MoveIt/whole-arm stages','new ROS calls or hardware'],
        full_task_speedup_claimed=False,all_ik_or_candidates_rejected=False,other_ik_branches_still_require_full_check=True,
        live_source_changed=False,tests_or_ROS_or_hardware_run=False,temperature_read=False)
    p=D/'terminal_screen_original_replay_r2.json'
    with p.open('x') as f:json.dump(out,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(dict(path=str(p),sha256=sha(p),rows=[dict(plan=r['plan_id'],original_sec=r['trials'][0]['elapsed_sec'],screen_sec=r['trials'][1]['elapsed_sec'],ratio=r['ratio_full_to_screen']) for r in rows])))
if __name__=='__main__':main()
