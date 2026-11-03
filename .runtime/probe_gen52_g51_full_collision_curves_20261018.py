"""Original continuous quintic CAD gate on exact stored G51 curves, no ROS."""
from pathlib import Path
from types import SimpleNamespace as NS
from dataclasses import dataclass
from copy import deepcopy
import ast, hashlib, json, gzip, math, os, time
import xml.etree.ElementTree as ET
import numpy as np

W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
D=V/'gen52_contact_collision_review_r1'
R=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):
    with p.open('x') as f:json.dump(v,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    assert json.loads(p.read_text())==v
def load(p,names,env):
    nodes=[n for n in ast.parse(p.read_text()).body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names]
    assert {n.name for n in nodes}==set(names)
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(p),'exec'),env)
class Duration:
    def __init__(self,ns):self.ns=ns
    def to_sec(self):return self.ns/1e9
    def to_nsec(self):return self.ns
def plan_from_curve(c,offset=None):
    pts=[]
    for r in c['points']:
        pos=np.array(r['positions'])
        if offset is not None:pos=pos-offset
        pts.append(NS(positions=pos.tolist(),velocities=r['velocities'],accelerations=r['accelerations'],time_from_start=Duration(r['time_from_start_ns'])))
    return NS(joint_trajectory=NS(header=NS(stamp=Duration(0)),joint_names=c['joint_names'],points=pts),multi_dof_joint_trajectory=NS(points=[]))
def digest(plan):
    # Metadata-only adapter. The gate never uses this digest for admission.
    value=[dict(positions=p.positions,velocities=p.velocities,accelerations=p.accelerations,time_ns=p.time_from_start.to_nsec()) for p in plan.joint_trajectory.points]
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def main():
    assert os.path.ismount('/media/zhuyupei/SanStand')
    env=dict(__name__='__main__',np=np,math=math,time=time,hashlib=hashlib,ET=ET,dataclass=dataclass,
        stamp_nanoseconds=lambda stamp:stamp.to_nsec(),wire_digest=digest)
    g=R/'grasp/gripper_geometry.py';p=R/'robot/observation_path_guard.py'
    constants={'ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M','ANALYTICAL_PALM_CENTER_TOOL_XYZ_M','ANALYTICAL_FINGER_BOX_PADDING_XYZ_M','ANALYTICAL_MAX_INNER_GAP_M'}
    for n in ast.parse(g.read_text()).body:
        if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in constants for t in n.targets):
            exec(compile(ast.Module(body=[n],type_ignores=[]),str(g),'exec'),env)
    load(g,{'_readonly_vector','_finite_number','GripperGeometry','parse_tool_axis','_validated_rotation','gripper_box_centers','_stage_boxes','_box_corners','_obb_overlap'},env)
    load(p,{'ObservationPathError','_array','_axis_rotation','SerialUrdfFk','segment_coefficients','_polynomial','_polynomial_bounds','_derivative_bound','validate_observation_trajectory'},env)
    refs_path=V/'gen51_actual_handoff_monitor_r1/terminal_and_completed_diagnostics.json'
    refs=json.loads(refs_path.read_text())['unique_failure_replay_refs'][:3]
    rows=[]
    for item in refs:
        ref=item['reference'];f=Path(ref['store_root'])/ref['relative_path'];raw=gzip.decompress(f.read_bytes())
        assert sha(f)==ref['gzip_sha256'] and hashlib.sha256(raw).hexdigest()==ref['object_sha256']
        d=json.loads(raw);fk=env['SerialUrdfFk'](d['model']['xml'],d['following_sample']['joint_names'])
        s=d['scene'];scene=NS(**{k:np.array(v) if k in ['center','normal','rotation','size'] else v for k,v in s.items()})
        gripper=env['GripperGeometry'](**{k:np.array(v) if isinstance(v,list) else v for k,v in d['config']['evaluated_gripper'].items()})
        sample=d['following_sample'];bias=np.array(sample['sdk_positions_rad'])-sample['accepted_positions_rad']
        for kind in ('original_command','constant_bias_actual'):
            original=d['checked_curve']; offset=None if kind=='original_command' else bias
            curve=plan_from_curve(original,offset);h=d['controller_desired']
            hold=NS(positions=(np.array(h['positions'])-(bias if offset is not None else 0)).tolist(),velocities=h['velocities'],accelerations=h['accelerations'])
            started=time.monotonic();row=dict(plan_id=d['binding']['plan_id'],curve=kind,original_failure_object_sha256=ref['object_sha256'],adapter_geometry_sha256=digest(curve))
            try:report=env['validate_observation_trajectory'](curve,scene,fk,gripper,d['opening_m'],hold,**d['first_failed_sample']['checker_options'])
            except env['ObservationPathError'] as exc:
                row.update(passed=False,reason=str(exc),first_failed_sample=getattr(exc,'contact_replay_sample',None))
            else:row.update(passed=True,report=report)
            row['elapsed_sec']=time.monotonic()-started;rows.append(row)
    result=dict(passed=all(r['passed'] for r in rows if r['curve']=='constant_bias_actual') and all(not r['passed'] for r in rows if r['curve']=='original_command'),
       scope='same_original_continuous_geometry_gate_on_sealed_curves',rows=rows,
       reviewed_source_sha256={str(x):sha(x) for x in [g,p,refs_path,Path(__file__)]},
       notes=['Original AST continuous checker, FK, CAD boxes, spline coefficients and interval enclosure are unchanged.',
       'Adapters supply recorded trajectory values without ROS. wire_digest is metadata-only canonical curve digest, not ROS serialization.',
       'Actual curve equals original actuator-reference curve minus one original stationary observed bias at all points and hold; this is conditional, not a measured future trace.',
       'These are full continuous gripper CAD/target/support proofs for the three recorded grasp curves only. No MoveIt whole-arm validity, actual tracking, current feedback, other-stage proof, or execution permission is asserted.'],
       tests_or_ROS_or_hardware_run=False,live_source_modified=False,temperature_read=False)
    write(D/'continuous_curve_probe.json',result)
    print(json.dumps(dict(path=str(D/'continuous_curve_probe.json'),sha256=sha(D/'continuous_curve_probe.json'),passed=result['passed'],rows=[{k:r[k] for k in ['plan_id','curve','passed','elapsed_sec']}|({'reason':r['reason']} if 'reason'in r else {'clearance':r['report']['certified_minimum_clearance_m']}) for r in rows])))
if __name__=='__main__':main()
