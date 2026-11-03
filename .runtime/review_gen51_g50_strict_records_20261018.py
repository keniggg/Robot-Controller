"""Small deterministic arithmetic on sealed G50 records; no ROS/planning."""
from pathlib import Path
import ast, base64, gzip, hashlib, json, math, os, time
import xml.etree.ElementTree as ET
import zlib
import numpy as np
W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
OUT=V/'gen51_g50_strict_algorithm_review_r1'
ROOT=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp'
Q=2*math.pi/4096

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p): return json.loads(Path(p).read_text())
def write(p,x):
    with Path(p).open('x') as f:
        json.dump(x,f,ensure_ascii=False,indent=2,allow_nan=False)
        f.write('\n');f.flush();os.fsync(f.fileno())
    assert read(p)==x

def pose_matrix(v):
    x,y,z,qx,qy,qz,qw=v
    q=np.array([qx,qy,qz,qw],float);q/=np.linalg.norm(q);qx,qy,qz,qw=q
    t=np.eye(4);t[:3,3]=[x,y,z]
    t[:3,:3]=[[1-2*(qy*qy+qz*qz),2*(qx*qy-qz*qw),2*(qx*qz+qy*qw)],
              [2*(qx*qy+qz*qw),1-2*(qx*qx+qz*qz),2*(qy*qz-qx*qw)],
              [2*(qx*qz-qy*qw),2*(qy*qz+qx*qw),1-2*(qx*qx+qy*qy)]]
    return t

def errors(fk,q,goal):
    a=fk(q);t=pose_matrix(goal)
    return dict(position_m=float(np.linalg.norm(a[:3,3]-t[:3,3])),
      orientation_rad=math.acos(float(np.clip((np.trace(a[:3,:3].T@t[:3,:3])-1)/2,-1,1))))

def sat_margins(ca,ra,sa,cb,rb,sb):
    axes=[('body%d'%i,ra[:,i]) for i in range(3)]+[('target%d'%i,rb[:,i]) for i in range(3)]
    for i in range(3):
        for j in range(3):
            x=np.cross(ra[:,i],rb[:,j]);n=np.linalg.norm(x)
            if n>1e-9:axes.append(('cross%d%d'%(i,j),x/n))
    values=[]
    for label,axis in axes:
        separation=abs(float((cb-ca)@axis))-float((sa/2)@abs(ra.T@axis))-float((sb/2)@abs(rb.T@axis))
        values.append(dict(axis=label,separation_m=separation))
    return max(values,key=lambda x:x['separation_m'])

def main():
    OUT.mkdir(exist_ok=True)
    refs_path=V/'gen50_actual_handoff_monitor_r1/completed_contact_diagnostic_refs_r2.json'
    refs=read(refs_path)
    assert sha(refs_path)=='c4ae7a914869b7501c6287c085d1c01d0ff75e93485733f4ccdf29970433df91'
    guard=ROOT/'robot/observation_path_guard.py'
    names={'_array','_axis_rotation','SerialUrdfFk'}
    tree=ast.parse(guard.read_text())
    selected=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names]
    assert {n.name for n in selected}==names
    env=dict(np=np,math=math,hashlib=hashlib,ET=ET,ObservationPathError=RuntimeError)
    exec(compile(ast.Module(body=selected,type_ignores=[]),str(guard),'exec'),env)
    geometry=ROOT/'grasp/gripper_geometry.py'
    sat=next(n for n in ast.parse(geometry.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='_obb_overlap')
    exec(compile(ast.Module(body=[sat],type_ignores=[]),str(geometry),'exec'),env)
    records=[]
    evidence={str(p):sha(p) for p in [refs_path,guard,geometry,Path(__file__)]}
    fk=None
    for item in refs['unique_contact_failure_replays']:
        ref=item['reference'];p=Path(ref['store_root'])/ref['relative_path']
        compressed=p.read_bytes();raw=gzip.decompress(compressed)
        assert sha(p)==ref['gzip_sha256'] and hashlib.sha256(raw).hexdigest()==ref['object_sha256']
        assert len(raw)==ref['uncompressed_bytes']
        d=json.loads(raw);evidence[str(p)]=sha(p)
        s=d['scene'];f=d['first_failed_sample'];t=np.array(f['T_base_tool0'])
        ca=np.array(f['body_center_base_m']);sa=np.array(f['body_size_xyz_m'])
        cb=np.array(s['center']);rb=np.array(s['rotation']);sb=np.array(s['size'])
        fk=env['SerialUrdfFk'](d['model']['xml'],d['following_sample']['joint_names'])
        assert fk.model_sha256==d['model']['sha256']
        fd=float(np.max(abs(fk(f['joint_positions_rad'])-t)));assert fd<1e-12
        overlap=env['_obb_overlap'](ca,t[:3,:3],sa,cb,rb,sb);assert overlap
        encoded=json.loads(d['plan_data']['contact_geometry_evidence'])
        c=json.loads(zlib.decompress(base64.b64decode(encoded['payload_base64'])))
        box=c['box_model_evidence']
        records.append(dict(object_sha256=ref['object_sha256'],binding=d['binding'],
          curve=f['curve'],body=f['body'],label=f['label'],original_overlap_reproduced=bool(overlap),
          original_fk_max_absolute_difference=fd,closest_SAT_axis=sat_margins(ca,t[:3,:3],sa,cb,rb,sb),
          scene_size_m=s['size'],nominal_box_size_m=box['size_xyz_m'],
          box_edge_uncertainty_m=box['empirical_edge_uncertainty_m'],
          box_yaw_uncertainty_rad=box['empirical_yaw_uncertainty_rad'],
          inferred_width_m=c['inferred_contact_width_m'],width_uncertainty_m=c['width_uncertainty_m'],
          required_open_width_m=c['required_open_width_m'],actual_opening_m=d['opening_m'],
          following_position_error_m=d['following_sample']['position_error_m'],
          following_joint_error_sdk_counts=d['following_sample']['joint_error_sdk_counts'],
          support_clearance_m=f['support_clearance_m'],tool_pose_xyz_m=t[:3,3].tolist(),
          grasp_goal=d['plan_data']['poses_xyz_xyzw'][2],contact_height_bounds_m=c['contact_height_bounds_m']))
    consumer=Path(refs['consumers'])
    # The same deployed consumers log can append during a later task. Require
    # an exact byte prefix matching the formerly closed original log SHA.
    prefix=[];hasher=hashlib.sha256();matched=False
    with consumer.open('rb') as stream:
        for line in stream:
            prefix.append(line);hasher.update(line)
            if hasher.hexdigest()==refs['consumers_sha256']:
                matched=True;break
    assert matched, 'No exact original consumer-log SHA prefix remains'
    frozen=OUT/'original_G50_consumers_exact_prefix.log'
    frozen.write_bytes(b''.join(prefix))
    assert sha(frozen)==refs['consumers_sha256']
    evidence[str(frozen)]=sha(frozen)
    lines=frozen.read_text().splitlines();guides=[]
    for delivery in refs['deliveries']:
        if delivery['classification']!='held_axes_fixed_goal_prediction_not_converged':continue
        raw=lines[delivery['line']-1];d=json.loads(raw[raw.index('{'):])
        pre=d['precheck'];s=pre['held_pregrasp_selection'];trials=[]
        offset=np.array(s['sdk_minus_encoder_rad']);pg=s['fixed_pregrasp_goal'];ag=s['fixed_approach_goal'];free=[0,2,3,5]
        for trial in s['trials']:
            x=np.array(trial['solver_endpoint']);command=np.array(trial['quantized_command_rad'])
            actual=command-offset;delta=np.zeros(6,dtype=int);delta[free]=np.rint(x[6:]).astype(int)
            after=actual+delta*Q;continuous=x[:6].copy();continuous[free]+=x[6:]*Q
            pe=errors(fk,actual,pg);ae=errors(fk,after,ag);before=errors(fk,actual,ag)
            ok=(pe['position_m']<=.010 and pe['orientation_rad']<=math.radians(5)
              and ae['position_m']<=.006 and ae['orientation_rad']<=math.radians(5)
              and ae['position_m']<before['position_m']-1e-6 and max(abs(delta))<=32 and delta[1]==delta[4]==0)
            trials.append(dict(solver_success=trial['solver_success'],normalized_fallback=trial['normalized_fallback'],
              solver_guide_delta_counts=x[6:].tolist(),rounded_solver_guide_delta_counts=delta.tolist(),
              pregrasp_quantized_errors=pe,approach_rounded_solver_guide_errors=ae,
              continuous_solver_approach_errors=errors(fk,continuous,ag),
              existing_greedy_rejection=trial.get('rejection'),
              existing_greedy_delta_counts=trial.get('rejection_evidence',{}).get('proposed_delta_counts'),
              existing_greedy_position_error_m=trial.get('rejection_evidence',{}).get('predicted_position_error_m'),
              endpoint_only_witness_passed=bool(ok),does_not_certify_full_path_or_current_execution_policy=True))
        guides.append(dict(plan_id=delivery['plan_id'],line=delivery['line'],
          elapsed_sec=s['elapsed_sec'],original_failure=s['original_failure']['evidence'],trials=trials))
    write(OUT/'nine_collision_arithmetic.json',records);write(OUT/'four_guide_arithmetic.json',guides)
    result=dict(kind='G50_sealed_strict_records_bounded_arithmetic_review',created_ns=time.time_ns(),
      passed=True,passed_meaning='Nine stored first-failure points reproduce using original FK and SAT; guide arithmetic grants no path or authority.',
      collision_records=len(records),guide_failures=len(guides),
      endpoint_only_alternate_guide_witness_count=sum(t['endpoint_only_witness_passed'] for r in guides for t in r['trials']),
      no_global_reachability_claim=True,full_path_or_motion_authorized=False,
      evidence_sha256=evidence,outputs_sha256={str(p):sha(p) for p in [OUT/'nine_collision_arithmetic.json',OUT/'four_guide_arithmetic.json']},
      ROS_initialized=False,hardware_contacted=False,temperature_read=False,production_or_tests_modified=False)
    write(OUT/'bounded_arithmetic_review.json',result)
    print(json.dumps(dict(result_sha256=sha(OUT/'bounded_arithmetic_review.json'),
      collisions=[dict(curve=r['curve'],separation_mm=r['closest_SAT_axis']['separation_m']*1000,
        nominal_box_mm=[x*1000 for x in r['nominal_box_size_m']],width_mm=r['inferred_width_m']*1000,
        uncertainty_mm=r['width_uncertainty_m']*1000) for r in records],
      guides=[dict(plan=r['plan_id'],trials=[dict(solver=t['solver_success'],
        endpoint_witness=t['endpoint_only_witness_passed'],pre_mm=t['pregrasp_quantized_errors']['position_m']*1000,
        alt_mm=t['approach_rounded_solver_guide_errors']['position_m']*1000,
        old_mm=t['existing_greedy_position_error_m']*1000) for t in r['trials']]) for r in guides]),ensure_ascii=False))

if __name__=='__main__':main()
