"""Bounded arithmetic on three sealed G51 first failures; no ROS imports."""
from pathlib import Path
import ast, base64, gzip, hashlib, json, math, os, time, zlib
import xml.etree.ElementTree as ET
import numpy as np

W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
D=V/'gen52_contact_collision_review_r1'
ROOT=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,x):
    with p.open('x') as f:
        json.dump(x,f,ensure_ascii=False,indent=2,allow_nan=False); f.write('\n');f.flush();os.fsync(f.fileno())
    assert json.loads(p.read_text())==x

def sat(ca,ra,sa,cb,rb,sb):
    axes=[('body%d'%i,ra[:,i]) for i in range(3)]+[('target%d'%i,rb[:,i]) for i in range(3)]
    for i in range(3):
        for j in range(3):
            a=np.cross(ra[:,i],rb[:,j]);n=float(np.linalg.norm(a))
            if n>1e-9: axes.append(('cross%d%d'%(i,j),a/n))
    vals=[dict(axis=k,separation_m=abs(float((cb-ca)@a))-float((sa/2)@abs(ra.T@a))-float((sb/2)@abs(rb.T@a))) for k,a in axes]
    return max(vals,key=lambda x:x['separation_m'])

def main():
    assert os.path.ismount('/media/zhuyupei/SanStand')
    D.mkdir(exist_ok=True)
    original=V/'gen51_actual_handoff_monitor_r1/terminal_and_completed_diagnostics.json'
    refs=json.loads(original.read_text())['unique_failure_replay_refs'][:3]
    guard=ROOT/'robot/observation_path_guard.py'; geom=ROOT/'grasp/gripper_geometry.py'
    env=dict(np=np,math=math,hashlib=hashlib,ET=ET,ObservationPathError=RuntimeError)
    names={'_array','_axis_rotation','SerialUrdfFk'}
    nodes=[n for n in ast.parse(guard.read_text()).body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names]
    assert {n.name for n in nodes}==names
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(guard),'exec'),env)
    gtree=ast.parse(geom.read_text()); n=next(n for n in gtree.body if isinstance(n,ast.FunctionDef) and n.name=='_obb_overlap')
    exec(compile(ast.Module(body=[n],type_ignores=[]),str(geom),'exec'),env)
    constants={}
    for node in gtree.body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id in {'ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M','ANALYTICAL_PALM_CENTER_TOOL_XYZ_M','ANALYTICAL_FINGER_BOX_PADDING_XYZ_M'} for t in node.targets):
            exec(compile(ast.Module(body=[node],type_ignores=[]),str(geom),'exec'),dict(np=np),constants)
    pair=constants['ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M'];pad=constants['ANALYTICAL_FINGER_BOX_PADDING_XYZ_M']
    def bodies(t,d):
        eg=d['config']['evaluated_gripper']; fs=np.array(eg['finger_size_xyz_m']);R=t[:3,:3]
        mid=t[:3,3]+R@pair; off=.5*(d['opening_m']+fs[1])*R[:,1]
        return [('left_finger',mid+off,fs+pad),('right_finger',mid-off,fs+pad),('palm',t[:3,3]+R@constants['ANALYTICAL_PALM_CENTER_TOOL_XYZ_M'],np.array(eg['palm_size_xyz_m']))]
    def report(t,d,target):
        cb,rb,sb=target
        tests=[]
        for name,ca,sa in bodies(t,d):
            margin=sat(ca,t[:3,:3],sa,cb,rb,sb)
            overlap=env['_obb_overlap'](ca,t[:3,:3],sa,cb,rb,sb)
            tests.append(dict(body=name,overlap=bool(overlap),**margin))
        mid=t[:3,3]+t[:3,:3]@pair; jaw=t[:3,1]
        width=float(sb@abs(rb.T@jaw)); center=float((cb-mid)@jaw)
        return dict(bodies=tests,target_jaw_projection_m=width,target_jaw_center_relative_finger_mid_m=center,
            padded_gap_required_at_this_rotation_m=width+2*abs(center)+float(pad[1]),
            actual_opening_m=d['opening_m'],tool_position_base_m=t[:3,3].tolist())
    results=[];evidence={str(p):sha(p) for p in [original,guard,geom,Path(__file__)]}
    for entry in refs:
        ref=entry['reference']; p=Path(ref['store_root'])/ref['relative_path']; compressed=p.read_bytes();raw=gzip.decompress(compressed)
        assert hashlib.sha256(compressed).hexdigest()==ref['gzip_sha256']
        assert hashlib.sha256(raw).hexdigest()==ref['object_sha256'] and len(raw)==ref['uncompressed_bytes']
        evidence[str(p)]=sha(p);d=json.loads(raw);s=d['scene'];f=d['first_failed_sample']
        fk=env['SerialUrdfFk'](d['model']['xml'],d['following_sample']['joint_names']);assert fk.model_sha256==d['model']['sha256']
        q=np.array(f['joint_positions_rad']);t=fk(q);assert float(np.max(abs(t-np.array(f['T_base_tool0']))))<1e-12
        bb=next(x for x in bodies(t,d) if x[0]==f['body']);assert np.max(abs(bb[1]-f['body_center_base_m']))<1e-12
        assert np.max(abs(bb[2]-f['body_size_xyz_m']))<1e-12
        target=tuple(np.array(s[k]) for k in ['center','rotation','size'])
        encoded=json.loads(d['plan_data']['contact_geometry_evidence']);contact=json.loads(zlib.decompress(base64.b64decode(encoded['payload_base64'])))
        box=contact['box_model_evidence'];nominal=tuple(np.array(box[k]) for k in ['center_base','axes_base','size_xyz_m'])
        fs=d['following_sample'];bias=np.array(fs['sdk_positions_rad'])-np.array(fs['accepted_positions_rad'])
        command=report(t,d,target);actual=report(fk(q-bias),d,target)
        assert next(x for x in command['bodies'] if x['body']==f['body'])['overlap']
        knots=[]
        for i,pt in enumerate(d['checked_curve']['points']):
            c=np.array(pt['positions']); row=dict(index=i,time_ns=pt['time_from_start_ns'])
            for name,qq in [('command',c),('constant_bias_actual',c-bias)]:
                rr=report(fk(qq),d,target);row[name]=dict(colliding_bodies=[b['body'] for b in rr['bodies'] if b['overlap']],minimum_sat_separation_m=min(b['separation_m'] for b in rr['bodies']),jaw_center_m=rr['target_jaw_center_relative_finger_mid_m'])
            knots.append(row)
        results.append(dict(plan_id=d['binding']['plan_id'],failure_object_sha256=ref['object_sha256'],curve=f['curve'],label=f['label'],body=f['body'],first_failure=command,
          same_q_nominal_box_diagnostic=report(t,d,nominal),constant_current_bias_actual_at_same_curve_sample_diagnostic=actual,
          constant_bias_tool_displacement_m=(fk(q-bias)[:3,3]-t[:3,3]).tolist(),bias_joint_sdk_counts=(bias/(2*math.pi/4096)).tolist(),
          physical_opening_m=d['opening_m'],scene_size_m=s['size'],nominal_box_size_m=box['size_xyz_m'],
          required_open_width_m=contact['required_open_width_m'],inferred_contact_width_m=contact['inferred_contact_width_m'],width_uncertainty_m=contact['width_uncertainty_m'],
          scene_center_m=s['center'],nominal_box_center_m=box['center_base'],fixed_grasp_goal=d['plan_data']['poses_xyz_xyzw'][2],curve_knots=knots,
          measured_following_position_error_m=fs['position_error_m'],support_clearance_first_failure_m=f['support_clearance_m'],
          diagnostic_only=True,certifies_complete_actual_path=False))
    write(D/'three_first_collision_arithmetic.json',results)
    result=dict(passed=True,review_scope='sealed_first_failure_fk_sat_arithmetic',created_ns=time.time_ns(),source_records=3,
      first_failure_fk_and_original_sat_reproduced=True,actual_paths_diagnostic_only=True,full_path_certified=False,motion_authorized=False,
      production_or_tests_modified=False,ROS_or_hardware_called=False,temperature_read=False,evidence_sha256=evidence,
      outputs_sha256={str(D/'three_first_collision_arithmetic.json'):sha(D/'three_first_collision_arithmetic.json')})
    write(D/'arithmetic_result.json',result)
    print(json.dumps(dict(result_sha256=sha(D/'arithmetic_result.json'),rows=[dict(plan=r['plan_id'],label=r['label'],first=r['first_failure']['bodies'][0],nominal_box_collision=r['same_q_nominal_box_diagnostic']['bodies'][0]['overlap'],same_actual=r['constant_current_bias_actual_at_same_curve_sample_diagnostic']['bodies'][0],required_gap_mm=r['first_failure']['padded_gap_required_at_this_rotation_m']*1000,actual_gap_required_mm=r['constant_current_bias_actual_at_same_curve_sample_diagnostic']['padded_gap_required_at_this_rotation_m']*1000,actual_knots_colliding=sum(bool(k['constant_bias_actual']['colliding_bodies']) for k in r['curve_knots'])) for r in results]),ensure_ascii=False))

if __name__=='__main__':main()
