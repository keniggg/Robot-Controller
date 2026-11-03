"""Original connected contact-height interval and finite same-branch IK probes."""
from pathlib import Path
import ast,gzip,hashlib,json,math,os,time,xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from probe_gen52_g51_full_collision_curves_20261018 import load
from review_gen51_g50_strict_records_20261018 import pose_matrix
from review_gen52_g51_contact_collisions_20261018 import sat
W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
D=V/'gen52_contact_collision_review_r1'
ROOT=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    assert os.path.ismount('/media/zhuyupei/SanStand')
    env=dict(__name__='__main__',np=np,math=math,time=time,hashlib=hashlib,ET=ET,ObservationPathError=RuntimeError)
    gpath=ROOT/'grasp/gripper_geometry.py';hpath=ROOT/'grasp/contact_height_margin.py';guard=ROOT/'robot/observation_path_guard.py'
    for n in ast.parse(gpath.read_text()).body:
        if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id.startswith('ANALYTICAL_') for t in n.targets):exec(compile(ast.Module(body=[n],type_ignores=[]),str(gpath),'exec'),env)
    load(gpath,{'_signed_polygon_margin'},env)
    env['PATCH']=env['ANALYTICAL_FINGER_CONTACT_PATCH_TOOL_XZ_M'];env['_EPS']=1e-12
    load(hpath,{'_merge','_line_intervals','_clip_linear','_contact_translation_intervals'},env)
    load(guard,{'_array','_axis_rotation','SerialUrdfFk'},env)
    refs=V/'gen51_actual_handoff_monitor_r1/terminal_and_completed_diagnostics.json';ref=json.loads(refs.read_text())['unique_failure_replay_refs'][0]['reference'];p=Path(ref['store_root'])/ref['relative_path'];raw=gzip.decompress(p.read_bytes());assert sha(p)==ref['gzip_sha256'] and hashlib.sha256(raw).hexdigest()==ref['object_sha256'];d=json.loads(raw)
    edata=V/'gen51_actual_handoff_monitor_r1/first_strict_contact_evidence_original_decoded.json';e=json.loads(edata.read_text())['decoded_original_contact_geometry_evidence'];t=pose_matrix(d['plan_data']['poses_xyz_xyzw'][2]);r=t[:3,:3];center=np.array(e['contact_center_base']);axis=np.array(e['contact_height_axis_base']);n=np.array(d['scene']['normal'])
    pair=env['ANALYTICAL_FINGER_PAIR_CENTER_TOOL_XYZ_M'];point=(r.T@(center-t[:3,3])-pair)[[0,2]];height=(r.T@axis)[[0,2]];normal=(r.T@n)[[0,2]]
    intervals=env['_contact_translation_intervals'](point,height,normal,*e['contact_height_bounds_m'],e['minimum_contact_overlap_m'])
    # The original actual grasp curve already proved >=3mm support. Recompute
    # its endpoint from the same CAD boxes for this exact connected interval.
    from dataclasses import dataclass
    env['dataclass']=dataclass
    load(gpath,{'_readonly_vector','_finite_number','GripperGeometry','parse_tool_axis','_validated_rotation','gripper_box_centers','_stage_boxes','_box_corners','_obb_overlap'},env)
    g=env['GripperGeometry'](**{k:np.array(v) if isinstance(v,list) else v for k,v in d['config']['evaluated_gripper'].items()})
    clearance=min(float(np.min(env['_box_corners'](c,t[:3,:3],s)@n+d['scene']['offset'])) for _,c,s in env['_stage_boxes'](t,g,d['opening_m'],'y','z'))
    allowed=[(max(lo,g.support_clearance_m-clearance),hi) for lo,hi in intervals if max(lo,g.support_clearance_m-clearance)<=hi+1e-12]
    lo,hi=next((lo,hi) for lo,hi in allowed if lo<=0<=hi);limit=d['config']['grasp']['handeye_translation_max_error_m'];lower,upper=lo+limit,hi-limit;assert lower<=0<=upper
    fk=env['SerialUrdfFk'](d['model']['xml'],d['following_sample']['joint_names']);s=d['following_sample'];bias=np.array(s['sdk_positions_rad'])-s['accepted_positions_rad'];q0=np.array(d['checked_curve']['points'][-1]['positions'])-bias
    qlo=np.maximum(fk.limits[:,0],fk.limits[:,0]-bias);qhi=np.minimum(fk.limits[:,1],fk.limits[:,1]-bias);rows=[]
    for label,delta in [('original',0.),('first_up_50pct',.5*upper),('recorded_up_75pct',.75*upper),('maximum_up_with_original_evidence',upper),('minimum_down_with_original_evidence',lower)]:
        goal=t.copy();goal[:3,3]+=delta*n
        def fun(q):
            x=fk(q);return np.r_[(x[:3,3]-goal[:3,3])/.0005,((goal[:3,:3].T@x[:3,:3])-np.eye(3)).ravel()/.005]
        start=time.monotonic();sol=least_squares(fun,q0,bounds=(qlo,qhi),max_nfev=80,ftol=1e-11,xtol=1e-11,gtol=1e-11);tt=fk(sol.x);pe=float(np.linalg.norm(tt[:3,3]-goal[:3,3]));ae=math.acos(float(np.clip((np.trace(goal[:3,:3].T@tt[:3,:3])-1)/2,-1,1)));assert pe<.0005 and ae<.005
        row=dict(label=label,normal_delta_m=delta,goal_position_error_m=pe,goal_orientation_error_rad=ae,goal_qualified=True,nominal_actual_q=sol.x.tolist(),command_q=(sol.x+bias).tolist(),elapsed_sec=time.monotonic()-start,curves={})
        for name,q in [('actual',sol.x),('command',sol.x+bias)]:
            tt=fk(q);body=[]
            for bodyname,c,size in env['_stage_boxes'](tt,g,d['opening_m'],'y','z'):
                margin=sat(c,tt[:3,:3],size,np.array(d['scene']['center']),np.array(d['scene']['rotation']),np.array(d['scene']['size']))
                body.append(dict(body=bodyname,overlap=bool(env['_obb_overlap'](c,tt[:3,:3],size,np.array(d['scene']['center']),np.array(d['scene']['rotation']),np.array(d['scene']['size']))),separation_m=margin['separation_m']))
            row['curves'][name]=body
        row['dual_endpoint_passed']=all(not b['overlap'] for a in row['curves'].values() for b in a);rows.append(row)
    out=dict(scope='same_original_goal_orientation_connected_height_interval_finite_endpoint_diagnostics',original_plan_id=d['binding']['plan_id'],contact_translation_intervals_m=intervals,physical_support_restricted_intervals_m=allowed,original_handeye_translation_evidence_limit_m=limit,admissible_planned_height_shift_interval_m=[lower,upper],rows=rows,global_all_ik_or_entire_interval_infeasibility_claimed=False,full_path_checked=False,motion_authorized=False,source_sha256={str(q):sha(q) for q in [gpath,hpath,guard,refs,p,edata,Path(__file__)]},ROS_or_hardware_called=False,tests_or_production_modified=False,temperature_read=False)
    p=D/'remaining_original_height_window.json'
    with p.open('x') as f:json.dump(out,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(dict(path=str(p),sha256=sha(p),limit_mm=limit*1000,allowed_shift_mm=[lower*1000,upper*1000],rows=[dict(label=x['label'],shift_mm=x['normal_delta_m']*1000,command_overlap=[dict(body=b['body'],sat_mm=b['separation_m']*1000) for b in x['curves']['command'] if b['overlap']],safe=x['dual_endpoint_passed']) for x in rows])))
if __name__=='__main__':main()
