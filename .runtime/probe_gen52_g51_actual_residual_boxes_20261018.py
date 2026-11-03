"""Exact corner counterexamples for prospective actual residual boxes."""
from pathlib import Path
from itertools import product
import ast,gzip,hashlib,json,math,os,time,xml.etree.ElementTree as ET
import numpy as np
from review_gen52_g51_contact_collisions_20261018 import sat
W=Path(__file__).resolve().parents[1]
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
D=V/'gen52_contact_collision_review_r1'
P=W/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_path_guard.py'
Q=2*math.pi/4096
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    assert os.path.ismount('/media/zhuyupei/SanStand')
    env=dict(np=np,math=math,hashlib=hashlib,ET=ET,ObservationPathError=RuntimeError)
    names={'_array','_axis_rotation','SerialUrdfFk'}
    nodes=[n for n in ast.parse(P.read_text()).body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names]
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(P),'exec'),env)
    refs=json.loads((V/'gen51_actual_handoff_monitor_r1/terminal_and_completed_diagnostics.json').read_text())['unique_failure_replay_refs'][:3]
    rows=[]
    for item in refs:
        ref=item['reference'];f=Path(ref['store_root'])/ref['relative_path'];raw=gzip.decompress(f.read_bytes())
        assert sha(f)==ref['gzip_sha256'] and hashlib.sha256(raw).hexdigest()==ref['object_sha256']
        d=json.loads(raw);fk=env['SerialUrdfFk'](d['model']['xml'],d['following_sample']['joint_names'])
        q=np.array(d['checked_curve']['points'][-1]['positions']);s=d['following_sample'];bias=np.array(s['sdk_positions_rad'])-s['accepted_positions_rad'];q-=bias
        scene=d['scene'];cb=np.array(scene['center']);rb=np.array(scene['rotation']);sb=np.array(scene['size'])
        opening=d['opening_m'];finger=np.array(d['config']['evaluated_gripper']['finger_size_xyz_m']);size=finger+.0012
        samples=[]
        for scale in (.25,.5,1.):
            epsilon=Q*scale;worst=None;colliding=0
            for signs in product((-1.,1.),repeat=6):
                residual=np.array(signs)*epsilon; t=fk(q+residual);r=t[:3,:3]
                mid=t[:3,3]+r@np.array([.0004,.0003,-.0302]);off=.5*(opening+finger[1])*r[:,1]
                for body,center in [('left_finger',mid+off),('right_finger',mid-off)]:
                    m=sat(center,r,size,cb,rb,sb)
                    witness=dict(body=body,joint_residual_rad=residual.tolist(),max_sat_separation_m=m['separation_m'],closest_axis=m['axis'])
                    if worst is None or witness['max_sat_separation_m']<worst['max_sat_separation_m']:worst=witness
                    if m['separation_m'] <= 1e-9:colliding+=1
            samples.append(dict(uniform_radius_sdk_counts=scale,uniform_radius_rad=epsilon,corner_body_tests=128,colliding_corner_body_count=colliding,worst_corner_witness=worst,
                certifies_interior=False,infeasible_box_proved_by_witness=colliding>0))
        rows.append(dict(plan_id=d['binding']['plan_id'],end_actual_joint_positions_rad=q.tolist(),samples=samples))
    result=dict(scope='finite_exact_fk_sat_counterexamples_not_full_uncertainty_proof',rows=rows,
       notes=['A single colliding included corner disproves a whole-box safety claim. No collisions among corners does not prove the interior.',
              'Residual radii are diagnostic fractions of the existing SDK quantum, not a proposed new tolerance or relaxed gate.',
              'Freshness, controller desired/actual synchronization, source quantization, commanded stop and physical overshoot bounds are not certified by this arithmetic.'],
       source_sha256={str(p):sha(p) for p in [P,Path(__file__)]},production_or_tests_changed=False,ROS_or_hardware_called=False,temperature_read=False)
    p=D/'actual_residual_box_counterexamples.json'
    with p.open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(dict(path=str(p),sha256=sha(p),rows=[dict(plan=r['plan_id'],samples=[dict(counts=s['uniform_radius_sdk_counts'],colliding=s['colliding_corner_body_count'],worst_mm=s['worst_corner_witness']['max_sat_separation_m']*1000) for s in r['samples']]) for r in rows])))
if __name__=='__main__':main()
