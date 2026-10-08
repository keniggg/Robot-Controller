#!/usr/bin/env python3
"""Finite offline old/new deltas and unchanged common-domain audit only."""
import itertools,json
from pathlib import Path
import numpy as np
import derive_expanded_metric_mjcf_review as D

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/grasp_repair_resume_20260928T120606Z')
OUT=BASE/'expanded_metric_mjcf_review_20260929';CAND=BASE/'expanded_metric_training_once/candidate_model'
def samples(lo,hi):
 middle=(lo+hi)/2;rng=np.random.default_rng(20260929)
 out=[*rng.uniform(lo,hi,size=(1024,6)),*np.asarray(list(itertools.product(*zip(lo,hi)))),middle,np.clip(np.zeros(6),lo,hi)]
 for i in range(6):
  for v in (lo[i],hi[i],np.nextafter(lo[i],hi[i]),np.nextafter(hi[i],lo[i])):
   q=middle.copy();q[i]=v;out.append(q)
 return out

def main():
 original=D.ET.fromstring(D.SOURCE.read_text());manifest=json.loads((CAND/'manifest.json').read_text());derived,_=D.derive(D.SOURCE.read_text(),manifest['origin_changes'])
 m0=D.compile_xml(original);m1=D.compile_xml(derived);d0=D.mujoco.MjData(m0);d1=D.mujoco.MjData(m1)
 old=D.SerialUrdfFk((CAND/'rollback_source/original_robot_description.urdf').read_text(),D.NAMES);new=D.SerialUrdfFk((CAND/'alicia_expanded_metric_candidate.urdf').read_text(),D.NAMES)
 X0=D.handeye(CAND/'rollback_source/original_validated_handeye.yaml');X1=D.handeye(CAND/'d405_expanded_metric_candidate_eye_on_hand.yaml')
 _,palm=D.fk(m0,d0,np.zeros(6));H=np.linalg.inv(palm)@old(np.zeros(6))
 report=json.loads((OUT/'report.json').read_text());ml=np.asarray(report['mjcf_limits_rad']);ul=new.limits
 lo=np.maximum(ml[:,0],ul[:,0]);hi=np.minimum(ml[:,1],ul[:,1]);assert np.all(lo<hi)
 groups={};outside_by_joint={n:0 for n in D.NAMES}
 for domain,qs in [('URDF_legal',samples(*ul.T)),('URDF_MJCF_intersection',samples(lo,hi))]:
  before={k:[] for k in ('raw_tcp','fixed_frame_tool','fixed_frame_camera')};after={k:[] for k in before}
  for q in qs:
   a,pa=D.fk(m0,d0,q);b,pb=D.fk(m1,d1,q);u0,u1=old(q),new(q)
   for k,aa,bb,uu0,uu1 in [('raw_tcp',a,b,u0,u1),('fixed_frame_tool',pa@H,pb@H,u0,u1),('fixed_frame_camera',pa@H@X0,pb@H@X1,u0@X0,u1@X1)]:
    before[k].append(D.err(aa,uu0));after[k].append(D.err(bb,uu1))
   if domain=='URDF_legal':
    for i,n in enumerate(D.NAMES):outside_by_joint[n]+=int(q[i]<ml[i,0] or q[i]>ml[i,1])
  delta={}
  for k in before:
   stats0=D.summarize(before[k]);stats1=D.summarize(after[k]);diff={m:np.array([b[m]-a[m] for a,b in zip(before[k],after[k])]) for m in before[k][0]}
   delta[k]={'old':stats0,'new':stats1,'max_statistic_new_minus_old':{m:stats1[m]['maximum']-stats0[m]['maximum'] for m in stats0},'same_q_error_norm_delta':{m:{'minimum':float(v.min()),'maximum':float(v.max()),'max_absolute':float(np.abs(v).max())} for m,v in diff.items()}}
  groups[domain]={'count':len(qs),'random_interior_count':1024,'all_64_limit_vertices':True,'boundary_and_nextafter_probes':24,'zero_and_middle':True,'metrics':delta}
 supplement={'schema':1,'deployed':False,'hardware_or_ROS_actions':False,'new_acceptance_gate_introduced':False,'strict_identity_is_diagnostic_not_runtime_gate':True,'deployment_status_reason':'Review only: prospective new-pose validation and root deployment orchestration are still pending. Micrometre residual is not itself a newly imposed blocker.','source_report_sha256':D.sha(OUT/'report.json'),'generator_sha256':D.sha(__file__),'URDF_MJCF_legal_intersection_rad':np.column_stack((lo,hi)).tolist(),'URDF_legal_outside_MJCF_by_joint':outside_by_joint,'limit_difference_note':'Existing Joint4 MJCF +/-2.7925 is narrower than URDF +/-3.14. Existing Joint5 MJCF upper +1.57 is wider than URDF +1.50; all other listed endpoints equal. Candidate derivation changes neither set.','groups':groups,'existing_tolerance_scope':{'unknown_initial_IK_position_m':0.0002,'unknown_initial_IK_orientation_error':0.005,'source':'tools/mujoco_digital_twin_server.py:75-76,1693-1703,2274-2275,2449-2500','meaning':'Within MuJoCo, requested Cartesian pose versus its own finger-midpoint/Link6 FK; not a cross-URDF model comparison. Orientation IK uses its existing cross-product error, not newly added geodesic requirement.','cross_URDF_MJCF_runtime_tolerance_found':None,'finger_mesh_gap_contract_m':0.0005,'finger_mesh_gap_scope':'Runtime zero-finger mesh gap/ranges only, not TCP.'},'claim_limit':'Finite deterministic model comparison, not global equivalence proof, hardware calibration truth, prospective acceptance or grasp/lift success.'}
 with (OUT/'old_new_delta_and_domain.json').open('x') as f:json.dump(supplement,f,indent=2)
 print(json.dumps({'outside':outside_by_joint,'intersection':supplement['URDF_MJCF_legal_intersection_rad'],'deltas':{k:{m:v['max_statistic_new_minus_old'] for m,v in group['metrics'].items()} for k,group in groups.items()}},indent=2))
if __name__=='__main__':main()
