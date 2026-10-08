"""Pure archived FK quantization scenarios, no ROS or runtime parameter writes."""
import itertools,json,math,hashlib,os
from pathlib import Path
import numpy as np
from audit_joint_metric_identifiability import MetricFk,rotation,from_matrix

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
OUT=ROOT/'grasp_repair_resume_20260928T120606Z/counterrotation_encoder_quantization'
SOURCE=ROOT/'grasp_repair_resume_20260928T120606Z/counterrotation_multiframe_131913'
SESSION=ROOT/'bounded_joint23_counterrotation_20260928T131913Z_197737'


def jacobian(fk,q,X):
 camera=fk(q)@X;point=camera[:3,3];pose=np.eye(4);J=np.zeros((3,6))
 for origin,index,axis in fk.chain:
  pose=pose@origin
  if index is not None:
   worldaxis=pose[:3,:3]@axis;J[:,index]=np.cross(worldaxis,point-pose[:3,3])
   pose[:3,:3]=pose[:3,:3]@rotation(axis,q[index])
 return J


def envelope(fk,q0,q1,X,half_width):
 P0,P1=fk(q0)@X,fk(q1)@X;d=P1[:3,3]-P0[:3,3];length=np.linalg.norm(d);unit=d/length
 J0,J1=jacobian(fk,q0,X),jacobian(fk,q1,X);H=np.column_stack((-J0,J1))
 S=sum(np.linalg.norm(origin[:3,3]) for origin,_,_ in fk.chain)+np.linalg.norm(X[:3,3])
 rho=6*half_width;remainder=S*rho*rho # two vector Taylor remainders .5*S*rho²
 errors=np.asarray(list(itertools.product((-half_width,half_width),repeat=12)))
 linear=d+errors@H.T
 # Norm is convex, so maximum on this linear zonotope occurs at some box vertex.
 upper=np.linalg.norm(linear,axis=1).max()+remainder+1e-12
 # A fixed supporting plane gives a rigorous lower bound without claiming a
 # numerical optimizer certifies the non-linear minimum.
 projection_allowance=half_width*np.sum(np.abs(unit@H))
 lower=max(0.,length-projection_allowance-remainder-1e-12)
 coordinate=half_width*np.sum(np.abs(H),axis=1)+remainder+1e-12
 angle=np.linalg.norm(from_matrix(P0[:3,:3].T@P1[:3,:3]).as_rotvec())
 # Each perturbed serial SO(3) factor contributes at most |dq| geodesic angle;
 # triangle inequality covers both endpoint rotations and their relative angle.
 angular_allowance=2*rho
 # The relative-coordinate vector also rotates with uncertain initial camera.
 relative_ball=(np.linalg.norm(half_width*np.sum(np.abs(H),axis=1))+remainder
                +2*math.sin(rho/2)*length)
 out=dict(per_joint_endpoint_half_width_deg=math.degrees(half_width),endpoint_L1_radius_deg=math.degrees(rho),
  global_camera_position_Hessian_vector_norm_bound_m=float(S),two_endpoint_Taylor_vector_remainder_mm=float(remainder*1000),
  nominal_base_displacement_mm=(d*1000).tolist(),nominal_length_mm=float(length*1000),
  base_displacement_coordinate_intervals_mm=np.column_stack((d-coordinate,d+coordinate)).tolist(),
  length_continuous_interval_mm=[float(lower*1000),float(upper*1000)],
  nominal_relative_rotation_angle_deg=math.degrees(angle),relative_angle_continuous_interval_deg=[math.degrees(max(0,angle-angular_allowance)),math.degrees(min(math.pi,angle+angular_allowance))],
  relative_camera_translation_vector_error_ball_radius_mm=float(relative_ball*1000),
  first_order_length_absolute_allowance_mm=float(projection_allowance*1000),
  linear_zonotope_vertices=4096,jacobian_base_m_per_rad_initial=J0.tolist(),jacobian_base_m_per_rad_final=J1.tolist())
 out['base_displacement_coordinate_intervals_mm']=(np.column_stack((d-coordinate,d+coordinate))*1000).tolist()
 # Independent finite difference and deterministic interior verification; the
 # proof is the global Taylor bound, not these samples.
 h=1e-6
 for q,J in ((q0,J0),(q1,J1)):
  numeric=np.column_stack([((fk(q+np.eye(6)[j]*h)@X)[:3,3]-(fk(q-np.eye(6)[j]*h)@X)[:3,3])/(2*h) for j in range(6)])
  if not np.allclose(numeric,J,atol=2e-9,rtol=2e-8):raise AssertionError('camera Jacobian mismatch')
 rng=np.random.RandomState(311);max_ratio=0.;lengths=[]
 for e in rng.uniform(-half_width,half_width,(500,12)):
  A=fk(q0+e[:6])@X;B=fk(q1+e[6:])@X;actual=B[:3,3]-A[:3,3];err=np.linalg.norm(actual-(d+H@e));max_ratio=max(max_ratio,err/remainder)
  if err>remainder+1e-12 or not lower<=np.linalg.norm(actual)<=upper:raise AssertionError('continuous enclosure violated')
  actual_relative=A[:3,:3].T@actual;nominal_relative=P0[:3,:3].T@d
  if np.linalg.norm(actual_relative-nominal_relative)>relative_ball+1e-12:raise AssertionError('relative frame enclosure violated')
  lengths.append(np.linalg.norm(actual)*1000)
 out['verification']=dict(analytic_jacobian_matches_finite_difference=True,deterministic_interior_pair_count=500,max_sample_Taylor_remainder_fraction=float(max_ratio),sample_length_range_mm=[min(lengths),max(lengths)],samples_not_the_continuous_proof=True)
 return out


def main():
 if not os.path.ismount('/media/zhuyupei/SanStand'):raise RuntimeError('external archive unavailable')
 samples=json.loads((SOURCE/'samples.json').read_text());invariants=json.loads((SOURCE/'counterrotation_invariants.json').read_text());g=json.loads((SESSION/'baseline_geometry.json').read_text())
 rows={label:[r for r in samples if r['window']==label] for label in ('baseline','positive_stationary')}
 qs={label:np.unique(np.array([r['accepted_rad'] for r in entries]),axis=0) for label,entries in rows.items()}
 if any(len(q)!=1 for q in qs.values()):raise RuntimeError('endpoint encoder values not unique; do not silently average quantization')
 fk=MetricFk((SESSION/'robot_description.urdf').read_text());X=np.array(g['T_tool_camera']);q0=qs['baseline'][0];q1=qs['positive_stationary'][0]
 one_word=2*math.pi/4096
 scenarios={name:envelope(fk,q0,q1,X,half) for name,half in [('nearest_word_half_bin',one_word/2),('conservative_one_word_each_endpoint',one_word)]}
 cam=invariants['mean_motion']['camera_tnorm_mm'];nominal=invariants['mean_motion']['fk_tnorm_mm']
 if abs(scenarios['nearest_word_half_bin']['nominal_length_mm']-nominal)>1e-7:raise RuntimeError('FK motion differs from frozen invariant report')
 result=dict(scope='offline conditional quantization envelope; no calibration/URDF/runtime changes',encoder_word_deg=360/4096.,q0_rad=q0.tolist(),q1_rad=q1.tolist(),delta_encoder_deg=np.degrees(q1-q0).tolist(),
  source_sample_counts={label:len(entries) for label,entries in rows.items()},reported_camera_motion=invariants['mean_motion'],
  scenarios=scenarios,interpretation='Both endpoint six-axis errors independently cover the declared bins. Repeated identical quantized samples do not shrink these unknown fixed offsets. Half-bin assumes nearest-word quantization; one-word scenario is also reported. This is not a bound on encoder accuracy, gear backlash, compliance, hand-eye, depth or PnP errors.',
  proof='Expand serial-chain camera center as sums of rotated constant translations. Each joint rotation occurs once per term; every Hessian vector has norm <= sum(origin translation norms)+norm(X translation). Integral Taylor remainder <= .5*S*L1(error)^2 at each endpoint. Linear displacement zonotope covers all endpoint boxes; convex vertex maximum and fixed direction supporting minimum plus remainders bound continuous length. SO(3) triangle inequality covers rotation.',
  source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [SOURCE/'samples.json',SOURCE/'counterrotation_invariants.json',SESSION/'baseline_geometry.json',SESSION/'robot_description.urdf']})
 for value in scenarios.values():value['observed_camera_length_inside_interval']=value['length_continuous_interval_mm'][0]<=cam<=value['length_continuous_interval_mm'][1]
 OUT.mkdir(exist_ok=True);(OUT/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');(OUT/'analysis_source.py').write_bytes(Path(__file__).read_bytes())
 print(json.dumps(dict(delta_encoder_deg=result['delta_encoder_deg'],scenarios={k:{a:b for a,b in v.items() if not a.startswith('jacobian')} for k,v in scenarios.items()}),ensure_ascii=False))

if __name__=='__main__':main()
