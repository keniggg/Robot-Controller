"""Bound half-count joint quantization and replay nearby sealed feedback only."""
from pathlib import Path
import itertools,hashlib,json,math
import numpy as np
import rosbag,rospy
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW=ROOT/'ros_log/full_latest_restart_20261010T002347Z'
INPUT=NEW/'gen13_handoff_analysis/support_plane_replay'
OUT=NEW/'gen13_handoff_analysis/offline_encoder_quantization'
assert Path('/media/zhuyupei/SanStand').is_mount()
r=json.loads((INPUT/'report.json').read_text());c=json.loads((INPUT/'coordinate_sensitivity_expanded_metric.json').read_text())
selected=[0,2];frames=[r['frames'][i] for i in selected];qs=[np.asarray(c['rows'][i]['source_interpolated_q_rad']) for i in selected]
model=SerialUrdfFk(Path(c['urdf_path']).read_text(),['Joint%d'%i for i in range(1,7)])
mount=c['static_mount'];X=quaternion_matrix(mount['quaternion']);X[:3,3]=mount['xyz']
points=np.vstack((r['reference']['anchor_base'],r['reference']['footprint_base']))
ph=np.column_stack((points,np.ones(len(points))))
half=math.pi/4096
corners=np.asarray(list(itertools.product([-half,half],repeat=6)))

def plane(q,camera_plane):
    T=model(q)@X;n=T[:3,:3]@camera_plane[:3]
    return np.r_[n,camera_plane[3]-n@T[:3,3]]
def skew(v):
    a,b,z=v;return np.asarray([[0.,-z,b],[z,0.,-a],[-b,a,0.]])
def analytic_gradient(q,pi):
    # Independent differential of the fixed-origin URDF product; derivatives
    # are evaluated at nominal q, without a finite-difference step.
    factors=[];derivatives=[]
    for origin,index,axis in model.chain:
        factors.append(origin);derivatives.append(None)
        if index is not None:
            K=skew(axis);angle=q[index]
            R=np.eye(4);R[:3,:3]=np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)
            dR=np.zeros((4,4));dR[:3,:3]=math.cos(angle)*K+math.sin(angle)*(K@K)
            factors.append(R);derivatives.append((index,dR))
    factors.append(X);derivatives.append(None)
    T=np.eye(4)
    for f in factors:T=T@f
    n=T[:3,:3]@pi[:3];grad=np.zeros((len(points),6))
    for where,derivative in enumerate(derivatives):
        if derivative is None:continue
        index,dR=derivative;product=np.eye(4)
        for k,f in enumerate(factors):product=product@(dR if k==where else f)
        dn=product[:3,:3]@pi[:3]
        grad[:,index]=(points-T[:3,3])@dn-float(n@product[:3,3])
    return grad

datasets=[];nominals=[];corner_values=[];bounds=[]
# For s(q)=pi_camera^T X^-1 product(R_j^-1 O_j^-1) p_h,
# every rotation and its first/second angular derivative has spectral norm <=1.
# Thus every Hessian entry is bounded by the product of constant factor norms.
constant_norm_product=float(np.linalg.norm(np.linalg.inv(X),2))
for origin,index,axis in model.chain:constant_norm_product*=float(np.linalg.norm(np.linalg.inv(origin),2))
for frame,q in zip(frames,qs):
    pi=np.asarray(frame['plane_camera']);nom=plane(q,pi);values=np.asarray([ph@plane(q+delta,pi) for delta in corners]);nominal=ph@nom
    gradient=analytic_gradient(q,pi)
    hessian_bound=np.linalg.norm(pi)*constant_norm_product*np.linalg.norm(ph,axis=1)
    remainder=.5*hessian_bound*(6*half)**2
    continuous_bound=half*np.abs(gradient).sum(axis=1)+remainder
    # Verify the analytic gradient against an independent centered difference;
    # the actual continuous bound below uses the analytic value.
    numerical=np.column_stack([(ph@plane(q+np.eye(6)[j]*1e-6,pi)-ph@plane(q-np.eye(6)[j]*1e-6,pi))/2e-6 for j in range(6)])
    assert np.max(abs(numerical-gradient))<1e-8
    assert np.all(np.max(abs(values-nominal),axis=0)<=continuous_bound+1e-12)
    nominals.append(nominal);corner_values.append(values);bounds.append(continuous_bound)
    datasets.append(dict(source_stamp_ns=frame['source_stamp_ns'],q_nominal_rad=q.tolist(),plane_camera=pi.tolist(),
        enumerated_corner_count=64,maximum_corner_height_perturbation_mm=float(np.max(abs(values-nominal)))*1000,
        continuous_half_count_cube_height_bound_mm=float(max(continuous_bound))*1000,
        maximum_second_order_remainder_mm=float(max(remainder))*1000,
        analytic_gradient_max_difference_from_central_difference=float(np.max(abs(numerical-gradient))),
        signed_distance_gradient_m_per_rad=gradient.tolist()))
nominal_delta=nominals[1]-nominals[0]
pair_delta=corner_values[1][:,None,:]-corner_values[0][None,:,:]
perturbation=pair_delta-nominal_delta
continuous_pair_bound=bounds[0]+bounds[1]
min_unavoidable=float(np.max(np.maximum(0.,abs(nominal_delta)-continuous_pair_bound)))
pair_report=dict(independent_corner_pair_count=4096,
    nominal_maximum_anchor_and_footprint_plane_difference_mm=float(max(abs(nominal_delta)))*1000,
    maximum_enumerated_change_in_any_point_plane_difference_mm=float(np.max(abs(perturbation)))*1000,
    minimum_enumerated_maximum_local_plane_difference_mm=float(np.min(np.max(abs(pair_delta),axis=2)))*1000,
    maximum_enumerated_maximum_local_plane_difference_mm=float(np.max(abs(pair_delta)))*1000,
    continuous_independent_half_count_cube_change_bound_mm=float(max(continuous_pair_bound))*1000,
    continuous_minimum_remaining_maximum_local_difference_mm=min_unavoidable*1000,
    unchanged_consistency_limit_mm=4.)

windows=[]
with rosbag.Bag(str(Path(r['record'])/'keyframes.bag')) as bag:
    for frame,q,nominal in zip(frames,qs,nominals):
        ns=frame['source_stamp_ns'];samples=[]
        for topic,msg,received in bag.read_messages(topics=['/alicia_d/accepted_joint_states'],
                start_time=rospy.Time.from_sec(ns*1e-9-1.15),end_time=rospy.Time.from_sec(ns*1e-9+1.15)):
            t=msg.header.stamp.to_nsec()
            if abs(t-ns)>1_000_000_000:continue
            state=dict(zip(msg.name,msg.position));v=np.asarray([state['Joint%d'%i] for i in range(1,7)])
            samples.append(dict(source_stamp_ns=t,source_offset_sec=(t-ns)*1e-9,q=v))
        assert samples
        positions=np.asarray([s['q'] for s in samples]);pi=np.asarray(frame['plane_camera'])
        wrongtime_values=np.asarray([ph@plane(row['q'],pi) for row in samples])
        windows.append(dict(measurement_source_stamp_ns=ns,accepted_count=len(samples),
            q_peak_to_peak_rad=np.ptp(positions,axis=0).tolist(),q_max_difference_from_actual_source_q_rad=np.max(abs(positions-q),axis=0).tolist(),
            maximum_same_frame_plane_shift_using_any_q_in_plusminus_1sec_mm=float(np.max(abs(wrongtime_values-nominal)))*1000,
            samples=[dict(source_stamp_ns=s['source_stamp_ns'],source_offset_sec=s['source_offset_sec'],q_rad=s['q'].tolist()) for s in samples]))
pair_window_bound=sum(w['maximum_same_frame_plane_shift_using_any_q_in_plusminus_1sec_mm'] for w in windows)
report=dict(input_report=str(INPUT/'report.json'),input_report_sha256=hashlib.sha256((INPUT/'report.json').read_bytes()).hexdigest(),
    model_sha256=c['urdf_sha256'],half_encoder_count_rad=half,half_encoder_count_deg=half*180/math.pi,
    point_count=len(points),reference_anchor_and_footprint_base_m=points.tolist(),stations=datasets,pair=pair_report,
    continuous_bound_proof=dict(constant_inverse_transform_norm_product=constant_norm_product,
        statement='Apply scalar multivariate Taylor theorem to signed camera-plane distance at each fixed base point. Bound each Hessian entry by ||pi_camera|| * ||X^-1||_2 * product(||O_j^-1||_2) * ||p_h||; revolute inverse rotations and their first/second derivatives have norm <= 1. Remainder <= 0.5*H_bound*(sum(abs(delta_q)))^2. Add independent station bounds.',
        uses_fixed_camera_plane_and_expanded_metric_model=True),
    source_plusminus_one_second=windows,pair_plusminus_one_second_plane_shift_sum_bound_mm=pair_window_bound,
    conclusion='Conditional half-count and recorded timing sensitivities only; inspect the numerical bounds below before judging whether each can explain the recorded disagreement.',
    limits=['The model is conditional on correct physical encoder zeroes, URDF and fixed hand-eye. Systematic zero bias, mechanical compliance/backlash and physical calibration errors are not half-count quantization.',
        'Sealed accepted feedback does not independently prove continuous physical joint location between samples.',
        'The +/-1 s replay changes only the pose assigned to the same measured camera plane; it is a timing-mismatch sensitivity, not an estimate of sensor exposure timing.',
        'No ROS master, devices, temperature data, live code, thresholds or calibration were accessed or changed.'])
OUT.mkdir(parents=True,exist_ok=True);path=OUT/'encoder_quantization_and_timing.json'
with path.open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False);f.write('\n')
print(json.dumps(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),pair=pair_report,windows=[{k:v for k,v in x.items() if k!='samples'} for x in windows])))
