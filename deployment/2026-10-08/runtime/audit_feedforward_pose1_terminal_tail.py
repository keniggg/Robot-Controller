"""Closed-bag FK tail audit. No ROS master, control or parameter access."""
import hashlib,json,math,shutil
from pathlib import Path
import numpy as np
import rosbag
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_flexible_grasp.robot.stationary_following import ARM_NAMES
ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/feedforward_speed_grasp_20260929T064011Z')
RUNNER=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20260929T064011.916461Z_30cc326ed02f4b1cbc5b3051ee2e8408/runner.log')
URDF=Path('.runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf')
OUT=ROOT/'offline_pose1_tail_audit'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def counts(q):return np.rint(np.asarray(q)*4096/(2*math.pi)+2048).astype(int).tolist()
def main():
    key='CONTACT_ENDPOINT_COMPENSATION_FAILED: '
    line=next(s for s in RUNNER.read_text().splitlines() if s.startswith('GRASP_RESULT ') and key in s)
    failed=json.JSONDecoder().raw_decode(line.split(key,1)[1])[0]
    assert failed['plan_id']=='62e92d05507d1e48a0f49773' and failed['fixed_goal_pose_index']==1
    fk=SerialUrdfFk(URDF.read_text(),ARM_NAMES);assert fk.model_sha256==failed['model_sha256']
    goal=failed['fixed_goal_pose'];goal_R=quaternion_matrix(goal[3:])[:3,:3]
    evidence=[];refs=[];epochs=[];states=[];statuses=[]
    with rosbag.Bag(str(ROOT/'scene_diagnostic.bag')) as bag:
        for topic,msg,t in bag.read_messages(topics=['/alicia_d/accepted_joint_states','/alicia_d/control_reference','/alicia_d/control_reference_epoch','/grasp/state','/alicia_d/actuation_status']):
            if topic=='/grasp/state':
                if key in msg.message and failed['plan_id'] in msg.message:
                    states.append(dict(source_ns=msg.header.stamp.to_nsec(),receipt_ns=t.to_nsec(),active=msg.active,state=msg.state))
                continue
            if topic=='/alicia_d/control_reference_epoch':epochs.append(msg.stamp.to_nsec());continue
            if topic=='/alicia_d/actuation_status':statuses.append((t.to_nsec(),msg.data));continue
            indices=[msg.name.index(n) for n in ARM_NAMES];q=np.array([msg.position[i] for i in indices])
            row=dict(source_ns=msg.header.stamp.to_nsec(),receipt_ns=t.to_nsec(),q_rad=q.tolist(),counts=counts(q))
            if topic=='/alicia_d/control_reference':refs.append(row);continue
            T=fk(q);delta=T[:3,3]-np.asarray(goal[:3]);rotation=math.acos(float(np.clip((np.trace(goal_R.T@T[:3,:3])-1)/2,-1,1)))
            row.update(position_error_m=float(np.linalg.norm(delta)),error_base_xyz_m=delta.tolist(),orientation_error_deg=math.degrees(rotation),
                       opening_m=float(msg.position[msg.name.index('right_finger')]))
            evidence.append(row)
    assert set(epochs)=={failed['epoch_ns']}
    terminal=next(s for s in states if not s['active']);t0=terminal['source_ns']
    measured=next(s for s in evidence if s['source_ns']==failed['accepted_stamp_ns'])
    assert abs(measured['position_error_m']-failed['position_error_m'])<1e-12
    for row in evidence:row['seconds_after_terminal']=(row['source_ns']-t0)*1e-9
    tail=[r for r in evidence if 0<=r['seconds_after_terminal']<=8.05]
    positions=np.array([r['error_base_xyz_m'] for r in tail]);cn=np.array([r['counts'] for r in tail]);errs=np.array([r['position_error_m'] for r in tail]);times=np.array([r['seconds_after_terminal'] for r in tail]);src=np.array([r['source_ns'] for r in tail],dtype=np.int64)
    changes=[];prev=measured
    for row in tail:
        diff=np.asarray(row['counts'])-np.asarray(prev['counts'])
        if np.any(diff):changes.append(dict(source_ns=row['source_ns'],seconds_after_terminal=row['seconds_after_terminal'],delta_counts=diff.tolist(),counts=row['counts'],position_error_m=row['position_error_m']))
        prev=row
    checkpoints=[]
    for sec in [1,2,5,8]:
        row=min(tail,key=lambda r:abs(r['seconds_after_terminal']-sec))
        checkpoints.append(dict(requested_seconds=sec,actual_seconds=row['seconds_after_terminal'],offset_seconds=row['seconds_after_terminal']-sec,
            exact_time_sample=False,selection='nearest recorded source; no extrapolation',source_ns=row['source_ns'],counts=row['counts'],position_error_m=row['position_error_m'],error_base_xyz_m=row['error_base_xyz_m'],orientation_error_deg=row['orientation_error_deg']))
    tail_refs=[r for r in refs if t0<=r['source_ns']<=src[-1]]
    sdk_unique=sorted({tuple(r['counts']) for r in tail_refs})
    last2=[r for r in tail if r['seconds_after_terminal']>=times[-1]-2]
    lastcounts=np.array([r['counts'] for r in last2]);lasterr=np.array([r['position_error_m'] for r in last2])
    report=dict(schema='pose1_terminal_tail_audit_v1',plan_id=failed['plan_id'],pose_index=1,epoch_ns=failed['epoch_ns'],
        scope='8-second post-rejection measured-encoder/FK observation; not speed causality or physical tool metrology',
        frozen_goal_pose=goal,model_sha256=fk.model_sha256,position_limit_m=failed['position_tolerance_m'],
        terminal=terminal,failed_result_accepted_source_ns=failed['accepted_stamp_ns'],
        failure_sample_before_terminal_sec=(t0-failed['accepted_stamp_ns'])*1e-9,
        failed_result_error_m=failed['position_error_m'],failed_sample_recomputed_error_m=measured['position_error_m'],failed_counts=failed['measured_counts'],
        accepted_sample_count=len(tail),first_post_terminal_seconds=float(times[0]),last_post_terminal_seconds=float(times[-1]),
        max_accepted_source_gap_sec=float(np.diff(src).max()*1e-9),
        post_terminal_error_range_m=[float(errs.min()),float(errs.max())],post_terminal_error_at_or_below_6mm_samples=int(np.count_nonzero(errs<=.006)),
        first=tail[0],last=tail[-1],checkpoints=checkpoints,encoder_range_counts=np.ptp(cn,axis=0).tolist(),
        endpoint_position_ptp_base_m=np.ptp(positions,axis=0).tolist(),encoder_changes_after_failure_sample=changes,
        sdk_reference_unique_counts=[list(x) for x in sdk_unique],sdk_reference_samples=len(tail_refs),
        last_2_seconds=dict(samples=len(last2),start_seconds=last2[0]['seconds_after_terminal'],end_seconds=last2[-1]['seconds_after_terminal'],
                            encoder_range_counts=np.ptp(lastcounts,axis=0).tolist(),error_range_m=[float(lasterr.min()),float(lasterr.max())]),
        post_terminal_status_values=sorted({s for t,s in statuses if t>=t0}),
        conclusion='No recorded post-terminal sample enters the unchanged 6 mm bound; final two-second encoder window is static at encoder resolution.' if errs.min()>.006 and not np.any(np.ptp(lastcounts,axis=0)) else 'See measured range; no extrapolation beyond recorded tail.',
        causal_attribution_to_speed=False,production_changed=False,ros_calls=0,motion_authorized=False,
        sources=dict(bag=str(ROOT/'scene_diagnostic.bag'),runner=str(RUNNER),runner_sha256=sha(RUNNER),urdf=str(URDF),
                     script_sha256=sha(Path(__file__))))
    OUT.mkdir(exist_ok=False);(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    (OUT/'accepted_tail_samples.json').write_text(json.dumps(tail,ensure_ascii=False,indent=2)+'\n');shutil.copy2(__file__,OUT/Path(__file__).name)
    print(json.dumps({k:report[k] for k in ('failed_result_error_m','accepted_sample_count','first_post_terminal_seconds','last_post_terminal_seconds','max_accepted_source_gap_sec','post_terminal_error_range_m','checkpoints','encoder_range_counts','encoder_changes_after_failure_sample','sdk_reference_unique_counts','last_2_seconds','conclusion')},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
