#!/usr/bin/env python3
"""Bind GEN60 resource windows to the already exported, closed state messages."""
from pathlib import Path
import hashlib
import json
import os

S=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/actual_attempt_r1')
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261022T060615.487250Z_8db865e4c1094eb1bbdc72457cb35e2e')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def header(row):return row['data']['header']['stamp']['stamp_ns']
def compact(row):
    return {k:row[k] for k in ('topic','bag_stamp_ns','wire_sha256','message_type','message_md5')}

def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    source=S/'terminal_evidence_r1/state_messages.jsonl'
    extraction=S/'terminal_evidence_r1/extraction.json'
    pins={str(source):'3befd6a315a798a482e420cd8b309c78014a76f7212084aa55086c5a8785b874',
          str(extraction):'e1ed440c768e87680551185ff62251f362a00c127dd5284f1173ee4b35eb9205',
          str(Path(__file__).resolve()):sha(__file__)}
    for p,h in pins.items():assert sha(p)==h,p
    x=json.loads(extraction.read_bytes())
    assert x['passed'] and x['bag_identity_unchanged'] and not x['full_bag_rehashed']
    assert x['manifest_sha256']==sha(R/'manifest.json')
    pins[str(R/'manifest.json')]=x['manifest_sha256']
    runner=R/'runner.log'
    pins[str(runner)]=sha(runner)
    runner_text=runner.read_text()
    assert 'CANDIDATE_COMPUTATION_START success=True' in runner_text
    assert 'FRESH_MODE_PREVIEW strategy=observe_return plan_id=3cbc2048ca6e80dbaca4b24e' in runner_text
    rows=[json.loads(line) for line in source.read_text().splitlines()]
    events=[r for r in rows if r['topic']=='/grasp/task_event']
    assert [r['data']['event_seq'] for r in events]==list(range(1,12))
    assert all(r['data']['selection']['generation']==4 and r['data']['task_run_id']=='0b56e0e032b44fd5ab44262b443a8e44' for r in events)
    assert all(r['data']['bound_plan_id']=='3cbc2048ca6e80dbaca4b24e' for r in events[:-1])
    assert events[-1]['data']['bound_plan_id'] is None and events[-1]['data']['message'].startswith('execution slot released: ')
    event={r['data']['event_seq']:r for r in events}
    states=[r for r in rows if r['topic']=='/grasp/state']
    for r in events:
        matches=[q for q in states if header(q)==r['data']['stamp_ns']]
        assert len(matches)==1 and matches[0]['data']['active']==r['data']['active']
    assert all(r['data']['active'] for r in events[:-1]) and not events[-1]['data']['active']
    assert event[6]['data']['message']=='moving 6D pregrasp'
    assert event[7]['data']['message']=='correcting observation endpoint'
    assert event[8]['data']['message']=='qualified observation station; checking range before contact planning'
    assert event[9]['data']['message']=='waiting for near-field 6D preview'
    start,end=event[1]['data']['stamp_ns'],event[11]['data']['stamp_ns']
    selection_start=event[1]['data']['selection']['stamp_ns']
    assert selection_start==1792649183269455671
    near=[r for r in rows if r['topic']=='/grasp/near_field_phase' and r['data']['phase_id']==3 and r['data']['active']]
    assert len(near)==1 and near[0]['data']['planning_stage']=='contact_replan'
    goals=[r for r in rows if r['topic']=='/alicia_controller/follow_joint_trajectory/goal']
    results=[r for r in rows if r['topic']=='/alicia_controller/follow_joint_trajectory/result']
    assert len(goals)==len(results)==1
    goal,result=goals[0],results[0]
    assert goal['data']['goal_id']==result['data']['status']['goal_id']
    assert result['data']['status']['status']==3 and result['data']['result']['error_code']==0
    points=goal['data']['goal']['trajectory']['points']
    duration=points[-1]['time_from_start']['stamp_ns']/1e9
    assert len(points)==25 and duration==29.100347393
    def window(a,b,description):
        assert selection_start<=a<b<=end
        return dict(start_ns=a,end_ns=b,exact_wall_sec=(b-a)/1e9,description=description)
    phases={
        'whole_automatic_attempt':window(selection_start,end,'Current selection submission through inactive FAILED; includes initial remote inference, candidate/view planning, audit handoff and task execution.'),
        'pre_active_computation_and_handoff':window(selection_start,start,'Selection through task-active; real automatic attempt work, not idle. No pure compute/wait decomposition is inferred.'),
        'task_active':window(start,end,'Exact current GraspState active header through inactive FAILED header.'),
        'initial_task_setup':window(start,event[6]['data']['stamp_ns'],'Initial capture and observation planning state events; no pure RPC decomposition.'),
        'observation_stage_including_dispatch':window(event[6]['data']['stamp_ns'],event[7]['data']['stamp_ns'],'moving 6D pregrasp through correcting observation endpoint; includes dispatch and status handling.'),
        'arm_action_goal_to_result':window(header(goal),header(result),'Nested action interval, not an additional disjoint phase; successful result for the same goal ID.'),
        'endpoint_qualification':window(event[7]['data']['stamp_ns'],event[8]['data']['stamp_ns'],'Endpoint correction event through qualified observation station event.'),
        'near_field':window(header(near[0]),end,'Active contact_replan phase through FAILED inactive task state.'),
        'waiting_for_preview':window(event[9]['data']['stamp_ns'],end,'Preview waiting state through FAILED inactive; not pure RPC wait time.')}
    statuses=[dict(**compact(r),message=r['data']) for r in rows if r['topic']=='/grasp_6d/status' and header(near[0])<=r['bag_stamp_ns']<=end]
    rpc=dict(action_goal_to_result_wall_sec=(header(result)-header(goal))/1e9,
        submitted_action_trajectory_duration_sec=duration,action_goal=compact(goal),action_result=compact(result),
        action_goal_header_ns=header(goal),action_result_header_ns=header(result),action_result_status=3,action_error_code=0,
        action_point_count=len(points),goal_id=goal['data']['goal_id']['id'],
        moving_stage_to_action_goal_wall_sec=(header(goal)-event[6]['data']['stamp_ns'])/1e9,
        action_result_to_endpoint_event_wall_sec=(event[7]['data']['stamp_ns']-header(result))/1e9,
        initial_status_messages=[dict(**compact(r),message=r['data']) for r in rows if r['topic']=='/grasp_6d/status' and selection_start<=r['bag_stamp_ns']<=start],
        runner_records_initial_candidate_computation_and_bound_preview=True,
        preview_source_stamp_is_not_preview_completion_wall_time=True,
        near_field_status_messages=statuses,contact_precheck_rpc_wall_measurement_available=False,
        planning_compute_vs_RPC_wait_decomposition_available=False,
        limitation='Exported allowlisted state/status events contain no timed contact-precheck request/response intervals. Preview waiting is elapsed state time, not pure RPC delay; absence in this export does not establish zero RPC calls.')
    notes=[
        '整次自动尝试按selection4提交→terminal统计61.018714905秒；其中task-active前24.163157701秒含真实远端推理、观察候选规划及审计交接，计入全链路负载，不称空闲。task-active为36.855557204秒。selection前的门等待/准备和terminal后采样另列。',
        'moving 6D pregrasp→correcting observation endpoint 为32.695504427秒，含分发与回执处理；唯一匹配的动作goal→成功result为29.150668631秒，提交轨迹时长29.100347393秒。两窗口重叠，不能相加。',
        'correcting observation endpoint→qualified observation station 为0.802790403秒；1Hz采样不足以独立估计此短阶段CPU。correcting事件本身不等同最终合格到位。',
        '近场contact_replan启动→任务inactive为2.667288065秒，waiting preview→inactive为1.916198731秒。期间状态记录continuous remote inference already running及DEPTH_UNSTABLE（mask centroid shift 11.087px > 5px）。这些是状态与接收时刻，不能拆成纯计算或RPC等待时间。',
        '终态文本的observed_stamp_ns=1792649244237768888是失去目标的发现戳，不当作相机RGBD源戳；源帧诊断由独立感知审计处理。',
        '旧selection generation 3的锁存状态未纳入本次generation 4事件边界。仅使用已导出的非图像消息，未再次读取bag。']
    out=dict(kind='GEN60_actual_resource_event_windows',record_root=str(R),input_sha256=pins,
        method='Whole automatic attempt uses generation-4 selection stamp through inactive terminal header, including computation before task-active. Use exact matching current-generation task-event stamp_ns and GraspState.header stamps for active bounds; action and near-field bounds use their message headers. String status times are explicitly bag receipt times. Resource counters use adjacent samples wholly inside each window, no interpolation; overlapping nested phases must not be summed.',
        phases=phases,planning_and_rpc=rpc,readable_stage_notes=notes,
        task_events=[dict(**compact(r),event=r['data']) for r in events],near_field_start=compact(near[0]),
        original_bag_rehashed_here=False,bag_read_here=False,ROS_contacted=False,network_contacted=False)
    for p,h in pins.items():assert sha(p)==h,p
    path=S/'resource_event_windows_r1.json'
    with path.open('x') as f:
        json.dump(out,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
    print(json.dumps(dict(path=str(path),sha256=sha(path),active_wall_sec=phases['task_active']['exact_wall_sec'])))

if __name__=='__main__':main()
