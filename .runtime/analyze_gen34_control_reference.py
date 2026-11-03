#!/usr/bin/env python3
"""Read sealed gen32--34 bags only; never connect to ROS or read thermal data."""
from bisect import bisect_right
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import rosbag

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SESSION = ROOT / 'ros_log/supplement_grasp_20261008T103117Z'
WORK = Path(__file__).resolve().parents[1]
RECORDS = {
    32: '20261008T122515.003686Z_ffbc823b23544d479e051ca3923e2df2',
    33: '20261008T123552.595043Z_704863368cdf4c2abd14255ee3daab9e',
    34: '20261008T124106.426913Z_8690434647cf4b4e838df3f34e6b250c',
}
TOPICS = ['/alicia_d/actuation_status', '/alicia_d/motion_enabled',
          '/alicia_d/accepted_joint_states', '/alicia_d/control_reference_epoch',
          '/alicia_controller/follow_joint_trajectory/status', '/rosout_agg']


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(4*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def analyze(generation, directory):
    manifest_path = directory/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    bag_path = directory/'keyframes.bag'
    bag_digest = sha(bag_path)
    expected = next(row['sha256'] for row in manifest['files'] if row['path']=='keyframes.bag')
    assert bag_digest == expected
    assert manifest['recorder_closed'] is True
    statuses, changes, strict, feedback, epochs = [], [], [], [], []
    enabled, actions = Counter(), Counter()
    with rosbag.Bag(str(bag_path), 'r') as bag:
        recorded_topics = bag.get_type_and_topic_info().topics
        inventory = {name:dict(type=recorded_topics[name].msg_type,
                              count=recorded_topics[name].message_count)
                     for name in TOPICS if name in recorded_topics}
        for topic, msg, stamp in bag.read_messages(topics=TOPICS):
            ns = stamp.to_nsec()
            if topic == '/alicia_d/actuation_status':
                row = dict(recorded_ns=ns, value=msg.data)
                statuses.append(row)
                if not changes or changes[-1]['value'] != msg.data:
                    changes.append(row)
            elif topic == '/alicia_d/motion_enabled':
                enabled[str(bool(msg.data)).lower()] += 1
            elif topic == '/alicia_d/accepted_joint_states':
                feedback.append(dict(recorded_ns=ns, source_ns=msg.header.stamp.to_nsec(),
                                     frame_id=msg.header.frame_id))
            elif topic == '/alicia_d/control_reference_epoch':
                epochs.append(dict(recorded_ns=ns, source_ns=msg.stamp.to_nsec(),frame_id=msg.frame_id))
            elif topic == '/alicia_controller/follow_joint_trajectory/status':
                for item in msg.status_list:
                    actions[str(item.status)] += 1
            elif (msg.name == '/motion_gateway' and
                  msg.msg.startswith(('check_pose_strict request', 'check_pose_strict result'))):
                strict.append(dict(log_source_ns=msg.header.stamp.to_nsec(),
                                   recorded_ns=ns, message=msg.msg))
    timeline = [row['recorded_ns'] for row in statuses]
    for row in strict:
        i = bisect_right(timeline, row['log_source_ns']) - 1
        row['latest_status_recorded_before_log_source'] = statuses[i] if i >= 0 else None
        row['next_status_recorded_after_log_source'] = statuses[i+1] if i+1 < len(statuses) else None
    count = Counter(row['value'] for row in statuses)
    command = json.loads((directory/'command_result.json').read_text())
    authorization = json.loads((directory/'authorization.json').read_text())
    return dict(generation=generation, record_dir=str(directory),
        manifest_sha256=sha(manifest_path), bag_sha256=bag_digest,
        bag_manifest_digest_verified=True, sealed=True, recorded_topics=inventory,
        actuation_status_counts=dict(count), status_changes_in_record=changes,
        first_recorded_status=statuses[0], last_recorded_status=statuses[-1],
        transition_into_stale_captured=False,
        first_recorded_status_is_already_stale=statuses[0]['value']=='UNCONFIRMED:CONFIRMED_FEEDBACK_STALE',
        all_recorded_statuses_stale=set(count)=={'UNCONFIRMED:CONFIRMED_FEEDBACK_STALE'},
        motion_enabled_counts=dict(enabled), controller_goal_status_counts=dict(actions),
        accepted_feedback=dict(count=len(feedback),first=feedback[0],last=feedback[-1],
            frame_id_counts=dict(Counter(row['frame_id'] for row in feedback))),
        control_reference_epochs=epochs, strict_logs=strict,
        strict_handler_success_count=sum(row['message'].startswith('check_pose_strict result success=True') for row in strict),
        final_strict_rpc_responses_recorded=False,
        selection=authorization['selection'], recorded_result=command['result'],
        recorded_failure_reason=command['failure_reason'],
        runner_returncode=command['evidence']['runner_returncode'])


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    output = SESSION/'gen34_control_reference_analysis.json'
    assert not output.exists(), 'Do not replace existing experiment evidence'
    reports = [analyze(gen,ROOT/'grasp_records'/record) for gen,record in RECORDS.items()]
    code_paths = [
        'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py',
        'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py',
        'src/alicia_grasp_modes/scripts/mode_aware_grasp_runner.py',
        'src/real-arm/alicia_d_driver/include/alicia_d_driver/actuation_confirmation.hpp',
        '.runtime/run_user_requested_grasp_20261008.py',
        '.runtime/capture_bounded_current_grasp_preview_20261007.py',
    ]
    preflight_path = SESSION/'preflight.json'
    preflight = json.loads(preflight_path.read_text())
    check_path = SESSION/'gen34_candidate0_full_check/result.json'
    candidate_check = json.loads(check_path.read_text())
    report = dict(schema_version=1, review_at_utc=datetime.now(timezone.utc).isoformat(),
        scope='historical sealed recordings only; no live ROS access',
        physical_motion_requested=False, execution_authority=False,
        current_session_source_or_control_reference_inheritance_claimed=False,
        thermal_diagnostic_topics_read=False,
        records=reports,
        earliest_stale_observation_in_review=reports[0]['first_recorded_status'],
        actual_first_stale_transition_ns=None,
        transition_limit='All three bags start stale. First recorded status is an upper bound on the original transition, not the transition time.',
        original_preflight=dict(path=str(preflight_path),sha256=sha(preflight_path),
            at_utc=preflight.get('at_utc'),
            topic_names=list(preflight.get('topics',{})),
            records_actuation_status='/alicia_d/actuation_status' in preflight.get('topics',{}),
            records_motion_enabled='/alicia_d/motion_enabled' in preflight.get('topics',{})),
        direct_readonly_candidate_check=dict(path=str(check_path),sha256=sha(check_path),
            state_validity=candidate_check['state_validity'],final_rpc_response=candidate_check['strict']),
        current_review_code_sha256={p:sha(WORK/p) for p in code_paths},
        code_scope='Current local code is used for call-order interpretation; this list alone does not certify an entire historical loaded process tree.',
        call_chain=[
            'motion_gateway._with_observation_planner calls handler(req). The handler writes check_pose_strict result success=True.',
            'Only after handler success, execute=False appends _observation_preview_path_evidence to the response.',
            '_observation_preview_path_evidence -> _select_controller_sync_reference -> _command_reference_snapshot -> _fresh_actuation_status.',
            '_command_reference_snapshot requires CONFIRMED:* or an explicitly listed pending-bootstrap state; stale-unconfirmed is rejected.',
            'The wrapper replaces handler success with final success=False and OBSERVATION_PREVIEW_PATH_UNAVAILABLE when evidence construction raises.',
        ],
        conclusions_zh=[
            'gen32、gen33、gen34 的全部控制确认状态分别为 1280、1816、1974 条，均为 UNCONFIRMED:CONFIRMED_FEEDBACK_STALE；各录制均从该状态开始，没有记录到首次转失效事件。',
            'gen34 六次 handler 的 planned/success=True 日志只证明内部规划成功。最终 RPC 仍需后置控制参考与路径证据；这些日志不能证明最终 strict 成功，更不能证明 following/view 已执行或失败。',
            '已保存的 candidate0 只读 RPC 实证：状态碰撞检查通过，但最终 strict 返回 driver reference has no fresh actuation epoch。纠正此前“规划前拒绝”的描述，应为 handler 后路径证据构造拒绝。',
            '原 session/preflight.json 只保存 accepted_joint_states；run_user_requested_grasp 的启动预检及 runner 请求候选计算之前未检查 actuation_status/motion_enabled。因此 stale 状态未阻止这些录制和候选规划开始。',
            '这属于早期预检与错误归因缺口，不能称为绕过执行准入：grasp_task 的 _automatic_actuation_gate 在启动抓取时仍检查确认状态；本次记录也没有 controller active goal。',
            '封存旧关节反馈持续到达，不会自动证明失效后的重新执行确认；不能用当前新 session 或新对准现场继承旧候选、旧目标身份或旧控制权限。',
        ],
        limitations=[
            'std_msgs/String actuation_status has no source timestamp; stored times are recorder receipt times. Late bag writes are not exact driver-transition times.',
            'Historical six final service wire responses were not recorded. Their exact individual rejection strings remain unproven; the post-handler failure route is supported by unchanged stale-status coverage, source call order and the saved direct RPC result.',
            'No claim is made about current live actuation state or physical scene. No live ROS calls, motion commands, parameter changes or node restarts occurred in this analysis.',
        ], script_sha256=sha(Path(__file__)))
    with output.open('x') as handle:
        json.dump(report,handle,ensure_ascii=False,indent=2);handle.write('\n')
    print(json.dumps(dict(output=str(output),sha256=sha(output),
        counts={r['generation']:r['actuation_status_counts'] for r in reports},
        strict_handler_success={r['generation']:r['strict_handler_success_count'] for r in reports},
        motion_enabled={r['generation']:r['motion_enabled_counts'] for r in reports},
        action_status={r['generation']:r['controller_goal_status_counts'] for r in reports}),ensure_ascii=False))


if __name__ == '__main__':
    main()
