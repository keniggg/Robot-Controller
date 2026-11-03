"""Read GEN11 sealed bag: execution acknowledgement versus measured motion."""
from pathlib import Path
import hashlib
import json

import numpy as np
import rosbag

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = ROOT / 'ros_log/full_latest_restart_20261010T002347Z'
REC = ROOT / 'grasp_records/20261010T045823.108983Z_afb71d5ad1424dad9de6275837bc5f78'
assert Path('/media/zhuyupei/SanStand').is_mount()
manifest = json.loads((REC / 'manifest.json').read_text())
assert manifest['state'] == 'complete' and manifest['recorder_closed']
events, measured, written, states, loss = [], [], [], {}, None
topics = ['/grasp_6d/preview_plan_enriched', '/grasp_6d/plan_enriched', '/grasp/state',
    '/alicia_d/actuation_status', '/alicia_d/control_reference_epoch',
    '/alicia_controller/follow_joint_trajectory/goal',
    '/alicia_controller/follow_joint_trajectory/result', '/alicia_controller/command',
    '/alicia_d/accepted_joint_states', '/perception/unknown/detector_status',
    '/rosout_agg', '/alicia_d/sdk_command', '/gui/joint_direct_mode']
with rosbag.Bag(str(REC / 'keyframes.bag')) as bag:
    topic_counts = {name: value.message_count for name, value in bag.get_type_and_topic_info().topics.items()}
    for topic, message, stamp in bag.read_messages(topics=topics):
        row = dict(topic=topic, received_ns=stamp.to_nsec())
        if topic in ('/alicia_d/accepted_joint_states', '/alicia_d/sdk_command'):
            row.update(source_ns=message.header.stamp.to_nsec(), frame_id=message.header.frame_id,
                q=list(message.position[:6]))
            (measured if topic.endswith('accepted_joint_states') else written).append(row)
            continue
        if topic == '/alicia_d/actuation_status':
            if states.get(topic) == message.data:
                continue
            states[topic] = message.data
            row['status'] = message.data
        elif topic == '/grasp/state':
            row.update(source_ns=message.header.stamp.to_nsec(), state=message.state, message=message.message)
        elif topic.endswith('control_reference_epoch'):
            row['epoch_ns'] = message.stamp.to_nsec()
        elif topic.endswith('plan_enriched'):
            if not message.valid:
                continue
            row.update(source_ns=message.header.stamp.to_nsec(), plan_id=message.plan_id,
                diagnostic=message.diagnostic, valid=True)
        elif topic.endswith('/goal'):
            trajectory = message.goal.trajectory
            row.update(goal_id=message.goal_id.id, source_ns=message.header.stamp.to_nsec(),
                q_start=list(trajectory.points[0].positions), q_end=list(trajectory.points[-1].positions),
                duration_sec=trajectory.points[-1].time_from_start.to_sec())
        elif topic.endswith('/result'):
            row.update(action_status=message.status.status, error_code=message.result.error_code,
                error_string=message.result.error_string)
        elif topic == '/alicia_controller/command':
            row.update(points=len(message.points), endpoint=list(message.points[-1].positions))
        elif topic == '/gui/joint_direct_mode':
            row['manual'] = message.data
        elif topic == '/rosout_agg':
            if not any(value in message.msg for value in (
                    'INITIAL_PROBE_TARGET_RETIRED', 'Actuation state changed:',
                    'ACTUATION_PREFIX_NO_RESPONSE', 'Unknown target first loss')):
                continue
            row.update(source_ns=message.header.stamp.to_nsec(), node=message.name, text=message.msg)
        elif topic.endswith('detector_status'):
            status = json.loads(message.data)
            if loss is not None or 'initial_station_current_support_invalidated' not in status.get('detail', ''):
                continue
            timing = status['frame_timing']
            row.update(detail=status['detail'], first_loss=status['first_loss'],
                frame_timing={key: timing[key] for key in (
                    'source_stamp_ns', 'processing_start_ros_ns', 'failed_stage',
                    'failure_ros_ns', 'failure_source_age_sec', 'processing_wall_sec',
                    'stage_end_elapsed_sec')})
            loss = row
        events.append(row)
assert loss is not None and measured and written
origin = np.asarray(measured[0]['q'])
measured_delta = np.max(np.abs(np.array([r['q'] for r in measured]) - origin), axis=0)
written_delta = np.max(np.abs(np.array([r['q'] for r in written]) - origin), axis=0)
epochs = [r for r in events if r['topic'].endswith('control_reference_epoch')]
assert len(epochs) == 2
max_gap = max((b['source_ns']-a['source_ns'])*1e-9 for a, b in zip(measured, measured[1:]))
report = dict(record=str(REC), record_result='failure',
    manifest_sha256=hashlib.sha256((REC / 'manifest.json').read_bytes()).hexdigest(),
    events=events, measured_sample_count=len(measured), written_command_count=len(written),
    measured_first=measured[0], measured_last=measured[-1],
    maximum_measured_delta_per_joint_rad=measured_delta.tolist(),
    maximum_written_delta_per_joint_rad=written_delta.tolist(),
    maximum_measured_source_gap_sec=max_gap,
    last_written_command=written[-1],
    epoch_to_perception_failure_sec=(loss['frame_timing']['failure_ros_ns']-epochs[-1]['epoch_ns'])*1e-9,
    observation_measurement_request_count=topic_counts.get('/grasp_mode/observe_return_measurement_request', 0),
    findings=[
        'A valid observation preview and execution plan were published before the task requested its bounded actuation-confirmation prefix.',
        'The controller received a three-second prefix with a planned maximum endpoint displacement of approximately 0.025 rad. Successful SDK write telemetry changed by up to 0.0245436926 rad, but all 199 accepted six-joint measurements remained exactly equal.',
        'The trajectory action reported success before the driver encoder-response timeout. Controller action completion therefore does not establish physical measured motion or actuation confirmation.',
        'At 1791608319.688858540 the driver retired the failed initial probe and issued a new reference epoch. Its log explicitly says accepted feedback and torque request remain unchanged. The driver then published UNCONFIRMED:ENCODER_RESPONSE_TIMEOUT.',
        'The perception frame had started at 1791608319.506439208 and failed at post_refine at 1791608319.695053815, approximately 6.20 ms after the new epoch. Its source was only 0.21229124 seconds old.',
        'The epoch callback clears the locked initial station reference and accepted-reference history. The observed ordering and zero measured motion support epoch revocation as the reason the in-flight initial support check failed, rather than physical station movement or frame-age expiry.',
        'No observation measurement request was recorded. Code starts bootstrap before execute(), while the ordinary measurement lease is begun in the later strict execution wrapper. This is a scope gap worth reviewing for a future physically moving prefix, but it is not demonstrated as the cause of this no-response attempt.',
        'The task reported ACTUATION_BOOTSTRAP_FAILED / ACTUATION_PREFIX_NO_RESPONSE only after the driver timeout and secondary perception invalidation. No approach, close or physical grasp success is evidenced.',
    ],
    limitations=[
        'Accepted encoder feedback demonstrates no observed joint displacement; it cannot alone determine whether the device ignored commands, reported frozen feedback or has another hardware/protocol fault.',
        'Subscriber receipt times use separate topic connections. Source timestamps and driver logs are preserved, but exact callback interleaving inside perception is inferred from the code and recorded timing, not captured by a thread trace.',
        'This analysis makes no changes, calls no live ROS service, sends no command and reads no temperature data.',
    ], physical_actions=False, production_edited=False, temperature_queried=False)
output = NEW / 'gen11_handoff_analysis/prefix_measurement_epoch_timeline.json'
assert not output.exists()
output.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + '\n')
print(json.dumps(dict(path=str(output), sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
    measured_samples=len(measured), maximum_measured_delta_rad=float(max(measured_delta)),
    maximum_written_delta_rad=float(max(written_delta)), epoch_to_failure_sec=report['epoch_to_perception_failure_sec'])))
