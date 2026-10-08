#!/usr/bin/env python3
"""Whitelisted-topic offline bag analysis; no ROS master, SDK or thermal fields."""
import sys
sys.dont_write_bytecode = True
import collections
import hashlib
import json
import pathlib
import re
import numpy as np
import rosbag

ROOT = pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD = ROOT / 'grasp_records/20261006T032914.897053Z_b4aed3dc145a43409c8562c79cdc0fe1'
OUTPUT = ROOT / 'ros_log/grasp_close_wire_retry_20261006T032914Z/diagnostic_motion_support_timeline.json'
SOURCE = 1791257419787791252
TOPICS = ['/alicia_d/accepted_joint_states', '/alicia_d/sdk_command',
          '/alicia_d/control_reference', '/alicia_controller/state',
          '/alicia_controller/follow_joint_trajectory/goal',
          '/alicia_controller/follow_joint_trajectory/result',
          '/grasp/state', '/perception/unknown/detector_status']


def main():
    assert pathlib.Path('/media/zhuyupei/SanStand').is_mount()
    assert OUTPUT.parent.is_dir()
    series = collections.defaultdict(list)
    goal = result = None
    states, observations = [], []
    with rosbag.Bag(str(RECORD / 'keyframes.bag')) as bag:
        for topic, message, bag_time in bag.read_messages(topics=TOPICS):
            stamp = message.header.stamp.to_nsec() if hasattr(message, 'header') else None
            if topic in TOPICS[:3]:
                assert len(message.position) >= 6
                series[topic].append(dict(stamp_ns=stamp, bag_ns=bag_time.to_nsec(),
                                         frame_id=message.header.frame_id,
                                         q=list(message.position[:6])))
            elif topic == '/alicia_controller/state':
                series[topic].append(dict(stamp_ns=stamp, bag_ns=bag_time.to_nsec(),
                    desired_q=list(message.desired.positions), actual_q=list(message.actual.positions),
                    desired_velocity=list(message.desired.velocities)))
            elif topic.endswith('/goal'):
                goal = dict(stamp_ns=stamp, bag_ns=bag_time.to_nsec(),
                    goal_id=message.goal_id.id,
                    duration_ns=message.goal.trajectory.points[-1].time_from_start.to_nsec(),
                    first_q=list(message.goal.trajectory.points[0].positions),
                    terminal_q=list(message.goal.trajectory.points[-1].positions))
            elif topic.endswith('/result'):
                result = dict(stamp_ns=stamp, bag_ns=bag_time.to_nsec(),
                    status=int(message.status.status), error_code=int(message.result.error_code))
            elif topic == '/grasp/state':
                states.append(dict(stamp_ns=stamp, state=message.state, active=message.active,
                                   success=message.success, message=message.message))
            else:
                d = json.loads(message.data)
                observations.append(dict(bag_ns=bag_time.to_nsec(),
                    source_stamp_ns=d.get('source_stamp_ns'), detail=d.get('detail'),
                    state=d.get('state'), target_lost=d.get('target_lost'),
                    generation=d.get('generation'),
                    status_source_age_sec=d.get('frame_timing', {}).get('status_source_age_sec'),
                    support_metrics={k:v for k,v in d.get('geometry_metrics', {}).items()
                                     if k.startswith('support_')}))
    assert goal and result
    terminal = np.array(goal['terminal_q'])
    accepted = series[TOPICS[0]]
    sdk = series[TOPICS[1]]

    def near(rows, ns):
        return min(rows, key=lambda r:abs(r['stamp_ns']-ns))

    def joint_snapshot(ns):
        a, s, c = near(accepted, ns), near(sdk, ns), near(series[TOPICS[3]], ns)
        aq, sq = np.array(a['q']), np.array(s['q'])
        return dict(requested_stamp_ns=ns, accepted=a, sdk_transmitted=s,
            accepted_time_error_ms=(a['stamp_ns']-ns)/1e6,
            sdk_time_error_ms=(s['stamp_ns']-ns)/1e6,
            accepted_minus_terminal_deg=np.degrees(aq-terminal).tolist(),
            sdk_minus_terminal_deg=np.degrees(sq-terminal).tolist(),
            accepted_minus_sdk_deg=np.degrees(aq-sq).tolist(),
            accepted_max_terminal_error_deg=float(np.max(abs(np.degrees(aq-terminal)))),
            accepted_max_sdk_error_deg=float(np.max(abs(np.degrees(aq-sq)))),
            controller=c)

    def window(lo, hi):
        rows = [r for r in accepted if lo <= r['stamp_ns'] <= hi]
        if not rows:
            return None
        qs = np.array([r['q'] for r in rows])
        return dict(start_stamp_ns=rows[0]['stamp_ns'], end_stamp_ns=rows[-1]['stamp_ns'],
            accepted_samples=len(rows), q_peak_to_peak_deg=np.degrees(np.ptp(qs, axis=0)).tolist(),
            net_q_change_deg=np.degrees(qs[-1]-qs[0]).tolist(),
            maximum_accepted_gap_ms=float(np.max(np.diff([r['stamp_ns'] for r in rows]))/1e6) if len(rows)>1 else None)

    contradict = [o for o in observations if 'support_plane_local_distance_inconsistent' in (o['detail'] or '')]
    cited = next(o for o in contradict if o['source_stamp_ns'] == SOURCE)
    after_result = [o for o in observations if (o['source_stamp_ns'] or 0)>result['stamp_ns']]
    failure = next(s for s in states if s['state']=='FAILED')
    diagnostic = dict(schema_version=1,
        scope='Offline closed-bag analysis only; no ROS calls, motion, thresholds or source edits; no temperature topics or fields read.',
        bag_path=str(RECORD/'keyframes.bag'),
        bag_sha256_from_sealed_manifest=next(f['sha256'] for f in json.loads((RECORD/'manifest.json').read_text())['files'] if f['path']=='keyframes.bag'),
        script_sha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),
        whitelist_topics=TOPICS, goal=goal, action_result=result, task_states=states,
        nominal_end_stamp_ns=goal['stamp_ns']+goal['duration_ns'],
        cited_contradiction=cited,
        cited_source_before_action_result_sec=(result['stamp_ns']-SOURCE)/1e9,
        cited_source_before_failure_sec=(failure['stamp_ns']-SOURCE)/1e9,
        first_contradiction=contradict[0], last_contradiction=contradict[-1],
        contradictions=contradict,
        snapshots={name:joint_snapshot(ns) for name,ns in [('cited_source',SOURCE),
            ('action_result',result['stamp_ns']),('task_failure',failure['stamp_ns']),
            ('bag_final_feedback',accepted[-1]['stamp_ns'])]},
        feedback_windows={name:window(lo,hi) for name,lo,hi in [
            ('around_source_plus_minus_half_sec',SOURCE-500000000,SOURCE+500000000),
            ('last_second_before_action_result',result['stamp_ns']-1000000000,result['stamp_ns']),
            ('first_second_after_action_result',result['stamp_ns'],result['stamp_ns']+1000000000),
            ('last_recorded_second',accepted[-1]['stamp_ns']-1000000000,accepted[-1]['stamp_ns'])]},
        observations_after_action_result=dict(count=len(after_result),
            reason_counts=dict(collections.Counter(o['detail'] for o in after_result)),
            observations=after_result),
        full_detector_timeline=observations,
        conclusion='The cited 4.268 mm contradiction was acquired during nonzero commanded motion, 3.180421 s before trajectory SUCCEEDED. The task later reported this earlier frame. All post-result detector observations in this bag lack support spatial coverage; they do not establish a new stopped-pose plane disagreement or prove the plane consistent. Accepted SDK feedback must be used to distinguish controller success from physical settling; see measured windows and endpoint errors.')
    OUTPUT.write_text(json.dumps(diagnostic,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in diagnostic.items() if k not in ['full_detector_timeline','contradictions','observations_after_action_result','task_states','whitelist_topics']},ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
