#!/usr/bin/env python3
"""Read a sealed attempt; no ROS initialization, services, publishers or tuning."""
import collections
from decimal import Decimal
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import sys
sys.dont_write_bytecode = True
import numpy as np
import rosbag

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD = ROOT/'grasp_records/20261006T040304.829106Z_1f3530bcf6b345fbab0305a8c730a064'
RUN = ROOT/'ros_log/grasp_support_fixed_20261006T040304Z'
OUT = RUN/'independent_motion_diagnostic'
NODE = ROOT/'ros_log/support_source_task_deployment_20261006T040240Z/node.log'
NAMES = ['Joint%d' % i for i in range(1, 7)]
Q = 2*np.pi/4096
TOPICS = ['/alicia_d/accepted_joint_states', '/alicia_d/sdk_command',
          '/alicia_d/control_reference', '/alicia_controller/state',
          '/alicia_controller/follow_joint_trajectory/goal',
          '/alicia_controller/follow_joint_trajectory/result',
          '/gui/joint_direct_mode', '/grasp/state', '/rosout_agg']


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(2**20), b''):
            h.update(block)
    return h.hexdigest()


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    manifest = json.loads((RECORD/'manifest.json').read_text())
    assert manifest['recorder_closed'] and manifest['state'] == 'complete'
    method = Path(__file__).with_name('analyze_wrist_oscillation_20261006.py')
    spec = importlib.util.spec_from_file_location('wrist_method', str(method))
    previous = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(previous)
    metrics = previous.numerical_metrics
    data = collections.defaultdict(list)
    goals, results, modes, states, trims, finger = [], {}, [], [], [], []
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
        for topic, m, at in bag.read_messages(topics=TOPICS):
            if topic in TOPICS[:3]:
                idx = {name: i for i, name in enumerate(m.name)}
                if not all(name in idx for name in NAMES):
                    continue
                data[topic].append((m.header.stamp.to_nsec(),
                                    [m.position[idx[name]] for name in NAMES]))
                if topic == TOPICS[0] and 'right_finger' in idx:
                    finger.append((m.header.stamp.to_nsec(), m.position[idx['right_finger']]))
            elif topic == '/alicia_controller/state':
                data['desired'].append((m.header.stamp.to_nsec(), list(m.desired.positions)))
            elif topic.endswith('/goal'):
                goals.append(dict(goal_id=m.goal_id.id, stamp_ns=m.header.stamp.to_nsec(),
                    duration_ns=m.goal.trajectory.points[-1].time_from_start.to_nsec(),
                    start_q=list(m.goal.trajectory.points[0].positions),
                    end_q=list(m.goal.trajectory.points[-1].positions),
                    trajectory_points=len(m.goal.trajectory.points)))
            elif topic.endswith('/result'):
                results[m.status.goal_id.id] = dict(stamp_ns=m.header.stamp.to_nsec(),
                    status=int(m.status.status), error_code=int(m.result.error_code),
                    error_string=m.result.error_string)
            elif topic == '/gui/joint_direct_mode':
                modes.append(dict(bag_ns=at.to_nsec(), manual_direct=bool(m.data)))
            elif topic == '/grasp/state':
                states.append(dict(stamp_ns=m.header.stamp.to_nsec(), bag_ns=at.to_nsec(),
                                   stage=int(m.stage), message=m.message))
            elif m.name == '/alicia_d_driver_node' and m.msg.startswith('Endpoint trim transition'):
                trims.append(dict(stamp_ns=m.header.stamp.to_nsec(), message=m.msg))
    assert len(goals) == len(results) == 4
    arrays = {key: (np.array([r[0] for r in rows], dtype=np.int64),
                    np.array([r[1] for r in rows])) for key, rows in data.items()}
    terminal = next(s for s in states if s['stage'] == 7)
    first = goals[0]['stamp_ns']
    labels = ['pregrasp', 'pregrasp_fixed_endpoint_compensation',
              'bounded_approach_compensation', 'linear_grasp_pose']
    action_reports = []
    for i, (label, goal) in enumerate(zip(labels, goals)):
        result = results[goal['goal_id']]
        t0 = goal['stamp_ns']
        end = t0+goal['duration_ns']
        hi = goals[i+1]['stamp_ns']-1 if i+1 < len(goals) else terminal['stamp_ns']
        direction = np.sign(np.array(goal['end_q'])-np.array(goal['start_q']))
        report = dict(label=label, goal=goal, result=result,
                      actual_action_duration_sec=(result['stamp_ns']-t0)/1e9,
                      post_result_window_sec=(hi-result['stamp_ns'])/1e9,
                      metrics={}, endpoint={})
        for win, lo, stop in [('trajectory', t0, end),
                              ('final_1s_and_hold', end-10**9, hi),
                              ('after_action_result', result['stamp_ns'], hi)]:
            report['metrics'][win] = {}
            for key, (times, qs) in arrays.items():
                mask = (times >= lo) & (times <= stop)
                if not np.any(mask):
                    continue
                report['metrics'][win][key] = {name: metrics(qs[mask], (times[mask]-t0)/1e9,
                    axis, direction[axis], epsilon=1e-9 if key == 'desired' else Q*.49)
                    for axis, name in enumerate(NAMES)}
        times, qs = arrays[TOPICS[0]]
        mask = (times >= t0) & (times <= hi)
        ts, vals = times[mask], qs[mask]
        for axis, name in enumerate(NAMES):
            changed = np.flatnonzero(abs(np.diff(vals[:, axis])) > Q*.49)
            last = int(ts[changed[-1]+1]) if len(changed) else None
            near = vals[ts >= end-10**9, axis]
            report['endpoint'][name] = dict(
                final_minus_command_goal_deg=float(np.degrees(vals[-1, axis]-goal['end_q'][axis])),
                last_observed_change_ns=last,
                last_observed_change_after_nominal_end_sec=(last-end)/1e9 if last else None,
                maximum_terminal_direction_overrun_deg=float(np.degrees(max(0,
                    np.max(direction[axis]*(near-goal['end_q'][axis]))))) if len(near) else None)
        action_reports.append(report)
    qualifications = collections.defaultdict(list)
    timeline = []
    compensation = []
    contact = None
    for line in NODE.read_text().splitlines():
        match = re.search(r'\[(\d+\.\d+)\]: (.*)', line)
        if not match:
            continue
        stamp = int(Decimal(match.group(1))*10**9)
        text = match.group(2)
        if 'Support source qualification: ' in text:
            d = json.loads(text.split('Support source qualification: ', 1)[1])
            if d.get('generation') == 4:
                qualifications[d['source_stamp_ns']].append(dict(log_stamp_ns=stamp, evidence=d))
        elif any(s in text for s in ['[Grasp]', 'Grasp motion settled',
                                    'Measured rich-6D endpoint', 'Measured fixed endpoint compensation completed',
                                    'Measured contact before closure']):
            timeline.append(dict(stamp_ns=stamp, after_first_action_sec=(stamp-first)/1e9, message=text))
            if 'Measured fixed endpoint compensation completed: ' in text:
                compensation.append(dict(stamp_ns=stamp, evidence=json.loads(text.split('completed: ', 1)[1])))
            if 'Measured contact before closure: ' in text:
                contact = dict(stamp_ns=stamp, evidence=json.loads(text.split('closure: ', 1)[1]))
    source_report = []
    for source, events in sorted(qualifications.items()):
        resolved = next((event for event in events if event['evidence']['source_qualification']['state'] in ('moving', 'stationary')), None)
        source_report.append(dict(source_stamp_ns=source, events=events,
            final_state=events[-1]['evidence']['source_qualification']['state'],
            delay_from_first_log_to_resolved_ms=(resolved['log_stamp_ns']-events[0]['log_stamp_ns'])/1e6 if resolved else None))
    cadence = {}
    for key, (times, qs) in arrays.items():
        delta = np.diff(times)/1e6
        cadence[key] = dict(samples=len(times), median_ms=float(np.median(delta)),
                            max_ms=float(np.max(delta)), nonpositive_intervals=int(np.count_nonzero(delta <= 0)))
    accepted_times, accepted_q = arrays[TOPICS[0]]
    post = (accepted_times >= terminal['stamp_ns'])
    after_failure = {name: metrics(accepted_q[post], (accepted_times[post]-first)/1e9, axis, 0)
                     for axis, name in enumerate(NAMES)} if np.any(post) else {}
    report = dict(schema_version=1, scope='Sealed bag and saved node.log only; no ROS/device interaction or tuning.',
        record=str(RECORD), run=str(RUN), source_bag_sha256=sha(RECORD/'keyframes.bag'),
        analysis_script_sha256=sha(Path(__file__)), inherited_metric_script_sha256=sha(method),
        source_manifest_sha256=sha(RECORD/'manifest.json'), node_log_snapshot_sha256=sha(NODE),
        outcome=manifest['result'], failure_reason=manifest['failure_reason'],
        feedback_provenance='sdk_measured accepted packets; controller actual velocities are dummy zero and unused.',
        action_count=len(goals), actions=action_reports, states=states, timeline=timeline,
        compensation=compensation, contact_before_closure=contact, cadence=cadence,
        support=dict(unique_sources=len(source_report), final_states=dict(collections.Counter(r['final_state'] for r in source_report)),
                     initially_unqualified=sum(r['events'][0]['evidence']['source_qualification']['state']=='unqualified' for r in source_report),
                     sources=source_report, action_blocked_by_support=False),
        manual_mode_messages=modes, trim_transition_logs=trims,
        after_failure_encoder_metrics=after_failure,
        measured_finger=dict(samples=len(finger), minimum_m=min(x[1] for x in finger), maximum_m=max(x[1] for x in finger),
                             initial_m=finger[0][1], final_m=finger[-1][1]),
        physical_grip_verified=False, gripper_closure_stage_observed=False, lift_stage_observed=False,
        no_temperature_topics_read=True, controls_sent=False, production_files_changed=False,
        limitations=['Encoder reversals include SDK quantization and are not proof of mechanical oscillation.',
                     'The settled model endpoint and nominal overlap are model evidence, not externally measured grip.',
                     'Node timestamps are ROS log creation times; bag delivery times approximate callback timing.',
                     'No PID or actuator parameter change is derived from this analysis.'])
    OUT.mkdir(exist_ok=True)
    os.environ['MPLCONFIGDIR'] = str(OUT/'matplotlib_cache')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for label, lo, hi, ref in [('wrist_full', first-10**9, accepted_times[-1], np.zeros(6))] + [
            ('wrist_'+a['label']+'_settle', a['goal']['stamp_ns']+a['goal']['duration_ns']-2*10**9,
             goals[i+1]['stamp_ns']-1 if i+1 < len(goals) else accepted_times[-1], np.array(a['goal']['end_q']))
            for i,a in enumerate(action_reports)]:
        fig, axes = plt.subplots(3,1,figsize=(11,8),sharex=True,constrained_layout=True)
        for axis, ax in zip(range(3,6), axes):
            for key, name, color in [(TOPICS[0],'Accepted SDK feedback','#125db5'),
                                     (TOPICS[1],'SDK transmitted','#dd7919'), ('desired','Controller desired','#333333')]:
                times, qs = arrays[key]
                mask=(times>=lo)&(times<=hi)
                ax.plot((times[mask]-first)/1e9,np.degrees(qs[mask,axis]-ref[axis]),label=name,color=color,lw=1.2)
            for a in action_reports:
                end=a['goal']['stamp_ns']+a['goal']['duration_ns']
                if lo<=end<=hi:ax.axvline((end-first)/1e9,color='#b73434',ls=':')
                if lo<=a['result']['stamp_ns']<=hi:ax.axvline((a['result']['stamp_ns']-first)/1e9,color='#653b91',ls=':')
            ax.grid(alpha=.25);ax.set_ylabel('J%d%s (deg)'%(axis+1,'' if label=='wrist_full' else ' - goal'))
        axes[0].legend(ncol=3,fontsize=9)
        axes[-1].set_xlabel('Seconds after first trajectory goal; red=planned end, purple=action result')
        fig.suptitle('Sealed attempt: wrist commands and real feedback')
        fig.savefig(str(OUT/(label+'.png')),dpi=150);plt.close(fig)
    path=OUT/'analysis.json'
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(report=str(path), action_count=len(goals), support={k:v for k,v in report['support'].items() if k!='sources'},
                         measured_finger=report['measured_finger'], cadence=cadence),ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
