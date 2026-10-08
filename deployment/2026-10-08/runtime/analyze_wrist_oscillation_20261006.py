#!/usr/bin/env python3
"""Historical encoders/commands only. No device, thermal fields or tuning."""
import sys
sys.dont_write_bytecode = True
import os
import collections
import hashlib
import json
import pathlib
import numpy as np
import rosbag

ROOT = pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD = ROOT/'grasp_records/20261006T032914.897053Z_b4aed3dc145a43409c8562c79cdc0fe1'
OUT = ROOT/'ros_log/grasp_close_wire_retry_20261006T032914Z/wrist_oscillation_diagnostic'
NAMES = ['Joint%d'%i for i in range(1,7)]
QUANTUM = 2*np.pi/4096
TOPICS = ['/alicia_d/accepted_joint_states', '/alicia_d/sdk_command', '/alicia_d/control_reference',
          '/alicia_controller/state', '/alicia_controller/follow_joint_trajectory/goal',
          '/alicia_controller/follow_joint_trajectory/result', '/gui/joint_direct_mode', '/rosout_agg']


def numerical_metrics(q, t, axis, goal_direction, epsilon=QUANTUM*.49):
    v = np.asarray(q)[:,axis]
    dv = np.diff(v)
    changing = np.flatnonzero(abs(dv)>epsilon)
    signs = np.sign(dv[changing])
    reversals = changing[1:][signs[1:] != signs[:-1]] if len(signs)>1 else []
    opposite = dv*goal_direction < -epsilon
    runs=[]
    for i in changing:
        sign=int(np.sign(dv[i]))
        if runs and runs[-1]['sign']==sign:
            runs[-1]['end']=i+1
        else:
            runs.append(dict(sign=sign,start=int(i),end=int(i+1)))
    opposite_runs=[]
    for r in runs:
        if r['sign']*goal_direction<0:
            delta=float(v[r['end']]-v[r['start']])
            opposite_runs.append(dict(start_relative_s=float(t[r['start']]),end_relative_s=float(t[r['end']]),
                delta_deg=float(np.degrees(delta)),delta_quantums=float(delta/QUANTUM)))
    return dict(samples=len(v), change_threshold_rad=float(epsilon), peak_to_peak_deg=float(np.degrees(np.ptp(v))),
        net_motion_deg=float(np.degrees(v[-1]-v[0])),
        total_variation_deg=float(np.degrees(np.sum(abs(dv)))),
        changed_samples=int(len(changing)),direction_reversals=int(len(reversals)),
        reversal_times_sec=[float(t[i+1]) for i in reversals],
        opposite_to_goal_steps=int(np.count_nonzero(opposite)),
        max_opposite_run_deg=max([abs(r['delta_deg']) for r in opposite_runs] or [0.]),
        opposite_runs=opposite_runs,
        interpretation='One SDK quantum is 0.087890625 deg. Reversals count sign changes after omitting repeated quantized positions; this does not by itself establish mechanical oscillation.')


def main():
    assert pathlib.Path('/media/zhuyupei/SanStand').is_mount()
    OUT.mkdir(exist_ok=True)
    os.environ['MPLCONFIGDIR']=str(OUT/'matplotlib_cache')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    data=collections.defaultdict(list)
    goals=[];results=[];modes=[];trim_logs=[]
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as b:
        for topic,m,bt in b.read_messages(topics=TOPICS):
            if topic in TOPICS[:3]:
                data[topic].append((m.header.stamp.to_nsec(),list(m.position[:6])))
            elif topic == '/alicia_controller/state':
                data['desired'].append((m.header.stamp.to_nsec(),list(m.desired.positions)))
                data['desired_velocity'].append((m.header.stamp.to_nsec(),list(m.desired.velocities)))
            elif topic.endswith('/goal'):
                goals.append(dict(stamp_ns=m.header.stamp.to_nsec(),duration_ns=m.goal.trajectory.points[-1].time_from_start.to_nsec(),
                    start_q=list(m.goal.trajectory.points[0].positions),end_q=list(m.goal.trajectory.points[-1].positions)))
            elif topic.endswith('/result'):
                results.append(dict(stamp_ns=m.header.stamp.to_nsec(),status=int(m.status.status)))
            elif topic == '/gui/joint_direct_mode':
                modes.append(dict(bag_ns=bt.to_nsec(),manual_direct=bool(m.data)))
            elif m.name == '/alicia_d_driver_node' and m.msg.startswith('Endpoint trim transition'):
                trim_logs.append(dict(stamp_ns=m.header.stamp.to_nsec(),message=m.msg))
    assert len(goals)==len(results)==1
    goal,result=goals[0],results[0]
    t0=goal['stamp_ns'];end=goal['stamp_ns']+goal['duration_ns'];end_relative=(end-t0)/1e9
    goal_direction=np.sign(np.array(goal['end_q'])-np.array(goal['start_q']))
    arrays={k:(np.array([row[0] for row in rows],dtype=np.int64),np.array([row[1] for row in rows])) for k,rows in data.items()}
    report=dict(scope='Historical offline diagnostics, no live control or tuning; no temperature data read.',
        record=str(RECORD),script_sha256=hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),
        feedback_topic='/alicia_d/accepted_joint_states',feedback_provenance='sdk_measured accepted packet; controller actual velocities are dummy zero and not usable for settling.',
        sdk_quantum_deg=float(np.degrees(QUANTUM)),goal=goal,result=result,manual_mode_messages=modes,
        endpoint_trim_transition_logs=trim_logs, metrics={},cadence={})
    windows=[('trajectory',t0,end),('final_5s_and_settle',end-5000000000,arrays[TOPICS[0]][0][-1]),
             ('after_action_result',result['stamp_ns'],arrays[TOPICS[0]][0][-1]),
             ('before_goal',t0-4000000000,t0)]
    for label,lo,hi in windows:
        report['metrics'][label]={}
        for key in (TOPICS[0],TOPICS[1],TOPICS[2],'desired'):
            times,qs=arrays[key];mask=(times>=lo)&(times<=hi);times=times[mask];qs=qs[mask]
            if not len(times):continue
            report['metrics'][label][key]={name:numerical_metrics(qs,(times-t0)/1e9,i,goal_direction[i],
                epsilon=1e-9 if key=='desired' else QUANTUM*.49) for i,name in enumerate(NAMES)}
    for key,(times,qs) in arrays.items():
        intervals=np.diff(times)/1e6
        report['cadence'][key]=dict(count=len(times),median_interval_ms=float(np.median(intervals)),max_interval_ms=float(np.max(intervals)))
    for label,lim in [('wrist_full',(-1.,(arrays[TOPICS[0]][0][-1]-t0)/1e9)),
                      ('wrist_settle',(end_relative-6,(arrays[TOPICS[0]][0][-1]-t0)/1e9))]:
        fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True,constrained_layout=True)
        for i,ax in zip(range(3,6),axes):
            for key,name,color,style in [(TOPICS[0],'Accepted SDK feedback','#125db5','-'),
                    (TOPICS[1],'SDK transmitted','#dd7919','-'),('desired','Controller desired','#333333','--')]:
                times,qs=arrays[key];t=(times-t0)/1e9
                y=np.degrees(qs[:,i]-goal['end_q'][i]) if label=='wrist_settle' else np.degrees(qs[:,i])
                visible=(t>=lim[0])&(t<=lim[1])
                ax.plot(t[visible],y[visible],label=name,color=color,lw=1.2,ls=style)
            ax.axvline(end_relative,color='#b73434',ls=':',label='Planned end')
            ax.axvline((result['stamp_ns']-t0)/1e9,color='#653b91',ls=':',label='Action success')
            ax.set_ylabel('J%d %s (deg)'%(i+1,'- target' if label=='wrist_settle' else 'position'))
            ax.grid(alpha=.25);ax.set_xlim(*lim)
        axes[0].legend(ncol=3,fontsize=9)
        axes[-1].set_xlabel('Seconds after recorded trajectory goal (bag clock)')
        fig.suptitle('Wrist commands and real feedback — offline sealed bag')
        fig.savefig(str(OUT/(label+'.png')),dpi=150)
        plt.close(fig)
    end_metrics={}
    for i,name in enumerate(NAMES):
        times,qs=arrays[TOPICS[0]];mask=times>=t0;v=qs[mask,i];ts=times[mask]
        final=qs[-1,i];changed=np.flatnonzero(abs(np.diff(v))>QUANTUM*.49)
        lastchange=int(ts[changed[-1]+1]) if len(changed) else None
        end_metrics[name]=dict(final_minus_target_deg=float(np.degrees(final-goal['end_q'][i])),
            last_observed_change_stamp_ns=lastchange,
            last_observed_change_after_nominal_end_sec=(lastchange-end)/1e9 if lastchange else None,
            maximum_terminal_direction_overrun_deg=float(max(0,np.max(goal_direction[i]*(v[ts>=end-1000000000]-goal['end_q'][i])))*180/np.pi))
    matched_reference = dict(data[TOPICS[2]])
    exact_reference_pairs = [(q,matched_reference[t]) for t,q in data[TOPICS[1]] if t in matched_reference]
    report['sdk_reference_exact_match'] = dict(sdk_count=len(data[TOPICS[1]]),
        matched_header_timestamps=len(exact_reference_pairs),
        all_positions_identical=all(a==b for a,b in exact_reference_pairs),
        interpretation='These two topics are copies of the same successful write in driver source. Equality does not independently prove trim inactive.')
    sdk_times,sdk_q=arrays[TOPICS[1]]
    desired_times,desired_q=arrays['desired']
    during=(sdk_times>=t0)&(sdk_times<=arrays[TOPICS[0]][0][-1])
    interpolated=np.stack([np.interp((sdk_times[during]-t0)/1e9,(desired_times-t0)/1e9,desired_q[:,i]) for i in range(6)],axis=1)
    report['sdk_minus_interpolated_desired_deg']=dict(
        maximum_absolute_by_axis=np.degrees(np.max(abs(sdk_q[during]-interpolated),axis=0)).tolist(),
        median_absolute_by_axis=np.degrees(np.median(abs(sdk_q[during]-interpolated),axis=0)).tolist(),
        interpretation='Offline timestamp interpolation only; asynchronous command timing remains a small confound. No feedback was replaced.')
    report['endpoint_metrics']=end_metrics
    (OUT/'encoder_command_analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    compact={label:{key:{j:{k:v for k,v in vals.items() if k in ['peak_to_peak_deg','net_motion_deg','changed_samples','direction_reversals','max_opposite_run_deg','opposite_to_goal_steps']} for j,vals in axes.items()} for key,axes in sources.items()} for label,sources in report['metrics'].items()}
    print(json.dumps(dict(output=str(OUT),metrics=compact,endpoint=end_metrics,cadence=report['cadence']),indent=2))


if __name__=='__main__':main()
