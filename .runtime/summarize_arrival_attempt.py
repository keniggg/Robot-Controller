"""Summarize a sealed record, preserving machine and human outcomes separately."""
from pathlib import Path
import argparse, hashlib, json, math, os, statistics
p=argparse.ArgumentParser();p.add_argument('--attempt',default='');a=p.parse_args()
R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/actual_gen63_baseline_validation_20261023T041149Z/high_accuracy_grasp_r1/arrival_margin_repair_r1')
root=R/a.attempt if a.attempt else R
out=root/'actual_validation'
record=Path(next(s.split('=',1)[1] for s in (out/'wrapper.log').read_text().splitlines() if s.startswith('GRASP_RECORD_DIR=')))
auth=json.loads((record/'authorization.json').read_text())
closure=json.loads((record/'recorder_outcome.json').read_text());assert closure['bag_closed']
os.environ['OFFLINE_BOUNDARY_LOG']=str(out/'summary_boundary.jsonl');exec((R/'guard.py').read_text())
import rosbag
selected=auth['selection']['stamp_ns']
states=[];goals=[];results={};plans=[];endpoints=[];margin=[];following=[]
topics=['/grasp/state','/alicia_controller/follow_joint_trajectory/goal',
    '/alicia_controller/follow_joint_trajectory/result','/grasp_6d/plan_enriched',
    '/experiment/runtime_execution_error_snapshot','/rosout_agg']
with rosbag.Bag(str(record/'keyframes.bag')) as b:
    for topic,m,t in b.read_messages(topics=topics):
        stamp=m.header.stamp.to_nsec() if hasattr(m,'header') else t.to_nsec()
        if topic.endswith('snapshot'):
            d=json.loads(m.data)
            if d.get('stamp_sec',0)*1e9>=selected:endpoints.append(d)
            continue
        if stamp<selected:continue
        if topic=='/grasp/state':states.append(dict(ns=stamp,active=m.active,state=m.state,message=m.message))
        elif topic.endswith('/goal'):
            goals.append(dict(ns=stamp,goal_id=m.goal_id.id,names=list(m.goal.trajectory.joint_names),
                endpoint=list(m.goal.trajectory.points[-1].positions),
                duration_sec=m.goal.trajectory.points[-1].time_from_start.to_sec()))
        elif topic.endswith('/result'):results[m.status.goal_id.id]=dict(ns=stamp,status=m.status.status,error_code=m.result.error_code)
        elif topic.endswith('/plan_enriched') and m.valid:
            if not any(row['plan_id']==m.plan_id for row in plans):
                plans.append(dict(plan_id=m.plan_id,phase=m.diagnostic,source_ns=stamp,received_ns=t.to_nsec()))
        elif topic=='/rosout_agg':
            for prefix,destination in [('Contact pregrasp arrival margin: ',margin),
                ('Pregrasp feedforward proposal: ',following),('Pregrasp feedforward measured result: ',following)]:
                if m.msg.startswith(prefix):destination.append(dict(ns=stamp,kind=prefix.rstrip(': '),data=json.loads(m.msg[len(prefix):])))
for goal in goals:goal['result']=results.get(goal['goal_id'])
active=[s['ns'] for s in states if s['active']]
inactive=[s['ns'] for s in states if not s['active']]
start=min(active) if active else selected
finish=max(inactive) if inactive else auth['finished_ns']
first_goal=min((g['ns'] for g in goals),default=finish)
resources={};rows=[json.loads(s) for s in (out/'resources/samples.jsonl').read_text().splitlines()]
windows={'initial_planning':(selected,start),'task_preparation':(start,first_goal),'whole_active_task':(start,finish)}
for label,(lo,hi) in windows.items():
    grouped={}
    for row in rows:
        if not lo<=row['wall_ns']<=hi:continue
        for name,process in row['processes'].items():
            if process.get('valid') and 'cpu_core_equivalents' in process:
                grouped.setdefault(name,[]).append(process)
    resources[label]=dict(wall_sec=(hi-lo)*1e-9,processes={name:dict(samples=len(items),
        mean_cpu_cores=statistics.mean(i['cpu_core_equivalents'] for i in items),
        peak_rss_mib=max(i['rss_bytes'] for i in items)/2**20,
        mean_read_bytes_sec=statistics.mean(i.get('io_per_sec',{}).get('read_bytes',0) for i in items),
        mean_write_bytes_sec=statistics.mean(i.get('io_per_sec',{}).get('write_bytes',0) for i in items)) for name,items in grouped.items()})
result=dict(record_dir=str(record),selection=auth['selection'],machine_result=json.loads((record/'command_result.json').read_text()),
    manual_restored=auth.get('manual_after'),manual_ack=auth.get('manual_ack'),final_idle=auth.get('final_idle'),
    task_states=states,goals=goals,plans=plans,endpoint_snapshots=endpoints,margin=margin,following=following,
    resources=resources,scope='Resource stages use recorded task and controller times; endpoint snapshots exclude old latched plans. No remote-server CPU measurement.')
confirmation=root/'physical_grip_confirmation.json'
if confirmation.exists():result['user_confirmation']=json.loads(confirmation.read_text())
(out/'task_resources_and_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(record=str(record),machine_result=result['machine_result']['result'],manual_restored=result['manual_restored'],
    goals=len(goals),phases=[row['phase'] for row in plans],
    endpoints=[dict(stage=e['stage_label'],position_mm=e['position_error_m']*1000,orientation_deg=math.degrees(e['orientation_error_rad'])) for e in endpoints],
    resource_stage_wall={n:v['wall_sec'] for n,v in resources.items()}),ensure_ascii=False))
