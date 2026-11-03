from pathlib import Path
import hashlib,json
NEW=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
OUT=NEW/'gen7_handoff_analysis'
files=[OUT/'station_compute_7_1791601238579388978.jsonl',OUT/'motion_boundary_7_1791601240571248348.jsonl']
rows=[]
for p in files:
 for line in p.open():
  r=json.loads(line);r['_observer_file']=p.name;rows.append(r)
rows.sort(key=lambda r:r['received_ros_ns'])
failed=next(r for r in rows if r['kind']=='detector' and r.get('target_lost'))
t=failed['received_ros_ns']
lastready=[r for r in rows if r['kind']=='detector' and r.get('state')=='ready' and r['received_ros_ns']<t][-5:]
q=[r for r in rows if r['kind']=='accepted_q']
stationq=[r for r in q if 1791601358711609125<=r['received_ros_ns']<=t]
qs=[r['q'] for r in stationq]
span=[max(v[i] for v in qs)-min(v[i] for v in qs) for i in range(6)]
events=[r for r in rows if r['kind'] in ('session','measurement_request','measurement_ack','task','initial_preflight_request','initial_preflight_ack','initial_preflight_attempt','preview','selection')]
geometries=[r for r in rows if r['kind']=='geometry' and r['received_ros_ns']<t][-6:]
postsecond=[r for r in stationq if r['received_ros_ns']>=1791601379267964839]
report=dict(generation=7,record='/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261010T030141.809371Z_7724ffb5910c4565bf751bd9c281954c',
 first_loss=failed,previous_ready_statuses=lastready,events=events,geometry_before_first_loss=geometries,
 real_motion=dict(maximum_accepted_joint_delta_rad=max(r['maximum_delta_rad'] for r in q),sample_count=len(q),
 qualified_to_failure_samples=len(stationq),qualified_to_failure_joint_ranges_rad=span,
 first_station_sample=stationq[0],last_station_sample=stationq[-1],samples_after_second_view_session=postsecond),
 observations=dict(first_observation_requalified=True,task_reached_nearfield_preview_wait=True,
 contact_plan_observed=any(r['kind']=='preview' and 'CONTACT' in r.get('diagnostic','') for r in rows),
 second_measurement_executing_observed=False,
 first_loss_reason=failed['frame_timing'].get('arrival_station_invalidated_reason'),
 first_loss_current_source_age_sec=failed['frame_timing'].get('failure_source_age_sec')),
 limitations=['observer receipt timestamps are not subscriber callback ingress timestamps',
 'ObjectPose and complete detector status require sealed bag; watcher carries selected fields only',
 'q stability establishes actual station motion separately from phase/view_index changes',
 'no CONTACT success or physical object grasp is claimed'])
out=OUT/'first_loss_observer_evidence.json';out.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
seal=dict(observers_closed=True,processes=[68894,68978],exit_codes=[0,0],files={str(p):sha(p) for p in files},report=str(out),report_sha256=sha(out),no_hardware_commands=True)
(OUT/'observers_closed_seal.json').write_text(json.dumps(seal,indent=2,sort_keys=True)+'\n')
print(json.dumps(dict(report=str(out),sha256=sha(out),station_q_span_rad=span,station_samples=len(stationq),geometry_latest=[(r['source_ns'],r['valid'],r['reason'][:100]) for r in geometries],max_delta_rad=report['real_motion']['maximum_accepted_joint_delta_rad'])))
