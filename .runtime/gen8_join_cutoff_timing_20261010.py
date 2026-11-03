import hashlib,json
from pathlib import Path
import numpy as np
NEW=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
p=NEW/'gen8_handoff_analysis/submission_handoff_fixture.json';f=json.loads(p.read_text())
assert hashlib.sha256(p.read_bytes()).hexdigest()=='513eae1df28b72128d563a3a6cc1898e0e087feafdd276eb3c0f94167cc41b5b'
j=f['all_ready_three_source_joins'];ps=f['pending_geometry_timeline'];intervals=np.diff([r['source_ns'] for r in j])
rows=[]
for i,r in enumerate(j[:16]):
 d=dict(r);d['complete_received_ns']=max(r['received_ns'].values());d['source_interval_ms']=None if i==0 else (r['source_ns']-j[i-1]['source_ns'])/1e6
 d['source_cutoff_if_this_row_commits_as_recovery']=r['source_ns']-1
 d['last_three_completed_source_ns']=[v['source_ns'] for v in j[max(0,i-2):i+1]]
 d['sources_rejected_by_that_cutoff']=[s for s in d['last_three_completed_source_ns'] if s<=r['source_ns']-1]
 rows.append(d)
pending=[]
for r in ps[:12]:
 x=dict(r);later=[v for v in j if v['source_ns']>r['contract_source_ns']]
 if later:
  nextrow=later[0];join=max(nextrow['received_ns'].values());x.update(next_ready_source_ns=nextrow['source_ns'],next_complete_received_ns=join,
   next_join_minus_old_geometry_deadline_ms=(join-r['geometry_deadline_ns'])/1e6,
   pending_receive_minus_next_complete_ms=(r['received_ns']-join)/1e6,
   pending_message_stamp_minus_next_complete_ms=(r['message_source_ns']-join)/1e6)
 pending.append(x)
report=dict(source_fixture=str(p),source_fixture_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
 source_interval_ms=dict(min=float(intervals.min()/1e6),median=float(np.median(intervals)/1e6),p90=float(np.quantile(intervals,.9)/1e6),max=float(intervals.max()/1e6)),
 complete_join_age_sec=f['join_summary'],first_ready_rows=rows,first_pending_vs_next_ready=pending,
 code_mapping=dict(recovery_commit='station_compute_wait_remote.py:_station_compute_cutoff_ns=max(previous,ns-1)',submission='mode_aware_remote_grasp6d.py:submit_stream_snapshot rejects any sample_stamp_ns <= _station_compute_cutoff_ns'),
 limitations=['Cutoff values above are conditional arithmetic from the deployed source144 recovery branch, not instrumented assignments from the remote process.',
 'Recorded complete joins prove publication/recorder delivery only; separate real-callback replay must determine whether each recovery commit actually executed.',
 'No reconstructed source timestamp, modified expiry, ROS publisher or physical command is used.'])
out=NEW/'gen8_handoff_analysis/join_cutoff_timing.json';out.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
print(json.dumps(dict(path=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest(),intervals=report['source_interval_ms'],first_pending=pending[0])))
