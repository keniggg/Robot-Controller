"""Retain bounded, read-only audit/log findings for the sealed third attempt."""
import hashlib,json,re
from pathlib import Path
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
N=E/'ros_log/full_latest_gen22_restart_20261015T003700Z'
R=E/'grasp_records/20261015T075503.791411Z_24849e8304a44341adebca3867d29fdd'
O=N/'verification/gen27_attempt3_initial_inference_review'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert json.loads((R/'manifest.json').read_text())['recorder_closed']
initial=R/'keyframes_audits/17d4f9d7ee2d80d84edb4031f085f42bf0fea965f9b6ec1802bd7054e890f14c.json'
contact=R/'keyframes_audits/bc7ee7f0499f35034b423b0892e93e7cd7741457ce0056902b3c9f0e4d46cd9a.json'
a=json.loads(initial.read_text());c=json.loads(contact.read_text())
def named(obj,wanted,path=''):
    out=[]
    if isinstance(obj,dict):
        for k,v in obj.items():
            if k in wanted:out.append(dict(path=path+'/'+k,value=v))
            else:out.extend(named(v,wanted,path+'/'+k))
    elif isinstance(obj,list):
        for i,v in enumerate(obj):out.extend(named(v,wanted,path+'/'+str(i)))
    return out
lines=[];path_audits=[]
ansi=re.compile(r'\x1b\[[0-9;]*m')
with (N/'core.log').open('rb') as f:
    while True:
        offset=f.tell();raw=f.readline()
        if not raw:break
        line=ansi.sub('',raw.decode('utf8','replace')).strip()
        m=re.search(r'\[(1792050\d{3}\.\d+)',line)
        if not m or not 1792050904<float(m[1])<1792051079:continue
        if 'Frozen observation continuous path audit: ' in line:
            data=json.loads(line.split('Frozen observation continuous path audit: ',1)[1])
            keys=['trajectory_sha256','plan_id','scene_sha256','certified_minimum_clearance_m','controller_hold_provenance','bridge','bridge_start_joint_positions_rad']
            path_audits.append(dict(log_byte_offset=offset,raw_line_sha256=hashlib.sha256(raw).hexdigest(),stamp_sec=m[1],audit={k:data[k] for k in keys if k in data}))
        elif any(s in line for s in ['Control ownership synchronized','manual mode','Manual mode','synchronization held','controller synchronization','station_compute_wait_expired_or_future']):
            lines.append(dict(log_byte_offset=offset,raw_line_sha256=hashlib.sha256(raw).hexdigest(),line=line))
report=dict(record=str(R),initial_audit=dict(path=str(initial),sha256=hashlib.sha256(initial.read_bytes()).hexdigest(),snapshot_stamp_ns=a['snapshot_stamp_ns'],planning_origin=a['planning_origin'],
    trigger=a['observe_return']['trigger_code'],trigger_evidence=a['observe_return']['trigger_evidence'],remote_diagnostics=a['remote_diagnostics'],
    retained_regular_box_entries=named(a,{'regular_box_inference'})),
    contact_audit=dict(path=str(contact),sha256=hashlib.sha256(contact.read_bytes()).hexdigest(),snapshot_stamp_ns=c['snapshot_stamp_ns'],
      retained_box_entries=named(c,{'regular_box_inference','regular_box_planning_enclosure'})),handoff_path_audits=path_audits,non_temperature_log_lines=lines,
    findings=[
      'Initial preflight materialized zero contact candidates; contact search and learned candidate request were not completed before observation transition.',
      'Initial trigger points to weak measured side evidence; original initial box qualification audit was not retained in this observation audit.',
      'Initial_three_frame_replay.json is a qualified offline reconstruction with explicit limits; its BOX_TOP_NOT_RECTANGULAR result is not relabeled as the exact original decision.',
      'Near-field contact audit records box qualification false; current_inferred_box only exposes ok models, so that collision did not use a qualified inferred-box enclosure.',
      'The observed automatic movement used completed_frozen_sdk_handoff with previously successful SDK words, not a new pending positive-enable prefix.',
      'Power-restart temporal association does not establish a hardware root cause; GUI and automatic trajectory evidence are kept distinct.'
    ],no_ros_calls=True,no_hardware_access=True,no_temperature_reads=True)
p=O/'handoff_and_gate_findings.json';assert not p.exists();p.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),handoff_audits=len(path_audits),selected_log_lines=len(lines)),ensure_ascii=False))
