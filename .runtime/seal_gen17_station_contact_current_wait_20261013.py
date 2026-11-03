#!/usr/bin/env python3
from pathlib import Path
import difflib,hashlib,json,sys,time
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen17_station_contact_current_wait_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification/gen17_station_contact_current_wait_delivery'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir()
release=N/'verification/gen16_success_first_release_manifest.json';prior=json.loads(release.read_text());sealed=prior['source_sha256']
assert len(sealed)==915 and all(sha(W/r)==v for r,v in sealed.items())
evidence={}
for name in ('gen17_station_contact_current_wait_r1','gen17_station_contact_current_wait_boundary_r2'):
 p=N/'verification'/name/'result.json';d=json.loads(p.read_text());assert d['passed'] and d['actual_unchanged'] and d['source_unchanged']
 assert all(sha(C/r)==v for r,v in d['candidate_sha256'].items())
 evidence[str(p)]=sha(p)
# Bind original raw-wire extraction without rereading or hashing the bag.
wire=N/'verification/attempt5_station_timeline_r1/wire.json';wire_result=wire.with_name('result.json')
rows=json.loads(wire.read_text());recorded_source=1791890537728708505
source=next(r for r in rows if r['topic']=='/perception/unknown/detector_status' and r['message'].get('source_stamp_ns')==recorded_source)
next_source=next(r for r in rows if r['topic']=='/perception/unknown/detector_status' and r['message'].get('source_stamp_ns')==1791890537992609739)
log=next(r for r in rows if r['topic']=='/rosout_agg' and 'station_compute_wait_registration_too_late' in r['message'].get('msg',''))
sys.path.insert(0,str(W/'src/alicia_grasp_modes/src'))
from alicia_grasp_modes.station_compute_reacquisition import validate_contract,validate_registration
contract=source['message']['station_compute_wait'];now=log['message']['header']['stamp']
assert validate_contract(contract,binding=contract['binding'],now_ns=now)=='PENDING'
try:validate_registration(contract,binding=contract['binding'],now_ns=now)
except ValueError as exc:assert str(exc)=='station_compute_wait_registration_too_late'
else:raise AssertionError('original late registration not reproduced from exact wire')
audit=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261013T112006.120233Z_d2b04edd648643b4b57d8e9febc4d88c/keyframes_audits/dc4c84e52e3a3b299bdc27a77afaa0504ce70b0f3ceeee26251a519f9e835c8e.json')
a=json.loads(audit.read_text());r=a['station_contact_computation']
diag=dict(kind='R5_ALREADY_REGISTERED_CURRENT_SOURCE_EXPIRY',source_ns=recorded_source,source_wire_index=source['index'],source_received_ns=source['received_ns'],contract=contract,
 log_wire_index=log['index'],log_stamp_ns=now,log_received_ns=log['received_ns'],age_at_log_ns=now-recorded_source,
 geometry_overrun_at_log_ns=now-contract['geometry_deadline_ns'],next_ready_wire_index=next_source['index'],next_ready_received_ns=next_source['received_ns'],next_source_ns=next_source['message']['source_stamp_ns'],
 audit=str(audit),audit_sha256=sha(audit),receipt_issued_ns=r['issued_ns'],receipt_current_source_ns=r['current']['contract']['source_ns'],receipt_current_remaining_ns=r['current']['contract']['geometry_deadline_ns']-r['issued_ns'],
 original_wire_validation_result='validate_contract=PENDING; validate_registration raises station_compute_wait_registration_too_late',
 evidence_scope='The wire confirms current source and elapsed deadline at logging, not the exact per-line internal clock or callback lock acquisition. Deterministic offline tests establish that boundary behavior and recovery with fresh real callbacks.',
 physical_result='FAILED; no physical grasp success claimed',wire=str(wire),wire_sha256=sha(wire),wire_extraction_result=str(wire_result),wire_extraction_result_sha256=sha(wire_result))
p=O/'diagnosis.json';p.write_text(json.dumps(diag,indent=2)+'\n');evidence[str(p)]=sha(p)
files=['src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py','src/alicia_grasp_modes/tests/test_station_contact_current_wait.py'];records=[];patch=[]
for rel in files:
 old=W/rel;new=C/rel;records.append(dict(path=rel,base_sha256=sha(old) if old.exists() else None,candidate_sha256=sha(new)))
 patch.extend(difflib.unified_diff(old.read_text().splitlines(True) if old.exists() else [],new.read_text().splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
patchfile=O/'candidate.patch';patchfile.write_text(''.join(patch));evidence[str(patchfile)]=sha(patchfile)
scope=list(prior['test_scope']);assert len(scope)==176
for rel in files:
 if '/tests/' in rel and rel not in scope:scope.append(rel)
manifest=dict(schema=1,kind='GEN17_ALREADY_REGISTERED_STATION_CURRENT_WAIT',passed=True,candidate_root=str(C),files=records,test_scope=scope,
 focused_test_scope=json.loads((N/'verification/gen17_station_contact_current_wait_r1/result.json').read_text())['test_scope'],
 proof_sha256=evidence,baseline_release=str(release),baseline_release_sha256=sha(release),baseline_source_sha256=sealed,
 baseline_expected_failures=3,candidate_matched_cases=3,focused_passed=165,
 changes=['Only already independently registered current geometry expiry becomes pending with public geometry quarantined; no contract deadline is renewed.',
 'Source witness and identity/owner/contract checks remain rejecting; initial and historical receipt registration validators are unchanged.',
 'Pending during the last metadata freeze before super mutates bound authority is caught only by the contact metadata commit wait; it releases the guard and rechecks the next READY under original deadlines.'],
 limitations=['Offline three-actor fixture uses synthetic RGBD and translated recorded geometry; it is not a replay of the complete physical attempt or proof of grasp success.',
 'Historical R5 wire confirms the observed rejection and deadline relationship; internal per-call start/end clocks and task lock callback delivery are not recorded.',
 'A separate remote receipt issuance candidate is needed for the analogous producer-side current-window crossing; it is not included in these two files.',
 'boundary_r1 exited before collecting tests because the proof runner used a Python 3.9-only Path method; corrected r2 retains and supersedes that setup log without changing candidate.'],
 runner_sha256={str(W/'.runtime/run_gen17_station_contact_current_wait_20261013.py'):sha(W/'.runtime/run_gen17_station_contact_current_wait_20261013.py'),str(W/'.runtime/run_gen17_station_contact_current_wait_boundary_20261013.py'):sha(W/'.runtime/run_gen17_station_contact_current_wait_boundary_20261013.py')},recorded_ns=time.time_ns(),actual_unchanged=True,source_changes_installed=False,live_ros_contacted=False,motion_commands_sent=False,physical_grasp_verified=False)
p=C/'delivery_manifest.json';assert not p.exists();p.write_text(json.dumps(manifest,indent=2)+'\n');print(p,sha(p));print('files',records);print('scope',len(scope));print('diagnosis',sha(O/'diagnosis.json'))
