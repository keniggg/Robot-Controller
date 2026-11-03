#!/usr/bin/env python3
from pathlib import Path
import hashlib,json,difflib,time
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen17_station_status_witness_atomic_20261013';P=W/'.runtime/gen17_station_join_clock_pair_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z');O=N/'verification/gen17_station_status_witness_atomic_delivery'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir()
release=N/'verification/gen16_success_first_release_manifest.json';prior=json.loads(release.read_text());assert all(sha(W/r)==v for r,v in prior['source_sha256'].items())
inherit=P/'delivery_manifest.json';inherited=json.loads(inherit.read_text());assert sha(inherit)=='f24dde4f1f2384a258b30a15163f0fae9bf31b50d8b3c3a2859d76702d898b38'
proof=N/'verification/gen17_station_status_witness_atomic_r1/result.json';result=json.loads(proof.read_text());assert result['passed'] and result['only_existing_witness_branch_guard_AST'];assert all(sha(C/r)==v for r,v in result['candidate_sha256'].items())
files=['src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py','src/alicia_grasp_modes/tests/test_station_status_witness_atomic.py'];records=[];patch=[]
for r in files:
 a=W/r;b=C/r;records.append(dict(path=r,base_sha256=sha(a) if a.exists() else None,candidate_sha256=sha(b)));patch+=list(difflib.unified_diff(a.read_text().splitlines(True) if a.exists() else [],b.read_text().splitlines(True),fromfile='a/'+r,tofile='b/'+r))
p=O/'candidate.patch';p.write_text(''.join(patch))
r=files[0];delta=O/'incremental_over_sealed_clock_pair.patch';delta.write_text(''.join(difflib.unified_diff((P/r).read_text().splitlines(True),(C/r).read_text().splitlines(True),fromfile='sealed_clock_pair/'+r,tofile='status_witness_atomic/'+r)))
scope=list(inherited['test_scope']);scope.append(files[1]);assert len(scope)==len(set(scope))==179
m=dict(schema=1,kind='GEN17_STATION_STATUS_WITNESS_ATOMIC_INCREMENTAL',passed=True,candidate_root=str(C),files=records,test_scope=scope,
 inherits=dict(manifest=str(inherit),sha256=sha(inherit),task_sha256=sha(P/r),must_preserve_inherited_changes=True,upstream=inherited['inherits']),
 baseline_release=str(release),baseline_release_sha256=sha(release),baseline_source_sha256=prior['source_sha256'],
 proof_sha256={str(proof):sha(proof),str(p):sha(p),str(delta):sha(delta)},focused_test_scope=result['focused_scope'],
 baseline_expected_failures=2,focused_passed=131,new_test_cases=7,
 changes=['Wrap only the existing typed station-requalification status witness parent lookup, validation, station check and active quarantine in the existing plan RLock.',
 'All original statements and exception behavior are preserved; AST equality after removing this one with-block proves no new gate or changed drift/quarantine policy.',
 'The full task source retains sealed C17 geometry pending recovery, metadata retry, paired-clock and preview-copy guard changes unchanged.'],
 limitations=['Event regressions use actual offline producer/remote/task callbacks and deliberately interleave an incoming READY with a valid producer notice; the notice still quarantines before a subsequent serialized callback may replace the record.',
 'A producer witness is only enough to quarantine, never to enroll local measured drift recovery; local accepted feedback and original requalification remain required.',
 'Actual ROS and hardware have not been accessed; no grasp success is claimed. Earlier failed first boundary setup and 82/83 timing run remain preserved in inherited evidence.'],
 actual_unchanged=True,inherited_candidate_unchanged=True,source_changes_installed=False,live_ros_contacted=False,motion_commands_sent=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
p=C/'delivery_manifest.json';assert not p.exists();p.write_text(json.dumps(m,indent=2)+'\n');print('MANIFEST',p,sha(p));print('RESULT',proof,sha(proof));print('FILES',records)
