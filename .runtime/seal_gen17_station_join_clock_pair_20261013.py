#!/usr/bin/env python3
from pathlib import Path
import hashlib,json,difflib,time
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen17_station_join_clock_pair_20261013';P=W/'.runtime/gen17_station_contact_current_wait_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z');O=N/'verification/gen17_station_join_clock_pair_delivery'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir()
release=N/'verification/gen16_success_first_release_manifest.json';prior=json.loads(release.read_text());assert all(sha(W/r)==v for r,v in prior['source_sha256'].items())
inherit=P/'delivery_manifest.json';inherited=json.loads(inherit.read_text());assert sha(inherit)=='7bd0c5fcb055ff9b9c67be7632b46b77707651cd98066c8f908920bc4141a742'
proof=N/'verification/gen17_station_join_clock_pair_r2/result.json';result=json.loads(proof.read_text());assert result['passed'] and result['only_mono_reposition_and_first_check_guard_AST'];assert all(sha(C/r)==v for r,v in result['candidate_sha256'].items())
files=['src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_task.py','src/alicia_grasp_modes/tests/test_station_join_clock_pair.py'];records=[];patch=[]
for r in files:
 a=W/r;b=C/r;records.append(dict(path=r,base_sha256=sha(a) if a.exists() else None,candidate_sha256=sha(b)));patch+=list(difflib.unified_diff(a.read_text().splitlines(True) if a.exists() else [],b.read_text().splitlines(True),fromfile='a/'+r,tofile='b/'+r))
p=O/'candidate.patch';p.write_text(''.join(patch))
r=files[0];delta=O/'incremental_over_sealed_C17.patch';delta.write_text(''.join(difflib.unified_diff((P/r).read_text().splitlines(True),(C/r).read_text().splitlines(True),fromfile='sealed_C17/'+r,tofile='paired_clock/'+r)))
scope=list(inherited['test_scope']);scope.append(files[1]);assert len(scope)==len(set(scope))==178
probe=N/'verification/gen17_join_clock_pair_probe_r1/result.json'
m=dict(schema=1,kind='GEN17_STATION_JOIN_CLOCK_PAIR_INCREMENTAL',passed=True,candidate_root=str(C),files=records,test_scope=scope,
 inherits=dict(manifest=str(inherit),sha256=sha(inherit),task_sha256=sha(P/r),must_preserve_inherited_changes=True),
 baseline_release=str(release),baseline_release_sha256=sha(release),baseline_source_sha256=prior['source_sha256'],
 proof_sha256={str(proof):sha(proof),str(probe):sha(probe),str(N/'verification/gen17_current_record_interleave_probe_r1/result.json'):sha(N/'verification/gen17_current_record_interleave_probe_r1/result.json'),str(p):sha(p),str(delta):sha(delta)},focused_test_scope=result['focused_scope'],
 baseline_expected_failures=2,focused_passed=result['focused']['summary']['tests'],
 changes=['Pair the existing task join monotonic clock read immediately before its final ROS clock read; preserve every existing comparison/deadline. Guard only the initial preview-copy station check with the existing RLock so its old record cannot quarantine a new callback registration.',
 'Full production candidate includes every sealed C17 task change unchanged; the separate incremental patch contains only that monotonic assignment movement and the initial-check guard plus comments.',
 'Existing C17 final monotonic geometry check now quarantines the source and returns PENDING under the reproduced paused-ROS registration delay; no extra commit hard gate is added.'],
 limitations=['F4 requires elapsed work between paired clocks while ROS time is paused; no evidence asserts that R5 encountered this condition.',
 'No producer or remote join code is changed; their static clock pairing similarities are not claimed fixed.',
 'focused r1 retained one original concurrent fresh-copy timeout (82/83); the deterministic Event probe established old-record quarantine of a new READY, repaired by the narrow existing guard. r2 includes this regression.',
 'Tests use actual offline producer/remote/task callbacks, metadata freeze and retirement validators with an inert action sentinel; no physical grasp outcome is claimed.'],
 actual_unchanged=True,inherited_candidate_unchanged=True,source_changes_installed=False,live_ros_contacted=False,motion_commands_sent=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
p=C/'delivery_manifest.json';assert not p.exists();p.write_text(json.dumps(m,indent=2)+'\n');print('MANIFEST',p,sha(p));print('RESULT',proof,sha(proof));print('FILES',records)
