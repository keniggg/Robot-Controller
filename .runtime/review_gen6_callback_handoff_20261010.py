"""Seal read-only independent review of the frozen GEN6 callback patch."""
from pathlib import Path
import datetime
import hashlib
import json

WORK=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
STAGE=WORK/'.runtime/gen6_callback_handoff_staging_20261010'
NEW=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z')
EXPECTED={
 'src/alicia_grasp_modes/src/alicia_grasp_modes/initial_preflight_task.py':'96370a5a721b48d6e4b729e6333c1983059724fe46f844ad80d8ba5bc7d1dc45',
 'src/alicia_grasp_modes/tests/test_motion_delayed_object.py':'ceea9a5727d28af9d570c11b05397f3bad10a7ec168c2dadaa6008bd09dd7de5',
}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
files=[]
for name,expected in EXPECTED.items():
 actual=sha(STAGE/name)
 assert actual==expected,(name,actual,expected)
 files.append(dict(path=name,sha256=actual))
delivery=json.loads((STAGE/'delivery.json').read_text())
assert delivery['changes']==files and delivery['source_frozen'] is True
fixture=NEW/'gen6_handoff_analysis/arrival_lock_fixture/fixture.json'
fixture_data=json.loads(fixture.read_text())
assert sha(fixture)=='63725db1ca2a75ca2ec08b58a5d3981abcc11b498f4d3c32c278f03817e79896'
replay=NEW/'gen6_handoff_analysis/callback_handoff_replay.json'
assert sha(replay)=='91448c09c8926a5075a8d9045a2ee4a2c879f7571eccf39615f35f61a1ab8dc1'
report=dict(
 schema=1,reviewer='support_residual_audit',created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
 disposition='no_blocking_findings_in_frozen_patch',passed=True,files=files,
 stage=str(STAGE),delivery_sha256=sha(STAGE/'delivery.json'),
 production_change_count=1,no_live_edits_or_control_actions=True,
 scope='Independent source/diff and test review of the GEN6 initial-preflight callback-lock lifecycle fix; no threshold or motion policy change.',
 evidence=dict(
  fixture_path=str(fixture),fixture_sha256=sha(fixture),
  old_positive_source_ns=fixture_data['old_positive_source_ns'],
  independent_recorder_received_ns=fixture_data['old_positive_received_ns'],
  age_at_independent_recorder_sec=fixture_data['old_positive_age_at_recorder_sec'],
  association_checks=fixture_data['association_checks'],
  observed_original_log='Grasp ignored stale object source age 38.377s outside [0, 8.000]s',
  original_log_then_failure='Reached observation rejected because execution authority had already been revoked.'),
 reviewed_invariants=[
  'Committed, retired and absent initial tokens no longer acquire an extra preparation lock before original ObjectPose freshness validation. Initial direct behavior retains the original base callback path.',
  'Initial refresh returns immediately outside armed/registering. It does not wait on the synchronous physical-action lock during committed motion.',
  'An armed-to-committed callback race still enters the original plan RLock. Only an originally timely positive captured between frozen plan source and original executing lease start, now old after waiting, can be discarded.',
  'Discard requires active actually-submitted initial attempt, committed token, matching current and submitted lease/selection/session/plan/track/epoch, and both canonical measurement-plan and original ROS-wire plan digests verified in their own domains.',
  'Existing source/frame association, finite confidence, configured confidence threshold, target drift, jump limits, reference binding and fatal collector rejection remain in force. No new geometric tolerance is introduced.',
  'Only normal measured_pose_changed diagnostics may coexist with completed initial submission. Other attempt disqualifications, revoked authority, manual ownership, epoch change and owner/selection change reject the exception.',
  'Cheap source and binding checks precede the rare original authoritative owner lookup. Normal committed messages do not use that added owner RPC. Following the lookup, revoked/epoch/hard-disqualification state is rechecked.',
  'No source stamp, freshness timestamp, geometry, lease deadline or authority latch is renewed or restored by discarding the superseded object. Only a bounded diagnostic source list and log are written.',
  'Invalid/false/new-source/old-on-entry frames retain the original callback rejection path. Later fresh post-motion observations still enter the original callback.',
  'No new attempt-lock to plan-lock ordering is introduced. The rare helper follows pre-existing plan-to-collector/following-evidence ordering; no newly added wait for ROS ACK or service execution occurs.'
 ],
 resolved_review_findings=[
  'Removed unconditional committed preparation lock and committed refresh lock.',
  'Separated canonical lease plan hash from full ROS-wire frozen-plan hash; exact recorded hashes are asserted by tests.',
  'Moved authoritative owner verification to the rare successful-candidate branch; normal callbacks keep their original hot path.',
  'Added fatal source-collector rejection and authoritative selection-change negative coverage.',
  'Corrected real bag measurement-request topic and parsed the existing string-encoded session deadline via positive_ns.'
 ],
 test_evidence=dict(
  implementation_agent_run='22 passed in 1.09s',
  report_path=str(replay),report_sha256=sha(replay),
  independently_reviewed_not_rerun=True,
  covered=['real ModeAware callback with recorded old ObjectPose and original frozen plan/lease',
   'two real Python threads crossing the existing RLock with explicit simulated ROS time advancement',
   'old callback failure reproduced when superseded helper is disabled',
   'committed entry restores original pre-lock age evaluation',
   'committed refresh returns while another thread owns the action lock',
   'fresh recorded post-motion ObjectPose accepted afterwards',
   '14 parameterized hard-failure negatives plus invalid geometry, authoritative selection change, fatal collector',
   'recorded canonical and ROS-wire hash-domain distinction'],
  combined_regression='Run and receipt owned separately by gen3_support_monitor; this review does not claim those tests have completed.'),
 limitations=[
  'Recorder receipt is not a recorded task callback-entry timestamp. The source age and scheduling fixture support the reproduced causal seam; they do not directly instrument the original 38-second lock acquisition/release.',
  'No real hardware movement or actual grasp success is established by this offline patch review.',
  'The new bag-backed tests require sealed GEN6 on SanStand and skip if that fixture is absent; acceptance must count passes with the fixture present.',
  'The rare authoritative owner RPC is synchronous and not forcibly cancelled. This accepted narrow lookup can delay callback completion; the patch does not extend its lease/session/capture deadlines or grant motion authority.',
  'The patch does not change the original action lock or general false/geometry invalidation ordering during a synchronous motion.'
 ])
destination=NEW/'verification/gen6_callback_handoff_independent_review.json'
assert destination.parent.is_dir()
with destination.open('x',encoding='utf-8') as stream:json.dump(report,stream,ensure_ascii=False,indent=2);stream.write('\n')
print(json.dumps(dict(path=str(destination),sha256=sha(destination),disposition=report['disposition'],files=files),ensure_ascii=False))
