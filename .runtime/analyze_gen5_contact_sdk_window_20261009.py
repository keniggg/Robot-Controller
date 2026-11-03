#!/usr/bin/env python3
"""Read only closed recording and replay original following-window gates."""
from pathlib import Path
import argparse,hashlib,json
import numpy as np
import rosbag,rospy
from alicia_flexible_grasp.robot.stationary_following import _window,ARM_NAMES,SDK_QUANTUM_RAD

p=argparse.ArgumentParser();p.add_argument('--record',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
manifest=json.loads((args.record/'manifest.json').read_text());assert manifest['state']=='complete' and manifest['recorder_closed'] is True
samples={t:[] for t in ('/alicia_d/sdk_command','/alicia_d/accepted_joint_states')}
with rosbag.Bag(str(args.record/'keyframes.bag')) as bag:
 for topic,msg,received in bag.read_messages(topics=list(samples),start_time=rospy.Time.from_sec(1791598200.),end_time=rospy.Time.from_sec(1791598216.)):
  samples[topic].append((received.to_nsec(),msg))
epoch=1791591831641651789;checks=[]
for now_ns in range(1791598213800000000,1791598215270000000,10000000):
 histories={topic:[msg for recv,msg in values if recv<=now_ns][-128:] for topic,values in samples.items()}
 now=now_ns/1e9
 try:
  sdk=_window(histories['/alicia_d/sdk_command'],'sdk_transmitted',ARM_NAMES,now,epoch,.5)
  actual=_window(histories['/alicia_d/accepted_joint_states'],'sdk_measured',ARM_NAMES+('right_finger',),now,epoch,.5)
  end=min(sdk[-1][0],actual[-1][0]);start=end-300000000
  sdk=_window(histories['/alicia_d/sdk_command'],'sdk_transmitted',ARM_NAMES,now,epoch,.5,start)
  actual=_window(histories['/alicia_d/accepted_joint_states'],'sdk_measured',ARM_NAMES+('right_finger',),now,epoch,.5,start)
  assert all(np.array_equal(q,sdk[-1][1]) for _,q in sdk)
  assert np.all(np.ptp(np.asarray([q for _,q in actual]),axis=0)<=SDK_QUANTUM_RAD+1e-12)
  checks.append(dict(now_ns=now_ns,passes_original_window_rules=True,sdk_count=len(sdk),accepted_count=len(actual),sdk_latest_ns=sdk[-1][0],accepted_latest_ns=actual[-1][0]))
 except (ValueError,AssertionError) as exc:checks.append(dict(now_ns=now_ns,passes_original_window_rules=False,reason=str(exc)))
telemetry={}
for topic,values in samples.items():
 selected=[(recv,msg) for recv,msg in values if 1791598211000000000<=recv<=1791598216000000000]
 stamps=[msg.header.stamp.to_nsec() for _,msg in selected]
 telemetry[topic]=dict(count=len(selected),maximum_source_gap_seconds=max((b-a)/1e9 for a,b in zip(stamps,stamps[1:])),
  maximum_recorder_receipt_delay_seconds=max((recv-msg.header.stamp.to_nsec())/1e9 for recv,msg in selected),
  exact_source_messages=[dict(received_ns=recv,source_stamp_ns=msg.header.stamp.to_nsec(),frame=msg.header.frame_id,names=list(msg.name),positions=list(msg.position)) for recv,msg in selected])
result=dict(schema_version=1,generation=5,record=str(args.record),record_closed=True,manifest_sha256=hashlib.sha256((args.record/'manifest.json').read_bytes()).hexdigest(),
 first_failed_candidate=dict(plan_id='4599f654ea474c23ecfb39d4',logged_time_sec=1791598215.274355,reason='missing independent sdk_transmitted samples',failed_stage='',scope='read_only_candidate_constant_bias_prediction'),
 telemetry=telemetry,original_window_replays=checks,successful_window_replays=sum(x['passes_original_window_rules'] for x in checks),
 interpretation='Recorder saw continuous successful SDK-write telemetry and accepted feedback. At every tested receipt cutoff, the original independent common stationary-window rules pass. Original gateway local history is unrecorded, so the specific callback/lock/scheduling mechanism is unproven.',
 minimum_fix_proposal=['Keep all existing source-frame, joint identity, epoch, freshness, stationary range, opening and real SDK write requirements.',
 'In the read-only candidate sample collection, allow a narrowly typed temporary incomplete-window wait within both original candidate deadline and a fixed monotonic sampling sub-budget; release feedback lock during waiting. Re-evaluate a newly received actual two-channel snapshot, not replayed messages.',
 'Reject changed source identity/epoch/manual ownership/target or invalid data immediately; require original context again and original freshness after FK.',
 'Capture SDK/accepted latest stamps, history counts, maximum adjacent gaps, receive delay and capture-lock timing in the failure audit so a future miss is attributable.',
 'Do not repair by relabeling accepted feedback as sdk_transmitted, synthesizing keepalive timestamps, clearing old revocations, changing physical gates, or automatically executing a predicted path.'],
 limits=['Bag receive times reflect recorder delivery, not gateway callback admission times.','This analysis replays only pure original independent feedback windows, not FK/contact collision or hardware execution.','Initial preflight arm/commit passed in this trial, but no support-gap occurred while token armed; this is not hardware validation of gap recovery.'])
args.output.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
print(json.dumps(dict(output=str(args.output),sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),original_window_replay_passes=result['successful_window_replays'],replays=len(checks),telemetry={k:{a:b for a,b in v.items() if a!='exact_source_messages'}for k,v in telemetry.items()}),sort_keys=True))
