"""Interpret already saved metadata; no ROS imports, no hardware access."""
import hashlib,json
from pathlib import Path
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
p=E/'ros_log/explicit_grasp_20261007T122934Z/raw_support/parallel_stream_gap_analysis'
r=(p/'report.json').read_bytes();d=json.loads(r);starts={}
for topic,v in d['gen24']['topics'].items():
    events=[]
    for a,b in zip(v['rows'],v['rows'][1:]):
        before=max(a['source_stamps_ns']);after=min(b['source_stamps_ns'])
        if after-before>150000000:
            events.append(dict(start_source_ns=before,end_source_ns=after,source_gap_ms=(after-before)/1e6,
                bag_receipt_gap_ms=(b['bag_time_ns']-a['bag_time_ns'])/1e6,
                before_bag_lag_ms=(a['bag_time_ns']-before)/1e6,after_bag_lag_ms=(b['bag_time_ns']-after)/1e6))
    starts[topic]=events
source=E/'ros_log/aligned_grasp_20261007T112309.952794Z/recorded_retry_20261007T120118Z/raw_support/analysis/first_loss_bag_metadata/report.json'
sr=source.read_bytes();s=json.loads(sr);ft=s['first_loss_timing'];base=ft['processing_start_ros_ns']
tfstart=base+round(ft['stage_end_elapsed_sec']['before_tf']*1e9)
tfend=base+round(ft['stage_end_elapsed_sec']['tf_lookup']*1e9)
nextrow=d['gen23']['first_loss_tf_query_bracket_diagnostic']['first_saved_tf_at_or_after_image_source']
nextstamp=max(nextrow['source_stamps_ns'])
report=dict(diagnostic_only=True,production_modified=False,source_report_sha256=hashlib.sha256(r).hexdigest(),
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),long_source_gaps=starts,
    gen23_first_loss_timing_report_sha256=hashlib.sha256(sr).hexdigest(),
    gen23_tf_wait=dict(image_source_ns=s['first_loss']['source_stamp_ns'],
        inferred_tf_lookup_start_ros_ns=tfstart,inferred_tf_lookup_end_ros_ns=tfend,
        inference_method='Original processing ROS start plus monotonic per-stage elapsed, rounded to integer nanoseconds; not a new exact ROS clock observation.',
        next_actual_feedback_and_tf_source_ns=nextstamp,next_tf_bag_receipt_ns=nextrow['bag_time_ns'],
        inferred_wait_until_next_feedback_ms=(nextstamp-tfstart)/1e6,
        inferred_next_feedback_to_tf_lookup_return_ms=(tfend-nextstamp)/1e6,
        observed_tf_stage_wall_ms=ft['stage_wall_sec']['tf_lookup']*1000),
    conclusions=[
        'During the 1.641-second camera SDK wait, real accepted feedback and TF still arrived. A whole-VM freeze spanning that entire interval is not supported.',
        'Three shorter 427-478ms accepted-feedback gaps coincide with three 296-324ms driver heartbeat gaps, ending within about 12 microseconds of parser feedback timestamps. The driver stream was not otherwise completely smooth.',
        'Small bag receipt lags at gap endpoints show these are producer-header gaps, not merely recorder callback backlogs.',
        'Accepted feedback, TF and joint_states share the driver as a source. Their correlated gaps cannot distinguish a driver mutex/serial blockage from shorter scheduling or host/guest pauses.',
        'In the separate gen23 window, accepted feedback and TF are one-to-one at 10Hz. The observed 42ms TF wait closely spans the interval until the next actual measured sample required for interpolation; it is consistent with the configured 10Hz polling, not additional TF downsampling.',
        'Do not replace measured TF with 50Hz heartbeat repeats or remove exact-time requirements. No configuration or thresholds were changed.'
    ],limits=['Current launch explicitly polls states at10Hz; runtime parameter was not queried.',
              'No device-level USB/host scheduler trace was recorded here. The root cause of camera interruption is not yet localized.'])
raw=(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode();out=p/'interpretation.json'
with out.open('xb') as f:f.write(raw)
(p/'interpretation.sha256').write_text(hashlib.sha256(raw).hexdigest()+'  interpretation.json\n')
print(json.dumps({'sha256':hashlib.sha256(raw).hexdigest(),'gen23_tf_wait':report['gen23_tf_wait']},ensure_ascii=False))
