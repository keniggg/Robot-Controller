"""Read sealed gen18 metadata and logs; no ROS node or hardware calls."""
from pathlib import Path
import hashlib
import json
import re

import rosbag

E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R = E/'grasp_records/20261006T110552.716537Z_6050e2d687f8496fb41dd6d13f7ef9b1'
C = E/'ros_log/regional_production_20261006T105318.899385Z'
OUT = E/'ros_log/grasp_supplement_view_20261006T110552Z/gen18_status_diagnostic'
assert Path('/media/zhuyupei/SanStand').is_mount()
manifest = json.loads((R/'manifest.json').read_text())
assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
statuses = json.loads((OUT/'raw_status_window.json').read_text())
last_good = 1791284754386728048
pending = [row for row in statuses if row['status'].get('source_stamp_ns', 0) > last_good
           and row['status'].get('detail') == 'support_measurement_pending_revalidation']
metadata = []
with rosbag.Bag(str(R/'keyframes.bag')) as bag:
    for _, message, stamp in bag.read_messages(topics=['/grasp_record/camera_runtime_metadata']):
        metadata.append(dict(receipt_ns=stamp.to_nsec(), raw=json.loads(message.data)))
events = []
for line in (C/'camera.stderr.log').open(errors='replace'):
    match = re.search(r'\[(179128\d+\.\d+)\]', line)
    if not match or not 1791284751. <= float(match.group(1)) <= 1791284756.3:
        continue
    entry = dict(log_wall_sec=float(match.group(1)), raw_line=line.rstrip())
    for prefix in ('Camera RGB-D pair unavailable: ', 'Camera backend read stages: '):
        if prefix in line:
            entry['raw_read_diagnostics'] = json.loads(line.split(prefix, 1)[1])
    events.append(entry)
first = last = None
with (C/'read_diagnostics.jsonl').open() as source:
    for line in source:
        current = json.loads(line)
        if first is None:
            first = current['captured_wall_time_ns']
        last = current['captured_wall_time_ns']

late = [row for row in events if row.get('raw_read_diagnostics', {}).get('result') == 'complete']
assert len(late) == 1
diagnostic = late[0]['raw_read_diagnostics']
current = diagnostic['raw_frames']['color']
previous = diagnostic['previous_completed_raw_frames']['raw_frames']['color']
filter_host = diagnostic['stationary_pair_filter_host_monotonic']
read_enter = diagnostic['host_wait']['enter']['monotonic_sec']
# In this sequential read loop the previous filter observation cannot be
# later than the NEXT read's wait entry. This is a lower bound, not the
# unavailable exact previous filter timestamp.
host_gap_lower_bound = filter_host-read_enter
temporal = metadata[0]['raw']['parameters']['runtime_profile']['stationary_temporal']
center = next(row for row in temporal['regions'] if row['name'] == 'center')
report = dict(
    record=str(R), sealed_manifest=manifest, no_images_read=True,
    no_hardware_calls=True, no_temperature_queries=True,
    first_loss_observed_ns=1791284755208718538,
    last_good_source_ns=last_good,
    exact_pending_sources=[row['status']['source_stamp_ns'] for row in pending],
    pending_filter_qualification='unavailable: no per-read trace or source-bound regional evidence recorded',
    pending_reset_cause='unavailable; do not substitute neighboring frames',
    camera_metadata_snapshot_count=len(metadata),
    stale_parameter_snapshot=dict(receipt_ns=metadata[0]['receipt_ns'],
        center_sdk_frame_number=center['visual']['frame_number'],
        center_sdk_timestamp_ms=center['visual']['device_timestamp_ms'],
        pair_history_present='pair_history' in temporal,
        current_state_not_inferred=True,
        reason='ROS parameter updates fail with int exceeds XML-RPC limits; snapshot is not gen18 per-source evidence'),
    startup_direct_observer_wall_range_ns=[first, last],
    startup_observer_overlaps_gen18=False,
    later_read=dict(log_wall_sec=late[0]['log_wall_sec'],
        host_wait_enter_wall_sec=diagnostic['host_wait']['enter']['wall_time_sec'],
        before_or_during_first_loss=False,
        source_color_number_delta=current['frame_number']-previous['frame_number'],
        source_color_device_delta_ms=current['device_timestamp_ms']-previous['device_timestamp_ms'],
        current_filter_host=filter_host,
        previous_filter_host_unavailable=True,
        host_gap_lower_bound_sec=host_gap_lower_bound,
        unchanged_host_gap_limit_sec=max(.1, 2.5/current['stream_fps']),
        spatial_processing_sec=diagnostic['stages_sec']['depth_filter_0_spatial'],
        gate_reason=diagnostic['regional_stationary_evidence']['global_history_reset_reason'],
        filtered_pixel_count=diagnostic['regional_stationary_evidence']['filtered_pixel_count'],
        attribution='original host continuity gate correctly resets this later read; cannot explain the earlier three pending frames'),
    source_binding_limits=['SDK device timestamps are not converted into ROS exposure stamps',
        'throttled rejected-pair logs do not prove the next complete depth source',
        'absence of normal-read logs is not evidence of stable filter qualification'],
    raw_events=events, raw_camera_metadata=metadata,
    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(OUT/'camera_filter_evidence.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({key: report[key] for key in (
    'exact_pending_sources','pending_filter_qualification','stale_parameter_snapshot','later_read')},
    ensure_ascii=False,indent=2))
