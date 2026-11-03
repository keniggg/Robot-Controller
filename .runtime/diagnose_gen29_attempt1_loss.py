"""Closed-bag loss chronology only. No ROS connection, pixels exported or motion."""
import hashlib
import json
import os
from pathlib import Path

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M / '实验记录/Robot-Controller'
R = E / 'grasp_records/20261015T111226.886093Z_0a4de2be16394344b0fa84a6ad2f8318'
N = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z'
O = N / 'verification/gen29_attempt1_loss_diagnosis'
assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY and not O.exists()
def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b''): h.update(block)
    return h.hexdigest()
manifest = json.loads((R/'manifest.json').read_text())
command = json.loads((R/'command_result.json').read_text())
outcome = json.loads((R/'recorder_outcome.json').read_text())
assert manifest['state']=='complete' and manifest['recorder_closed'] is True
assert manifest['result']==command['result']=='failure'
assert command['evidence']['runner_returncode']==6 and command['recorder_returncode']==0
assert outcome['bag_closed'] is True and outcome['interrupted'] is False
entry = next(x for x in manifest['files'] if x['path']=='keyframes.bag')
bag = R/'keyframes.bag'; before=bag.stat()
assert before.st_size==entry['size_bytes'] and sha(bag)==entry['sha256']
O.mkdir(); (O/'tmp').mkdir()
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1', TMPDIR=str(O/'tmp'), PYTHONDONTWRITEBYTECODE='1')
import rosbag
import numpy as np

TOPICS = ['/perception/unknown/detector_status','/perception/object','/perception/object_mask',
    '/grasp_6d/object_geometry','/grasp/state','/grasp/near_field_phase',
    '/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack', '/rosout_agg']
statuses=[]; task=[]; measures=[]; objects=[]; masks=[]; geometries=[]; logs=[]; phases=[]
def header(msg): return dict(source_ns=msg.header.stamp.to_nsec(), frame_id=msg.header.frame_id)
def pick(value, names): return {name:value[name] for name in names if name in value}
with rosbag.Bag(str(bag)) as stream:
    counts={t:stream.get_type_and_topic_info().topics[t].message_count for t in TOPICS}
    for topic,msg,stamp in stream.read_messages(topics=TOPICS):
        ns=stamp.to_nsec()
        if topic=='/perception/unknown/detector_status':
            statuses.append(dict(record_ns=ns, value=json.loads(msg.data)))
        elif topic=='/grasp/state':
            task.append(dict(record_ns=ns, **header(msg), active=bool(msg.active), state=msg.state,
                stage=msg.stage, success=bool(msg.success), message=msg.message))
        elif topic.startswith('/grasp_mode/'):
            measures.append(dict(topic=topic,record_ns=ns,value=json.loads(msg.data)))
        elif topic=='/perception/object':
            objects.append(dict(record_ns=ns,**header(msg),detected=bool(msg.detected),label=msg.label,
                bbox_xywh=[msg.bbox_x,msg.bbox_y,msg.bbox_width,msg.bbox_height]))
        elif topic=='/perception/object_mask':
            assert msg.encoding=='mono8'
            pixels=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width]
            norm=np.ascontiguousarray(pixels!=0,dtype=np.uint8)
            masks.append(dict(record_ns=ns,**header(msg),shape=[msg.height,msg.width],
                nonzero_pixels=int(np.count_nonzero(norm)), normalized_sha256=hashlib.sha256(norm.tobytes()).hexdigest()))
        elif topic=='/grasp_6d/object_geometry':
            geometries.append(dict(record_ns=ns,**header(msg),valid=bool(msg.valid),
                failure_reason=msg.failure_reason,target_track_id=msg.target_track_id))
        elif topic=='/grasp/near_field_phase':
            phases.append(dict(record_ns=ns,**header(msg),active=bool(msg.active),phase_id=msg.phase_id,
                planning_stage=msg.planning_stage,deadline_ns=msg.deadline.to_nsec(),
                reference_source_ns=msg.reference_source_stamp.to_nsec()))
        elif topic=='/rosout_agg':
            if msg.name not in ('/unknown_tabletop_perception','/remote_grasp6d_node','/grasp_task_node','/motion_gateway'):continue
            # Do not extract or inspect temperature numbers; reject such text
            # before the narrow non-temperature event selection and persistence.
            low=msg.msg.lower()
            if any(word in low for word in ('temperature','thermal','温度')):continue
            if not any(word in low for word in ('station','target','registration','support','observ','geometry','rgb','perception','handoff')):continue
            if ns>=1792062832000000000:
                logs.append(dict(record_ns=ns,source_ns=msg.header.stamp.to_nsec(),name=msg.name,level=msg.level,message=msg.msg))

current=[row for row in statuses if row['value'].get('generation')==11]
lost=next(row for row in current if row['value'].get('target_lost') is True)
v=lost['value']; first=v['first_loss']; timing=v['frame_timing']
assert first['reason']=='perception_failure: station_compute_recovery_rgb_identity_required'
assert v['selection_stamp_ns']==1792062749180071115
last_good=next(row for row in current if row['value'].get('source_stamp_ns')==first['last_good_source_stamp_ns'])
appearance=timing['observation_appearance_identity'][-1]
assert appearance['code']=='SAME_STATION_COMPLETE_OUTLINE_MISMATCH'
assert appearance['same_station_mask_iou']<appearance['same_station_mask_iou_minimum']==.85
assert appearance['colour_distance']<appearance['max_colour_distance']==.4
assert appearance['contour_distance']<appearance['max_contour_distance']==.4
qualified=next(row for row in measures if row['topic'].endswith('_request') and
    row['value'].get('view_index')==2 and row['value']['state']=='qualified')
terminal=next(row for row in task if row['state']=='FAILED' and row['active'])
parent=last_good['value']['station_compute_wait']
after_arrival=[row for row in current if row['record_ns']>=qualified['record_ns'] and row['record_ns']<=lost['record_ns']]

def compact_status(row):
    d=row['value']; ft=d.get('frame_timing') or {}; contract=d.get('station_compute_wait') or {}
    return dict(record_ns=row['record_ns'], **pick(d,('source_stamp_ns','generation','selection_stamp_ns','state','detail','target_lost')),
        appearance_identity=ft.get('observation_appearance_identity'),rgb_identity_hold=d.get('rgb_identity_hold'),
        source_age_sec=ft.get('status_source_age_sec'), failed_stage=ft.get('failed_stage'),
        station_contract=pick(contract,('source_ns','geometry_deadline_ns','identity_deadline_ns','phase_deadline_ns','session_deadline_ns')),
        geometry_metrics=pick(d.get('geometry_metrics') or {},('mask_kind','rgb_instance_pixels','rgb_mask_pixels_without_valid_depth',
            'support_current_audit_inlier_ratio','support_current_audit_residual_m','support_current_audit_quadrants',
            'support_reference_max_distance_m','support_reference_angle_deg')))

def save(name,value):
    p=O/name;assert not p.exists()
    p.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return dict(path=str(p),sha256=sha(p))
first_receipt=save('first_current_generation_loss.json',lost)
good_receipt=save('last_good_status.json',last_good)
window_receipt=save('arrival_to_loss_timeline.json',dict(task=task,measurement=measures,phases=phases,
    detector=[compact_status(row) for row in after_arrival],
    object=[r for r in objects if r['record_ns']>=1792062832000000000],
    mask=[r for r in masks if r['record_ns']>=1792062832000000000],
    geometry=[r for r in geometries if r['record_ns']>=1792062832000000000], non_temperature_rosout=logs))
after=bag.stat(); assert (before.st_ino,before.st_size,before.st_mtime_ns)==(after.st_ino,after.st_size,after.st_mtime_ns)
sources=['src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observation_appearance.py']
report=dict(record=str(R),manifest_sha256=sha(R/'manifest.json'),command_result_sha256=sha(R/'command_result.json'),
    recorder_outcome_sha256=sha(R/'recorder_outcome.json'),bag_sha256=entry['sha256'],bag_hash_verified=True,
    bag_size_bytes=entry['size_bytes'],bag_identity_unchanged=True,read_topics=TOPICS,topic_message_counts=counts,
    current_generation=11,current_selection_stamp_ns=v['selection_stamp_ns'],
    excluded_old_generation=dict(generation=10,reason='Old latched loss before this attempt selection; not the current failure.',
        first_loss=next(r['value']['first_loss'] for r in statuses if r['value'].get('generation')==10 and r['value'].get('first_loss'))),
    second_view_qualified_request=qualified, first_loss=first, decisive_appearance=appearance,
    causal_timing=dict(parent_source_ns=parent['source_ns'],parent_geometry_deadline_ns=parent['geometry_deadline_ns'],
        current_source_ns=v['source_stamp_ns'],source_processing_start_ns=timing['processing_start_ros_ns'],
        failure_ns=timing['failure_ros_ns'],status_build_ns=timing['status_build_ros_ns'],status_bag_ns=lost['record_ns'],
        task_failure_ns=terminal['source_ns'],current_source_age_at_failure_sec=timing['failure_source_age_sec'],
        parent_deadline_crossed_by_sec=(timing['failure_ros_ns']-parent['geometry_deadline_ns'])/1e9,
        qualified_to_failure_sec=(timing['failure_ros_ns']-qualified['value']['stamp_ns'])/1e9,
        source_completion=timing.get('arrival_frame_completion')),
    conclusion=[
        'The second observation view reached its endpoint, supplied three source frames, and received both qualified participant acknowledgements.',
        'The current failure originated in the unknown perception producer, not in the downstream task or the old generation-10 support-plane failure.',
        'The decisive RGB identity gate compared the current complete instance mask to the frozen qualified-station template and failed IoU 0.810808807 < 0.85. Color and contour-distance thresholds passed.',
        'Previous frames already recorded the same outline mismatch while still READY: optional recovery proof failure is tolerated until the previous registration expires or a gap requires recovery.',
        'This frame remained individually fresh (~0.330 s). Processing crossed the previous READY geometry deadline by ~49.6 ms; the existing recovery RGB proof then became mandatory and failure was latched.',
        'Support/unique-geometry validation precedes this recorded appearance audit and was passed in this recovery call; the direct cause is not support-plane rejection or registration RMSE.',
        'The output object/mask after failure is withholding, not the rejected candidate mask. Candidate mask hash is retained in the appearance audit; do not infer a blank physical scene from the empty published mask.',
        'Lighting or shadows may affect RGB segmentation, but these records do not establish that cause. The recorded color-distance test itself did not fail. Image/mask spatial comparison is needed before attributing the mask change.'
    ],
    missing_evidence=['The rejected refined candidate mask pixels are not published as an accepted mask; source RGB/depth replay would be needed to reconstruct their spatial change.',
        'No controlled lighting comparison or illumination measurement is available; no lighting causation is claimed.'],
    evidence=[first_receipt,good_receipt,window_receipt],source_sha256={p:sha(W/p) for p in sources},
    script_sha256=sha(__file__),live_ros_contacted=False,production_modified=False,motion_commands_sent=False,
    temperature_topics_read=False,temperature_values_used=False,physical_grasp_verified=False)
receipt=save('findings.json',report)
print(json.dumps(receipt,ensure_ascii=False))
print(json.dumps(report['causal_timing'],ensure_ascii=False))
