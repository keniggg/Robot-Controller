"""Read only the sealed GEN24 detector status; no ROS master or hardware."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import rosbag
import rospy

BASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD = BASE/'grasp_records/20261010T145523.713830Z_9463dd99db1449279fafc4fa41b57be4'
OUT = BASE/'ros_log/full_latest_restart_20261010T002347Z/gen24_analysis'
OUT.mkdir(parents=True, exist_ok=True)
TOPIC = '/perception/unknown/detector_status'
rows = []
with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
    for _, message, stamp in bag.read_messages(topics=[TOPIC],
            start_time=rospy.Time.from_sec(1791644189.),
            end_time=rospy.Time.from_sec(1791644251.)):
        payload = json.loads(message.data)
        measurement = payload.get('observation_measurement')
        if not measurement:
            continue
        rows.append(dict(bag_stamp_ns=stamp.to_nsec(),
            source_stamp_ns=payload.get('source_stamp_ns'),
            state=payload.get('state'), detail=payload.get('detail'),
            observation_measurement=measurement))
second = [row for row in rows if row['observation_measurement'].get('request', {}).get('view_index') == 2]
first = [row for row in rows if row['observation_measurement'].get('request', {}).get('view_index') == 1]
def metrics(group):
    distinct = {}
    for row in group:
        for item in row['observation_measurement'].get('appearance_identity', []):
            distinct[(item.get('current_source_stamp_ns'),item.get('current_mask_sha256'))] = item
    evidence = list(distinct.values())
    result = dict(status_count=len(group),
        reasons=dict(Counter(row['observation_measurement'].get('reason', '') for row in group)),
        unique_appearance_check_count=len(evidence),
        unique_appearance_sources_ns=sorted(set(item.get('current_source_stamp_ns') for item in evidence)),
        appearance_codes=dict(Counter(item.get('code') for item in evidence)))
    for key in ('colour_distance', 'contour_distance', 'chroma_distance'):
        values = [item[key] for item in evidence if isinstance(item.get(key), (int, float))]
        if values:
            result[key+'_range'] = [min(values), max(values)]
    return result
report = dict(record=str(RECORD), topic=TOPIC, hardware_access=False,
    second_arrival_ns=1791644246435057163, second_capture_deadline_ns=1791644249435057163,
    first_view_summary=metrics(first), second_view_summary=metrics(second),
    first_view=first, second_view=second,
    conclusion='Second-view first rejection is matching_fragment_competitor_not_owned, while both recorded appearance checks and current support audit passed. ACK timeout follows repeated qualification rejection; exact fragment ownership still needs geometry/source replay.',
    limitations=['Recorded appearance_identity may contain only checks reached before rejection, not proof that all candidates passed.',
                 'Status republishes retained measurement diagnostics after evaluation; appearance checks are deduplicated by source and mask SHA, while status_count is raw messages.',
                 'measurement_evaluated and exact_tf_consumed false are conservative status authority flags; they do not by themselves prove no internal computation ran.'])
target=OUT/'arrival_measurement_reasons.json'
target.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
print(json.dumps(dict(path=str(target),sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
    first=report['first_view_summary'],second=report['second_view_summary']), ensure_ascii=False))
