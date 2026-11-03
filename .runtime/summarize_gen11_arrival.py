from pathlib import Path
import collections
import json
import rosbag

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD=ROOT/'grasp_records/20261009T034234.183696Z_9e74e994513542879c9e556c6ccd2180'
OUT=ROOT/'ros_log/full_latest_ros_20261009T003407Z/gen11_arrival_analysis'
manifest=json.loads((RECORD/'manifest.json').read_text())
assert manifest['recorder_closed'] and manifest['state']=='complete'
requests=[]; acks=[]; statuses=[]; pairs=[]
with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
    for topic,msg,stamp in bag.read_messages(topics=[
        '/grasp_mode/observe_return_measurement_request','/grasp_mode/observe_return_measurement_ack',
        '/perception/unknown/detector_status','/experiment/observation_rgbd/metadata']):
        row=dict(received_ns=stamp.to_nsec(),message=json.loads(msg.data))
        if topic.endswith('measurement_request'):requests.append(row)
        elif topic.endswith('measurement_ack'):acks.append(row)
        elif topic.endswith('detector_status'):statuses.append(row)
        else:pairs.append(row)
request=next(r['message'] for r in requests if r['message']['state']=='requalifying')
start=request['arrival_stamp_ns']; deadline=request['capture_deadline_ns']
within=[r for r in statuses if start<=r['received_ns']<=deadline]
near=[r for r in statuses if start-300_000_000<=r['received_ns']<=deadline+500_000_000]
saved_pairs=[r for r in pairs if start<=r['received_ns']<=deadline+250_000_000]
counts=dict(collections.Counter(r['message'].get('observation_measurement',{}).get('reason','') for r in within))
report=dict(record=str(RECORD),manifest_closed=True,result=manifest['result'],
    terminal=manifest['failure_reason'],selection=request['selection'],arrival_ns=start,
    capture_deadline_ns=deadline,settle=request['settle'],within_capture_status_count=len(within),
    within_capture_reasons=counts,qualified_ack_seen=any(r['message'].get('qualified') for r in acks),
    requests=requests,acks=acks,arrival_statuses=near,recorded_arrival_rgbd_metadata=saved_pairs,
    no_ros_actions=True,temperature_values_read=False,
    limitations=['Recorded RGBD pairs and completed detector statuses are different collections; neither substitutes for the other.'])
OUT.mkdir(parents=True,exist_ok=True)
with (OUT/'analysis.json').open('x') as output:json.dump(report,output,indent=2,ensure_ascii=False)
print(json.dumps(dict(path=str(OUT/'analysis.json'),counts=counts,status_count=len(within),
    saved_pair_metadata_count=len(saved_pairs),acks=[r['message'] for r in acks],
    first_status=within[0] if within else None),ensure_ascii=False))
