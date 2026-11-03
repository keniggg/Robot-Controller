#!/usr/bin/env python3
"""Extract original accepted pre-motion ROS messages; no new success receipt."""
import io,json,hashlib
from pathlib import Path
import rosbag

N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
F=N/'verification/gen14_arrival_independent'
R=N.parents[1]/'grasp_records/20261013T025604.344486Z_af3ecb9373264cc9b19a98143f7b5df5'
PRE=1791860176945733070

def main():
    output=F/'three_owner_seed';output.mkdir(exist_ok=False)
    fixture=json.loads((F/'fixture.json').read_text())
    executing=next(x['message']for x in fixture['wire']if x['topic'].endswith('measurement_request')and x['message']['state']=='executing')
    rows=[]
    def save(kind,topic,message,received):
        name=kind+'_'+str(received)+'.rosmsg';data=io.BytesIO();message.serialize(data);raw=data.getvalue();(output/name).write_bytes(raw)
        rows.append(dict(kind=kind,topic=topic,received_ns=received,file=str(output/name),message_type=message._type,md5sum=message._md5sum,sha256=hashlib.sha256(raw).hexdigest()))
    with rosbag.Bag(str(R/'keyframes.bag'))as bag:
        for topic,msg,t in bag.read_messages(topics=['/perception/object','/perception/object_mask','/perception/unknown/detector_status','/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched','/grasp_mode/observe_return_session']):
            if topic in ('/perception/object','/perception/object_mask') and msg.header.stamp.to_nsec()==PRE:
                save('pre_object' if topic.endswith('/object')else'pre_mask',topic,msg,t.to_nsec())
            elif topic.endswith('/detector_status') and json.loads(msg.data).get('source_stamp_ns')==PRE:
                save('pre_status',topic,msg,t.to_nsec())
            elif 'plan_enriched' in topic and msg.valid and msg.plan_id==executing['view_plan_id']:
                save('original_view_plan',topic,msg,t.to_nsec())
            elif topic.endswith('observe_return_session'):
                payload=json.loads(msg.data)
                if payload.get('session_id')==executing['session_id']:save('original_session',topic,msg,t.to_nsec())
    assert all(any(x['kind']==k for x in rows)for k in ('pre_object','pre_mask','pre_status','original_view_plan','original_session'))
    recorded={k:min(x['received_ns']for x in rows if x['kind']==k)for k in ('pre_object','pre_mask','pre_status')}
    first=min(recorded.values())
    result=dict(scope='Historical accepted PRE and committed observation plan/session only. No arrival qualification or remote ACK is synthesized.',source_ns=PRE,rows=rows,record_bag_sha256=fixture['record_bag_sha256'],original_intertopic_receive_offsets_ns={k:v-first for k,v in recorded.items()},transport_scope='Bag receive spacing preserves recorded inter-topic ordering only; it does not measure ROS publish-to-receive latency.',script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (output/'manifest.json').write_text(json.dumps(result,indent=2)+'\n');print(output/'manifest.json')

if __name__=='__main__':main()
