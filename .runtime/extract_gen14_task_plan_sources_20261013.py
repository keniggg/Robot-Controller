import json,base64,hashlib,io
from pathlib import Path
import rosbag
W=Path(__file__).resolve().parent.parent;N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z');F=N/'verification/gen14_arrival_independent';R=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261013T025604.344486Z_af3ecb9373264cc9b19a98143f7b5df5');out=F/'task_plan_sources';out.mkdir(exist_ok=False)
def plain(value):
 if hasattr(value,'to_nsec'):return value.to_nsec()
 if hasattr(value,'__slots__'):return{k:plain(getattr(value,k))for k in value.__slots__}
 if isinstance(value,(list,tuple)):return[plain(v)for v in value]
 return value
rows=[];topics=['/grasp_6d/plan_enriched','/grasp_6d/preview_plan_enriched','/grasp_mode/observe_return_session']
with rosbag.Bag(str(R/'keyframes.bag'))as b:
 for topic,msg,stamp in b.read_messages(topics=topics):
  buffer=io.BytesIO();msg.serialize(buffer);data=buffer.getvalue();rows.append(dict(topic=topic,received_ns=stamp.to_nsec(),message_type=msg._type,md5sum=msg._md5sum,serialized_ros_base64=base64.b64encode(data).decode(),serialized_ros_sha256=hashlib.sha256(data).hexdigest(),message=plain(msg)))
wire=json.loads((F/'wire.json').read_text());executing=next(r for r in wire if r['topic'].endswith('measurement_request')and r['message']['state']=='executing');planid=executing['message']['view_plan_id'];selection=executing['message']['selection'];matching=[r for r in rows if r['message'].get('plan_id')==planid];report=dict(scope='Read-only exact ROS message exports from sealed attempt2 keyframes, no live connection; selection is exact executing-request binding because no selection topic was recorded',bag=str(R/'keyframes.bag'),bag_sha256=json.loads((F/'fixture.json').read_text())['record_bag_sha256'],topics=topics,selection=selection,executing_request=executing,requested_view_plan_id=planid,matched_plan_count=len(matching),rows=rows,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
(out/'result.json').write_text(json.dumps(report,indent=2)+'\n');(out/'requested_plan_rows.json').write_text(json.dumps(matching,indent=2)+'\n');print(json.dumps(dict(output=str(out),rows=len(rows),matched_plan_count=len(matching),view_plan_id=planid)))
