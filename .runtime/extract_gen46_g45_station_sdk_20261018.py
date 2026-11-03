"""Exact non-temperature station SDK ingress provider from sealed G45."""
import base64,hashlib,io,json,socket,sys
from pathlib import Path
W=Path(__file__).resolve().parents[1];E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller');V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
G=E/'grasp_records/20261018T070056.706765Z_9d704cdd7a93474ea90deecc79af09ce';O=V/'gen46_g45_station_sdk_inputs_r1'
sys.path[:0]=[str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages'];attempts=[]
class OfflineSocket(socket.socket):
 def __new__(cls,*a,**k):attempts.append('socket');raise RuntimeError('offline bag only')
socket.socket=OfflineSocket
import rosbag

def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb')as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def save(p,d):p.write_text(json.dumps(d,indent=2,ensure_ascii=False,allow_nan=False)+'\n');assert json.loads(p.read_text())==d

def main():
 assert not O.exists()and Path('/media/zhuyupei/SanStand').is_mount();m=json.loads((G/'manifest.json').read_text());assert m['recorder_closed']and m['state']=='complete'
 entry=next(x for x in m['files']if x['path']=='keyframes.bag');assert sha(G/'keyframes.bag')==entry['sha256']
 rows=[];epochs=[]
 with rosbag.Bag(str(G/'keyframes.bag'),'r')as bag:
  for topic,msg,t in bag.read_messages(topics=['/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch']):
   stamp=msg.header.stamp.to_nsec()if hasattr(msg,'header')else msg.stamp.to_nsec()
   if topic.endswith('accepted_joint_states')and not 1792306909000000000<=stamp<=1792306921000000000:continue
   s=io.BytesIO();msg.serialize(s);wire=s.getvalue()
   r=dict(topic=topic,bag_timestamp_ns=t.to_nsec(),source_stamp_ns=stamp,message_type=msg._type,md5sum=msg._md5sum,wire_sha256=hashlib.sha256(wire).hexdigest(),wire_base64=base64.b64encode(wire).decode())
   if hasattr(msg,'position'):r.update(frame_id=msg.header.frame_id,names=list(msg.name),positions=list(msg.position));rows.append(r)
   else:r.update(frame_id=msg.frame_id);epochs.append(r)
 assert rows and not attempts and sha(G/'keyframes.bag')==entry['sha256']
 station=json.loads((V/'gen45_station_owner_timeline_r1/exact_old_and_latest_contracts.json').read_text())['old_pending']['station_compute_wait']['binding']['station_q_rad']
 delta=[]
 for row in rows:
  q=dict(zip([n.lower()for n in row['names']],row['positions']));v=[q['joint%d'%i]for i in range(1,7)]
  delta.append(dict(source_ns=row['source_stamp_ns'],maximum_station_delta_rad=max(abs(a-b)for a,b in zip(v,station))))
 O.mkdir();save(O/'accepted_joint_wires.json',rows);save(O/'control_epoch_wires.json',epochs);save(O/'station_joint_delta_diagnostics.json',delta)
 result=dict(passed=True,bag_sha256=entry['sha256'],manifest_sha256=sha(G/'manifest.json'),whitelist=['/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch'],accepted_count=len(rows),epochs_count=len(epochs),maximum_station_delta_rad=max(x['maximum_station_delta_rad']for x in delta),above_original_004_threshold=[x for x in delta if x['maximum_station_delta_rad']>.004],external_boundary_attempts=attempts,ROS_initialized=False,temperature_read=False,hardware_contacted=False,production_or_record_modified=False,files={p.name:sha(p)for p in O.glob('*.json')},helper_sha256=sha(__file__))
 save(O/'result.json',result);print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),accepted_count=len(rows),maximum_station_delta_rad=result['maximum_station_delta_rad'],above_004=len(result['above_original_004_threshold'])),ensure_ascii=False))
if __name__=='__main__':main()
