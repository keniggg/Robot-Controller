"""Read only mask headers/descriptors from sealed G45; never export pixels."""
import hashlib
import json
from pathlib import Path
import socket
import sys
import time
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
G=E/'grasp_records/20261018T070056.706765Z_9d704cdd7a93474ea90deecc79af09ce'
I=V/'gen45_station_owner_messages_r1';O=V/'gen45_station_mask_fanin_r1'
sys.path[:0]=[str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
attempts=[]
class OfflineSocket(socket.socket):
 def __new__(cls,*args,**kwargs):
  attempts.append('socket');raise RuntimeError('offline sealed-bag analysis forbids sockets')
socket.socket=OfflineSocket
import rosbag
import numpy as np

def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb')as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()

def save(path,data):
 path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
 assert json.loads(path.read_text())==data

def main():
 assert Path('/media/zhuyupei/SanStand').is_mount()and not O.exists()
 m=json.loads((G/'manifest.json').read_text());assert m['state']=='complete'and m['recorder_closed']is True
 brow=next(x for x in m['files']if x['path']=='keyframes.bag')
 assert sha(G/'keyframes.bag')==brow['sha256']
 extraction=json.loads((I/'result.json').read_text())
 def rows(topic):
  info=extraction['files'][topic];assert sha(info['path'])==info['sha256']
  return json.loads(Path(info['path']).read_text())['messages']
 masks=[]
 with rosbag.Bag(str(G/'keyframes.bag'),'r')as bag:
  for topic,msg,t in bag.read_messages(topics=['/perception/object_mask']):
   assert topic=='/perception/object_mask'and msg.encoding=='mono8'and msg.height>0 and msg.width>0 and msg.step>=msg.width
   raw=bytes(msg.data);assert len(raw)==msg.height*msg.step
   raster=np.frombuffer(raw,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width]
   normal=np.ascontiguousarray(raster!=0,dtype=np.uint8)
   masks.append(dict(source_stamp_ns=msg.header.stamp.to_nsec(),bag_timestamp_ns=t.to_nsec(),header_seq=msg.header.seq,
    frame_id=msg.header.frame_id,shape=[msg.height,msg.width],step=msg.step,encoding=msg.encoding,
    data_bytes=len(raw),raw_data_sha256=hashlib.sha256(raw).hexdigest(),normalized_binary_sha256=hashlib.sha256(normal.tobytes()).hexdigest(),nonzero_pixels=int(np.count_nonzero(normal))))
 maskby={};objby={}
 for i,row in enumerate(masks):maskby.setdefault(row['source_stamp_ns'],[]).append((i,row))
 for i,row in enumerate(rows('/perception/object')):
  h=row['payload']['header']['stamp'];source=h['secs']*1000000000+h['nsecs'];objby.setdefault(source,[]).append((i,row))
 joined=[]
 for i,row in enumerate(rows('/perception/unknown/detector_status')):
  p=row['parsed_json']or{};c=p.get('station_compute_wait')
  if p.get('state')!='ready'or not isinstance(c,dict)or c['binding']['selection']['generation']!=5:continue
  source=c['source_ns'];mm=maskby.get(source,[]);oo=objby.get(source,[]);expected=p['station_compute_mask']
  match=bool(mm)and all(v['normalized_binary_sha256']==expected['sha256']and v['shape']==expected['shape']and v['frame_id']==expected['frame_id']and v['nonzero_pixels']>0 for _,v in mm)
  detected=bool(oo)and all(v['payload']['detected']is True for _,v in oo)
  joined.append(dict(source_ns=source,status_index=i,status_bag_timestamp_ns=row['received_ns'],status_wire_sha256=row['wire_sha256'],
   object_indices=[j for j,_ in oo],object_bag_timestamps_ns=[v['received_ns']for _,v in oo],object_wire_sha256=[v['wire_sha256']for _,v in oo],
   mask_indices=[j for j,_ in mm],mask_bag_timestamps_ns=[v['bag_timestamp_ns']for _,v in mm],
   nonzero_pixels=[v['nonzero_pixels']for _,v in mm],producer_status_mask_descriptor=expected,
   mask_exact_descriptor_match=match,object_detected_true=detected,recorder_three_topic_bundle_present=bool(mm and oo),
   three_topic_bag_latest_ns=max([row['received_ns']]+[v['received_ns']for _,v in oo]+[v['bag_timestamp_ns']for _,v in mm]),
   geometry_deadline_ns=c['geometry_deadline_ns'],identity_deadline_ns=c['identity_deadline_ns']))
 assert len(joined)==33 and all(v['mask_exact_descriptor_match']and v['object_detected_true']and v['recorder_three_topic_bundle_present']for v in joined)
 assert not attempts and sha(G/'keyframes.bag')==brow['sha256']
 O.mkdir();save(O/'mask_descriptors.json',masks);save(O/'station_READY_three_topic_fanin.json',joined)
 result=dict(passed=True,kind='G45_sealed_mask_descriptors_and_recorder_three_topic_fanin',created_ns=time.time_ns(),
  original_extraction_sha256=sha(I/'result.json'),bag_sha256=brow['sha256'],manifest_sha256=sha(G/'manifest.json'),
  bag_before_after_same=True,mask_topic_whitelist=['/perception/object_mask'],mask_count=len(masks),station_READY_join_count=len(joined),
  all_33_same_source_object_detected_and_nonempty_mask_match=True,
  first_hard_source_ns=1792306920504162788,last_READY=joined[-1],old_parent_source_ns=1792306915498465299,
  old_parent_join=next(v for v in joined if v['source_ns']==1792306915498465299),
  pending_parent_joins=[next(v for v in joined if v['source_ns']==s)for s in (1792306914631758928,1792306915229280233,1792306915498465299)],
  recorder_timestamp_semantics='Status metadata bag timestamp is its callback write. Object/mask/RGB/depth same-stamp SOURCES bundle is written only when recorder has all four; its shared bag timestamp is recorder bundle completion, not original per-topic callback arrival. Ordering here never proves remote callback or lock-internal processing.',
  original_extraction_received_ns_correction='All earlier received_ns fields refer to bag timestamp; use recorder_timestamp_semantics above.',
  source_recorder=dict(path=str(W/'tools/record_grasp_keyframes_readonly.py'),sha256=sha(W/'tools/record_grasp_keyframes_readonly.py')),
  descriptor_file=dict(path=str(O/'mask_descriptors.json'),sha256=sha(O/'mask_descriptors.json')),
  fanin_file=dict(path=str(O/'station_READY_three_topic_fanin.json'),sha256=sha(O/'station_READY_three_topic_fanin.json')),
  helper_sha256=sha(__file__),external_boundary_attempts=attempts,ROS_initialized=False,hardware_contacted=False,
  temperature_read=False,RGB_or_depth_messages_read=False,mask_pixels_exported=False,production_or_tests_or_record_modified=False)
 save(O/'result.json',result);(O/'helper_source.py').write_bytes(Path(__file__).read_bytes())
 print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),mask_count=len(masks),joined=len(joined),latest_bundle_timestamp_ns=joined[-1]['three_topic_bag_latest_ns']),ensure_ascii=False))
if __name__=='__main__':main()
