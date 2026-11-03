"""Freeze source-specific evidence from the sealed gen15 bag, without ROS I/O."""
from pathlib import Path
import hashlib,json
import rosbag,rospy
ROOT=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
record=EXT/'grasp_records/20261006T093000.069892Z_2b67317c7fb54de5a1fd94d6b4f7bb49'
assert Path('/media/zhuyupei/SanStand').is_mount()
m=json.loads((record/'manifest.json').read_text());assert m['state']=='complete' and m['recorder_closed'] is True
out=EXT/'ros_log/grasp_supplement_view_20261006T092959Z/ingress_stale_diagnostic';out.mkdir(exist_ok=False)
rows=[];camera_logs=[]
with rosbag.Bag(str(record/'keyframes.bag')) as bag:
 for topic,msg,t in bag.read_messages(topics=['/perception/unknown/detector_status','/rosout_agg'],start_time=rospy.Time.from_sec(1791279028),end_time=rospy.Time.from_sec(1791279032.5)):
  if topic.endswith('detector_status'):
   s=json.loads(msg.data);rows.append(dict(bag_receipt_ns=t.to_nsec(),**{k:s[k] for k in ('source_stamp_ns','color_seq','depth_seq','state','detail','first_loss','frame_timing','rgbd_ingress_timing') if k in s}))
  elif msg.name=='/supervisor_camera':camera_logs.append(dict(bag_receipt_ns=t.to_nsec(),publisher_stamp_ns=msg.header.stamp.to_nsec(),message=msg.msg))
report={'schema':1,'record':str(record),'manifest_sha256':hashlib.sha256((record/'manifest.json').read_bytes()).hexdigest(),'diagnostic_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'rows':rows,'camera_logs':camera_logs,
 'observed':{'last_good_source_ns':1791279029345232009,'last_good_ingress_ns':1791279029349124670,'last_good_status_build_ns':1791279029508317708,'first_loss_observed_ns':1791279030377484798,'late_paired_source_ns':1791279029636935234,'late_pair_color_arrival_ns':1791279030381608009,'late_pair_depth_arrival_ns':1791279029806198120,'late_pair_ingress_ns':1791279030381643295,'late_pair_processing_start_ns':1791279030681557416,'late_pair_failure_stage':'input_age_check','late_pair_was_not_segmented':True},
 'conclusions':['The final ready-frame segmentation was 39.478 ms, not the gen14 953 ms case.','The exact-pair watchdog elapsed before a delayed same-source RGB arrived; subsequent old pair failed the unchanged 800 ms source-age limit before TF or segmentation.','Latest raw ingress at approximately 9031.49 had depth seq462270/source9031.4379 but color seq462245/source9029.9378. This locates the directly observed fault in unequal ROS image delivery to the unknown consumer, after acquisition of the source pair.','Exact synchronization callback and input-lock costs were microseconds on the recovered pair. Increasing matching queue depth would not make its 1.045 s processing-start source age fresh.'],
 'limits':['First-loss observed_stamp is the watchdog decision. Its first published raw-arrival snapshot was taken about 289 ms later; the two times must not be conflated.','Camera-side per-image publication exit times and TCP send/receive scheduling are absent; current evidence cannot uniquely separate publisher connection queue, receiver/deserialization scheduling, GIL waiting or VM scheduling.','A nearby 33 ms SDK-pair skew rejection is recorded, but is not a same-frame or long-wait explanation for color lag. No long SDK wait is logged in this narrow interval; absence of a throttled log is not a per-frame upper bound.','Later /proc/PSI and kernel queries are context only, not historical proof of the failure. No motion, quality threshold or freshness change was made.']}
(out/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n');print(json.dumps(dict(path=str(out/'report.json'),statuses=len(rows),camera_logs=len(camera_logs))))
