import rospy,json,pathlib,time,threading
from std_msgs.msg import String
from sensor_msgs.msg import JointState,Image
OUT=pathlib.Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_ros_latest_start_20261002')
rospy.init_node('latency_recovery_readonly',anonymous=True,disable_signals=True)
rows=[];lock=threading.Lock()
def status(m):
 d=json.loads(m.data);f=d.get('frame_timing') or {}
 with lock:rows.append({'topic':'status','arrival_ns':rospy.Time.now().to_nsec(),'generation':d.get('generation'),'source_ns':f.get('source_stamp_ns'),'processing_wall_sec':f.get('processing_wall_sec'),'processing_start_source_age_sec':f.get('processing_start_source_age_sec'),'status_source_age_sec':f.get('status_source_age_sec'),'failed_stage':f.get('failed_stage')})
def frame(t,m):
 with lock:rows.append({'topic':t,'arrival_ns':rospy.Time.now().to_nsec(),'source_ns':m.header.stamp.to_nsec()})
subs=[rospy.Subscriber('/perception/unknown/detector_status',String,status,queue_size=30)]
for t,typ in [('accepted',JointState),('rgb',Image)]:
 topic='/alicia_d/accepted_joint_states' if t=='accepted' else '/supervisor/camera/color/image_raw'
 subs.append(rospy.Subscriber(topic,typ,lambda m,t=t:frame(t,m),queue_size=5,buff_size=2**24))
rospy.sleep(20)
for s in subs:s.unregister()
summary={}
for t in ('accepted','rgb','status'):
 r=[x for x in rows if x['topic']==t];sources=sorted(set(x['source_ns'] for x in r if x['source_ns']));arrivals=[x['arrival_ns'] for x in r]
 summary[t]={'count':len(r),'max_source_gap_ms':max(((b-a)/1e6 for a,b in zip(sources,sources[1:])),default=None),'max_arrival_gap_ms':max(((b-a)/1e6 for a,b in zip(arrivals,arrivals[1:])),default=None),'max_source_age_on_receipt_ms':max(((x['arrival_ns']-x['source_ns'])/1e6 for x in r if x['source_ns']),default=None)}
 if t=='status':
  for k in ['processing_wall_sec','status_source_age_sec']:summary[t]['max_'+k]=max((x[k] for x in r if isinstance(x.get(k),(float,int))),default=None)
(OUT/'latency_recovery_before_retry.json').write_text(json.dumps({'summary':summary,'rows':rows,'duration_sec':20,'read_only':True},indent=2)+'\n')
print(json.dumps(summary,indent=2))
