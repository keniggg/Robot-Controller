"""Extract only original color/mask arrays from a sealed recording for review."""
import hashlib,json
from pathlib import Path
import cv2,rosbag,rospy
from cv_bridge import CvBridge
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
R=E/'grasp_records/20261015T075503.791411Z_24849e8304a44341adebca3867d29fdd'
O=E/'ros_log/full_latest_gen22_restart_20261015T003700Z/verification/gen27_attempt3_initial_inference_review'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert json.loads((R/'manifest.json').read_text())['recorder_closed']
ids=[1792050906773036718,1792050906972096443]
topics={'/supervisor/camera/color/image_raw':('color','bgr8'),'/perception/object_mask':('mask','mono8')}
bridge=CvBridge();rows=[]
with rosbag.Bag(str(R/'keyframes.bag')) as b:
    for topic,msg,rec in b.read_messages(topics=list(topics),start_time=rospy.Time(1792050906,500000000),end_time=rospy.Time(1792050907,600000000)):
        ns=msg.header.stamp.to_nsec()
        if ns not in ids:continue
        kind,encoding=topics[topic];p=O/('source_%d_%s.png'%(ns,kind));assert not p.exists()
        assert cv2.imwrite(str(p),bridge.imgmsg_to_cv2(msg,encoding))
        rows.append(dict(path=str(p),source_stamp_ns=ns,received_ns=rec.to_nsec(),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),topic=topic))
assert len(rows)==4
p=O/'additional_initial_color_mask_sources.json';assert not p.exists();p.write_text(json.dumps(dict(record=str(R),rows=rows,no_ros_calls=True,no_hardware_access=True),indent=2)+'\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())))
