"""Six exact G49 sources, including original observation-only RGBD transport."""
import importlib.util
import hashlib
import io
import json
from pathlib import Path

W=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('g50_g49_offline_guard',W/'.runtime/extract_gen49_arrival_visibility_20261018.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
O=g.E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification/gen50_g49_reference_source_availability_r1';g.O=O
NS=(1792321745616349697,1792321796018874883,1792321796208590507,1792321797421193838,1792321797686099529,1792321797820373773)
TOPICS=('/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask',
        '/experiment/observation_rgbd/color','/experiment/observation_rgbd/depth','/perception/object','/perception/unknown/detector_status')
def sha(p):return g.sha(p)
def put(p,v):g.save(p,v);return dict(path=str(p),sha256=sha(p))
def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()and not O.exists()
    manifest=json.loads((g.G/'manifest.json').read_text());bagref=next(x for x in manifest['files']if x['path']=='keyframes.bag')
    assert manifest['recorder_closed']is True and sha(g.G/'keyframes.bag')==bagref['sha256']
    O.mkdir();(O/'tmp').mkdir()
    import os,tempfile
    os.environ['TMPDIR']=str(O/'tmp');tempfile.tempdir=str(O/'tmp')
    found={n:[]for n in NS};states=[]
    with g.rosbag.Bag(str(g.G/'keyframes.bag'),'r')as bag:
        info=bag.get_type_and_topic_info().topics
        metadata={t:dict(type=info[t].msg_type,count=info[t].message_count)for t in TOPICS if t in info}
        for lower,upper in ((g.rospy.Time(1792321745,100000000),g.rospy.Time(1792321746,300000000)),
                            (g.rospy.Time(1792321795,900000000),g.rospy.Time(1792321798,300000000))):
            for topic,msg,received in bag.read_messages(topics=list(TOPICS),start_time=lower,end_time=upper):
                parsed=None
                if topic=='/perception/unknown/detector_status':
                    parsed=json.loads(msg.data);ns=parsed.get('source_stamp_ns')
                else:ns=msg.header.stamp.to_nsec()
                if ns not in NS:continue
                if msg._type=='sensor_msgs/Image':
                    row=g.image_meta(msg,received.to_nsec());row['topic']=topic
                    dtype={'bgr8':g.np.uint8,'rgb8':g.np.uint8,'mono8':g.np.uint8,'8UC1':g.np.uint8,'16UC1':g.np.uint16,'mono16':g.np.uint16,'32FC1':g.np.float32}[msg.encoding]
                    channels=3 if msg.encoding in('bgr8','rgb8')else 1
                    a=g.np.frombuffer(msg.data,dtype=dtype).reshape(msg.height,msg.step//g.np.dtype(dtype).itemsize)[:,:msg.width*channels]
                    a=g.np.array(a.reshape((msg.height,msg.width,3)if channels==3 else(msg.height,msg.width)),copy=True)
                    kind='color'if channels==3 else('mask'if topic=='/perception/object_mask'else'depth')
                    p=O/('%s_%s_%s.npy'%(ns,kind,row['data_sha256'][:12]))
                    if not p.exists():g.np.save(p,a,allow_pickle=False)
                    row.update(array_path=str(p),array_sha256=sha(p),decoded_array_sha256=hashlib.sha256(a.tobytes()).hexdigest(),array_dtype=str(a.dtype),array_shape=list(a.shape))
                    if kind=='mask':row['nonzero_count']=int(g.np.count_nonzero(a))
                    if kind=='color':
                        bgr=a if msg.encoding=='bgr8'else a[:,:,::-1];png=p.with_suffix('.png')
                        if not png.exists():assert g.cv2.imwrite(str(png),bgr)
                        assert g.np.array_equal(g.cv2.imread(str(png)),bgr)
                        row.update(png_path=str(png),png_sha256=sha(png),resampled=False,overlay_added=False)
                    found[ns].append(row)
                else:
                    stream=io.BytesIO();msg.serialize(stream);raw=stream.getvalue()
                    states.append(dict(topic=topic,source_stamp_ns=ns,received_ns=received.to_nsec(),wire_sha256=hashlib.sha256(raw).hexdigest(),payload=g.payload(msg),parsed_json=parsed))
    assert sha(g.G/'keyframes.bag')==bagref['sha256']and not g.ATTEMPTS
    refs=put(O/'original_source_states.json',states)
    summary=[]
    for ns in NS:
        rows=found[ns];color=[r for r in rows if r['topic'].endswith('/color/image_raw')or r['topic']=='/experiment/observation_rgbd/color'];depth=[r for r in rows if r['topic'].endswith('/depth/image_raw')or r['topic']=='/experiment/observation_rgbd/depth'];mask=[r for r in rows if r['topic']=='/perception/object_mask']
        summary.append(dict(source_stamp_ns=ns,exact_color_available=bool(color),exact_depth_available=bool(depth),exact_published_mask_available=bool(mask),original_topics=[r['topic']for r in rows],standard_and_observation_color_equal=len({r['data_sha256']for r in color})<=1,standard_and_observation_depth_equal=len({r['data_sha256']for r in depth})<=1))
    report=dict(kind='G49_original_six_source_transport_availability',passed=True,record=str(g.G),manifest_sha256=sha(g.G/'manifest.json'),bag_sha256=bagref['sha256'],bag_before_after_same=True,selected_topics=metadata,summary=summary,exact_images={str(k):v for k,v in found.items()},source_states=refs,helper_sha256=sha(__file__),external_boundary_attempts=g.ATTEMPTS,ROS_initialized=False,hardware_contacted=False,temperature_read=False,source_or_control_or_original_record_modified=False,correction_scope='Prior arrival_visibility extraction checked standard RGB topic only. Any source found on original /experiment/observation_rgbd stream corrects a broader bag-missing interpretation; original earlier result remains unchanged.',limitations=['Observation-only RGBD are historical recorded original sensor messages and are not published to execution.','A missing exact mask cannot be recreated from a bounding box or a later mask as historical reference evidence.','Availability does not prove original appearance or identity confirmation succeeded.'])
    ref=put(O/'result.json',report);print(json.dumps(dict(result=ref,summary=summary),ensure_ascii=False))
if __name__=='__main__':main()
