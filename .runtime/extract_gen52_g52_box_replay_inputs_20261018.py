"""Exact four source RGBD/mask and metadata from sealed G52; offline only."""
import base64,hashlib,io,json,os,platform,struct,sys
from pathlib import Path
import extract_gen50_retry_r30_rgb_20261018 as boundary
W=Path(__file__).resolve().parents[1];E=boundary.E;V=boundary.N/'verification'
G=E/'grasp_records/20261018T143337.794533Z_bfea8721dba04526980035ef69ebe860'
OUT=V/'gen52_actual_handoff_monitor_r1/exact_box_inputs_r1'
STAMPS=(1792334020688617944,1792334065759709835,1792334066024749755,1792334066357594251)
def put(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n');assert json.loads(path.read_text())==value
def main():
 assert E.parent.parent.is_mount()and not OUT.exists();OUT.mkdir()
 os.environ.update(PYTHONDONTWRITEBYTECODE='1',ROS_MASTER_URI='http://127.0.0.1:1',ROS_LOG_DIR=str(OUT),ROS_HOME=str(OUT),TMPDIR=str(OUT),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
 sys.dont_write_bytecode=True;sys.path[:0]=[str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
 n=os.uname();platform.uname=lambda:platform.uname_result(*(tuple(n)+(n.machine,)));platform.architecture=lambda *a,**k:(str(8*struct.calcsize('P'))+'bit','ELF')
 boundary.OUT=OUT;sys.addaudithook(boundary.audit)
 import numpy as np,cv2,rosbag
 import genpy
 m=json.loads((G/'manifest.json').read_text());assert m['state']=='complete'and m['result']=='failure'and m['recorder_closed'];br=next(x for x in m['files']if x['path']=='keyframes.bag');bp=G/'keyframes.bag';assert boundary.sha(bp)==br['sha256']
 audit=G/'keyframes_audits/404d30752dbb490c83ce54c9d307676d52bb75355b3683a09825d294e293b402.json';assert boundary.sha(audit)==audit.stem;a=json.loads(audit.read_text());retained=a['remote_diagnostics']['tabletop_geometry']['regular_box_inference']['retained_pixel_sha256']
 topics={'/supervisor/camera/color/image_raw':'color','/experiment/observation_rgbd/color':'color','/supervisor/camera/depth/image_raw':'depth','/experiment/observation_rgbd/depth':'depth','/perception/object_mask':'mask'}
 rows=[];arrays={};contexts=[]
 with rosbag.Bag(str(bp),'r')as bag:
  for topic,msg,recv in bag.read_messages(topics=list(topics),start_time=genpy.Time(1792334019,0),end_time=genpy.Time(1792334070,0)):
   ns=msg.header.stamp.to_nsec()
   if ns not in STAMPS:continue
   kind=topics[topic];raw=bytes(msg.data);assert len(raw)==msg.height*msg.step
   if kind=='color':
    assert msg.encoding in('rgb8','bgr8');ar=np.frombuffer(raw,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()
    if msg.encoding=='rgb8':ar=ar[:,:,::-1].copy()
   elif kind=='depth':
    assert msg.encoding in('16UC1','mono16');dtype='>u2'if msg.is_bigendian else'<u2';ar=np.frombuffer(raw,dtype=dtype).reshape(msg.height,msg.step//2)[:,:msg.width].astype(np.uint16,copy=True)
   else:
    assert msg.encoding=='mono8';ar=np.frombuffer(raw,np.uint8).reshape(msg.height,msg.step)[:,:msg.width].copy()
   stream=io.BytesIO();msg.serialize(stream);key=(ns,kind)
   if key in arrays:assert np.array_equal(arrays[key],ar)
   else:
    arrays[key]=ar;np.save(str(OUT/('%d_%s.npy'%key)),ar,allow_pickle=False)
    if kind=='color'and ns==STAMPS[-1]:assert cv2.imwrite(str(OUT/'final_original_rgb.png'),ar)
   h=hashlib.sha256();h.update(str(ar.shape).encode('ascii'));h.update(b'\0'+ar.dtype.str.encode('ascii')+b'\0');h.update(ar.tobytes())
   rows.append(dict(topic=topic,kind=kind,source_stamp_ns=ns,bag_timestamp_ns=recv.to_nsec(),frame_id=msg.header.frame_id,shape=list(ar.shape),dtype=ar.dtype.str,encoding=msg.encoding,step=msg.step,raw_data_sha256=hashlib.sha256(raw).hexdigest(),wire_sha256=hashlib.sha256(stream.getvalue()).hexdigest(),canonical_array_bare_sha256=hashlib.sha256(ar.tobytes()).hexdigest(),canonical_array_shaped_sha256=h.hexdigest(),npy_path=str(OUT/('%d_%s.npy'%key)),npy_sha256=boundary.sha(OUT/('%d_%s.npy'%key))))
  for topic,msg,recv in bag.read_messages(topics=['/grasp_record/camera_runtime_metadata','/perception/unknown/detector_status','/perception/object']):
   if msg._type=='std_msgs/String':
    try:d=json.loads(msg.data)
    except ValueError:continue
    if topic=='/perception/unknown/detector_status'and d.get('source_stamp_ns')not in STAMPS:continue
    if topic=='/grasp_record/camera_runtime_metadata'and contexts and any(x['topic']==topic for x in contexts):continue
   else:
    if msg.header.stamp.to_nsec()not in STAMPS:continue
    d=None
   stream=io.BytesIO();msg.serialize(stream);contexts.append(dict(topic=topic,bag_timestamp_ns=recv.to_nsec(),message_type=msg._type,md5sum=msg._md5sum,wire_sha256=hashlib.sha256(stream.getvalue()).hexdigest(),wire_base64=base64.b64encode(stream.getvalue()).decode(),parsed_json=d))
 assert boundary.sha(bp)==br['sha256']and not boundary.BOUNDARY
 final={kind:arrays.get((STAMPS[-1],kind))for kind in('color','depth','mask')};assert all(x is not None for x in final.values())
 final_checks={}
 for kind in('color','depth','mask'):
  bare=hashlib.sha256(final[kind].tobytes()).hexdigest();final_checks[kind]=dict(actual_bare_sha256=bare,expected_retained_sha256=retained.get(kind+'_sha256'),exact=bare==retained.get(kind+'_sha256'))
 assert all(v['exact']for v in final_checks.values())
 put(OUT/'message_index.json',rows);put(OUT/'original_nonimage_context.json',contexts)
 result=dict(passed=True,record=str(G),manifest_sha256=boundary.sha(G/'manifest.json'),bag_sha256=br['sha256'],bag_before_after_same=True,audit=dict(path=str(audit),sha256=audit.stem),required_stamps=list(STAMPS),source_availability=[dict(source_stamp_ns=s,available={k:(s,k)in arrays for k in('color','depth','mask')})for s in STAMPS],final_retained_pixels=final_checks,message_index=dict(path=str(OUT/'message_index.json'),sha256=boundary.sha(OUT/'message_index.json')),nonimage_context=dict(path=str(OUT/'original_nonimage_context.json'),sha256=boundary.sha(OUT/'original_nonimage_context.json')),snapshot_transform=a['snapshot_transform'],cuboid_T_base_optical=a['remote_diagnostics']['tabletop_geometry']['regular_box_inference']['cuboid_rgb_interval']['T_base_optical'],helper_sha256=boundary.sha(__file__),external_boundary_attempts=boundary.BOUNDARY,ROS_initialized=False,hardware_contacted=False,temperature_read=False,production_or_record_modified=False,full_request_or_grasp_reproduced=False)
 put(OUT/'result.json',result);print(json.dumps(dict(result=str(OUT/'result.json'),sha256=boundary.sha(OUT/'result.json'),availability=result['source_availability'],retained_exact=final_checks),ensure_ascii=False))
if __name__=='__main__':main()
