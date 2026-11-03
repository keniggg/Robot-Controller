"""Three recorded G49 frames: exact support audit; no tracking reset or authority."""
import importlib.util
import hashlib
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

W=Path(__file__).resolve().parents[1]
path=W/'.runtime/extract_gen49_arrival_visibility_20261018.py'
spec=importlib.util.spec_from_file_location('g49_station_offline_guard',path)
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
V=g.E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
O=V/'gen49_same_station_reacquisition_feasibility_r2';g.O=O
STAMPS=(1792321796686522722,1792321797021150112,1792321797155105352)
IMAGE_TOPICS={'/supervisor/camera/color/image_raw':'color','/supervisor/camera/depth/image_raw':'depth','/perception/object_mask':'mask',
              '/experiment/observation_rgbd/color':'observation_color','/experiment/observation_rgbd/depth':'observation_depth'}
TOPICS=tuple(IMAGE_TOPICS)+('/tf','/tf_static')
sys.path.insert(0,str(W/'src/alicia_grasp_modes/src'))
import alicia_grasp_modes.tabletop as table
import tf2_py
import numpy as np
def sha(p):return g.sha(p)
def put(p,v):g.save(p,v);return dict(path=str(p),sha256=sha(p))
def array(m):
    dtype={'bgr8':np.uint8,'rgb8':np.uint8,'mono8':np.uint8,'8UC1':np.uint8,'16UC1':np.uint16,'mono16':np.uint16,'32FC1':np.float32}[m.encoding]
    channels=3 if m.encoding in ('bgr8','rgb8')else 1
    a=np.frombuffer(m.data,dtype=dtype).reshape(m.height,m.step//np.dtype(dtype).itemsize)[:,:m.width*channels]
    return np.array(a.reshape((m.height,m.width,3)if channels==3 else(m.height,m.width)),copy=True)
def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()and not O.exists()
    manifest=json.loads((g.G/'manifest.json').read_text());ref=next(x for x in manifest['files']if x['path']=='keyframes.bag')
    assert manifest['recorder_closed']is True and sha(g.G/'manifest.json')=='747a7e1f2240a6e612763e4320a34a428975fb4fe022b278c479f695b8ad7ed0'
    assert sha(g.G/'keyframes.bag')==ref['sha256']
    release=V/'gen49_release_manifest.json';sealed=json.loads(release.read_text())
    rel=Path('src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py')
    assert sha(W/rel)==sealed['source_sha256'][str(rel)]and Path(table.__file__).resolve()==(W/rel).resolve()
    O.mkdir();(O/'tmp').mkdir()
    import os,tempfile
    os.environ['TMPDIR']=str(O/'tmp');tempfile.tempdir=str(O/'tmp')
    existing=V/'gen49_arrival_visibility_r1/arrival_messages.json';messages=json.loads(existing.read_text())
    good=next(x['parsed_json']for x in messages if x['topic']=='/perception/unknown/detector_status'and(x.get('parsed_json')or{}).get('source_stamp_ns')==1792321796341869592)
    statuses={x['parsed_json']['source_stamp_ns']:x['parsed_json']for x in messages if x['topic']=='/perception/unknown/detector_status'and(x.get('parsed_json')or{}).get('source_stamp_ns')in STAMPS}
    params=[x['parsed_json']['parameters']for x in messages if x['topic']=='/grasp_record/camera_runtime_metadata']
    camera_keys=('fx','fy','cx','cy','depth_scale','frame_id','width','height','align_depth_to_color')
    camera_variants={json.dumps({k:p[k]for k in camera_keys},sort_keys=True)for p in params}
    assert len(camera_variants)==1;camera=json.loads(next(iter(camera_variants)))
    cfgdoc=json.loads((g.G/'perception_execution_configuration.json').read_text())
    overrides=cfgdoc['parameters']['/unknown_perception']
    cfg=table.Config(**{k:v for k,v in overrides.items()if k in table.Config.__dataclass_fields__})
    anchor=SimpleNamespace(position_base=np.array(good['anchor_position_base'],dtype=float),support_footprint_base=np.array(good['reference_support_footprint_base'],dtype=float))
    plane=np.array(good['reference_support_plane_base'],dtype=float)
    buffer=tf2_py.BufferCore(g.rospy.Duration(120.0));seen={};tf_rows=[];frames={s:{}for s in STAMPS}
    with g.rosbag.Bag(str(g.G/'keyframes.bag'),'r')as bag:
        info=bag.get_type_and_topic_info().topics
        metadata={t:dict(type=info[t].msg_type,count=info[t].message_count)for t in TOPICS if t in info}
        for topic,m,received in bag.read_messages(topics=list(TOPICS)):
            recv=received.to_nsec()
            if topic in IMAGE_TOPICS:
                ns=m.header.stamp.to_nsec()
                if ns not in STAMPS:continue
                kind=IMAGE_TOPICS[topic];a=array(m);p=O/('%s_%s.npy'%(ns,kind));np.save(p,a,allow_pickle=False)
                meta=g.image_meta(m,recv);meta.update(topic=topic,array_path=str(p),array_file_sha256=sha(p),array_dtype=str(a.dtype),array_shape=list(a.shape),decoded_array_sha256=hashlib.sha256(a.tobytes()).hexdigest())
                frames[ns][kind]=dict(array=a,meta=meta)
            else:
                for tr in m.transforms:
                    ns=tr.header.stamp.to_nsec();key=(tr.header.frame_id,tr.child_frame_id,ns,topic)
                    body=g.payload(tr);encoded=json.dumps(body,sort_keys=True).encode()
                    if key in seen:
                        assert seen[key]==encoded
                        continue
                    seen[key]=encoded
                    if topic=='/tf_static':buffer.set_transform_static(tr,'sealed_G49_record')
                    else:buffer.set_transform(tr,'sealed_G49_record')
                    if topic=='/tf_static'or STAMPS[0]-2_000_000_000<=ns<=STAMPS[-1]+2_000_000_000:
                        tf_rows.append(dict(topic=topic,received_ns=recv,transform=body))
    tfref=put(O/'original_transform_subset.json',tf_rows)
    result=[]
    for ns in STAMPS:
        row=dict(source_stamp_ns=ns,original_status=dict(state=statuses[ns]['state'],detail=statuses[ns]['detail'],source_stamp_ns=statuses[ns]['source_stamp_ns'],frame_timing=statuses[ns]['frame_timing']),available_images={k:v['meta']for k,v in frames[ns].items()})
        depth_item=frames[ns].get('depth')or frames[ns].get('observation_depth')
        rgb_item=frames[ns].get('color')or frames[ns].get('observation_color')
        if depth_item is None or rgb_item is None:
            row['replay_possible']=False;row['missing']='recorded exact RGB/depth pair';result.append(row);continue
        for kind in ('color','depth'):
            a=frames[ns].get(kind);b=frames[ns].get('observation_'+kind)
            if a is not None and b is not None:assert a['meta']['data_sha256']==b['meta']['data_sha256']
        try:
            tr=buffer.lookup_transform_core('base_link','camera_link',g.rospy.Time(ns//10**9,ns%10**9))
        except Exception as exc:
            row['replay_possible']=False;row['missing']='original TF chain: '+str(exc);result.append(row);continue
        import tf.transformations
        q=tr.transform.rotation;t=tr.transform.translation
        T=tf.transformations.quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[t.x,t.y,t.z]
        row['original_T_base_camera']=T.tolist();row['TF_lookup_source_ns']=ns
        raw=depth_item['array'];enc=depth_item['meta']['encoding']
        depth=np.asarray(raw,dtype=np.float32)*float(camera['depth_scale'])if enc in('16UC1','mono16')else np.asarray(raw,dtype=np.float32)
        intrinsics=[camera[k]for k in('fx','fy','cx','cy')]
        traces=[]
        def trace(frame,event,arg):
            if frame.f_code is table._validate_support_plane_update.__code__ and event in('return','exception'):
                values={k:(v.tolist()if isinstance(v,np.ndarray)else v)for k,v in frame.f_locals.items()if k in('reference','current','angle','anchor_shift','footprint_shift','maximum','max_shift')}
                if event=='exception':values['exception']=str(arg[1])
                traces.append(values)
            return trace
        sys.settrace(trace)
        try:
            audit=table._audit_current_support(depth,intrinsics,T,cfg,anchor,plane)
            row['support_audit']=audit
        except ValueError as exc:row['support_error']=str(exc)
        finally:sys.settrace(None)
        row['support_validation_trace']=traces
        if row.get('support_audit',{}).get('support_current_audit_available')is True:
            try:
                segmentation=table.segment(depth,intrinsics,T,cfg,target_position_base=anchor.position_base,plane_base=plane,target_footprint_base=anchor.support_footprint_base,fit_current_support=True)
                row['current_support_segment']=dict(candidate_count=len(segmentation.candidates),metrics=segmentation.metrics,plane_base=segmentation.plane_base.tolist(),candidates=[dict(position_base=x.position_base.tolist(),extent_m=x.extent_m.tolist(),bbox=list(x.bbox),quality=x.quality,metrics=x.metrics)for x in segmentation.candidates])
            except ValueError as exc:row['current_support_segment_error']=str(exc)
        row['replay_possible']=True;result.append(row)
    assert sha(g.G/'keyframes.bag')==ref['sha256']and sha(W/rel)==sealed['source_sha256'][str(rel)]and not g.ATTEMPTS
    report=dict(kind='G49_recorded_same_station_RGBD_support_replay',passed=True,record=str(g.G),source_stamps_ns=list(STAMPS),bag_sha256=ref['sha256'],bag_before_after_same=True,release_sha256=sha(release),source_path=str(W/rel),source_sha256=sha(W/rel),source_unchanged=True,exact_tf_algorithm='original tf2 BufferCore with recorded dynamic/static transform messages; no ROS node/master',original_transform_subset=tfref,camera=camera,camera_scope='All archived arrival-window runtime parameter reads agree; no recorded CameraInfo topic in this bag',reference_anchor_position_base=anchor.position_base.tolist(),reference_support_plane_base=plane.tolist(),reference_support_footprint_base=anchor.support_footprint_base.tolist(),configuration_overrides=overrides,status_wire_source=dict(path=str(existing),sha256=sha(existing)),recorded_topic_metadata=metadata,results=result,helper_sha256=sha(__file__),guard_helper_sha256=sha(path),external_boundary_attempts=g.ATTEMPTS,ROS_initialized=False,hardware_contacted=False,temperature_read=False,tracker_loss_cleared=False,motion_or_candidate_authority_granted=False,production_or_original_record_modified=False,limitations=['Original status/timing proves which recorded pipeline stages completed; replay of current-support segment is separate, does not replace original tracking or registration.','Zero masks published for detected=False frames are not missing RGBD; pure support segmentation requires raw depth, not an invented foreground mask.','Passing support or finding depth candidates does not prove same identity, qualified cuboid, opening, path feasibility or physical grasp.'])
    print(json.dumps(put(O/'result.json',report),ensure_ascii=False))
if __name__=='__main__':main()
