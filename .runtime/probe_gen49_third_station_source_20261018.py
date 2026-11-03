"""Add one exact recorded source, reusing sealed TF and two prior support results."""
import importlib.util
import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

W=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('g49_station_prior_probe',W/'.runtime/probe_gen49_same_station_sources_20261018.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
g=m.g;np=m.np;table=m.table;V=m.V
O=V/'gen49_same_station_reacquisition_feasibility_r3';g.O=O
NS=1792321797421193838
def sha(p):return g.sha(p)
def put(p,v):g.save(p,v);return dict(path=str(p),sha256=sha(p))
def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()and not O.exists()
    prior_path=V/'gen49_same_station_reacquisition_feasibility_r2/result.json'
    assert sha(prior_path)=='a1e650e336e5d3daccf327c821cfc8f9f53e00f2c8e7d6f4a4a88d6a69d48212'
    prior=json.loads(prior_path.read_text());assert sha(g.G/'keyframes.bag')==prior['bag_sha256']
    assert sha(prior['source_path'])==prior['source_sha256']
    O.mkdir();(O/'tmp').mkdir()
    import os,tempfile
    os.environ['TMPDIR']=str(O/'tmp');tempfile.tempdir=str(O/'tmp')
    from geometry_msgs.msg import TransformStamped
    buffer=m.tf2_py.BufferCore(g.rospy.Duration(120))
    tfref=prior['original_transform_subset'];assert sha(tfref['path'])==tfref['sha256']
    for row in json.loads(Path(tfref['path']).read_text()):
        d=row['transform'];tr=TransformStamped();tr.header.seq=d['header']['seq'];tr.header.frame_id=d['header']['frame_id']
        tr.header.stamp=g.rospy.Time(d['header']['stamp']['secs'],d['header']['stamp']['nsecs']);tr.child_frame_id=d['child_frame_id']
        for key in('x','y','z'):setattr(tr.transform.translation,key,d['transform']['translation'][key])
        for key in('x','y','z','w'):setattr(tr.transform.rotation,key,d['transform']['rotation'][key])
        if row['topic']=='/tf_static':buffer.set_transform_static(tr,'original_G49_archived_TF')
        else:buffer.set_transform(tr,'original_G49_archived_TF')
    images={}
    with g.rosbag.Bag(str(g.G/'keyframes.bag'),'r')as bag:
        for topic,msg,received in bag.read_messages(topics=['/supervisor/camera/color/image_raw','/supervisor/camera/depth/image_raw','/perception/object_mask'],start_time=g.rospy.Time(1792321797,300000000),end_time=g.rospy.Time(1792321797,900000000)):
            if msg.header.stamp.to_nsec()!=NS:continue
            kind={'/supervisor/camera/color/image_raw':'color','/supervisor/camera/depth/image_raw':'depth','/perception/object_mask':'mask'}[topic]
            a=m.array(msg);p=O/('%s_%s.npy'%(NS,kind));np.save(p,a,allow_pickle=False)
            row=g.image_meta(msg,received.to_nsec());row.update(topic=topic,array_path=str(p),array_file_sha256=sha(p),decoded_array_sha256=hashlib.sha256(a.tobytes()).hexdigest(),array_dtype=str(a.dtype),array_shape=list(a.shape))
            if kind=='mask':row['nonzero_count']=int(np.count_nonzero(a))
            images[kind]=(a,row)
    assert set(images)=={'color','depth','mask'}
    png=O/('original_color_%s.png'%NS);assert g.cv2.imwrite(str(png),images['color'][0])and np.array_equal(g.cv2.imread(str(png)),images['color'][0])
    tr=buffer.lookup_transform_core('base_link','camera_link',g.rospy.Time(NS//10**9,NS%10**9))
    import tf.transformations
    q=tr.transform.rotation;t=tr.transform.translation;T=tf.transformations.quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[t.x,t.y,t.z]
    raw=images['depth'][0];enc=images['depth'][1]['encoding'];camera=prior['camera']
    depth=np.asarray(raw,dtype=np.float32)*float(camera['depth_scale'])if enc in('16UC1','mono16')else np.asarray(raw,dtype=np.float32)
    cfg=table.Config(**{k:v for k,v in prior['configuration_overrides'].items()if k in table.Config.__dataclass_fields__})
    messages=json.loads(Path(prior['status_wire_source']['path']).read_text());assert sha(prior['status_wire_source']['path'])==prior['status_wire_source']['sha256']
    good=next(x['parsed_json']for x in messages if x['topic']=='/perception/unknown/detector_status'and(x.get('parsed_json')or{}).get('source_stamp_ns')==1792321796341869592)
    original=next(x for x in messages if x['topic']=='/perception/unknown/detector_status'and(x.get('parsed_json')or{}).get('source_stamp_ns')==NS)
    anchor=SimpleNamespace(position_base=np.array(good['anchor_position_base']),support_footprint_base=np.array(good['reference_support_footprint_base']),extent_m=np.array(good['anchor_extent_m']))
    last=SimpleNamespace(position_base=np.array(good['position_base']))
    plane=np.array(good['reference_support_plane_base']);K=[camera[k]for k in('fx','fy','cx','cy')]
    audit=table._audit_current_support(depth,K,T,cfg,anchor,plane)
    seg=table.segment(depth,K,T,cfg,target_position_base=anchor.position_base,plane_base=plane,target_footprint_base=anchor.support_footprint_base,fit_current_support=True)
    summaries=[]
    for old in prior['results'][1:]:
        c=old['current_support_segment']['candidates'];objects=[SimpleNamespace(position_base=np.array(x['position_base']),extent_m=np.array(x['extent_m']))for x in c]
        matches=table._matching_candidates(objects,anchor,last,cfg)
        summaries.append(dict(source_stamp_ns=old['source_stamp_ns'],support_max_distance_m=old['support_audit']['support_reference_max_distance_m'],depth_candidate_count=len(c),original_geometry_gate_matching_count=len(matches),matching_anchor_distances_m=[x[0]for x in matches],source='sealed prior r2 exact numeric result'))
    matches=table._matching_candidates(seg.candidates,anchor,last,cfg)
    summaries.append(dict(source_stamp_ns=NS,support_max_distance_m=audit['support_reference_max_distance_m'],depth_candidate_count=len(seg.candidates),original_geometry_gate_matching_count=len(matches),matching_anchor_distances_m=[x[0]for x in matches],source='this original segment replay'))
    intervening=[dict(source_stamp_ns=x['parsed_json']['source_stamp_ns'],detail=x['parsed_json']['detail'],wire_sha256=x['wire_sha256'])for x in messages if x['topic']=='/perception/unknown/detector_status'and 1792321797155105352<(x.get('parsed_json')or{}).get('source_stamp_ns',0)<NS]
    assert sha(g.G/'keyframes.bag')==prior['bag_sha256']and sha(prior['source_path'])==prior['source_sha256']and not g.ATTEMPTS
    report=dict(kind='G49_third_original_support_frame_before_capture_deadline',passed=True,passed_meaning='Original input integrity and bounded support/geometry replay completed; no identity or execution acceptance',record=str(g.G),source_stamp_ns=NS,capture_deadline_ns=1792321798873908758,after_required_new_settle_ns=1792321796914392082,prior_result=dict(path=str(prior_path),sha256=sha(prior_path)),reused_original_TF=tfref,original_T_base_camera=T.tolist(),exact_images={k:v[1]for k,v in images.items()},original_RGB_png=dict(path=str(png),sha256=sha(png)),original_status=dict(detail=original['parsed_json']['detail'],state=original['parsed_json']['state'],wire_sha256=original['wire_sha256'],frame_timing=original['parsed_json']['frame_timing']),support_audit=audit,current_support_segment=dict(candidate_count=len(seg.candidates),metrics=seg.metrics,candidates=[dict(position_base=x.position_base.tolist(),extent_m=x.extent_m.tolist(),bbox=list(x.bbox),quality=x.quality)for x in seg.candidates]),three_selected_support_frame_summary=summaries,intervening_original_rejections=intervening,source_sha256=prior['source_sha256'],helper_sha256=sha(__file__),external_boundary_attempts=g.ATTEMPTS,tracker_loss_cleared=False,original_false_mask_used_as_object_foreground=False,ROS_initialized=False,hardware_contacted=False,temperature_read=False,gate_or_production_or_control_changed=False,limitations=['Three selected support-valid frames are not three uninterrupted eligible frames: the recorded intervening7.257mm hard support failure is retained explicitly.','Read-only original geometry matching uses archived anchor extent/position and last-good published position only. It does not reproduce full appearance/registration/confirmation state or grant identity.','Original published mask is retained as zero-mask failure evidence and is never used to create foreground.','Passing support and having candidates does not establish cuboid, opening, collision/path acceptance or physical success.'])
    print(json.dumps(put(O/'result.json',report),ensure_ascii=False))
if __name__=='__main__':main()
