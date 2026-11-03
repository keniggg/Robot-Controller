"""Bounded pure appearance feasibility from recorded reference and three frames."""
import importlib.util
import hashlib
import json
import signal
import sys
import time
from pathlib import Path
from types import SimpleNamespace

W=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('g50_appearance_guard',W/'.runtime/extract_gen49_arrival_visibility_20261018.py')
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
V=g.E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
O=V/'gen50_g49_appearance_material_probe_r1';g.O=O
sys.path[:0]=[str(W/'src/alicia_grasp_modes/src'),str(W/'src/alicia_flexible_grasp_supervisor/src')]
import numpy as np
import tf2_py
import tf.transformations
from geometry_msgs.msg import TransformStamped
from alicia_grasp_modes import tabletop as table
from alicia_grasp_modes.rgb_observation_handoff import _mask,_appearance
from alicia_grasp_modes.rgb_instance import refine_candidate
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance,match_observation_appearance
from alicia_flexible_grasp.vision.rgbd_rectification import rectification_rgb_valid_mask
STAMPS=(1792321797421193838,1792321797686099529,1792321797820373773)
def sha(p):return g.sha(p)
def put(p,v):g.save(p,v);return dict(path=str(p),sha256=sha(p))
def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()and not O.exists()
    availpath=V/'gen50_g49_reference_source_availability_r1/result.json';assert sha(availpath)=='8ba5c3aeec73bf6e131ae319dba43719ee2fdb3a9ff171565c835fea553c70f4'
    availability=json.loads(availpath.read_text());priorpath=V/'gen49_same_station_reacquisition_feasibility_r2/result.json';assert sha(priorpath)=='a1e650e336e5d3daccf327c821cfc8f9f53e00f2c8e7d6f4a4a88d6a69d48212';prior=json.loads(priorpath.read_text())
    release=json.loads((V/'gen49_release_manifest.json').read_text());source_pins={}
    for name in ('tabletop.py','rgb_observation_handoff.py','rgb_instance.py','observation_appearance.py'):
        rel='src/alicia_grasp_modes/src/alicia_grasp_modes/'+name;assert sha(W/rel)==release['source_sha256'][rel];source_pins[str(W/rel)]=sha(W/rel)
    O.mkdir();(O/'tmp').mkdir()
    import os,tempfile
    os.environ['TMPDIR']=str(O/'tmp');tempfile.tempdir=str(O/'tmp')
    started=time.monotonic();deadline=started+50.
    def check():
        if time.monotonic()>=deadline:raise ValueError('offline_diagnostic_budget_expired')
    def alarm(*args):raise TimeoutError('offline_diagnostic_60sec_cap')
    signal.signal(signal.SIGALRM,alarm);signal.alarm(60)
    states=json.loads(Path(availability['source_states']['path']).read_text())
    old=next(x['parsed_json']for x in states if x['topic']=='/perception/unknown/detector_status'and x['source_stamp_ns']==1792321745616349697)
    archive=json.loads(Path(prior['status_wire_source']['path']).read_text())
    camera=next(x['parsed_json']['parameters']for x in archive if x['topic']=='/grasp_record/camera_runtime_metadata')
    def images(ns):
        refs=availability['exact_images'][str(ns)];result={}
        for topic,suffix in [('/supervisor/camera/color/image_raw','color'),('/supervisor/camera/depth/image_raw','depth'),('/perception/object_mask','mask')]:
            ref=next(x for x in refs if x['topic']==topic);assert sha(ref['array_path'])==ref['array_sha256'];result[suffix]=(np.load(ref['array_path'],allow_pickle=False),ref)
        return result
    reference=images(1792321745616349697);rgb=reference['color'][0];mask=_mask(reference['mask'][0],rgb.shape[:2])
    assert hashlib.sha256(mask.tobytes()).hexdigest()==old['initial_station_measurement']['refined_mask_sha256']
    assert int(np.count_nonzero(mask))==old['geometry_metrics']['rgb_instance_pixels']==6655
    histogram,contour=_appearance(rgb,mask);valid=rectification_rgb_valid_mask(camera,rgb.shape[:2])
    frozen=FrozenObservationAppearance(rgb,mask,valid,1792321745616349697,histogram,contour)
    np.save(O/'derived_reference_histogram.npy',histogram,allow_pickle=False);np.save(O/'derived_reference_contour.npy',contour,allow_pickle=False)
    buffer=tf2_py.BufferCore(g.rospy.Duration(120));tfref=prior['original_transform_subset'];assert sha(tfref['path'])==tfref['sha256']
    for row in json.loads(Path(tfref['path']).read_text()):
        d=row['transform'];tr=TransformStamped();tr.header.frame_id=d['header']['frame_id'];tr.child_frame_id=d['child_frame_id'];tr.header.stamp=g.rospy.Time(d['header']['stamp']['secs'],d['header']['stamp']['nsecs'])
        for key in ('x','y','z'):setattr(tr.transform.translation,key,d['transform']['translation'][key])
        for key in ('x','y','z','w'):setattr(tr.transform.rotation,key,d['transform']['rotation'][key])
        if row['topic']=='/tf_static':buffer.set_transform_static(tr,'original_G49_TF_subset')
        else:buffer.set_transform(tr,'original_G49_TF_subset')
    cfg=table.Config(**{k:v for k,v in prior['configuration_overrides'].items()if k in table.Config.__dataclass_fields__})
    anchor=SimpleNamespace(position_base=np.asarray(old['anchor_position_base']),support_footprint_base=np.asarray(old['reference_support_footprint_base']))
    plane=np.asarray(old['reference_support_plane_base']);spatial=dict(footprint_base=anchor.support_footprint_base,support_plane_base=plane,height_m=old['reference_object_height_m'])
    rows=[]
    for ns in STAMPS:
        check();items=images(ns);raw=items['depth'][0];encoding=items['depth'][1]['encoding'];depth=np.asarray(raw,dtype=np.float32)*float(camera['depth_scale'])if encoding in('16UC1','mono16')else np.asarray(raw,dtype=np.float32)
        tr=buffer.lookup_transform_core('base_link','camera_link',g.rospy.Time(ns//10**9,ns%10**9));q=tr.transform.rotation;t=tr.transform.translation;T=tf.transformations.quaternion_matrix([q.x,q.y,q.z,q.w]);T[:3,3]=[t.x,t.y,t.z]
        K=[camera[k]for k in('fx','fy','cx','cy')]
        audit=table._audit_current_support(depth,K,T,cfg,anchor,plane)
        seg=table.segment(depth,K,T,cfg,target_position_base=anchor.position_base,plane_base=plane,target_footprint_base=anchor.support_footprint_base,fit_current_support=True)
        current_rgb=items['color'][0];current_valid=rectification_rgb_valid_mask(camera,current_rgb.shape[:2]);candidates=[]
        for index,candidate in enumerate(seg.candidates):
            check();row=dict(index=index,raw_bbox=list(candidate.bbox),raw_position_base=candidate.position_base.tolist(),raw_extent_m=candidate.extent_m.tolist(),raw_quality=candidate.quality)
            try:
                refined=refine_candidate(current_rgb,depth,K,T,candidate,cfg,observation_arrival_retry=True,observation_spatial_reference=spatial,**({}if current_valid is None else{'rgb_valid':current_valid}))
                appearance=match_observation_appearance(current_rgb,refined.mask,histogram,contour,rgb_valid=current_valid,appearance_context=dict(reference=frozen,source_ns=ns,deadline_monotonic=deadline,check_current=check))
                row.update(refined_bbox=list(refined.bbox),refined_mask_pixels=int(np.count_nonzero(refined.mask)),refined_quality=refined.quality,appearance=appearance)
            except (ValueError,TimeoutError)as exc:row['refinement_or_appearance_error']=str(exc)
            candidates.append(row)
        rows.append(dict(source_stamp_ns=ns,original_false_mask_pixels=int(np.count_nonzero(items['mask'][0])),support_audit=audit,candidate_count=len(seg.candidates),candidate_appearance=candidates))
    signal.alarm(0)
    assert not g.ATTEMPTS and all(sha(p)==v for p,v in source_pins.items())
    report=dict(kind='pure_G49_original_reference_and_three_source_appearance_material',passed=True,passed_meaning='Source/reference bindings and bounded pure support/refinement/appearance calls completed; not a Task/producer acceptance test',elapsed_sec=time.monotonic()-started,availability_result=dict(path=str(availpath),sha256=sha(availpath)),prior_support_result=dict(path=str(priorpath),sha256=sha(priorpath)),reference=dict(source_stamp_ns=1792321745616349697,raw_RGB_sha256=reference['color'][1]['decoded_array_sha256'],published_mask_sha256=reference['mask'][1]['decoded_array_sha256'],original_bound_binary_mask_sha256=old['initial_station_measurement']['refined_mask_sha256'],reconstructed_binary_mask_sha256=frozen.mask_sha256,reference_histogram_sha256=sha(O/'derived_reference_histogram.npy'),reference_contour_sha256=sha(O/'derived_reference_contour.npy'),recorded_histogram_or_contour_digest_available=False,source_id=old['initial_station_measurement']['source_id']),source_stamps_ns=list(STAMPS),results=rows,source_sha256=source_pins,helper_sha256=sha(__file__),external_boundary_attempts=g.ATTEMPTS,tracker_loss_cleared=False,original_false_masks_used_as_foreground=False,Task_or_producer_thread_schedule_reconstructed=False,measured_face_source_reconstructed=False,ROS_initialized=False,hardware_contacted=False,temperature_read=False,execution_authority=False,limitations=['Histogram and contour are deterministically recomputed from exact recorded RGB and the original mask whose normalized SHA is in the original reference status; their own historical serialized digests were not recorded.','This diagnostic checks every original current-depth candidate; it does not select/discard rivals or perform the original fragment-consensus/three-frame confirmation.','It retains an original-data FrozenObservationAppearance but does not reconstruct the optional premotion measured-face cache or historical lifetime/owner callback. The diagnostic check callback enforces only its bounded offline computation.','Anchor spatial footprint, plane and height are recorded values; no missing anchor mask or raw historical tracker instance is invented.','Original source stamps remain historical. This pure result grants no freshness, identity, geometry or motion authority.'])
    print(json.dumps(put(O/'result.json',report),ensure_ascii=False))
if __name__=='__main__':main()
