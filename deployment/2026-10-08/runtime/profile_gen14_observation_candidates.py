#!/usr/bin/env python3
"""Offline endpoint construction profile of nearest sealed gen14 audit; no ROS IO."""
import argparse, cProfile, hashlib, json, pstats, sys, time
from collections import Counter
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import rosbag, rospy

ROOT=Path(__file__).resolve().parents[1]
for p in ['src/alicia_grasp_modes/tests','src/alicia_grasp_modes/src',
          'src/alicia_flexible_grasp_supervisor/src']:
    sys.path.insert(0,str(ROOT/p))
import test_remote_modes as fixture
from _pytest.monkeypatch import MonkeyPatch
from alicia_grasp_modes.observation_policy import observation_config

RECORD=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261006T085709.051090Z_bc8e72c457f542a18a60e9e94fb6b3e1')
AUDIT='0e8cd17a6329a2074364ee07bfea9991161f2f9d15e7d77016b00fbe65d8e8e2.json'

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((RECORD/'manifest.json').read_text())
    assert manifest['recorder_closed'] and manifest['state']=='complete'
    a=json.loads((RECORD/'keyframes_audits'/AUDIT).read_text())
    assert hashlib.sha256((RECORD/'keyframes_audits'/AUDIT).read_bytes()).hexdigest()==AUDIT[:-5]
    tool_camera=None; metadata=None
    with rosbag.Bag(str(RECORD/'keyframes.bag')) as bag:
        for topic,msg,t in bag.read_messages(topics=['/tf_static','/grasp_record/camera_runtime_metadata'],
                end_time=rospy.Time.from_sec(a['snapshot_stamp_sec'])):
            if topic=='/tf_static':
                for tr in msg.transforms:
                    if tr.header.frame_id=='tool0' and tr.child_frame_id=='camera_link':
                        tool_camera=fixture.base.quaternion_matrix([getattr(tr.transform.rotation,k)for k in 'xyzw'])
                        tool_camera[:3,3]=[getattr(tr.transform.translation,k)for k in 'xyz']
            else: metadata=json.loads(msg.data)
    assert tool_camera is not None and metadata is not None
    patch=MonkeyPatch(); node,params=fixture.configured.__wrapped__(patch)
    node._mode_selection=dict(a['grasp_mode_selection']); node._mode_ready=True
    node._observe_return_phase='observation_prefix'; node._observe_return_paused=False
    node._observe_return_diagnostic_observation_only=False
    node._observe_return_view_gaps=tuple(a['observe_return']['measured_missing_sides'])
    node._observe_return_view_uncertainty_m=.003
    node._observe_return_visited_camera_origins=()
    node._current_stream_target_identity=lambda:NS(epoch=a['target_epoch'],track_id=a['observe_return']['target_track_id'])
    node._require_stream_ticket_current=lambda *_:None
    node.near_field_planning_active=False
    node.grasp_config=observation_config({},node._mode_selection)
    node.gripper_geometry=fixture.base.GripperGeometry(.05,.002,[.0434,.0286,.0600],[.1175,.1550,.0774],.003)
    node.gripper_tool_jaw_axis='y'; node.gripper_tool_finger_length_axis='z'
    node.gripper_physical_open_width_m=.05; node.observation_envelope_gate_enabled=True
    node.observation_camera_roll_offsets_deg=fixture.base.DEFAULT_OBSERVATION_CAMERA_ROLL_OFFSETS_DEG
    node.observation_camera_tilt_offsets_deg=fixture.base.DEFAULT_OBSERVATION_CAMERA_TILT_OFFSETS_DEG
    node._tool_from_camera_matrix=lambda:tool_camera
    params['/camera']=metadata['parameters']
    counts=Counter()
    def param(name,default=None):counts[name]+=1; return params.get(name,default)
    patch.setattr(fixture.module.rospy,'get_param',param)
    g=a['replay_geometry']
    geometry=NS(center_base=np.array(g['obb_center_base_m']),axes_base=np.array(g['R_base_obb']),
        size_xyz_m=np.array(g['obb_size_xyz_m']),support_normal_base=np.array(g['support_normal_base']),
        support_offset_m=g['support_offset_m'])
    snapshot=NS(stamp_ns=a['snapshot_stamp_ns'],quality=NS(depth_repeatability_m=.001))
    prepared=NS(pose_estimator=NS(T_base_camera_link=np.array(a['snapshot_transform']['T_base_camera_link'])),
        stamp=rospy.Time(a['snapshot_stamp_ns']//10**9,a['snapshot_stamp_ns']%10**9),ticket=None)
    reference=node._frozen_observation_reference_pose(prepared)
    # Use one real adaptive profile from this source. Contact family construction
    # remains real; varying yaw later keeps this actual uncertainty/insertion.
    profile=a['remote_diagnostics']['tabletop_geometry']['adaptive_stage_profiles'][0]
    node._contact_stage_profile=lambda *a,**k:NS(**profile)
    cache={}; rows=[]; profiler=cProfile.Profile(); profiler.enable()
    for i in range(14):
        # Seven stable seeds with the existing two symmetric jaw variants.
        pose=fixture.base.make_pose_stamped('base_link',geometry.center_base,
            fixture.base.quaternion_from_euler(0.,0.,i*np.pi/7),stamp=prepared.stamp)
        before=Counter(counts); start=time.perf_counter()
        passing,rejected=node._far_field_observation_variants(pose,geometry,-geometry.support_normal_base,
            snapshot,reference,prepared,view_cache=cache)
        fingerprint=[]
        for v in passing:
            fingerprint.append(dict(preference_index=v['preference_index'],
                pose=fixture.base.pose_matrix(v['sequence'].pregrasp).tolist(),
                observation_view=v['observation_view'],gain=v.get('observe_return_missing_side_gain')))
        payload=json.dumps(dict(passing=fingerprint,rejected=rejected),sort_keys=True,default=str).encode()
        (args.output/('family_%02d.json'%i)).write_bytes(payload)
        rows.append(dict(family=i,elapsed_sec=time.perf_counter()-start,passing=len(passing),rejected=len(rejected),
            param_calls=dict(counts-before),candidate_sha256=hashlib.sha256(payload).hexdigest()))
    profiler.disable(); profiler.dump_stats(str(args.output/'profile.pstats'))
    with (args.output/'profile.txt').open('w') as f:pstats.Stats(profiler,stream=f).strip_dirs().sort_stats('cumtime').print_stats(45)
    result=dict(audit=str(RECORD/'keyframes_audits'/AUDIT),request_id=a['request_id'],
        purpose='Nearest preceding request5 endpoint workload; not exact request9 or37 geometry/timing replay',
        assumptions=['14 contact families (7 stable seeds x2 symmetry), source5 frozen adaptive profile',
            'ROS params are dictionary stubs, so RPC wall overhead is excluded','Stream ticket cancellation mocked; endpoint profiling gives no authority'],
        calls=dict(counts),families=rows,directed_views=len(node._observe_return_directed_views),
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in
            [ROOT/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
             ROOT/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',Path(__file__).resolve()]})
    (args.output/'sources').mkdir()
    for relative, expected in result['source_sha256'].items():
        payload=(ROOT/relative).read_bytes()
        assert hashlib.sha256(payload).hexdigest()==expected
        (args.output/'sources'/Path(relative).name).write_bytes(payload)
    (args.output/'camera_metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(output=str(args.output),families=len(rows),
        candidate_hashes=sorted(set(row['candidate_sha256']for row in rows)),
        calls=dict(counts),total_family_elapsed_sec=sum(row['elapsed_sec']for row in rows),
        source_sha256=result['source_sha256']),ensure_ascii=False));patch.undo()

if __name__=='__main__':main()
