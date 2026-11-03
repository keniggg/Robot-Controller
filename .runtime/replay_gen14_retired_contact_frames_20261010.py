"""Offline actual RGB-D/TF replay after exact station retirement; no ROS graph."""
from pathlib import Path
from copy import deepcopy
from types import SimpleNamespace as NS
from dataclasses import asdict
from collections import deque
import hashlib
import json
import sys
import time

import numpy as np
from _pytest.monkeypatch import MonkeyPatch
from sensor_msgs.msg import JointState
from std_msgs.msg import String

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK/'src/alicia_grasp_modes/tests'))
from test_unknown_stream import rgb_interest_node, module
from test_locked_rgb_hold import latest_status
from alicia_grasp_modes.contact_station_retirement import ack
from alicia_grasp_modes.observe_return_measurement import measurement_digest
from alicia_grasp_modes.rgb_instance import freeze_observation_spatial_reference


def main():
    mount = Path('/media/zhuyupei/SanStand')
    assert mount.is_mount()
    output = mount/'实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen14_analysis'
    inputs = output/'replay_inputs'
    camera = json.loads((inputs/'camera_configuration.json').read_text())
    measurement = json.loads((inputs/'qualified_measurement_request.json').read_text())
    ready = json.loads((inputs/'last_ready_status.json').read_text())
    initial_status = json.loads((inputs/'initial_acquisition_status.json').read_text())
    feedback = json.loads((inputs/'accepted_feedback_window.json').read_text())
    frames = {name: dict(np.load(inputs/(name+'.npz'))) for name in
              ('initial_anchor','last_good','failed_frame','after_200ms','after_400ms','after_800ms')}
    additional = '--additional' in sys.argv
    continuous = '--continuous' in sys.argv
    if additional:
        feedback += json.loads((inputs/'accepted_feedback_additional_windows.json').read_text())
        feedback = sorted({row['source_stamp_ns']:row for row in feedback}.values(), key=lambda row:row['source_stamp_ns'])
        for name in ('motion_plus_2s','motion_plus_10s','motion_plus_30s','motion_plus_50s',
                     'pregrasp_before_transition','pregrasp_after_transition'):
            frames[name] = dict(np.load(inputs/(name+'.npz')))
    replay_entries = None
    if continuous:
        continuous_manifest = json.loads((inputs/'continuous/manifest.json').read_text())
        replay_entries = [row for row in continuous_manifest['frames']
                          if row['source_stamp_ns'] > int(frames['last_good']['source_stamp_ns'])]
        for row in replay_entries:
            for key in ('feedback_before', 'feedback_after'):
                if row.get(key):
                    ns, q = row[key]
                    feedback.append(dict(source_stamp_ns=ns, q=q))
        feedback = sorted({row['source_stamp_ns']:row for row in feedback}.values(), key=lambda row:row['source_stamp_ns'])
    patch = MonkeyPatch()
    real_monotonic = time.monotonic
    try:
        node, clock, _, _, params = rgb_interest_node(patch)
        params['/camera'] = camera
        params['/grasp_mode/selection'] = deepcopy(measurement['selection'])
        node.generation, node.selection_ns = measurement['selection']['generation'], measurement['selection']['stamp_ns']
        node._rgb_interest_epoch = measurement['control_epoch_ns']
        node._rgb_interest_feedback.clear()
        node.last_shape = frames['initial_anchor']['depth_raw'].shape
        now = [int(frames['last_good']['source_stamp_ns'])+100_000_000]
        process_clock = [False, real_monotonic()]
        def elapsed_ns():
            return int((real_monotonic()-process_clock[1])*1e9) if continuous and process_clock[0] else 0
        def ros_now():
            value=now[0]+elapsed_ns()
            return module.rospy.Time(value//10**9,value%10**9)
        patch.setattr(module.rospy.Time, 'now', staticmethod(ros_now))
        if continuous:
            patch.setattr(module.time,'monotonic',lambda:clock[0]+elapsed_ns()*1e-9)
        # The dynamic image transform is sealed exact bag TF. Static context
        # provenance is fixture-owned, not claimed as a hardware attestation.
        current = [frames['last_good']]
        patch.setattr(module, 'transform_matrix', lambda args:
            current[0]['T_base_camera'].copy() if args[0]=='base_link' else np.eye(4))
        def arrays(frame):
            return frame['depth_raw'].astype(np.float32)*float(frame['depth_scale']), frame['intrinsics'], frame['T_base_camera']
        initial = frames['initial_anchor']
        r = module.segment_with_support_fallback(*arrays(initial), node.config)
        anchor, _ = node.tracker.choose(r, node.last_shape)
        anchor_error = float(np.max(abs(anchor.position_base-np.array(initial_status['anchor_position_base']))))
        plane_error = float(np.max(abs(node.tracker.plane_base-np.array(initial_status['reference_support_plane_base']))))
        assert anchor_error < 1e-12 and plane_error < 1e-12
        node.anchor_source_stamp_ns = int(initial['source_stamp_ns'])
        last = frames['last_good']
        r = module.segment_with_support_fallback(*arrays(last), node.config, anchor=node.tracker.anchor,
            last=node.tracker.last, reference_plane_base=node.tracker.plane_base, prefer_current_support=True)
        raw, _ = node.tracker.choose(r, node.last_shape)
        refined = module.refine_candidate(last['color'], *arrays(last), raw, node.config,
            observation_arrival_retry=True,
            observation_spatial_reference=freeze_observation_spatial_reference(anchor,node.tracker.plane_base),
            **module._rgb_source_kwargs(camera,node.last_shape))
        last_error = float(np.max(abs(refined.position_base-np.array(ready['position_base']))))
        assert last_error < 1e-12
        def pair(frame):
            msgs=(node.bridge.cv2_to_imgmsg(frame['color'],encoding='bgr8'),
                  node.bridge.cv2_to_imgmsg(frame['depth_raw'],encoding='16UC1'))
            ns=int(frame['source_stamp_ns'])
            for msg in msgs:
                msg.header.stamp=module.rospy.Time(ns//10**9,ns%10**9)
                msg.header.frame_id=node.camera_frame
            return msgs
        consumed = set()
        def deliver_feedback(cutoff):
            for row in feedback:
                ns=row['source_stamp_ns']
                if ns <= cutoff and ns not in consumed:
                    msg=JointState();msg.header.frame_id='sdk_measured'
                    msg.header.stamp=module.rospy.Time(ns//10**9,ns%10**9)
                    msg.name=list(module.JOINT_NAMES);msg.position=row['q']
                    node._rgb_interest_joint_cb(msg);consumed.add(ns)
        deliver_feedback(now[0])
        reference=node._prepare_locked_rgb_hold_reference(pair(last),refined,camera)
        assert reference is not None
        node.last_good_source_stamp_ns=int(last['source_stamp_ns'])
        node._locked_rgb_hold_reference=reference
        marker=dict(measurement_binding={k:measurement[k] for k in (
            'selection','lease_id','session_id','view_plan_id','view_index','target_epoch','target_track_id','control_epoch_ns')},
            lease_id=measurement['lease_id'],qualified_source_ns=measurement['source_stamps_ns'][-1],
            arrival_q=np.asarray(measurement['settle']['accepted_positions_rad']),epoch_ns=measurement['control_epoch_ns'],
            source_id=reference['source_id'],config_sha256=reference['config_sha256'],
            camera_configuration_sha256=module.digest(module._camera_configuration_contract(camera)),
            spatial_reference=freeze_observation_spatial_reference(anchor,node.tracker.plane_base),
            anchor_fingerprint=reference['anchor_fingerprint'],reference_plane=node.tracker.plane_base.copy(),
            session_deadline_ns=measurement['deadline_ns'],session_deadline_monotonic=clock[0]+600)
        reference['observation_arrival_retry']=marker
        node._observation_measurement_status=dict(state='completed',request=deepcopy(measurement))
        phases=json.loads((inputs/'near_field_phases.json').read_text())
        p=[p for p in phases if p['active'] and p['planning_stage']=='contact_replan'][-1]
        def stamp(value):return module.rospy.Time(value['secs'],value['nsecs'])
        node._station_compute_phase=NS(active=True,planning_stage='contact_replan',phase_id=p['phase_id'],
            header=NS(stamp=stamp(p['header']['stamp'])),deadline=stamp(p['deadline']),
            reference_target_track_id=p['reference_target_track_id'])
        node._station_compute_task_state=NS(state='MOVE_PREGRASP',active=True)
        from unittest.mock import Mock
        node._contact_station_ack_pub=Mock()
        request=dict(version=1,state='prepare',request_id='recorded-replay-contact',selection=measurement['selection'],
            session_id=measurement['session_id'],target_track_id=measurement['target_track_id'],target_epoch=measurement['target_epoch'],
            control_epoch_ns=measurement['control_epoch_ns'],measurement_sha256=measurement_digest(measurement),
            plan_id='212cd5c13c08337a1a24a79b',plan_sha256='b'*64,plan_source_ns=1791612166323238849,
            phase_id=p['phase_id'],phase_started_ns=stamp(p['header']['stamp']).to_nsec(),stage='MOVE_PREGRASP',
            stamp_ns=now[0],deadline_ns=now[0]+800_000_000,metric_geometry_authority=False,motion_authorized=False,
            controller_motion_confirmed=False)
        node._contact_station_request_cb(String(data=json.dumps(request)))
        response=json.loads(node._contact_station_ack_pub.publish.call_args[0][0].data)
        assert response['accepted'],response
        retire=dict(request,state='retire',prepared_acks={p:ack(request,p) for p in ('unknown','remote')})
        node._contact_station_request_cb(String(data=json.dumps(retire)))
        response=json.loads(node._contact_station_ack_pub.publish.call_args[0][0].data)
        assert response['accepted'],response
        segment_evidence = [None]
        original_segment = module.segment_with_support_fallback
        def segment(*args, **kwargs):
            value=original_segment(*args, **kwargs)
            matches=module._matching_candidates(value.candidates,node.tracker.anchor,node.tracker.last,node.config)
            segment_evidence[0]=dict(candidate_count=len(value.candidates),metric_matches=len(matches),
                candidates=[dict(position_base=c.position_base.tolist(),quality=c.quality,
                    bbox_xyxy=[int(np.where(c.mask)[1].min()),int(np.where(c.mask)[0].min()),
                               int(np.where(c.mask)[1].max())+1,int(np.where(c.mask)[0].max())+1],
                    pixels=int(np.count_nonzero(c.mask))) for c in value.candidates],
                support_metrics=value.metrics)
            return value
        patch.setattr(module,'segment_with_support_fallback',segment)
        ambiguity_probe=[]
        if '--probe-fragments' in sys.argv:
            original_transition=node._metric_ambiguity_transition
            def probe(source,result,camera,depth,intrinsics,transform,**kwargs):
                from alicia_grasp_modes.rgb_observation_handoff import _appearance
                from alicia_grasp_modes import rgb_instance
                reference=node._locked_rgb_hold_reference
                histogram,contour=_appearance(reference['bgr'],reference['mask'])
                spatial=node._contact_spatial_context['spatial_reference']
                # Offline probe explicitly uses contact's original two-attempt
                # policy, not the arrival-mode _FrameRefinement.refine shortcut.
                original_refine=rgb_instance._FrameRefinement.refine
                def fresh_contact(self,rgb,candidate,*,rgb_valid=None):
                    return module.refine_candidate(rgb,depth,intrinsics,transform,candidate,node.config,
                        rgb_valid=rgb_valid,contact_spatial_reference=spatial)
                rgb_instance._FrameRefinement.refine=fresh_contact
                try:
                    recovered,pair=module.recover_arrival_matching_fragments(result,source[0] and current[0]['color'],
                        depth,intrinsics,transform,node.config,node.tracker.anchor,node.tracker.last,spatial,
                        histogram,contour,node.minimum_quality,**module._rgb_source_kwargs(camera,depth.shape))
                    ambiguity_probe.append(dict(source_stamp_ns=source[0].header.stamp.to_nsec(),accepted=pair is not None,
                        remaining_candidates=len(recovered.candidates),metrics=None if pair is None else pair[1].metrics))
                except ValueError as exc:
                    ambiguity_probe.append(dict(source_stamp_ns=source[0].header.stamp.to_nsec(),accepted=False,reason=str(exc)))
                finally:rgb_instance._FrameRefinement.refine=original_refine
                return original_transition(source,result,camera,depth,intrinsics,transform,**kwargs)
            node._metric_ambiguity_transition=probe
        rows=[]
        initial_clock=clock[0]
        if replay_entries is None:
            replay_entries=[dict(label=k,source_stamp_ns=int(frames[k]['source_stamp_ns'])) for k in frames
                            if k not in ('initial_anchor','last_good')]
        for entry in sorted(replay_entries,key=lambda row:row['source_stamp_ns']):
            name=entry.get('label',str(entry['source_stamp_ns']))
            frame=dict(np.load(entry['path'])) if continuous else frames[name]
            current[0]=frame;ns=int(frame['source_stamp_ns'])
            now[0]=ns+100_000_000
            clock[0]=(initial_clock+(ns-int(frames['last_good']['source_stamp_ns']))*1e-9
                      if continuous else clock[0]+.2)
            deliver_feedback(now[0])
            node.latest=pair(frame)
            before_ref=node._locked_rgb_hold_reference
            before=dict(reference_source_ns=None if before_ref is None else before_ref['source_stamp_ns'],
                        reference_pose_invalidated=None if before_ref is None else before_ref['pose_invalidated'])
            start=real_monotonic();process_clock[:]=[True,start]
            node.process(None);elapsed=real_monotonic()-start;process_clock[0]=False
            status=latest_status(node)
            rows.append(dict(label=name,source_stamp_ns=ns,state=status['state'],detail=status['detail'],
                target_lost=status['target_lost'],published_source_ns=node.pubs['object'].publish.call_args[0][0].header.stamp.to_nsec(),
                detected=node.pubs['object'].publish.call_args[0][0].detected,processing_wall_sec=elapsed,
                frame_timing=status.get('frame_timing'),geometry_metrics=status.get('geometry_metrics'),
                before=before,first_loss=status.get('first_loss'),segmentation=segment_evidence[0],
                station_marker_present='observation_arrival_retry' in (node._locked_rgb_hold_reference or {})))
            if '--first-failure' in sys.argv and status['state']!='ready':
                break
        report=dict(scope='No ROS graph. Actual sealed RGB-D, exact dynamic TF and accepted feedback; real producer retirement and process callbacks.',
            inputs_manifest_sha256=hashlib.sha256((inputs/'manifest.json').read_bytes()).hexdigest(),
            source_config=asdict(node.config),anchor_position_max_error_m=anchor_error,
            initial_plane_max_error=plane_error,last_refined_position_max_error_m=last_error,frames=rows,
            all_replayed_fresh_frames_ready=all(row['state']=='ready' and row['detected'] and not row['target_lost']
                and row['source_stamp_ns']==row['published_source_ns'] and not row['station_marker_present'] for row in rows),
            limitations=['Config uses source defaults with authorized 7 mm hard / 4 mm warning; complete recorded unknown_perception parameter dictionary was not available. Exact initial anchor/plane and last refined center reproduce within 1e-12.',
                'Clock is replayed at source + 100 ms; measured offline wall runtime is reported separately, not a real-time hardware qualification.',
                'Static context/model uses inert fixture transport; exact dynamic camera transform, pixels and accepted joint feedback come from the sealed recording.',
                'Remote preparation acknowledgements and plan digest are inert test inputs; full remote plan/digest and task pre-submit boundaries are covered separately by callback regression tests.',
                'Sparse added frames have deliberate replay gaps, not virtual fresh observations. No claim of continuous real-time tracking through omitted frames or of physical grasp success.'])
        if continuous:
            report['continuous_inputs_manifest_sha256']=hashlib.sha256((inputs/'continuous/manifest.json').read_bytes()).hexdigest()
            stamps=[int(frames['last_good']['source_stamp_ns'])]+[r['source_stamp_ns'] for r in rows]
            report['maximum_replay_source_gap_sec']=max(b-a for a,b in zip(stamps,stamps[1:]))*1e-9
            report['limitations'][1]='Source age starts at 100 ms and advances by actual processing wall time; all original ROS/monotonic processing-age gates remain active. Other nodes/hardware do not run.'
            report['limitations'][-1]='Sequential sealed image sampling through the recorded contact segment. Source gaps are reported; omitted camera frames and callback scheduling are not reconstructed. This is not physical grasp success.'
        report['offline_fragment_probe']=ambiguity_probe
        delivery=json.loads((WORK/'.runtime/gen14_contact_station_retirement_20261010/delivery_v2.json').read_text())
        report['production_source_sha256']={row['path']:hashlib.sha256((WORK/row['path']).read_bytes()).hexdigest() for row in delivery['files']}
        assert all(report['production_source_sha256'][row['path']]==row['sha256'] for row in delivery['files'])
        report['source_sha256']={str(Path(m.__file__).resolve()):hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest()
            for name,m in tuple(sys.modules.items()) if getattr(m,'__file__',None) and (name.startswith('alicia_grasp_modes.contact_station_')
            or name in ('alicia_grasp_modes.rgb_instance',module.__name__))}
        path=output/('contact_station_retirement/recorded_continuous_spatial_roi_replay.json' if continuous else
                    'contact_station_retirement/recorded_additional_spatial_roi_replay.json' if additional else
                     'contact_station_retirement/recorded_spatial_roi_replay.json')
        if '--sealed' in sys.argv:
            path=path.with_name('sealed_continuous_spatial_roi_replay.json')
        if '--first-failure' in sys.argv:
            path=path.with_name('recorded_spatial_roi_fragment_probe.json' if '--probe-fragments' in sys.argv else 'recorded_spatial_roi_first_ambiguity.json')
        path.write_text(json.dumps(report,ensure_ascii=False,indent=2,default=lambda v:v.tolist() if hasattr(v,'tolist') else str(v))+'\n')
        print(json.dumps(dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            all_replayed_fresh_frames_ready=report['all_replayed_fresh_frames_ready'],
            frame_count=len(rows), ready_count=sum(row['state']=='ready' for row in rows),
            first_not_ready=next(({k:v for k,v in row.items() if k in ('label','state','detail','processing_wall_sec','before')}
                                  for row in rows if row['state']!='ready'),None)),ensure_ascii=False))
    finally:
        patch.undo()


if __name__=='__main__':main()
