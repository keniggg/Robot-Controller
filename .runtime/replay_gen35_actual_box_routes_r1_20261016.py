#!/usr/bin/env python3
"""One sealed source, existing v1/v2 qualification only. No live interfaces."""
import hashlib
import json
import os
from pathlib import Path
import socket
import sys
import time
import traceback
from types import SimpleNamespace

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
C = E / 'ros_log/full_latest_gen29_restart_20261016T000728Z'
N = E / 'ros_log/full_latest_gen22_restart_20261015T003700Z/verification'
R = E / 'grasp_records/20261016T121453.757769Z_87f1feb8ab9847419f85b9ab3d7237fb'
O = C / 'verification/gen35_actual_attempt1_box_routes_r1'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for v in iter(lambda:f.read(1048576),b''):h.update(v)
    return h.hexdigest()
def read(p):return json.loads(Path(p).read_text())

class ForbiddenBoundary(BaseException):pass
assert Path('/media/zhuyupei/SanStand').is_mount()
O.mkdir(exist_ok=False)
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1', ROS_LOG_DIR=str(O/'ros'),
    TMPDIR=str(O/'tmp'), PYTHONDONTWRITEBYTECODE='1', PYTHONNOUSERSITE='1')
(O/'tmp').mkdir()
calls=[]
def forbidden(*args,**kwargs):
    calls.append(repr(args)[:200]);raise ForbiddenBoundary('No live network/ROS/RPC')
socket.socket.connect=forbidden
socket.socket.connect_ex=forbidden
socket.socket.sendto=forbidden
socket.create_connection=forbidden
socket.getaddrinfo=forbidden
for p in ('src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/src','devel/lib/python3/dist-packages'):
    sys.path.insert(0,str(W/p))
sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')
report=dict(kind='gen35_one_recorded_source_existing_box_routes', passed=False,
    live_ros_contacted=False, production_modified=False, motion_commands_sent=0,
    physical_grasp_success=False, full_task_replay=False)
try:
    import numpy as np
    import cv2
    import rosbag
    import rospy
    for name in ('init_node','Publisher','Subscriber','Service','ServiceProxy','get_master',
                 'get_param','get_param_cached','set_param','delete_param','wait_for_service'):
        setattr(rospy,name,forbidden)
    from cv_bridge import CvBridge
    from alicia_flexible_grasp.vision import regular_box_inference as box
    from alicia_flexible_grasp.vision.object_geometry import GeometryEstimate
    from alicia_flexible_grasp.vision.target_observation import TargetTrackIdentity
    from alicia_flexible_grasp.grasp.gripper_geometry import (GripperGeometry,
        ANALYTICAL_FINGER_SIZE_XYZ_M, ANALYTICAL_PALM_SIZE_XYZ_M,
        evaluate_inferred_box_contact, _obb_line_interval)
    manifest=read(R/'manifest.json');assert manifest['state']=='complete' and manifest['recorder_closed']
    ap=R/'keyframes_audits/1a2bda4538b3516d34934347bace57621290bccc6b808aef779b2bd19d3d0919.json'
    cp=C/'gen35_grasp_scene_r2/camera_configuration.json'
    inputs={str(p):sha(p) for p in (R/'manifest.json',R/'keyframes.bag',ap,cp)}
    for row in manifest['files']:
        if row['path'] in ('keyframes.bag',str(ap.relative_to(R))):assert inputs[str(R/row['path'])]==row['sha256']
    a=read(ap);t=a['remote_diagnostics']['tabletop_geometry'];q=t['regular_box_inference'];en=t['regular_box_planning_enclosure'];g=a['replay_geometry'];s=a['snapshot_stamp_ns']
    camera=read(cp);K=SimpleNamespace(**{k:camera[k] for k in ('width','height','fx','fy','cx','cy')});scale=camera['depth_scale']
    arrays={};message_counts={}; infos=[];bridge=CvBridge()
    topics={'/supervisor/camera/color/image_raw':'color','/supervisor/camera/depth/image_raw':'depth','/perception/object_mask':'mask'}
    with rosbag.Bag(str(R/'keyframes.bag'),'r') as b:
        for topic,msg,received in b.read_messages(topics=list(topics)+['/supervisor/camera/color/camera_info']):
            if topic not in topics:
                infos.append(dict(stamp_ns=msg.header.stamp.to_nsec(),frame_id=msg.header.frame_id,
                    width=msg.width,height=msg.height,K=list(msg.K)));continue
            if msg.header.stamp.to_nsec()!=s:continue
            key=topics[topic];value=bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
            if key in arrays:assert np.array_equal(arrays[key],value),'conflicting duplicate source'
            arrays[key]=value;message_counts[key]=message_counts.get(key,0)+1
    assert set(arrays)=={'color','depth','mask'}
    rawhash={k:hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k,v in arrays.items()}
    assert all(rawhash[k]==q['retained_pixel_sha256'][k+'_sha256'] for k in arrays)
    assert infos and all(row['width']==K.width and row['height']==K.height and
        np.allclose(np.asarray(row['K'])[[0,4,2,5]], [K.fx,K.fy,K.cx,K.cy],rtol=0,atol=1e-12) for row in infos)
    points=np.asarray(g['object_points_base_m'],float)
    assert hashlib.sha256(np.ascontiguousarray(points).tobytes()).hexdigest()==g['object_points_sha256']
    geometry=GeometryEstimate(True,'','',np.asarray(en['original_center_base']),np.asarray(en['original_axes_base']),
        np.asarray(en['original_size_xyz_m']),np.asarray(g['support_normal_base']),g['support_offset_m'],
        g['support_inlier_ratio'],points,g['source_mode'])
    T=np.asarray(a['snapshot_transform']['T_base_optical'])
    identity=TargetTrackIdentity(**q['source_target_identity'])
    loaded={Path(m.__file__).resolve() for m in list(sys.modules.values()) if getattr(m,'__file__',None)
        and str(Path(m.__file__).resolve()).startswith(str(W/'src')) and str(m.__file__).endswith('.py')}
    release=read(N/'gen35_release_manifest.json')
    before={str(p.relative_to(W)):sha(p) for p in loaded}
    assert all(release['source_sha256'][p]==v for p,v in before.items())
    events=[]
    def trace(frame,event,arg):
        if event=='call' and frame.f_code.co_filename==box.__file__ and frame.f_code.co_name=='_qualify_regular_box_top':
            events.append(dict(function='_qualify_regular_box_top',outer_fallback=frame.f_locals.get('outer_fallback')))
        return None
    args=(arrays['depth'],arrays['mask'],K,scale,T,geometry,identity,s,geometry.source_mode)
    sys.setprofile(trace);wall=time.monotonic();cpu=time.process_time()
    normal=box.qualify_regular_box_top(*args,color_bgr=arrays['color'])
    normal_time=dict(wall_sec=time.monotonic()-wall,cpu_sec=time.process_time()-cpu)
    sys.setprofile(None)
    assert normal.ok and len(events)==1 and events[0]['outer_fallback'] is False
    normal_evidence=normal.model.to_evidence_dict()
    assert np.allclose(normal.model.size_xyz_m,q['inferred_size_xyz_m'],rtol=0,atol=1e-9)
    historical_narrow=next(x['targeted_view'] for x in t['surface_evidence_rejections'] if x['failure_code']=='BOX_INFERENCE_PRECISION_VIEW_REQUIRED')
    assert abs(normal.model.empirical_edge_uncertainty_m-historical_narrow['empirical_edge_uncertainty_m'])<1e-9
    events.clear();sys.setprofile(trace);wall=time.monotonic();cpu=time.process_time()
    outer=box._qualify_regular_box_top(*args,color_bgr=arrays['color'],outer_fallback=True)
    outer_time=dict(wall_sec=time.monotonic()-wall,cpu_sec=time.process_time()-cpu);sys.setprofile(None)
    outer_events=list(events)
    gripper=GripperGeometry(.05,.002,ANALYTICAL_FINGER_SIZE_XYZ_M,ANALYTICAL_PALM_SIZE_XYZ_M,.003)
    def summarize(result):
        data=dict(ok=result.ok,code=result.code,audit=dict(result.audit),model=result.model.to_evidence_dict() if result.model else None)
        if result.model:
            m=result.model;planning,enclosure=box.enclose_inferred_box_geometry(geometry,m)
            assert np.array_equal(planning.object_points_base,points)
            evidence=[]
            for i in (0,1):
                ev=evaluate_inferred_box_contact(m,m.center_base,m.axes_base[:,i],-m.support_normal_base,
                    gripper,support_normal_base=m.support_normal_base,opening_fit_clearance_each_side_m=.0005)
                interval=_obb_line_interval(m.center_base,m.axes_base[:,i],planning.center_base,planning.axes_base,planning.size_xyz_m)
                reach=[-interval[0]+.0005,interval[1]+.0005] if interval else None
                evidence.append(dict(axis=i,ok=ev.ok,code=ev.code,targeted_view=dict(ev.targeted_view),
                    union_interval_m=interval,union_reach_m=reach,early_reach_would_pass=bool(reach and max(reach)<=.025+1.001e-9)))
            data.update(enclosure=enclosure,side_evidence=evidence)
        return data
    normal_summary=summarize(normal);outer_summary=summarize(outer)
    after={p:sha(W/p) for p in before};assert before==after and not calls
    assert rawhash=={k:hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest() for k,v in arrays.items()}
    assert cv2.imwrite(str(O/'recorded_color.png'),arrays['color'])
    report.update(passed=True, inputs_sha256=inputs, script_sha256=sha(__file__),
        source_before=before,source_after=after,source_unchanged=True,source_stamp_ns=s,
        source_identity=q['source_target_identity'],raw_array_sha256=rawhash,exact_retained_source_arrays=True,
        duplicate_message_counts=message_counts,camera_info=infos,camera_parameters={k:camera[k] for k in ('width','height','fx','fy','cx','cy','depth_scale')},
        T_base_optical=T.tolist(),measured_point_count=len(points),measured_points_sha256=g['object_points_sha256'],
        measured_points_unchanged=True,geometry_restored_from='audit original OBB/support plus unchanged complete replay_geometry object points; not final planning enclosure',
        source_admission_or_registration_replayed=False, original_geometry=dict(center_base=geometry.center_base.tolist(),
            axes_base=geometry.axes_base.tolist(),size_xyz_m=geometry.size_xyz_m.tolist(),support_normal_base=geometry.support_normal_base.tolist(),
            support_offset_m=geometry.support_offset_m,support_inlier_ratio=geometry.support_inlier_ratio),
        actual_public_route=normal_summary,actual_public_call_trace=[dict(function='_qualify_regular_box_top',outer_fallback=False)],
        public_timing=normal_time,historical_model_sha256=q['model_sha256'],
        exact_historical_model_digest=normal.model.model_sha256==q['model_sha256'],
        direct_existing_outer_route=outer_summary,direct_outer_call_trace=outer_events,outer_timing=outer_time,
        actual_runtime_outer_called=False,actual_runtime_outer_not_called_reason='public wrapper preserves any qualified v1 and only retries two original failure codes',
        external_boundary_attempts=calls,collision_strict_path_checked=False,
        limitations=['Direct outer_fallback=True is a diagnostic invocation of an existing branch, not the deployed public routing behavior.',
            'Exact retained source pixels and all audit measured points are preserved, but historical temporal source admission/registration are not recomputed.',
            'Any recomputed model SHA is its own identity and does not replace the archived model SHA.',
            'No candidate materialization, complete collision sweep, IK, strict path, task state or motion is executed.'])
except BaseException as exc:
    sys.setprofile(None)
    report.update(error_type=type(exc).__name__,error=str(exc),traceback=traceback.format_exc(),external_boundary_attempts=calls)
finally:
    (O/'result.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(O),passed=report['passed'],sha256=sha(O/'result.json'),
        v1=report.get('actual_public_route',{}).get('code'),v2=report.get('direct_existing_outer_route',{}).get('code'),
        error=report.get('error')),ensure_ascii=False))
    if not report['passed']:sys.exit(1)
