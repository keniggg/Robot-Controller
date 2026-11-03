"""Passive same-stamp RGBD/mask/object capture; no publishes/services/commands."""
import hashlib,json,os
from copy import deepcopy
from pathlib import Path
import threading,time

from gen29_initial_helper_contract import (W,E,arguments,source_check,process_check,sha)
args,release,deployment,expected_processes,run_binding=arguments(__doc__)
O=args.output
helper_sha256={str(Path(__file__).resolve().relative_to(W)):sha(__file__),
 '.runtime/gen29_initial_helper_contract.py':sha(W/'.runtime/gen29_initial_helper_contract.py')}
source_check(args,release,helper_sha256)
assert not O.exists();O.mkdir();(O/'ros').mkdir();(O/'tmp').mkdir()
os.environ['ROS_LOG_DIR']=str(O/'ros');os.environ['TMPDIR']=str(O/'tmp')
import cv2,numpy as np,rospy,tf2_py
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,JointState
from std_msgs.msg import Header,Bool,String
from actionlib_msgs.msg import GoalStatusArray
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_matrix
from alicia_flexible_grasp_supervisor.msg import GraspState,ObjectPose
from alicia_flexible_grasp.robot.observation_view_gateway import _camera_configuration_contract

def canonical(v):return json.dumps(v,ensure_ascii=False,sort_keys=True,separators=(',',':'),allow_nan=False)
def digest(v):return hashlib.sha256(canonical(v).encode()).hexdigest()
def arrsha(v):return hashlib.sha256(np.ascontiguousarray(v).tobytes()).hexdigest()
def header(h):return dict(stamp_ns=h.stamp.to_nsec(),frame_id=h.frame_id,seq=h.seq)
def message(v):
    if hasattr(v,'secs') and hasattr(v,'nsecs'):return dict(stamp_ns=v.to_nsec())
    if hasattr(v,'__slots__'):return {k:message(getattr(v,k)) for k in v.__slots__}
    if isinstance(v,(tuple,list)):return [message(x) for x in v]
    return v
def save(name,value):
    p=O/name;assert not p.exists();p.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())

rospy.init_node('gen29_current_initial_inputs_readonly',anonymous=True,disable_rosout=True)
started_ns=rospy.Time.now().to_nsec();started=time.monotonic();lock=threading.RLock()
buf=tf2_py.BufferCore(rospy.Duration(30.));frames={};qrows=[];events=[];latest={};status_rows={};counts={};tf_errors=[]
names=['Joint%d'%i for i in range(1,7)]
faults=[];sdk_rows=[];sdk_reference=None;state_received={}
processes_before=None;processes_after=None
BUSY=(0,1,6,7)
topics=[('color','/supervisor/camera/color/image_raw',Image),('depth','/supervisor/camera/depth/image_raw',Image),
        ('mask','/perception/object_mask',Image),('object','/perception/object',ObjectPose),
        ('accepted','/alicia_d/accepted_joint_states',JointState),('epoch','/alicia_d/control_reference_epoch',Header),
        ('manual','/gui/joint_direct_mode',Bool),('task','/grasp/state',GraspState),
        ('mode','/grasp_mode/status',String),('arm','/alicia_controller/follow_joint_trajectory/status',GoalStatusArray),
        ('hand','/hand_controller/follow_joint_trajectory/status',GoalStatusArray),('sdk','/alicia_d/sdk_command',JointState),
        ('detector_status','/perception/unknown/detector_status',String),('tf','/tf',TFMessage),('tf_static','/tf_static',TFMessage)]

def receive(kind,msg):
    global sdk_reference
    received=rospy.Time.now().to_nsec();caller=(getattr(msg,'_connection_header',{}) or {}).get('callerid','')
    with lock:
        counts[kind]=counts.get(kind,0)+1
        state_received[kind]=time.monotonic()
        if kind in ('tf','tf_static'):
            for tr in msg.transforms:
                try:
                    if kind=='tf_static':buf.set_transform_static(tr,caller)
                    else:buf.set_transform(tr,caller)
                except Exception as exc:
                    if len(tf_errors)<6:tf_errors.append(str(exc))
        elif kind in ('color','depth','mask','object'):
            ns=msg.header.stamp.to_nsec()
            if ns>=started_ns:
                frames.setdefault(ns,{})[kind]=dict(msg=msg,received_ns=received,caller_id=caller)
                for old in sorted(frames)[:-90]:del frames[old]
        elif kind in ('mode','arm','hand','sdk'):
            try:
                if kind=='mode':
                    d=json.loads(msg.data)
                    assert caller=='/grasp_mode_router'
                    assert not any(d.get(k) for k in ('active','pending','start_reserved'))
                    assert d.get('state') not in ('error','switch_queued','switching','executing','execution_response_unknown')
                    assert d.get('mode')=='unknown' and d.get('strategy')=='observe_return'
                    assert d.get('generation')==args.expected_generation and int(d.get('stamp_ns',0))==args.expected_selection_stamp_ns
                    value=dict(value=d,received_ns=received,caller_id=caller)
                elif kind=='sdk':
                    assert caller=='/alicia_d_driver_node' and msg.header.frame_id=='sdk_transmitted'
                    assert len(msg.name)==len(msg.position)==len(set(msg.name)) and set(names)<=set(msg.name)
                    assert np.all(np.isfinite(msg.position))
                    row=dict(names=list(msg.name),positions=list(msg.position),stamp_ns=msg.header.stamp.to_nsec(),received_ns=received,caller_id=caller)
                    if sdk_reference is None:sdk_reference=(row['names'],row['positions'])
                    assert row['names']==sdk_reference[0] and max(abs(x-y) for x,y in zip(row['positions'],sdk_reference[1]))<=1e-12
                    sdk_rows.append(row)
                    return
                else:
                    value=dict(statuses=[int(x.status) for x in msg.status_list],received_ns=received,caller_id=caller)
                    assert not any(x in BUSY for x in value['statuses'])
                latest[kind]=value;events.append(dict(kind=kind,**value))
            except Exception as exc:faults.append(dict(kind=kind,received_ns=received,error=repr(exc)))
        elif kind=='accepted':
            # Accepted kinematics only. Joint effort and all diagnostics are intentionally unread.
            if (caller!='/alicia_d_driver_node' or len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name) or not np.all(np.isfinite(msg.position))):
                faults.append(dict(kind=kind,received_ns=received,error='invalid accepted source or vector'));return
            positions=dict(zip(msg.name,msg.position))
            if msg.header.frame_id!='sdk_measured' or not all(n in positions for n in names):
                faults.append(dict(kind=kind,received_ns=received,error='invalid accepted frame or joint names'));return
            if msg.header.frame_id=='sdk_measured' and all(n in positions for n in names):
                row=dict(stamp_ns=msg.header.stamp.to_nsec(),received_ns=received,frame_id=msg.header.frame_id,
                    joint_names=names,positions_rad=[positions[n] for n in names],caller_id=caller)
                qrows.append(row);latest['accepted']=row
                if not 0<=received-row['stamp_ns']<=200_000_000:
                    faults.append(dict(kind=kind,received_ns=received,error='accepted feedback not fresh'))
                if len(qrows)>1 and row['stamp_ns']<=qrows[-2]['stamp_ns']:
                    faults.append(dict(kind=kind,received_ns=received,error='accepted feedback not increasing'))
                if float(np.max(np.ptp(np.asarray([q['positions_rad'] for q in qrows]),axis=0)))>.0025:
                    faults.append(dict(kind=kind,received_ns=received,error='accepted joints moved during capture'))
        elif kind=='detector_status':
            try:
                d=json.loads(msg.data);ns=d.get('source_stamp_ns')
                if type(ns) is int and ns>=started_ns:status_rows[ns]=dict(received_ns=received,caller_id=caller,value=d)
            except (TypeError,ValueError):pass
        else:
            if kind=='manual':value=dict(value=bool(msg.data),received_ns=received,caller_id=caller)
            elif kind=='epoch':value=dict(epoch_ns=msg.stamp.to_nsec(),received_ns=received,caller_id=caller)
            else:value=dict(active=bool(msg.active),state=msg.state,stage=msg.stage,success=bool(msg.success),message=msg.message,
                header=header(msg.header),received_ns=received,caller_id=caller)
            if ((kind=='manual' and (not value['value'] or caller!='/alicia_supervisor_gui')) or
                    (kind=='epoch' and (value['epoch_ns']!=args.expected_epoch or caller!='/alicia_d_driver_node')) or
                    (kind=='task' and (value['active'] or caller!='/grasp_task_node'))):
                faults.append(dict(kind=kind,received_ns=received,error='non-idle or unexpected owner/epoch'))
            latest[kind]=value;events.append(dict(kind=kind,**value))

subs=[rospy.Subscriber(topic,typ,lambda m,k=kind:receive(k,m),queue_size=50 if kind in ('accepted','tf') else 8,buff_size=2**24)
      for kind,topic,typ in topics]

remote_keys=['planning_snapshot_frames','near_field_planning_snapshot_frames','planning_snapshot_timeout_sec',
 'planning_snapshot_max_inference_latency_sec','planning_snapshot_max_age_sec','planning_snapshot_max_span_sec',
 'planning_mask_min_iou','planning_mask_max_centroid_shift_px','planning_max_joint_delta_rad','mask_erosion_px',
 'mask_internal_hole_max_area_px','depth_mad_scale','depth_mad_absolute_floor_m','support_bbox_expand_ratio',
 'support_distance_threshold_m','target_cloud_support_plane_inlier_distance_m','target_cloud_voxel_size_m',
 'target_cloud_outlier_neighbors','target_cloud_outlier_std_ratio','geometry_min_support_points','geometry_min_object_points',
 'target_cloud_min_points','geometry_min_size_m','geometry_max_size_m','geometry_max_height_m','gripper_geometry',
 'adaptive_stage_generation','tabletop_geometry','multiview']
parameter_names=['/grasp_6d/remote','/grasp','/gripper','/handeye','/camera','/perception/depth_min_m','/perception/depth_max_m','/unknown_perception',
 '/gui/joint_direct_mode','/grasp_mode/selection','/grasp/unknown_require_measured_bilateral_contact']+[
 '/grasp_6d/remote/'+x for x in remote_keys]
def parameters():
    begin=rospy.Time.now().to_nsec();values={p:rospy.get_param(p,None) for p in parameter_names}
    stable=deepcopy(values);stable['/camera']=_camera_configuration_contract(stable['/camera'])
    return dict(read_start_ns=begin,read_end_ns=rospy.Time.now().to_nsec(),parameters=values,
                parameters_sha256=digest(values),stable_configuration_sha256=digest(stable),
                stable_configuration=stable,configuration_contract='alicia_flexible_grasp.robot.observation_view_gateway._camera_configuration_contract',
                missing_parameters=[p for p,v in values.items() if v is None])
def state_check(rows):
    assert not faults,'Latched state violation: '+repr(faults[:2])
    if not all(sub.get_num_connections()>0 for sub in subs):return None
    if not all(k in latest for k in ('mode','arm','hand')):return None
    if any(time.monotonic()-state_received[k]>=2.5 for k in ('mode','arm','hand')):return None
    if len(rows)<20:return None
    recent=rows[-20:]
    positions=np.asarray([r['positions_rad'] for r in recent],float)
    now=rospy.Time.now().to_nsec()
    if not (latest.get('manual',{}).get('value') is True and latest.get('task',{}).get('active') is False and 'epoch' in latest):return None
    if latest['manual']['caller_id']!='/alicia_supervisor_gui':return None
    if not 0<=now-recent[-1]['stamp_ns']<=200_000_000:return None
    if recent[-1]['stamp_ns']-recent[0]['stamp_ns']<1_500_000_000:return None
    span=float(np.max(np.ptp(positions,axis=0)))
    if not np.all(np.isfinite(positions)) or span>.0025:return None
    return dict(checked_ns=now,manual=dict(latest['manual']),task=dict(latest['task']),epoch=dict(latest['epoch']),
        accepted_count=len(recent),accepted_first=recent[0],accepted_last=recent[-1],maximum_joint_span_rad=span)

before=None;after=None;params_before=None;params_after=None;chosen=[];failure=None;tf_by_source={};q_by_source={}
try:
    processes_before=process_check(rospy,expected_processes)
    params_before=parameters()
    assert params_before['parameters']['/gui/joint_direct_mode'] is True,'manual parameter is not true'
    selected=params_before['parameters']['/grasp_mode/selection']
    assert selected['mode']=='unknown' and selected['strategy']=='observe_return' and int(selected['generation'])==args.expected_generation and int(selected['stamp_ns'])==args.expected_selection_stamp_ns,'expected generation/selection stamp differs'
    cam=params_before['parameters']['/camera']
    assert isinstance(cam,dict) and all(k in cam for k in ('fx','fy','cx','cy','depth_scale','width','height')),'camera calibration fields missing'
    assert all(np.isfinite(float(cam[k])) for k in ('fx','fy','cx','cy','depth_scale')) and min(float(cam[k]) for k in ('fx','fy','depth_scale'))>0,'camera calibration invalid'
    deadline=time.monotonic()+22.
    while time.monotonic()<deadline and not rospy.is_shutdown():
        with lock:
            if before is None:before=state_check(qrows)
            if before is not None:
                assert before['epoch']['epoch_ns']==args.expected_epoch,'unexpected current control epoch'
                assert latest.get('manual',{}).get('value') is True,'manual changed during capture'
                assert latest.get('task',{}).get('active') is False,'task became active during capture'
                assert latest.get('epoch',{}).get('epoch_ns')==before['epoch']['epoch_ns'],'control epoch changed'
                if len(chosen)<3:
                    for ns,packet in sorted(frames.items()):
                        if ns<=before['checked_ns'] or ns in chosen or set(packet)!=set(('color','depth','mask','object')):continue
                        if not all(packet[k]['msg'].header.stamp.to_nsec()==ns for k in packet):continue
                        if not packet['object']['msg'].detected:continue
                        status=status_rows.get(ns,{}).get('value',{})
                        if not (status.get('generation')==args.expected_generation and int(status.get('selection_stamp_ns',0))==args.expected_selection_stamp_ns and status.get('state')=='ready' and status.get('target_lost') is False and status.get('target_locked') is True):continue
                        pre=[x for x in qrows if x['stamp_ns']<=ns];post=[x for x in qrows if x['stamp_ns']>=ns]
                        if not pre or not post:continue
                        qb=max(pre,key=lambda x:x['stamp_ns']);qa=min(post,key=lambda x:x['stamp_ns'])
                        if qa['stamp_ns']-qb['stamp_ns']>200_000_000:continue
                        frame=packet['depth']['msg'].header.frame_id
                        try:tr=buf.lookup_transform_core('base_link',frame,rospy.Time(*divmod(ns,10**9)))
                        except Exception:continue
                        tf_by_source[ns]=tr;q_by_source[ns]=dict(before=qb,after=qa,span_ns=qa['stamp_ns']-qb['stamp_ns'])
                        chosen.append(ns)
                        if len(chosen)==3:break
                if len(chosen)==3:
                    end_rows=[x for x in qrows if x['stamp_ns']>chosen[-1]]
                    after=state_check(end_rows)
                    if after is not None:break
        time.sleep(.02)
    params_after=parameters()
    processes_after=process_check(rospy,expected_processes)
    source_check(args,release,helper_sha256)
    assert before is not None,'no manual/inactive/stable accepted-joint pre-window'
    assert len(chosen)==3,'three strictly synchronized complete source packets unavailable'
    assert after is not None,'no stable accepted-joint post-window'
    assert before['epoch']['epoch_ns']==after['epoch']['epoch_ns'],'epoch changed'
    assert params_before['stable_configuration_sha256']==params_after['stable_configuration_sha256'],'fixed or unknown configuration parameters changed'
    with lock:
        window=[q for q in qrows if before['accepted_first']['stamp_ns']<=q['stamp_ns']<=after['accepted_last']['stamp_ns']]
        span=float(np.max(np.ptp(np.asarray([q['positions_rad'] for q in window]),axis=0)))
        assert span<=.0025,'accepted joints moved across capture window'
        assert not any(e['kind']=='manual' and not e['value'] or e['kind']=='task' and e['active'] for e in events if e['received_ns']>=before['checked_ns']),'non-idle event observed'
except Exception as exc:failure=dict(type=type(exc).__name__,message=str(exc))
finally:
    for sub in subs:sub.unregister()
    with lock:
        if faults:failure=dict(type='LatchedStateViolation',message=repr(faults[:2]))

bridge=CvBridge();rows=[]
camera=(params_before or {}).get('parameters',{}).get('/camera') or {}
for ns in chosen:
    packet=frames[ns];color=bridge.imgmsg_to_cv2(packet['color']['msg'],'bgr8')
    depth=bridge.imgmsg_to_cv2(packet['depth']['msg'],'passthrough');mask=bridge.imgmsg_to_cv2(packet['mask']['msg'],'mono8')
    obj=packet['object']['msg'];tr=tf_by_source[ns];r=tr.transform.rotation;t=tr.transform.translation
    T=quaternion_matrix([r.x,r.y,r.z,r.w]);T[:3,3]=[t.x,t.y,t.z]
    optical=np.eye(4);optical[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]]
    frame=packet['depth']['msg'].header.frame_id;converted='optical' not in frame.lower();Topt=T@optical if converted else T
    K=np.asarray([camera.get(k,float('nan')) for k in ('fx','fy','cx','cy')],float);scale=float(camera.get('depth_scale',float('nan')))
    arrays=dict(color=color,depth_raw=depth,object_mask=mask,T_base_camera=T,T_base_optical=Topt,intrinsics=K,
        depth_scale_m=np.array(scale),source_stamp_ns=np.array(ns),bbox_xywh=np.array([obj.bbox_x,obj.bbox_y,obj.bbox_width,obj.bbox_height]))
    p=O/('source_%d.npz'%ns);np.savez_compressed(str(p),**arrays)
    row=dict(source_stamp_ns=ns,npz=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest(),array_sha256={k:arrsha(v) for k,v in arrays.items()},
        topics={k:dict(header=header(v['msg'].header),received_ns=v['received_ns'],caller_id=v['caller_id']) for k,v in packet.items()},
        object_pose_message=message(obj),bbox_convention='x_y_width_height',object_detected=bool(obj.detected),mask_nonzero_pixels=int(np.count_nonzero(mask)),
        accepted_joint_bracket=q_by_source[ns],tf=dict(requested_source_stamp_ns=ns,returned_header=header(tr.header),child_frame_id=tr.child_frame_id,
            transform=message(tr.transform),optical_conversion_applied=converted,source_from_optical=optical.tolist() if converted else np.eye(4).tolist()),
        camera_parameters_sha256=digest(camera),recorded_detector_status=status_rows.get(ns),
        diagnostic_identity_created=False,production_track_identity_claimed=False,
        image_shapes_match=color.shape[:2]==depth.shape==mask.shape,
        color_depth_frame_ids_match=packet['color']['msg'].header.frame_id==frame,
        mask_depth_frame_ids_match=packet['mask']['msg'].header.frame_id==frame)
    assert cv2.imwrite(str(O/('source_%d_color.png'%ns)),color)
    assert cv2.imwrite(str(O/('source_%d_mask.png'%ns)),mask)
    rows.append(row)
params_receipt=save('parameters.json',dict(before=params_before,after=params_after,
    source_config_path='src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml',
    source_config_sha256=hashlib.sha256(Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml').read_bytes()).hexdigest(),
    missing_values_not_silently_defaulted=True))
q_receipt=save('accepted_joint_states.json',dict(rows=qrows,manual_task_epoch_events=events,sdk_rows=sdk_rows,latched_faults=faults,joint_effort_read=False))
complete=failure is None and len(rows)==3
usable=complete and all(r['object_detected'] and r['mask_nonzero_pixels']>0 and r['image_shapes_match'] and r['color_depth_frame_ids_match'] and r['mask_depth_frame_ids_match'] for r in rows)
result=dict(run_binding=run_binding,helper_sha256=helper_sha256,release_source_count=len(release['source_sha256']),
    processes_before=processes_before,processes_after=processes_after,latched_faults=faults,observed_sdk_command_count=len(sdk_rows),sdk_target_unchanged=True if not faults else None,
    capture_complete=complete,usable_detected_target_triples=usable,failure=failure,started_ns=started_ns,finished_ns=rospy.Time.now().to_nsec(),elapsed_sec=time.monotonic()-started,
    before=before,after=after,frames=rows,received_message_counts=counts,tf_callback_errors=tf_errors,
    incomplete_source_summary=[dict(source_stamp_ns=ns,available_topics=sorted(packet),detected=bool(packet['object']['msg'].detected) if 'object'in packet else None) for ns,packet in sorted(frames.items())[-12:]],
    parameters_receipt=params_receipt,accepted_receipt=q_receipt,topics=[t for _,t,_ in topics],script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    camera_configuration_contract_sha256=hashlib.sha256(Path('src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/observation_view_gateway.py').read_bytes()).hexdigest(),
    readonly_subscriptions_and_parameter_reads_only=True,publishers_created=False,services_called=False,parameters_written=False,
    motion_commands_sent=False,enable_requested=False,temperature_queried=False,execution_authority=False,
    purpose='Current initial-view offline geometry/planning inputs only; not a grasp launch or production target-track authority.',
    production_track_identity_claimed=False)
source_check(args,release,helper_sha256)
p=save('result.json',result)
print(json.dumps(dict(**p,capture_complete=complete,usable_detected_target_triples=usable,failure=failure,source_stamps_ns=chosen,message_counts=counts),ensure_ascii=False),flush=True)
