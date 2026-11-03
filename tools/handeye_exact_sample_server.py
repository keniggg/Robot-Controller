#!/usr/bin/env python3
"""GUI-compatible hand-eye sampling with same-frame evidence on SanStand.

This server publishes no robot commands or calibration transforms. Existing GUI
services can be aliases of a fresh session; raw captures are never removed.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import threading
import time
from collections import OrderedDict, Counter
from datetime import datetime, timezone
from copy import deepcopy

import numpy as np
import rospy
import rosbag
import rosgraph
import tf2_ros
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Bool, Header
from tf2_msgs.msg import TFMessage
from easy_handeye_msgs import srv as services
from std_srvs.srv import Empty, EmptyResponse
from easy_handeye.handeye_server import HandeyeServer
from easy_handeye.handeye_calibration import HandeyeCalibration
from alicia_flexible_grasp_supervisor.msg import GraspState

ROOT = Path(__file__).resolve().parents[1]
TRACKER = ROOT/'src/real-arm/alicia_d_calibration/scripts/charuco_tracker.py'
SPEC = importlib.util.spec_from_file_location('exact_capture_tracker_model', TRACKER)
MODEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODEL)
MOUNT = Path('/media/zhuyupei/SanStand')
ARCHIVE_ROOT = MOUNT/'实验记录/Robot-Controller'
TOPICS = dict(rgb='/supervisor/camera/color/image_raw', depth='/supervisor/camera/depth/image_raw',
              quality='/charuco/quality', joints='/alicia_d/accepted_joint_states')


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def write_json(path, value):
    payload = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n'
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_text(payload)
    os.replace(str(temporary), str(path))


def require_archive(path):
    resolved = Path(path).resolve()
    if not os.path.ismount(str(MOUNT)) or ARCHIVE_ROOT not in resolved.parents:
        raise ValueError('SanStand 未挂载或目录不属于实验外盘；停止采样')
    if not resolved.is_dir():
        raise ValueError('本次外盘会话目录不存在')
    return resolved


def transform_matrix(msg):
    q, p = msg.transform.rotation, msg.transform.translation
    result = quaternion_matrix([q.x,q.y,q.z,q.w])
    result[:3,3] = [p.x,p.y,p.z]
    if not np.isfinite(result).all():
        raise ValueError('非有限 TF')
    return result


def rotation_deg(a,b):
    return float(np.degrees(np.arccos(np.clip((np.trace(a[:3,:3].T @ b[:3,:3])-1)/2,-1,1))))


def model_binding(parameters, camera_frame, image_topic):
    evidence = MODEL.runtime_color_model_evidence(parameters,camera_frame,image_topic)
    fingerprint = hashlib.sha256(json.dumps(evidence,sort_keys=True,allow_nan=False).encode()).hexdigest()
    return evidence, fingerprint


def quality_error(q, fingerprint, board_frame):
    if not q.get('pose_ok') or q.get('camera_model_change_latched') or q.get('camera_model_error'):
        return '棋盘姿态无效'
    if type(q.get('source_stamp_ns')) is not int or q['source_stamp_ns'] <= 0:
        return '缺少精确图像源戳'
    if (q.get('camera_model_fingerprint') != fingerprint
            or q.get('intrinsics_source') != 'realsense_runtime'
            or q.get('board_frame') != board_frame):
        return '棋盘观测与当前相机模型不一致'
    if (q.get('board_size') != [12,9] or q.get('dictionary_id') != 'DICT_5X5_100'
            or abs(float(q.get('square_length_m',0))-.015)>1e-9
            or abs(float(q.get('marker_length_m',0))-.01125)>1e-9):
        return '棋盘参数不符 CC200'
    for name,low,high in [('charuco_count',35,np.inf),('edge_clearance_px',12,np.inf),
                          ('reprojection_rms_px',0,1.5)]:
        value=q.get(name)
        if type(value) not in (int,float) or not np.isfinite(value) or not low<=value<=high:
            return '棋盘质量不合格: '+name
    return None


def window_metrics(records):
    if len(records)<8:
        raise ValueError('静止窗口有效唯一帧少于 8 帧')
    stamps=[r['source_stamp_ns'] for r in records]
    if len(set(stamps))!=len(stamps) or stamps!=sorted(stamps):
        raise ValueError('源帧重复或时间倒退')
    duration=(stamps[-1]-stamps[0])/1e9
    if duration<1.0:
        raise ValueError('静止窗口不足 1 秒')
    metrics={'unique_frames':len(records),'duration_sec':duration}
    for key,prefix,max_m,max_deg in [('T_base_tool','tool',.0015,.25),
                                   ('T_camera_board','board',.002,.5)]:
        matrices=[np.asarray(r[key],dtype=float) for r in records]
        distance=max(float(np.linalg.norm(a[:3,3]-b[:3,3])) for a in matrices for b in matrices)
        angle=max(rotation_deg(a,b) for a in matrices for b in matrices)
        if not np.isfinite([distance,angle]).all() or distance>max_m or angle>max_deg:
            raise ValueError('%s 未停稳: %.3f mm / %.3f deg'%(prefix,distance*1000,angle))
        metrics[prefix+'_span_m']=distance
        metrics[prefix+'_span_deg']=angle
    return metrics


class ExactSampleServer(HandeyeServer):
    def __init__(self):
        self.destination=require_archive(rospy.get_param('~session_dir'))
        if (self.destination/'session.json').exists():
            raise ValueError('会话已有采集服务记录；拒绝覆盖，须建立新会话')
        self.action_lock=threading.RLock()
        self.cache_lock=threading.RLock()
        self.cache={name:OrderedDict() for name in ('rgb','depth','quality','board','joints')}
        self.feedback_ready=False
        self.protected=True
        self.task_active=None
        self.epoch=None
        self.epoch_generation=0
        self.guard_generation=0
        self.entries=[]
        self.capture_index=0
        self.initial_fingerprint=None
        self.initial_epoch=None
        self.alias=rospy.get_param('~gui_alias','').rstrip('/')
        self.pose_change_source=rospy.get_param('~pose_change_source','operator_manual')
        self.sample_role=rospy.get_param('~sample_role','training')
        if self.sample_role not in ('training','validation'):
            raise ValueError('sample_role must be training or validation')
        self.subscriptions=[]
        self.started=utc_now()
        self.ready=False
        super().__init__()
        self.board_frame=self.parameters.tracking_marker_frame
        self.camera_frame=self.parameters.tracking_base_frame
        self.buffer=self.sampler.tfBuffer
        for name in ('rgb','depth','joints'):
            kind=JointState if name=='joints' else Image
            self.subscriptions.append(rospy.Subscriber(TOPICS[name],kind,
                lambda msg,name=name:self.put(name,msg.header.stamp.to_nsec(),msg),
                queue_size=30,buff_size=2**24))
        self.subscriptions.extend([
            rospy.Subscriber(TOPICS['quality'],String,self.on_quality,queue_size=50),
            rospy.Subscriber('/tf',TFMessage,self.on_tf,queue_size=100),
            rospy.Subscriber('/alicia_d/feedback_ready',Bool,self.on_ready,queue_size=5),
            rospy.Subscriber('/alicia_d/protection_latched',Bool,self.on_protected,queue_size=5),
            rospy.Subscriber('/alicia_d/control_reference_epoch',Header,self.on_epoch,queue_size=5),
            rospy.Subscriber('/grasp/state',GraspState,self.on_task,queue_size=5),
        ])
        self.alias_services=[]
        if self.alias and self.alias+'/'!=rospy.get_namespace():
            for name,kind,handler in [
                ('get_sample_list',services.TakeSample,self.get_sample_lists),
                ('take_sample',services.TakeSample,self.take_sample),
                ('remove_sample',services.RemoveSample,self.remove_sample),
                ('list_algorithms',services.ListAlgorithms,self.list_algorithms),
                ('set_algorithm',services.SetAlgorithm,self.set_algorithm),
                ('compute_calibration',services.ComputeCalibration,self.compute_calibration),
                ('save_calibration',Empty,self.save_calibration)]:
                self.alias_services.append(rospy.Service(self.alias+'/'+name,kind,handler))
        description=rospy.get_param('/robot_description','')
        (self.destination/'robot_description.urdf').write_text(description)
        self.provenance={'server_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                         'tracker_sha256':hashlib.sha256(TRACKER.read_bytes()).hexdigest(),
                         'robot_description_sha256':hashlib.sha256(description.encode()).hexdigest()}
        for rel in ['src/alicia_flexible_grasp_supervisor/scripts/camera_node.py',
                    'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/realsense_manager.py']:
            self.provenance[rel]=hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()
        self.write_session()
        self.ready=True
        rospy.loginfo('Exact handeye session ready: %s (GUI alias %s)',self.destination,self.alias)

    def put(self,name,key,value):
        if key<=0:return
        with self.cache_lock:
            self.cache[name][key]=value
            while len(self.cache[name])>120:self.cache[name].popitem(last=False)

    def on_quality(self,msg):
        try:
            q=json.loads(msg.data)
            key=q.get('source_stamp_ns')
            if type(key) is int:self.put('quality',key,(q,msg))
        except (ValueError,TypeError):pass

    def on_tf(self,msg):
        for item in msg.transforms:
            if item.child_frame_id==self.board_frame and item.header.frame_id==self.camera_frame:
                self.put('board',item.header.stamp.to_nsec(),item)

    def on_ready(self,msg):
        with self.cache_lock:
            if not msg.data:self.guard_generation+=1
            self.feedback_ready=bool(msg.data)

    def on_protected(self,msg):
        with self.cache_lock:
            if msg.data:self.guard_generation+=1
            self.protected=bool(msg.data)

    def on_task(self,msg):
        with self.cache_lock:
            if msg.active:self.guard_generation+=1
            self.task_active=bool(msg.active)

    def on_epoch(self,msg):
        with self.cache_lock:
            value=msg.stamp.to_nsec()
            if self.epoch is not None and self.epoch.stamp.to_nsec()!=value:self.epoch_generation+=1
            self.epoch=msg

    def camera_parameters(self):
        return MODEL.read_runtime_camera_parameters('/camera',rospy.get_name(),rosgraph.get_master_uri())

    def guard(self):
        with self.cache_lock:
            if not self.ready or not self.feedback_ready or self.protected or self.task_active is not False:
                raise ValueError('机械臂反馈未就绪、保护生效或抓取任务未释放')
            if self.epoch is None:raise ValueError('缺少驱动反馈代际')
            if not self.cache['joints']:raise ValueError('缺少真实 accepted 关节反馈')
            latest=next(reversed(self.cache['joints'].values()))
            age=(rospy.Time.now()-latest.header.stamp).to_sec()
            if not 0<=age<=.25:raise ValueError('accepted 关节反馈过期')
            return (self.epoch_generation,self.guard_generation,self.epoch.stamp.to_nsec())

    def write_session(self):
        require_archive(self.destination)
        write_json(self.destination/'session.json',dict(
            schema_version=1,started_at_utc=self.started,updated_at_utc=utc_now(),
            primary_namespace=rospy.get_namespace(),gui_alias=self.alias,
            status='collecting',board_frame=self.board_frame,source_provenance=self.provenance,
            camera_model_fingerprint=self.initial_fingerprint,initial_epoch=self.initial_epoch,
            manual_pose_changes=getattr(self,'pose_change_source','operator_manual')=='operator_manual',
            pose_change_source=getattr(self,'pose_change_source','operator_manual'),
            sample_role=getattr(self,'sample_role','training'),robot_commands_sent_by_sampler=False,
            active_sample_count=len(self.sampler.samples),captures=self.entries,
            candidate_is_not_validated_calibration=True,calibration_deployed=False))

    def get_sample_lists(self,req):
        with self.action_lock:return super().get_sample_lists(req)

    def take_sample(self,req):
        if not self.action_lock.acquire(blocking=False):raise rospy.ServiceException('正在采样，请等待')
        try:
            return self.capture()
        except Exception as exc:
            rospy.logwarn('Exact handeye capture refused: %s',exc)
            raise rospy.ServiceException(str(exc))
        finally:self.action_lock.release()

    def capture(self):
        require_archive(self.destination)
        initial_guard=self.guard()
        before=self.camera_parameters()
        evidence,fingerprint=model_binding(before,self.camera_frame,TOPICS['rgb'])
        if self.initial_fingerprint is not None and fingerprint!=self.initial_fingerprint:
            raise ValueError('会话相机模型已变化，必须建立新会话')
        if self.initial_epoch is not None and self.initial_epoch!=initial_guard[2]:
            raise ValueError('驱动代际已变化，必须建立新会话')
        self.capture_index+=1
        directory=self.destination/('capture_%03d'%self.capture_index)
        directory.mkdir(exist_ok=False)
        first_stamp=rospy.Time.now().to_nsec()
        deadline=time.monotonic()+4.0
        records,frames,seen,failures=[],[],set(),Counter()
        error=None
        after,metrics=None,None
        with self.cache_lock: frozen_epoch=deepcopy(self.epoch)
        try:
            while time.monotonic()<deadline and not rospy.is_shutdown():
                if self.guard()!=initial_guard:raise ValueError('采样期间反馈状态或驱动代际变化')
                with self.cache_lock:
                    pending=[ns for ns in self.cache['quality'] if ns>=first_stamp and ns not in seen]
                for ns in sorted(pending):
                    with self.cache_lock:
                        values=[self.cache[name].get(ns) for name in ('rgb','depth','quality','board')]
                        joint_items=list(self.cache['joints'].items())
                    if any(v is None for v in values) or not joint_items:continue
                    rgb,depth,(q,qmsg),board=values
                    seen.add(ns)
                    reason=quality_error(q,fingerprint,self.board_frame)
                    if reason:failures[reason]+=1;continue
                    if (rgb.width,rgb.height)!=(evidence['color_intrinsics']['width'],evidence['color_intrinsics']['height']) or (depth.width,depth.height)!=(rgb.width,rgb.height):
                        raise ValueError('RGBD 尺寸与实际相机模型不符')
                    if rgb.header.frame_id!=self.camera_frame or depth.header.frame_id!=self.camera_frame:
                        raise ValueError('RGBD frame 不一致')
                    if (rospy.Time.now().to_nsec()-ns)/1e9>.5:failures['图像过期']+=1;continue
                    joint_ns,joints=min(joint_items,key=lambda item:abs(item[0]-ns))
                    if abs(joint_ns-ns)>.15e9:failures['缺少源戳附近 accepted 反馈']+=1;continue
                    stamp=rgb.header.stamp
                    try:
                        robot=self.buffer.lookup_transform('base_link','tool0',stamp,rospy.Duration(.05))
                    except Exception:
                        failures['同戳工具 TF 不可用']+=1;continue
                    a,b=transform_matrix(robot),transform_matrix(board)
                    record=dict(source_stamp_ns=ns,quality=q,T_base_tool=a.tolist(),T_camera_board=b.tolist(),
                                accepted_joint_stamp_ns=joint_ns,accepted_joint_time_offset_ns=joint_ns-ns,
                                accepted_joint_names=list(joints.name),accepted_joint_positions=list(joints.position),
                                control_reference_epoch_ns=initial_guard[2])
                    records.append(record)
                    frames.append((rgb,depth,qmsg,board,robot,joints))
                time.sleep(.01)
            if rospy.is_shutdown():raise ValueError('采样节点正在关闭')
            if self.guard()!=initial_guard:raise ValueError('窗口结束反馈状态变化')
            after=self.camera_parameters()
            if model_binding(after,self.camera_frame,TOPICS['rgb'])[1]!=fingerprint:
                raise ValueError('窗口前后相机模型不一致')
            metrics=window_metrics(records)
            representative=np.asarray(records[len(records)//2]['T_base_tool'])
            for previous in self.sampler.samples:
                prior=transform_matrix(previous['robot'])
                if (np.linalg.norm(representative[:3,3]-prior[:3,3])<.005
                        and rotation_deg(representative,prior)<3.):
                    raise ValueError('与已有姿态差异不足 5 mm / 3 度，请先调整到新姿态')
        except Exception as exc:
            error=str(exc)
        # Preserve failures as evidence too; never add an incomplete capture to GUI samples.
        require_archive(self.destination)
        with rosbag.Bag(str(directory/'sources.bag'),'w',compression='lz4') as bag:
            for rgb,depth,qmsg,board,robot,joints in frames:
                stamp=rgb.header.stamp
                for topic,message in [(TOPICS['rgb'],rgb),(TOPICS['depth'],depth),(TOPICS['quality'],qmsg),
                     ('/tf',TFMessage(transforms=[board])),('/handeye_capture/base_tool_at_source',robot),
                     (TOPICS['joints'],joints),('/alicia_d/control_reference_epoch',frozen_epoch)]:
                    bag.write(topic,message,t=stamp)
            for phase,parameters in [('before',before),('after',after)]:
                bag.write('/handeye_capture/camera_'+phase,String(data=json.dumps(parameters,sort_keys=True)),t=rospy.Time.now())
        digest=hashlib.sha256((directory/'sources.bag').read_bytes()).hexdigest()
        selected_index=len(records)//2 if records else None
        report=dict(schema_version=1,created_at_utc=utc_now(),accepted=error is None,rejection=error,
                    sample_role=getattr(self,'sample_role','training'),role_must_be_reserved_before_fitting=True,
                    camera_parameters_before=before,camera_parameters_after=after,
                    model_fingerprint=fingerprint,metrics=metrics,failures=dict(failures),
                    selected_frame_index=selected_index,samples=records,bag_sha256=digest,
                    parameter_binding='profile before/after + each accepted pose fingerprint; not sensor exposure metadata')
        write_json(directory/'capture.json',report)
        entry=dict(directory=directory.name,accepted=error is None,active=error is None,
                   role=getattr(self,'sample_role','training'),record_count=len(records),bag_sha256=digest,rejection=error)
        if error is None:
            _,_,_,board,robot,_=frames[selected_index]
            self.sampler.samples.append({'robot':robot,'optical':board})
            self.initial_fingerprint=fingerprint
            self.initial_epoch=initial_guard[2]
            entry['gui_index_1based']=len(self.sampler.samples)
            entry['source_stamp_ns']=records[selected_index]['source_stamp_ns']
            self.last_calibration=None
        self.entries.append(entry)
        try:
            self.write_session()
        except Exception:
            # A failed session commit must not be reported as an admitted GUI row.
            if error is None:self.sampler.samples.pop()
            entry['active']=False
            self.ready=False
            raise
        if error:raise ValueError(error)
        rospy.loginfo('Archived GUI sample %d: %s, %d unique frames',len(self.sampler.samples),directory,len(records))
        return services.TakeSampleResponse(self._retrieve_sample_list())

    def remove_sample(self,req):
        with self.action_lock:
            active=[entry for entry in self.entries if entry.get('active')]
            if not 0<=req.sample_index<len(active):raise rospy.ServiceException('样本索引无效')
            active[req.sample_index]['active']=False
            active[req.sample_index]['excluded_at_utc']=utc_now()
            self.sampler.remove_sample(req.sample_index)
            self.last_calibration=None
            self.write_session()
            return services.RemoveSampleResponse(self._retrieve_sample_list())

    def remove_last_sample(self,*_):
        if self.sampler.samples:self.remove_sample(services.RemoveSampleRequest(len(self.sampler.samples)-1))

    def compute_calibration(self,req):
        with self.action_lock:
            if len(self.sampler.samples)<10:raise rospy.ServiceException('至少需要 10 个不同姿态；连续帧不算独立姿态')
            return super().compute_calibration(req)

    def save_calibration(self,req):
        with self.action_lock:
            require_archive(self.destination)
            if self.last_calibration is None:raise rospy.ServiceException('没有计算结果')
            path=self.destination/('candidate_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.yaml')
            with path.open('x') as handle:handle.write(HandeyeCalibration.to_yaml(self.last_calibration))
            write_json(path.with_suffix('.json'),dict(candidate_only=True,validated=False,deployed=False,
                       sample_count=len(self.sampler.samples),model_fingerprint=self.initial_fingerprint))
            return EmptyResponse()


if __name__=='__main__':
    rospy.init_node('handeye_exact_sample_server')
    server=ExactSampleServer()
    rospy.spin()
