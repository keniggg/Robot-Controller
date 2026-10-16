import importlib.util
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS
from collections import OrderedDict
from unittest.mock import Mock
import numpy as np
import pytest
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Header

SPEC=importlib.util.spec_from_file_location('exact_server_test',Path(__file__).parents[1]/'handeye_exact_sample_server.py')
M=importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)

def parameters():
    return dict(frame_id='camera_link',width=640,height=480,color_topic=M.TOPICS['rgb'],
        runtime_profile=dict(source='realsense_active_profile',schema_version=1,sdk_version='2.56.5',
        device={'serial_number':'offline-test'},color_projection_correction={'enabled':False},
        color_intrinsics=dict(width=640,height=480,fx=395.,fy=395.,cx=320.,cy=240.,model='none',coeffs=[0.]*5)))

def quality(ns=100000000000):
    return dict(pose_ok=True,camera_model_change_latched=False,camera_model_error=None,
        source_stamp_ns=ns,camera_model_fingerprint=M.model_binding(parameters(),'camera_link',M.TOPICS['rgb'])[1],
        intrinsics_source='realsense_runtime',board_frame='board',board_size=[12,9],dictionary_id='DICT_5X5_100',
        square_length_m=.015,marker_length_m=.01125,charuco_count=54,edge_clearance_px=30.,reprojection_rms_px=.2)

def records():
    return [dict(source_stamp_ns=100000000000+index*200000000,
                 T_base_tool=np.eye(4).tolist(),T_camera_board=np.eye(4).tolist()) for index in range(8)]

def test_quality_binds_board_and_current_model():
    q=quality()
    assert M.quality_error(q,q['camera_model_fingerprint'],'board') is None

@pytest.mark.parametrize('field,value',[('pose_ok',False),('camera_model_change_latched',True),
    ('source_stamp_ns',100.),('camera_model_fingerprint','old'),('intrinsics_source','legacy'),
    ('board_size',[11,8]),('dictionary_id','DICT_4X4_100'),('marker_length_m',.011),
    ('reprojection_rms_px',float('nan')),('reprojection_rms_px',2.),('charuco_count',34),
    ('edge_clearance_px',11),('camera_model_error','timeout')])
def test_invalid_quality_is_rejected(field,value):
    q=quality(); expected=q['camera_model_fingerprint'];q[field]=value
    assert M.quality_error(q,expected,'board')

def test_stationary_window_counts_unique_source_frames():
    result=M.window_metrics(records())
    assert result['unique_frames']==8 and result['duration_sec']==1.4
    assert result['tool_span_m']==0.

@pytest.mark.parametrize('failure',['too_few','duplicate','reversed','short','tool_move','tool_rotate','board_move'])
def test_unstable_or_repeated_frames_rejected(failure):
    data=records()
    if failure=='too_few':data=data[:7]
    elif failure=='duplicate':data[-1]['source_stamp_ns']=data[-2]['source_stamp_ns']
    elif failure=='reversed':data.reverse()
    elif failure=='short':
        for i,r in enumerate(data):r['source_stamp_ns']=100000000000+i*10000000
    elif failure=='tool_move':data[-1]['T_base_tool'][0][3]=.002
    elif failure=='tool_rotate':
        a=.01
        data[-1]['T_base_tool'][:2]=[[np.cos(a),-np.sin(a),0,0],[np.sin(a),np.cos(a),0,0]]
    else:data[-1]['T_camera_board'][0][3]=.003
    with pytest.raises(ValueError):M.window_metrics(data)


def test_unmounted_or_system_disk_output_never_falls_back(monkeypatch,tmp_path):
    monkeypatch.setattr(M.os.path,'ismount',lambda _:False)
    with pytest.raises(ValueError):M.require_archive(tmp_path)
    monkeypatch.setattr(M.os.path,'ismount',lambda _:True)
    with pytest.raises(ValueError):M.require_archive(tmp_path)


def fake_capture(monkeypatch,tmp_path):
    server=M.ExactSampleServer.__new__(M.ExactSampleServer)
    server.destination=tmp_path
    server.ready=True
    server.guard=Mock(return_value=(0,0,42))
    server.camera_parameters=lambda:parameters()
    server.camera_frame='camera_link';server.board_frame='board'
    server.initial_fingerprint=server.initial_epoch=None
    server.capture_index=0;server.entries=[];server.last_calibration=None
    server.action_lock=threading.RLock();server.cache_lock=threading.RLock()
    server.cache={key:OrderedDict() for key in ['rgb','depth','quality','board','joints']}
    server.sampler=NS(samples=[])
    server.sampler.get_samples=lambda:server.sampler.samples
    server.epoch=Header(stamp=rospy.Time(0,42),frame_id='sdk_reference_epoch')
    server.write_session=Mock()
    for i in range(8):
        ns=100050000000+i*200000000
        stamp=rospy.Time(ns//10**9,ns%10**9)
        header=Header(stamp=stamp,frame_id='camera_link')
        for kind,encoding,size,step in [('rgb','rgb8',640*480*3,640*3),('depth','16UC1',640*480*2,640*2)]:
            server.cache[kind][ns]=Image(header=header,height=480,width=640,encoding=encoding,step=step,data=bytes(size))
        q=quality(ns);server.cache['quality'][ns]=(q,String(data=json.dumps(q)))
        board=TransformStamped(header=header,child_frame_id='board');board.transform.rotation.w=1;board.transform.translation.x=.3
        server.cache['board'][ns]=board
        server.cache['joints'][ns]=JointState(header=Header(stamp=stamp),name=['Joint%d'%j for j in range(1,7)],position=[0.]*6)
    def lookup(parent,child,stamp,*_):
        value=TransformStamped(header=Header(stamp=stamp,frame_id=parent),child_frame_id=child)
        value.transform.rotation.w=1
        return value
    server.buffer=NS(lookup_transform=lookup)
    monkeypatch.setattr(M,'require_archive',lambda p:p)
    monotonic=iter([0.,0.,5.])
    monkeypatch.setattr(M.time,'monotonic',lambda:next(monotonic))
    monkeypatch.setattr(M.time,'sleep',lambda _:None)
    times=iter([100.]+[100.15+i*.2 for i in range(8)]+[102.,102.])
    monkeypatch.setattr(M.rospy.Time,'now',staticmethod(lambda:rospy.Time.from_sec(next(times))))
    monkeypatch.setattr(M.rospy,'is_shutdown',lambda:False)
    monkeypatch.setattr(M.rospy,'loginfo',Mock())
    return server


def test_gui_sample_admitted_only_with_same_frame_archive(monkeypatch,tmp_path):
    server=fake_capture(monkeypatch,tmp_path)
    response=server.capture()
    assert len(response.samples.hand_world_samples)==1
    report=json.loads((tmp_path/'capture_001/capture.json').read_text())
    assert report['accepted'] and len(report['samples'])==8
    assert len({r['source_stamp_ns'] for r in report['samples']})==8
    import rosbag
    with rosbag.Bag(str(tmp_path/'capture_001/sources.bag')) as bag:
        images=[msg for topic,msg,_ in bag.read_messages(topics=[M.TOPICS['rgb']])]
    assert [msg.header.stamp.to_nsec() for msg in images]==[r['source_stamp_ns'] for r in report['samples']]


def test_session_commit_failure_does_not_increase_gui_count(monkeypatch,tmp_path):
    server=fake_capture(monkeypatch,tmp_path)
    server.write_session=Mock(side_effect=OSError('disk full'))
    with pytest.raises(OSError):server.capture()
    assert server.sampler.samples==[] and not server.ready
    assert (tmp_path/'capture_001/sources.bag').exists()


def test_invalid_window_preserved_without_admission(monkeypatch,tmp_path):
    server=fake_capture(monkeypatch,tmp_path)
    q,msg=next(iter(server.cache['quality'].values()));q['pose_ok']=False
    with pytest.raises(ValueError,match='少于 8'):server.capture()
    assert server.sampler.samples==[]
    assert not json.loads((tmp_path/'capture_001/capture.json').read_text())['accepted']


def test_candidate_save_is_external_and_never_overwrites_deployed_calibration(monkeypatch,tmp_path):
    server=M.ExactSampleServer.__new__(M.ExactSampleServer)
    server.action_lock=threading.RLock();server.destination=tmp_path
    server.last_calibration=object();server.sampler=NS(samples=[{}]*10);server.initial_fingerprint='model'
    monkeypatch.setattr(M,'require_archive',lambda p:p)
    monkeypatch.setattr(M.HandeyeCalibration,'to_yaml',lambda _: 'candidate: true\n')
    monkeypatch.setattr(M.HandeyeCalibration,'to_file',Mock(side_effect=AssertionError('must not use global calibration file')))
    server.save_calibration(None)
    assert len(list(tmp_path.glob('candidate_*.yaml')))==1
    status=json.loads(next(tmp_path.glob('candidate_*.json')).read_text())
    assert status['candidate_only'] and not status['validated'] and not status['deployed']


def test_repeated_pose_is_archived_but_not_counted_as_an_independent_sample(monkeypatch,tmp_path):
    server=fake_capture(monkeypatch,tmp_path)
    prior=TransformStamped();prior.transform.rotation.w=1.
    server.sampler.samples.append({'robot':prior,'optical':prior})
    with pytest.raises(ValueError,match='已有姿态'):
        server.capture()
    assert len(server.sampler.samples)==1
    report=json.loads((tmp_path/'capture_001/capture.json').read_text())
    assert report['accepted'] is False and len(report['samples'])==8
