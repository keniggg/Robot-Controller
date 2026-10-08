"""Offline freshness and concurrency regressions; creates no ROS node/publisher."""
from collections import OrderedDict
import importlib.util
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import pytest

@pytest.fixture(params=['bounded_joint_response_diagnostic.py','bounded_joint3_response_diagnostic.py'])
def mod(request):
    path=Path(__file__).with_name(request.param)
    spec=importlib.util.spec_from_file_location('pair_test_'+path.stem,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

def tuple_data():
    ns=10_000_000_000
    q=dict(board_frame='board',pose_ok=True,charuco_count=80,edge_clearance_px=122,
           reprojection_rms_px=.11,camera_model_change_latched=False,camera_model_error=None,
           camera_model_fingerprint='current',intrinsics_source='realsense_runtime',
           board_size=[12,9],dictionary_id='DICT_5X5_100',square_length_m=.015,
           marker_length_m=.01125)
    return ns,{ns:object()},OrderedDict([(ns,q)]),{(ns,'board'):object()}

@pytest.mark.parametrize('age_ns,allowed',[(0,True),(349999999,True),(350000000,True),(350000001,False),(-1,False)])
def test_source_age_boundary(mod,age_ns,allowed):
    ns,rgb,quality,board=tuple_data()
    assert bool(mod.paired_board_candidates(rgb,quality,board,ns+age_ns,'current')) is allowed

@pytest.mark.parametrize('mutation',['rgb_stamp','tf_stamp','tf_frame','fingerprint','pose','count','edge','rms','changed','error','source','size','dictionary','square','marker'])
def test_exact_tuple_and_model_required(mod,mutation):
    ns,rgb,quality,board=tuple_data();q=quality[ns]
    if mutation=='rgb_stamp':rgb[ns+1]=rgb.pop(ns)
    elif mutation=='tf_stamp':board[(ns+1,'board')]=board.pop((ns,'board'))
    elif mutation=='tf_frame':board[(ns,'other')]=board.pop((ns,'board'))
    else:
        key,value={'fingerprint':('camera_model_fingerprint','old'),'pose':('pose_ok',False),
          'count':('charuco_count',34),'edge':('edge_clearance_px',19),'rms':('reprojection_rms_px',1.51),
          'changed':('camera_model_change_latched',True),'error':('camera_model_error','error'),
          'source':('intrinsics_source','yaml'),'size':('board_size',[8,6]),
          'dictionary':('dictionary_id','DICT_4X4_50'),'square':('square_length_m',.014),
          'marker':('marker_length_m',.011)}[mutation]
        q[key]=value
    assert mod.paired_board_candidates(rgb,quality,board,ns+100_000_000,'current')==[]

def skeletal(mod):
    d=mod.Diagnostic.__new__(mod.Diagnostic)
    d.lock=threading.RLock();d.pair_changed=threading.Condition(d.lock);d.pair_generation=0
    d.bag_lock=threading.Lock();d.closed=False;d.fault=None;d.deadline=time.monotonic()+.5
    d.guard=lambda:None
    d.rgb=OrderedDict();d.quality=OrderedDict();d.board=OrderedDict();d.state={}
    return d

def test_notification_between_attempt_and_wait_not_lost(mod):
    d=skeletal(mod);attempts=[]
    def attempt(label):
        attempts.append(time.monotonic())
        if len(attempts)==1:
            with d.pair_changed:
                d.pair_generation+=1;d.pair_changed.notify_all()
            raise RuntimeError('fresh paired board quality/image/TF unavailable; no fallback')
        return 'new exact pair'
    d._snapshot_once=attempt
    start=time.monotonic();assert d.snapshot('test')=='new exact pair'
    assert time.monotonic()-start<.15

def test_callback_notification_wakes_wait(mod):
    d=skeletal(mod);attempted=threading.Event();ready=threading.Event()
    def attempt(label):
        if ready.is_set():return 'new exact pair'
        attempted.set();raise RuntimeError('fresh paired board quality/image/TF unavailable')
    d._snapshot_once=attempt
    def produce():
        assert attempted.wait(.2)
        with d.pair_changed:
            ready.set();d.pair_generation+=1;d.pair_changed.notify_all()
    t=threading.Thread(target=produce);t.start()
    assert d.snapshot('test')=='new exact pair';t.join(.2);assert not t.is_alive()

def test_no_notification_cannot_use_old_tuple(mod):
    d=skeletal(mod);d.deadline=time.monotonic()+.03;ns,rgb,quality,board=tuple_data()
    def attempt(label):
        assert not mod.paired_board_candidates(rgb,quality,board,ns+350_000_001,'current')
        raise RuntimeError('fresh paired board quality/image/TF unavailable')
    d._snapshot_once=attempt
    with pytest.raises(RuntimeError,match='fresh paired'):d.snapshot('old')

def test_blocked_depth_write_does_not_block_rgb_or_quality_cache(mod,monkeypatch):
    d=skeletal(mod);entered=threading.Event();release=threading.Event();writes=[]
    monkeypatch.setattr(mod.rospy.Time,'now',lambda:mod.rospy.Time(10,0))
    def write(topic,msg,stamp):
        writes.append(topic)
        if topic=='/supervisor/camera/depth/image_raw':
            entered.set();assert release.wait(1)
    d.bag=SimpleNamespace(write=write)
    depth=threading.Thread(target=d.receive,args=('/supervisor/camera/depth/image_raw',object()))
    depth.start();assert entered.wait(.2)
    msg=SimpleNamespace(header=SimpleNamespace(stamp=mod.rospy.Time(10,0)))
    d.receive(mod.RGB,msg)
    qthread=threading.Thread(target=d.receive,args=('/charuco/quality',SimpleNamespace(data='{"source_stamp_ns":10000000000}')))
    qthread.start()
    try:
        deadline=time.monotonic()+.2
        while 10000000000 not in d.quality and time.monotonic()<deadline:time.sleep(.001)
        with d.lock:
            assert 10000000000 in d.rgb and 10000000000 in d.quality
            assert d.pair_generation==2
        assert qthread.is_alive()  # Its disk write is serialized, only cache is prompt.
    finally:
        release.set();depth.join(1);qthread.join(1)
    assert d.fault is None and len(writes)==2

def test_close_serializes_with_writer_and_rejects_late_write(mod):
    d=skeletal(mod);calls=[]
    d.bag=SimpleNamespace(write=lambda *args:calls.append('write'),close=lambda:calls.append('close'))
    d.events=SimpleNamespace(close=lambda:None);d.subs=[]
    d.close();d.write_bag('late',None,None)
    assert calls==['close']
