"""Pure tests; never initializes ROS, a SDK device, or a service."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import zipfile
import pytest

P=Path(__file__).with_name('camera_fps_trial_20261008.py')
spec=importlib.util.spec_from_file_location('camera_fps_trial_under_test',P)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def fixture():
    c={'fps':30,'publish_fps':20,'stream_width':640,'stream_height':480,'runtime_profile':{
        'geometry_activation_stamp_ns':'10','mode_depth_preset':{'mode':'unknown','preset':'high_accuracy',
        'configured_unknown_preset':'high_accuracy','baseline_sha256':'original'}}}
    for key in ('device','color_intrinsics','native_depth_intrinsics','depth_to_color_extrinsics','alignment',
                'geometry_intrinsics','geometry_intrinsics_source','depth_scale_m_per_unit','color_projection_correction','rectification'):
        c['runtime_profile'][key]={'original':key}
    a=copy.deepcopy(c);a['fps']=15;a['runtime_profile']['geometry_activation_stamp_ns']='200'
    e={'source_stamp_ns':300,'geometry_activation_stamp_ns':'200','evidence':{'result':'complete',
       'pair_timing':{'accepted':True},'raw_frames':{s:{'present':True,'stream_fps':15.} for s in ('color','depth')}}}
    payload={'schema':1,'geometry_activation_stamp_ns':'200','entries':[e]}
    envelope={'schema':1,'encoding':'json','payload_json':json.dumps(payload)}
    return c,a,payload,envelope


def confirm(c,a,p):
    return m.confirmed_profile(c,a,{'schema':1,'encoding':'json','payload_json':json.dumps(p)},15,
        old_activation='10',start_ns=100,now_ns=400)


def test_valid_compact_profile_without_profile_signature():
    c,a,p,_=fixture();assert confirm(c,a,p)


@pytest.mark.parametrize('kind',['old_activation','old_entry_activation','wrong_fps','one_stream_missing','rejected_pair','old_source','future_source'])
def test_pending_or_invalid_source_never_confirms(kind):
    c,a,p,_=fixture();e=p['entries'][0]
    if kind=='old_activation': a['runtime_profile']['geometry_activation_stamp_ns']='10'
    elif kind=='old_entry_activation': e['geometry_activation_stamp_ns']='10'
    elif kind=='wrong_fps': e['evidence']['raw_frames']['depth']['stream_fps']=30
    elif kind=='one_stream_missing': del e['evidence']['raw_frames']['depth']
    elif kind=='rejected_pair': e['evidence']['pair_timing']['accepted']=False
    elif kind=='old_source': e['source_stamp_ns']=99
    elif kind=='future_source': e['source_stamp_ns']=500
    assert confirm(c,a,p) is None


@pytest.mark.parametrize('kind',['geometry','preset','other_config'])
def test_other_change_rejected(kind):
    c,a,p,_=fixture()
    if kind=='geometry':a['runtime_profile']['geometry_intrinsics']={'changed':True}
    elif kind=='preset':a['runtime_profile']['mode_depth_preset']['preset']='other'
    else:a['publish_fps']=15
    with pytest.raises(RuntimeError):confirm(c,a,p)


def test_real_gen25_compact_envelope_structure():
    path=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/aligned_grasp_resume_20261008T023234Z/selection/raw_support/diagnostic.zip')
    if not path.is_file():pytest.skip('sealed real archive unavailable; synthetic coverage remains')
    with zipfile.ZipFile(path) as z:
        md=json.loads(z.read('metadata/0.json'))['parameters']
    c=md['/camera'];a=copy.deepcopy(c);p=json.loads(md['/camera_read_diagnostics']['payload_json'])
    old=c['runtime_profile']['geometry_activation_stamp_ns'];start=int(old)+1
    activation=str(start+1);a['fps']=15;a['runtime_profile']['geometry_activation_stamp_ns']=activation
    p['geometry_activation_stamp_ns']=activation
    for e in p['entries']:
        e['geometry_activation_stamp_ns']=activation
        for raw in e['evidence']['raw_frames'].values():
            assert 'profile_signature' not in raw
            raw['stream_fps']=15.
    now=max(e['source_stamp_ns'] for e in p['entries'])+10_000_000
    proof=m.confirmed_profile(c,a,{'schema':1,'encoding':'json','payload_json':json.dumps(p)},15,
        old_activation=old,start_ns=start,now_ns=now)
    assert proof
    # This is a schema compatibility fixture; edited FPS is synthetic, not real 15 FPS evidence.


class FakeIO:
    def __init__(self,fail=None):
        self.report={};self.stop_requested=False;self.leg=0;self.events=[];self.fail=fail
    def stop_current(self):self.events.append('stop');self.stop_requested=True
    def launch(self,fps):self.leg+=1;self.events.append(('launch',fps))
    def confirm(self,fps):
        self.events.append(('confirm',fps))
        if fps==self.fail:raise RuntimeError('test failure')
    def verify_core(self):self.events.append('core')


def test_15_failure_restores_30_but_does_not_claim_trial_success():
    io=FakeIO(15)
    with pytest.raises(RuntimeError):m.transaction(io,15)
    assert io.events==['stop',('launch',15),('confirm',15),'stop',('launch',30),('confirm',30),'core']
    assert io.report['restored_30_confirmed'] is True
    assert not io.report.get('requested_profile_confirmed')


def test_30_failure_no_retry_loop():
    io=FakeIO(30)
    with pytest.raises(RuntimeError):m.transaction(io,30)
    assert io.events==['stop',('launch',30),('confirm',30)]


def test_success_does_not_restore():
    io=FakeIO();m.transaction(io,15)
    assert io.report['requested_profile_confirmed'] is True
    assert io.events==['stop',('launch',15),('confirm',15),'core']


def test_pending_registration_retries(monkeypatch):
    count=[0]
    def predicate():
        count[0]+=1
        if count[0]==1:raise m.CameraPending('not registered yet')
        return 'ready'
    monkeypatch.setattr(m.time,'sleep',lambda _:None)
    assert m.wait(predicate,.2,'test')=='ready'
    assert count[0]==2


def pending_io(completed):
    io=object.__new__(m.Runtime);io.pending_unit='own-unit';io.current=None;io.start_command_completed=completed
    io.expected_argv=['python','camera'];io.original={'exe_sha256':'same'};io.report={'legs':[{}]}
    io.h=SimpleNamespace(identity=lambda pid:{'pid':pid,'start_ticks':4,'argv':['python','camera'],'exe_sha256':'same'})
    return io


def test_timeout_binds_existing_owned_child_before_recovery(monkeypatch):
    io=pending_io(False)
    monkeypatch.setattr(m.subprocess,'run',lambda *a,**k:SimpleNamespace(stdout='LoadState=loaded\nActiveState=active\nMainPID=77\n',returncode=0))
    io.bind_pending_owner();assert io.current['pid']==77


def test_timeout_unknown_unit_blocks_second_owner(monkeypatch):
    io=pending_io(False)
    monkeypatch.setattr(m.subprocess,'run',lambda *a,**k:SimpleNamespace(stdout='LoadState=not-found\nActiveState=inactive\nMainPID=0\n',returncode=0))
    with pytest.raises(RuntimeError,match='ownership unresolved'):io.bind_pending_owner()


def test_failed_completed_start_with_absent_unit_is_recoverable(monkeypatch):
    io=pending_io(True)
    monkeypatch.setattr(m.subprocess,'run',lambda *a,**k:SimpleNamespace(stdout='LoadState=not-found\nActiveState=inactive\nMainPID=0\n',returncode=0))
    io.bind_pending_owner();assert io.pending_unit is None


def test_wrong_child_argv_never_bound(monkeypatch):
    io=pending_io(True);io.expected_argv=['python','different']
    monkeypatch.setattr(m.subprocess,'run',lambda *a,**k:SimpleNamespace(stdout='MainPID=77\n',returncode=0))
    with pytest.raises(RuntimeError,match='argv mismatch'):io.bind_pending_owner()
    assert io.current is None


def test_only_mutating_rpc_is_fps():
    calls=[];rpc=object.__new__(m.Rpc)
    rpc.call=lambda *a:(calls.append(a) or True);rpc.get=lambda p:15
    rpc.fps(15)
    assert calls==[('setParam','/camera/fps',15)]
    with pytest.raises(RuntimeError):rpc.fps(10)
