import copy
import importlib.util
import io
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
WORK = HERE.parent

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

module = load('camera_preset_ab_observer', HERE/'camera_preset_ab_observer.py')
old = load('stage_fixtures', HERE/'test_camera_stage_observer.py')
setup = old.setup


class Evidence:
    def __init__(self, clock):
        self.clock = clock
        self.guard = module.EvidenceGuard()
        self.initial_context = dict(selection={'mode':'unknown','generation':14}, handeye={'frozen':True})
        self.context = copy.deepcopy(self.initial_context)
        self.qualified = True
        self.closed = False
        self.tick()
    def now_ns(self):return 10**12+int(self.clock()*10**9)
    def tick(self):
        ns = self.now_ns()
        self.guard.observe('epoch',10**11,None)
        self.guard.observe('sdk',ns,[0.]*6)
        self.guard.observe('accepted',ns,[0.]*7)
        self.guard.observe('task',ns,dict(active=False,state='IDLE',stage=0,success=False,message=''))
    def check_context(self):
        if self.context != self.initial_context:
            self.guard.fail('context_changed')
            raise ValueError('context_changed')
    def at_anchor(self, ns):
        return dict(evidence_complete=self.qualified,feedback=self.guard.bracket(ns,timeout=0),
                    exact_tf={'source_stamp_ns':ns} if self.qualified else None)
    def close(self):self.closed=True


@pytest.fixture
def ab(setup, tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(WORK/'src/alicia_flexible_grasp_supervisor/src'))
    presets = load('real_preset_fixtures', WORK/'src/alicia_flexible_grasp_supervisor/tests/test_mode_depth_preset.py')
    make, camera_class, original = setup
    cam, previous, clock = make()
    previous.shutdown()
    policy,sensor,advanced = presets.setup_policy(tmp_path)
    sensor.supports=lambda _:True
    policy.apply('unknown')
    cam.mode_depth_preset=policy
    original_wait=cam.pipeline.wait_for_frames
    def wait():
        frames=original_wait()
        for frame in (frames.depth,frames.color):
            frame.get_frame_number=lambda:cam.pipeline.calls
            frame.get_timestamp=lambda:cam.pipeline.calls*33.333
        return frames
    cam.pipeline.wait_for_frames=wait
    cam.rs.option=policy.rs.option
    def mode(value):
        changed=policy.apply(value)
        if changed:cam.runtime_profile['mode_depth_preset']=policy.evidence()
        return changed
    cam.set_depth_preset_mode=mode
    class Node:
        def __init__(self):
            self.cam=cam;self.cfg={'runtime_profile':{'geometry_intrinsics':{'width':8,'height':8},
                                                      'geometry_activation_stamp_ns':'999999'}}
            self.publications=[];self.syncs=0
        def _sync_depth_preset(self,publish=True,timings=None):
            self.syncs+=1;cam.set_depth_preset_mode('unknown')
        def _publish_runtime_camera_params(self):self.publications.append(copy.deepcopy(cam.runtime_profile))
    node=Node();cam.runtime_profile['geometry_intrinsics']=copy.deepcopy(node.cfg['runtime_profile']['geometry_intrinsics'])
    evidence=Evidence(clock);observers=[]
    def factory(**kw):
        opts=dict(start_thread=False,clock=clock,samples_per_phase=1)
        opts.update(kw)
        obs=module.PresetABObserver(node,tmp_path/('ab%d'%len(observers)),evidence,**opts)
        observers.append(obs)
        return obs
    yield SimpleNamespace(cam=cam,node=node,clock=clock,evidence=evidence,factory=factory,
                          sensor=sensor,advanced=advanced,policy=policy,original=original)
    for observer in observers:observer.shutdown()


def step(ab, observer, *, seconds=.31):
    ab.clock.step(seconds)
    ab.evidence.tick()
    ab.node._sync_depth_preset()
    result=ab.cam.read()
    rows=[]
    while not observer.queue.empty():rows.append(observer.queue.get_nowait())
    return result,rows


def next_sample(ab, obs):
    for _ in range(40):
        result,rows=step(ab,obs)
        if rows:return result,rows[0]
    raise AssertionError('no selected sample')


def test_full_three_phase_real_policy_restoration_and_owned_arrays(ab):
    obs=ab.factory();rows=[]
    for label in ('A1','B','A2'):
        result,row=next_sample(ab,obs);rows.append(row)
        assert result is ab.cam.last_result
        assert row['phase']==label and row['complete']
        expected='carton' if label=='B' else 'unknown'
        assert ab.policy.mode==expected
        # Normal node sync is held by the diagnostic, no fake global selection.
        ab.node._sync_depth_preset();assert ab.policy.mode==expected
        assert ab.evidence.context==ab.evidence.initial_context
        assert row['published_camera_parameters']['runtime_profile']['geometry_activation_stamp_ns']=='999999'
        assert 'rectification canvas' in row['array_coordinate_spaces']['final_depth']
        assert np.all(row['arrays']['raw_depth']==2400)
        assert np.all(row['arrays']['aligned_depth']==2410)
        result[0][:]=0;result[1][:]=0
        assert np.all(row['arrays']['raw_rgb']==80) and np.all(row['arrays']['final_depth']==2460)
    step(ab,obs)
    assert obs.closed and obs.restore['succeeded']
    assert ab.policy.mode=='unknown' and ab.sensor.preset==3.
    assert 'read' not in ab.cam.__dict__ and '_sync_depth_preset' not in ab.node.__dict__
    for row in rows:obs.queue.put_nowait(row);obs._write_row(obs.queue.get_nowait())
    obs._writer()
    summary=json.loads((obs.output/'summary.json').read_text())
    assert summary['experiment_valid'] and summary['capture_complete']
    assert summary['counts']['selected']==3 and summary['counts']['written']==3
    assert [p['sdk_policy_mode'] for p in summary['phases']]==['unknown','carton','unknown']
    with np.load(obs.output/'sample_02.npz') as data:assert np.all(data['raw_depth']==2400)


def test_warmup_requires_both_time_and_ten_complete_reads(ab):
    obs=ab.factory()
    step(ab,obs,seconds=.01)
    ab.clock.step(4);ab.evidence.tick()
    assert not ab.cam.read() is None
    assert obs.counts['selected']==0
    for _ in range(9):step(ab,obs,seconds=.01)
    assert obs.counts['selected']==1


@pytest.mark.parametrize('kind',['sdk','accepted','epoch','task'])
def test_callback_excursion_latches_even_after_return_and_restores(ab,kind):
    obs=ab.factory();step(ab,obs)
    ns=ab.evidence.now_ns()+1
    if kind in ('sdk','accepted'):
        n=6 if kind=='sdk' else 7
        ab.evidence.guard.observe(kind,ns,[.01]+[0.]*(n-1))
        ab.evidence.guard.observe(kind,ns+1,[0.]*n)
    elif kind=='epoch':
        ab.evidence.guard.observe(kind,10**11+1,None)
        ab.evidence.guard.observe(kind,10**11,None)
    else:
        original=ab.evidence.guard.current['task']
        ab.evidence.guard.observe(kind,ns,dict(original,active=True))
        ab.evidence.guard.observe(kind,ns+1,original)
    result,rows=step(ab,obs)
    assert result is ab.cam.last_result and not rows
    assert obs.closed and obs.restore['succeeded'] and not obs.summary()['experiment_valid']
    assert ab.evidence.guard.faults


def test_timer_only_requests_stop_and_restoration_on_owner(ab):
    obs=ab.factory();next_sample(ab,obs);step(ab,obs)
    assert ab.policy.mode=='carton'
    writes=len(ab.sensor.writes)
    obs.finish('deadline')
    assert len(ab.sensor.writes)==writes and not obs.closed
    result,_=step(ab,obs)
    assert result is ab.cam.last_result and obs.closed and obs.restore['succeeded']
    assert ab.policy.mode=='unknown'


def test_deadline_never_admits_new_sample(ab):
    obs=ab.factory();step(ab,obs)
    ab.clock.value=75.1;ab.evidence.tick();ab.cam.read()
    assert obs.closed and obs.counts['selected']==0 and obs.close_reason=='deadline'


def test_restore_failure_is_explicit_and_does_not_claim_success(ab):
    obs=ab.factory();next_sample(ab,obs);step(ab,obs)
    ab.sensor.fail=True
    obs.finish('stop');step(ab,obs)
    assert obs.closed and not obs.restore['succeeded']
    assert 'sensor write failed' in obs.restore['error']
    assert not obs.summary()['experiment_valid']


def test_phase_apply_failure_still_restores_unknown(ab,monkeypatch):
    obs=ab.factory();step(ab,obs)
    next_sample(ab,obs)
    original=ab.cam.set_depth_preset_mode
    failed=[False]
    def once(mode):
        if mode=='carton' and not failed[0]:
            failed[0]=True
            original(mode)
            raise RuntimeError('diagnostic transition failure')
        return original(mode)
    monkeypatch.setattr(ab.cam,'set_depth_preset_mode',once)
    result,_=step(ab,obs)
    assert result is ab.cam.last_result and obs.restore['succeeded'] and ab.policy.mode=='unknown'


def test_original_read_error_propagates_after_restore(ab):
    obs=ab.factory();step(ab,obs)
    ab.cam.raise_original=True
    with pytest.raises(RuntimeError,match='production failure'):step(ab,obs)
    assert obs.closed and obs.restore['succeeded'] and obs.counts['original_read_errors']==1


def test_raw_pairs_saved_when_feedback_or_tf_unavailable(ab):
    obs=ab.factory();_,row=next_sample(ab,obs)
    obs.queue.put_nowait(row);ab.evidence.qualified=False
    obs.shutdown();obs._writer()
    assert (obs.output/'sample_01.npz').exists()
    saved=json.loads((obs.output/'sample_01.json').read_text())
    assert not saved['source_evidence']['evidence_complete'] and saved['complete']
    assert not json.loads((obs.output/'summary.json').read_text())['experiment_valid']


def test_writer_roundtrip_async_does_not_modify_results(ab):
    obs=ab.factory(start_thread=True)
    for _ in range(80):
        ab.clock.step(.31);ab.evidence.tick();result=ab.cam.read()
        assert result is ab.cam.last_result
        if obs.closed:break
        # Let the async writer obtain its bounded queued rows.
        import time;time.sleep(.001)
    obs.thread.join(3)
    assert not obs.thread.is_alive()
    summary=json.loads((obs.output/'summary.json').read_text())
    assert summary['experiment_valid'] and summary['capture_complete']
    assert sum(p.stat().st_size for p in obs.output.iterdir())<obs.maximum_bytes


def test_foreign_owner_not_configured_or_overwritten(ab):
    obs=ab.factory();step(ab,obs)
    foreign=lambda **kw:None
    ab.node._sync_depth_preset=foreign
    obs.finish('owner_changed');ab.cam.read()
    assert obs.closed and not obs.restore['succeeded']
    assert ab.node._sync_depth_preset is foreign


def test_context_change_invalidates_without_changing_original_return(ab):
    obs=ab.factory();step(ab,obs)
    ab.evidence.context['selection']['generation']=15
    result=None
    for _ in range(12):
        result,_=step(ab,obs)
        if obs.closed:break
    assert obs.closed and obs.restore['succeeded'] and result is ab.cam.last_result
    assert 'context_changed' in ab.evidence.guard.faults


def test_exact_bracket_and_over_200ms_failure():
    g=module.EvidenceGuard();epoch=10**9
    g.observe('epoch',epoch,None)
    g.observe('task',epoch,dict(active=False))
    g.observe('sdk',epoch,[0.]*6)
    g.observe('accepted',epoch,[0.]*7);assert g.check(epoch,freeze=True)
    g.observe('accepted',epoch+200_000_000,[0.]*7)
    assert g.bracket(epoch+100_000_000,0)['qualified']
    g.observe('accepted',epoch+400_000_001,[0.]*7)
    result=g.bracket(epoch+300_000_000,0)
    assert not result['qualified'] and result['span_ns']==200_000_001
    assert result['accepted_before'] and result['accepted_after']


def test_budget_24_native_and_rectified_pairs_strictly_below_160MB():
    arrays={k:np.zeros((480,640,3),np.uint8) if k.endswith('rgb') else np.zeros((480,640),np.uint16)
            for k in module._stage.REQUIRED}
    arrays['final_rgb']=np.zeros((489,653,3),np.uint8)
    arrays['final_depth']=np.zeros((489,653),np.uint16)
    buffer=io.BytesIO();np.savez(buffer,**arrays)
    assert len(buffer.getvalue())+128*1024 < 8*module.MIB
    assert 24*(len(buffer.getvalue())+128*1024)+256*1024+64*1024 < module.MAX_BYTES < 160_000_000


@pytest.mark.parametrize('kw',[{'seconds':76},{'warmup':2},{'stable_reads':9},
                               {'samples_per_phase':9},{'interval':.1},{'maximum_bytes':160_000_000}])
def test_invalid_bounds_do_not_patch_owner(ab,kw):
    with pytest.raises(ValueError):ab.factory(**kw)
    assert 'read' not in ab.cam.__dict__ and '_sync_depth_preset' not in ab.node.__dict__


def test_default_entry_never_imports_ros_or_installs_observer(tmp_path,monkeypatch):
    entry=tmp_path/'camera.py';entry.write_text('pass\n')
    calls=[]
    monkeypatch.setattr(module.runpy,'run_path',lambda path,run_name:calls.append((path,run_name)))
    module.main(['--camera-script',str(entry),'--','__name:=supervisor_camera'])
    assert calls==[(str(entry),'__main__')]


def test_explicit_enable_requires_external_quiescence_assertion():
    with pytest.raises(SystemExit):module.arguments(['--camera-script','camera.py','--enable-preset-ab','--output','/tmp/not-external'])


def test_repeated_raw_source_keeps_owned_raw_but_invalidates(ab):
    obs=ab.factory(samples_per_phase=2)
    _,first=next_sample(ab,obs)
    obs.last_raw_sources={key:(10**6,10**9,value[2]) for key,value in obs.last_raw_sources.items()}
    _,second=next_sample(ab,obs)
    assert obs.closed and second['arrays']['raw_depth'].size and not second['complete']
    assert any('SDK source' in error['error'] for error in obs.errors)
    obs.queue.put_nowait(second);obs._writer()
    assert (obs.output/'sample_02.npz').exists()
    assert not json.loads((obs.output/'summary.json').read_text())['experiment_valid']


@pytest.mark.parametrize('kind',['unserializable','huge','nan'])
def test_sample_metadata_failure_preserves_raw_under_budget(ab,kind):
    obs=ab.factory();_,row=next_sample(ab,obs)
    row['bad'] = object() if kind=='unserializable' else ('x'*(130*1024) if kind=='huge' else float('nan'))
    obs.queue.put_nowait(row);obs.shutdown();obs._writer()
    meta=json.loads((obs.output/'sample_01.json').read_text())
    assert meta['context_incomplete'] and not meta['complete']
    with np.load(obs.output/'sample_01.npz') as arrays:assert np.all(arrays['raw_depth']==2400)
    assert not json.loads((obs.output/'summary.json').read_text())['experiment_valid']
    assert sum(p.stat().st_size for p in obs.output.iterdir())<obs.maximum_bytes


def test_finalize_deepcopy_error_keeps_raw_and_original_result(ab):
    class Bad:
        def __deepcopy__(self,memo):raise ValueError('bad camera context')
    obs=ab.factory();step(ab,obs)
    ab.node.cfg['unavailable']=Bad()
    result,row=next_sample(ab,obs)
    assert result is ab.cam.last_result and obs.closed and row['context_incomplete']
    obs.queue.put_nowait(row);obs._writer()
    assert (obs.output/'sample_01.npz').exists()
    assert not json.loads((obs.output/'summary.json').read_text())['experiment_valid']


def test_budget_failure_writes_no_pair_outside_cap(ab):
    obs=ab.factory();_,row=next_sample(ab,obs)
    obs.queue.put_nowait(row);obs.shutdown()
    obs.maximum_bytes=module.MIB
    row['arrays']['raw_depth']=np.ones((480,640),np.uint16)
    row['arrays']['raw_rgb']=np.ones((480,640,3),np.uint8)
    obs._writer()
    assert not (obs.output/'sample_01.npz').exists()
    summary=json.loads((obs.output/'summary.json').read_text())
    assert summary['counts']['write_errors']==1 and not summary['experiment_valid']
    assert sum(p.stat().st_size for p in obs.output.iterdir())<obs.maximum_bytes


def test_thread_start_failure_does_not_leave_hooks(ab,monkeypatch):
    monkeypatch.setattr(module.threading.Thread,'start',lambda self:(_ for _ in ()).throw(RuntimeError('thread failure')))
    with pytest.raises(RuntimeError,match='thread failure'):ab.factory(start_thread=True)
    assert 'read' not in ab.cam.__dict__ and '_sync_depth_preset' not in ab.node.__dict__


def test_full_parameter_readback_mismatch_invalidates_restore(ab):
    obs=ab.factory();step(ab,obs)
    original=ab.cam.set_depth_preset_mode
    def corrupt(value):
        changed=original(value)
        if value=='unknown':ab.advanced.parameters['unexpected']='modified'
        return changed
    ab.cam.set_depth_preset_mode=corrupt
    obs.finish('stop');step(ab,obs)
    assert not obs.restore['succeeded'] and 'restoration mismatch' in obs.restore['error']


def test_guard_rejects_missing_before_epoch_and_stale_feedback():
    g=module.EvidenceGuard();g.observe('epoch',10**9,None)
    g.observe('sdk',10**9,[0.]*6);g.observe('task',10**9,{'active':False})
    assert not g.check(10**9,freeze=True)
    g.observe('accepted',10**9-1,[0.]*7)
    with pytest.raises(ValueError,match='before_epoch'):g.check(10**9,freeze=True)
    g=module.EvidenceGuard()
    for k,v in [('sdk',[0.]*6),('accepted',[0.]*7),('task',{'active':False}),('epoch',None)]:g.observe(k,10**9,v)
    assert g.check(10**9,freeze=True)
    with pytest.raises(ValueError,match='stale'):g.check(1_500_000_001)


def joint_message(kind='accepted',names=None,frame=None,positions=None):
    names=names or list(module.ARM)+( ['right_finger'] if kind=='accepted' else [])
    return SimpleNamespace(name=names,position=positions or [0.]*len(names),
        header=SimpleNamespace(frame_id=frame or ('sdk_measured' if kind=='accepted' else 'sdk_transmitted'),
                               stamp=SimpleNamespace(to_nsec=lambda:10**9)))


@pytest.mark.parametrize('message',[joint_message(frame='commanded'),
    joint_message(names=list(module.ARM)),joint_message(positions=[0.]*6+[float('nan')]),
    joint_message(names=['Joint1']*7)])
def test_actual_ros_joint_parser_rejects_invalid_evidence(message):
    obj=module.RosEvidence.__new__(module.RosEvidence);obj.guard=module.EvidenceGuard()
    obj.joints('accepted',message)
    assert obj.guard.faults and obj.guard.current['accepted'] is None


def test_actual_ros_joint_parser_normalizes_order():
    obj=module.RosEvidence.__new__(module.RosEvidence);obj.guard=module.EvidenceGuard()
    msg=joint_message(names=list(reversed(module.ARM+('right_finger',))),positions=list(reversed([0.,1.,2.,3.,4.,5.,6.])))
    obj.joints('accepted',msg)
    assert obj.guard.current['accepted']['q']==[0.,1.,2.,3.,4.,5.,6.]


def test_selected_original_failure_retains_completed_raw_stages(ab):
    obs=ab.factory();step(ab,obs)
    for _ in range(10):step(ab,obs,seconds=.1)
    ab.cam.raise_original=True
    ab.clock.step(3);ab.evidence.tick()
    with pytest.raises(RuntimeError,match='production failure'):ab.cam.read()
    row=obs.queue.get_nowait()
    assert not row['complete'] and row['anchor_ros_stamp_ns'] is None
    assert set(row['arrays'])=={'raw_depth','raw_rgb','aligned_depth'}
    assert obs.closed and obs.restore['succeeded']
    obs.queue.put_nowait(row);obs._writer()
    assert (obs.output/'sample_01.npz').exists()
    assert not json.loads((obs.output/'summary.json').read_text())['experiment_valid']


@pytest.mark.parametrize('bad',['stamp','parent','child','nan','rotation','missing',None])
def test_exact_anchor_tf_is_not_latest_or_fabricated(bad):
    stamp=10**9
    obj=module.RosEvidence.__new__(module.RosEvidence)
    obj.parent='base_link';obj.child='camera_link'
    obj.guard=SimpleNamespace(bracket=lambda _:dict(qualified=True))
    obj.rospy=SimpleNamespace(Time=lambda s,n:SimpleNamespace(sec=s,nsec=n),Duration=lambda s:s)
    transform=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_nsec=lambda:stamp+(bad=='stamp')),
                          frame_id='wrong' if bad=='parent' else obj.parent),
        child_frame_id='wrong' if bad=='child' else obj.child,
        transform=SimpleNamespace(translation=SimpleNamespace(x=float('nan') if bad=='nan' else .1,y=.2,z=.3),
            rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=0. if bad=='rotation' else 1.)))
    requests=[]
    def lookup(parent,child,t,timeout):
        requests.append((parent,child,t.sec,t.nsec,timeout))
        if bad=='missing':raise RuntimeError('no exact TF')
        return transform
    obj.buffer=SimpleNamespace(lookup_transform=lookup)
    value=obj.at_anchor(stamp)
    assert value['evidence_complete']==(bad is None)
    assert requests==[('base_link','camera_link',1,0,.35)]
    if bad is not None:assert value['exact_tf'] is None and value['tf_error']
    assert value['tf_stamp_audit'] == dict(
        requested_stamp_ns=stamp,
        returned_stamp_ns=None if bad=='missing' else stamp+(bad=='stamp'),
        delta_ns=None if bad=='missing' else int(bad=='stamp'),
        exact_match=bad not in ('missing','stamp'))


@pytest.mark.parametrize('delta_ns', [-1_000_000, -119, -1, 0, 1, 118, 1_000_000])
def test_tf_stamp_audit_never_promotes_a_nonexact_transform(delta_ns):
    stamp=1791125615215571641
    obj=module.RosEvidence.__new__(module.RosEvidence)
    obj.parent='base_link';obj.child='camera_link'
    obj.guard=SimpleNamespace(bracket=lambda _:dict(qualified=True))
    obj.rospy=SimpleNamespace(Time=lambda s,n:(s,n),Duration=lambda s:s)
    transform=SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(to_nsec=lambda:stamp+delta_ns),frame_id=obj.parent),
        child_frame_id=obj.child,
        transform=SimpleNamespace(translation=SimpleNamespace(x=.1,y=.2,z=.3),
                                  rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
    obj.buffer=SimpleNamespace(lookup_transform=lambda *args:transform)
    value=obj.at_anchor(stamp)
    assert value['tf_stamp_audit'] == dict(requested_stamp_ns=stamp,
        returned_stamp_ns=stamp+delta_ns,delta_ns=delta_ns,exact_match=delta_ns==0)
    assert value['evidence_complete'] == (delta_ns==0)
    if delta_ns:
        assert value['exact_tf'] is None
        assert value['tf_error']=='TF timestamp differs from exact requested anchor'
    else:
        assert value['exact_tf']['source_stamp_ns']==stamp


@pytest.mark.parametrize('stamp', [1791125615215571641,1791125615123456789,1791125615999999999])
def test_actual_ros_binding_roundtrip_is_audited_without_network(monkeypatch,stamp):
    rospy=pytest.importorskip('rospy')
    tf2_ros=pytest.importorskip('tf2_ros')
    geometry=pytest.importorskip('geometry_msgs.msg')
    import socket
    def no_socket(*args,**kwargs):
        raise AssertionError('pure-memory TF diagnostic must not contact a ROS master')
    monkeypatch.setattr(socket,'socket',no_socket)
    buffer=tf2_ros.Buffer(cache_time=rospy.Duration(10),debug=False)
    for i,ns in enumerate((1791125615000000000,1791125616000000000)):
        msg=geometry.TransformStamped()
        msg.header.stamp=rospy.Time(ns//10**9,ns%10**9)
        msg.header.frame_id='base_link';msg.child_frame_id='camera_link'
        msg.transform.translation.x=float(i);msg.transform.rotation.w=1.
        buffer.set_transform(msg,'offline_test')
    request=rospy.Time(stamp//10**9,stamp%10**9)
    actual=buffer.lookup_transform_core('base_link','camera_link',request)
    obj=module.RosEvidence.__new__(module.RosEvidence)
    obj.parent='base_link';obj.child='camera_link';obj.rospy=rospy
    obj.guard=SimpleNamespace(bracket=lambda _:dict(qualified=True))
    # Exercise the real C++ binding without rospy polling clocks or a listener.
    obj.buffer=SimpleNamespace(lookup_transform=lambda p,c,t,timeout:buffer.lookup_transform_core(p,c,t))
    value=obj.at_anchor(stamp)
    returned=actual.header.stamp.to_nsec()
    assert request.to_nsec()==stamp
    assert value['tf_stamp_audit']==dict(requested_stamp_ns=stamp,returned_stamp_ns=returned,
                                       delta_ns=returned-stamp,exact_match=returned==stamp)
    assert value['evidence_complete']==(returned==stamp)
    if returned!=stamp:
        assert value['exact_tf'] is None
        assert value['tf_error']=='TF timestamp differs from exact requested anchor'


def test_sdk_source_and_host_anchor_stay_explicitly_distinct(ab):
    obs=ab.factory();_,row=next_sample(ab,obs)
    assert row['sdk_frames']['depth']['timestamp_ms'] != row['anchor_ros_stamp_ns']/1e6
    assert 'not exposure' in row['timestamp_semantics']
    assert 'source_stamp_ns' not in row
    assert 'not calibrated' in obs.summary()['clock_scope']


def test_stale_sdk_with_fresh_accepted_cannot_validate_experiment():
    g=module.EvidenceGuard();ns=10**9
    for k,v in [('sdk',[0.]*6),('accepted',[0.]*7),('task',{'active':False}),('epoch',None)]:g.observe(k,ns,v)
    assert g.check(ns,freeze=True)
    g.observe('accepted',ns+1_000_000_001,[0.]*7)
    with pytest.raises(ValueError,match='sdk_command_stale'):g.check(ns+1_000_000_001)
    assert g.faults==['sdk_command_stale']
    # A stale SDK reference before capture cannot be frozen either.
    g=module.EvidenceGuard()
    for k,v in [('sdk',[0.]*6),('accepted',[0.]*7),('task',{'active':False}),('epoch',None)]:g.observe(k,ns,v)
    g.observe('accepted',ns+1_000_000_001,[0.]*7)
    assert not g.check(ns+1_000_000_001,freeze=True) and g.frozen is None


def test_first_fsync_failure_after_npz_bytes_stops_all_further_pair_writes(ab,monkeypatch):
    obs=ab.factory(samples_per_phase=2)
    _,first=next_sample(ab,obs);_,second=next_sample(ab,obs)
    obs.queue.put_nowait(first);obs.queue.put_nowait(second);obs.shutdown()
    real=module.os.fsync
    calls=[]
    def fail_first(fd):
        calls.append(fd)
        if len(calls)==1:raise OSError('fsync failed after bytes were written')
        return real(fd)
    monkeypatch.setattr(module.os,'fsync',fail_first)
    obs._writer()
    assert (obs.output/'sample_01.npz').stat().st_size>0
    assert not (obs.output/'sample_01.json').exists()
    assert not (obs.output/'sample_02.npz').exists()
    summary=json.loads((obs.output/'summary.json').read_text())
    assert summary['counts']['write_errors']==1 and summary['counts']['queue_dropped']==1
    assert summary['actual_artifact_bytes_before_summary']>summary['written_bytes_before_summary']
    assert not summary['experiment_valid']
    assert sum(p.stat().st_size for p in obs.output.iterdir())<=obs.maximum_bytes


def test_lost_owner_never_publishes_recovery_metadata(ab):
    obs=ab.factory();step(ab,obs)
    before=len(ab.node.publications)
    ab.node.cam=SimpleNamespace()
    obs.finish('owner_changed');obs.shutdown()
    assert not obs.restore['succeeded']
    assert obs.restore['metadata_publication_skipped']=='camera_owner_changed'
    assert len(ab.node.publications)==before


def test_context_rpc_deadline_does_not_admit_new_sample(ab):
    obs=ab.factory();step(ab,obs)
    for _ in range(10):step(ab,obs,seconds=.1)
    def delayed_context():
        ab.clock.value=76.;ab.evidence.tick();obs.finish('deadline')
    ab.evidence.check_context=delayed_context
    ab.clock.step(3);ab.evidence.tick()
    result=ab.cam.read()
    assert result is ab.cam.last_result and obs.closed and obs.restore['succeeded']
    assert obs.counts['selected']==0 and not obs.queue.qsize()


def test_phase_context_rpc_deadline_does_not_start_new_baseline(ab):
    obs=ab.factory();next_sample(ab,obs)
    writes=[];original=ab.cam.set_depth_preset_mode
    def record(mode):writes.append((mode,obs.restore['attempted']));return original(mode)
    ab.cam.set_depth_preset_mode=record
    def delayed_context():
        ab.clock.value=76.;ab.evidence.tick();obs.finish('deadline')
    ab.evidence.check_context=delayed_context
    ab.clock.step(.1);ab.evidence.tick();ab.cam.read()
    assert obs.closed and [p['label'] for p in obs.phases]==['A1']
    assert writes==[('carton',True),('unknown',True)]  # Only finally restoration.


def test_deadline_passthrough_original_failure_not_retried(ab):
    obs=ab.factory();step(ab,obs);obs.finish('deadline')
    before=ab.cam.calls;ab.cam.raise_original=True
    with pytest.raises(RuntimeError,match='production failure'):ab.cam.read()
    assert ab.cam.calls==before+1 and obs.restore['succeeded']


def test_rectified_final_shape_has_independent_native_geometry(ab):
    obs=ab.factory();_,row=next_sample(ab,obs)
    row['arrays']['final_depth']=np.ones((11,12),np.uint16)
    row['arrays']['final_rgb']=np.ones((11,12,3),np.uint8)
    k={'height':11,'width':12}
    row['runtime_profile']['geometry_intrinsics']=k
    row['published_camera_parameters']['runtime_profile']['geometry_intrinsics']=copy.deepcopy(k)
    module.validate_sample_geometry(row)
    assert row['arrays']['raw_depth'].shape==(8,8)
    assert row['sdk_frames']['depth']['intrinsics']['width']==8
    row['published_camera_parameters']['runtime_profile']['geometry_intrinsics']['width']=13
    with pytest.raises(ValueError,match='geometry differ'):module.validate_sample_geometry(row)


def test_final_geometry_mismatch_preserves_raw_and_invalidates(ab):
    obs=ab.factory();step(ab,obs)
    ab.node.cfg['runtime_profile']['geometry_intrinsics']['width']=10
    result,row=next_sample(ab,obs)
    assert result is ab.cam.last_result and not row['complete'] and row['context_incomplete']
    assert obs.closed and obs.restore['succeeded']
    assert 'geometry differ' in row['metadata_error']


def camera_source_fixture(path):
    path.write_text('''from types import SimpleNamespace
rospy=SimpleNamespace(initialized=False)
class CameraNode:
    def __init__(self):
        raise AssertionError("Definition loader must not construct a camera")
if __name__ == '__main__':
    raise AssertionError("Definition loader must not run the camera main block")
''')
    return path


def relay_fixture(path,source):
    path.write_text("python_script = %r\n" % str(source)+'''with open(python_script, 'r') as fh:
    context = {'__builtins__': __builtins__, '__doc__': None,
               '__file__': python_script, '__name__': __name__, '__package__': None}
    exec(compile(fh.read(), python_script, 'exec'), context)
''')
    return path


def test_definition_loader_accepts_direct_source_without_constructing_camera(tmp_path):
    source=camera_source_fixture(tmp_path/'camera.py')
    namespace,actual=module.load_camera_definition(source)
    assert actual==source and namespace['CameraNode'].__name__=='CameraNode'
    assert namespace['rospy'].initialized is False


def test_definition_loader_accepts_realistic_relay_and_keeps_actual_source(tmp_path):
    source=camera_source_fixture(tmp_path/'source.py')
    relay=relay_fixture(tmp_path/'devel_camera.py',source)
    namespace,actual=module.load_camera_definition(relay)
    assert actual==source and namespace['__file__']==str(source)
    assert namespace['CameraNode'].__init__.__globals__ is namespace
    assert 'python_script' not in namespace


def test_actual_workspace_catkin_devel_relay_offline_without_ros_or_sdk_calls(monkeypatch):
    import sys
    source=WORK/'src/alicia_flexible_grasp_supervisor/scripts/camera_node.py'
    relay=WORK/'devel/lib/alicia_flexible_grasp_supervisor/camera_node.py'
    assert 'generated from catkin/cmake/template/script.py.in' in relay.read_text()
    monkeypatch.syspath_prepend(str(WORK/'src/alicia_flexible_grasp_supervisor/src'))
    calls=[]
    def forbidden(*a,**kw):
        calls.append((a,kw));raise AssertionError('offline definition load attempted ROS/SDK I/O')
    rospy=SimpleNamespace(init_node=forbidden,Publisher=forbidden,Subscriber=forbidden,
        ServiceProxy=forbidden,get_param=forbidden,get_param_cached=forbidden,set_param=forbidden)
    monkeypatch.setitem(sys.modules,'rospy',rospy)
    monkeypatch.setitem(sys.modules,'pyrealsense2',SimpleNamespace(pipeline=forbidden,context=forbidden))
    namespace,actual=module.load_camera_definition(relay)
    assert actual==source and namespace['rospy'] is rospy
    assert namespace['CameraNode'].__init__.__globals__['__file__']==str(source)
    assert calls==[]


def test_relay_definition_loader_resolves_relative_cli_path(tmp_path,monkeypatch):
    source=camera_source_fixture(tmp_path/'source.py')
    relay=relay_fixture(tmp_path/'relay.py',source)
    monkeypatch.chdir(tmp_path)
    namespace,actual=module.load_camera_definition(Path('relay.py'))
    assert actual==source and namespace['CameraNode'].__name__=='CameraNode'


@pytest.mark.parametrize('corruption',['target','context_file','class_source','no_context'])
def test_malformed_relay_context_refused_without_main_execution(tmp_path,monkeypatch,corruption):
    source=camera_source_fixture(tmp_path/'source.py')
    relay=relay_fixture(tmp_path/'relay.py',source)
    if corruption=='target':relay.write_text(relay.read_text()+"python_script = '/tmp/unrelated_camera.py'\n")
    elif corruption=='context_file':relay.write_text(relay.read_text()+"context['__file__']='/tmp/unrelated_camera.py'\n")
    elif corruption=='class_source':
        # A class imported from another file must not be relabelled as this source.
        relay.write_text(relay.read_text()+"context['CameraNode']=type('CameraNode', (), {})\n")
    else:relay.write_text('python_script="irrelevant"\n')
    with pytest.raises(ValueError,match='camera|catkin'):module.load_camera_definition(relay)


@pytest.fixture
def shadow_setup(ab):
    from collections import deque
    from types import MethodType
    from alicia_flexible_grasp.vision.stationary_depth import StationaryDepthWindow
    import cv2
    cv2.setNumThreads(1)
    cam=ab.cam
    cam.runtime_profile['mode_depth_preset']=copy.deepcopy(ab.policy.evidence())
    size=96
    model=dict(width=size,height=size,fx=80.,fy=80.,cx=48.,cy=48.)
    for key in ('geometry_intrinsics','native_depth_intrinsics','color_intrinsics'):
        cam.runtime_profile[key]=copy.deepcopy(model)
    ab.node.cfg['runtime_profile']['geometry_intrinsics']=copy.deepcopy(model)
    class Profile(old.Profile):
        def get_intrinsics(self):
            return SimpleNamespace(width=size,height=size,fx=80.,fy=80.,ppx=48.,ppy=48.,
                                   model='inverse_brown_conrady',coeffs=[.01,0,0,0,0])
    def frame(data):
        value=old.Frame(data);value.get_profile=lambda:Profile();return value
    class Queue:
        def __init__(self,capacity):self.frames=deque(maxlen=capacity)
        def enqueue(self,f):self.frames.append(f)
        def poll_for_frame(self):return self.frames.popleft() if self.frames else None
    class Block:
        def __init__(self,callback):self.callback=callback
        def start(self,callback):self.ready=callback
        def invoke(self,f):
            def allocate(profile,original,bpp,w,h,stride,kind):
                assert bpp==2 and stride==2*w and kind=='depth_frame'
                return frame(np.empty((h,w),np.uint16))
            self.callback(f,SimpleNamespace(allocate_video_frame=allocate,frame_ready=self.ready))
    created=[]
    class Temporal:
        def __init__(self):self.options={};self.previous=None;created.append(self)
        def set_option(self,k,v):self.options[k]=v
        def process(self,f):
            source=f.data.copy()
            if getattr(cam,'shadow_sdk_raises',False):raise RuntimeError('shadow filter failed')
            if self.previous is None:result=source.copy()
            else:
                result=source.copy();valid=(source>0)&(self.previous>0)&(abs(source.astype(int)-self.previous.astype(int))<=100)
                result[valid]=(.4*source[valid]+.6*self.previous[valid]).astype(np.uint16)
            self.previous=result.copy()
            if getattr(cam,'shadow_mutates_private',False):f.data[:]=999
            if getattr(cam,'shadow_fills_holes',False):result[source==0]=1
            return frame(result)
    cam.rs.frame_queue=Queue;cam.rs.processing_block=Block
    cam.rs.extension=SimpleNamespace(depth_frame='depth_frame');cam.rs.temporal_filter=Temporal
    for key in ('filter_smooth_alpha','filter_smooth_delta','holes_fill'):setattr(cam.rs.option,key,key)
    cam.mode_depth_preset_cfg=dict(stationary_temporal_enabled=True)
    ab.node._stationary_window=StationaryDepthWindow()
    def forbidden_provider_call():raise AssertionError('use exact copied real window, not a racing second provider call')
    cam.stationary_depth_provider=forbidden_provider_call
    rgb=np.random.RandomState(31).randint(0,256,(size,size,3),dtype=np.uint8)
    cam.pipeline.frames=old.Frames(np.full((size,size),2400,np.uint16),rgb)
    original_wait=cam.pipeline.wait_for_frames
    def wait():
        frames=original_wait()
        for f in (frames.depth,frames.color):f.get_profile=lambda:Profile()
        return frames
    cam.pipeline.wait_for_frames=wait
    def aligned(frames):
        result=old.Frames(frames.depth.data+np.uint16(10),frames.color.data)
        result.depth.get_profile=lambda:Profile();result.color.get_profile=lambda:Profile()
        return result
    cam.align.process=aligned
    def production_stationary(self,f,diagnostic=None,**kwargs):return frame(f.data+np.uint16(30))
    cam._filter_stationary_depth=MethodType(production_stationary,cam)
    def read(self):
        self.calls+=1
        frames=self.pipeline.wait_for_frames()
        number=self.pipeline.calls
        raw={k:dict(present=True,frame_number=number,device_timestamp_ms=number*1000./30,
                    timestamp_domain='global_time',stream_fps=30.) for k in ('depth','color')}
        if getattr(self,'depth_source_skip',False):raw['depth']['frame_number']+=3
        diagnostic=dict(result='error',pair_timing={'accepted':True},raw_frames=raw)
        if not self.complete:
            diagnostic.update(result='rejected_pair',pair_timing={'accepted':False})
            self.last_read_diagnostics=diagnostic;self.last_result=(None,None);return self.last_result
        frames=self.align.process(frames)
        if self.raise_original:raise RuntimeError('production failure')
        depth=frames.get_depth_frame()
        for filt in self.depth_filters:depth=filt.process(depth)
        depth.get_profile=lambda:Profile()
        depth.data[0,0]=0
        result=self._filter_stationary_depth(depth,diagnostic,color=frames.color.data,source=raw['color'])
        self.production_input=depth.data.copy()
        diagnostic['result']='complete';self.last_read_diagnostics=diagnostic
        self.last_result=(frames.color.data.copy(),result.data.copy());return self.last_result
    cam.read=MethodType(read,cam)
    original=cam.read
    def tick(seconds=1./30):
        ab.clock.step(seconds);ab.evidence.tick();ns=ab.evidence.now_ns()
        for kind in ('sdk','accepted'):
            names=module.ARM+(('right_finger',) if kind=='accepted' else ())
            q=[0.]*len(names)
            if getattr(cam,'encoder_moves',False):q[0]=ab.clock.value
            message=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_nsec=lambda:ns),
                frame_id='sdk_measured' if kind=='accepted' else 'sdk_transmitted'),name=names,position=q)
            ab.node._stationary_window.observe(kind,message)
        ab.node._sync_depth_preset()
        result=cam.read();rows=[]
        while hasattr(state,'obs') and not state.obs.queue.empty():rows.append(state.obs.queue.get_nowait())
        return result,rows
    state=SimpleNamespace(ab=ab,cam=cam,node=ab.node,tick=tick,created=created,original=original,
                          frame=frame,rgb=rgb)
    return state


def shadow_observer(state,**kwargs):
    state.obs=state.ab.factory(matched_stationary_shadow=True,**kwargs)
    return state.obs


def test_shadow_requires_explicit_diagnostic_and_defaults_off():
    with pytest.raises(SystemExit):module.arguments(['--camera-script','camera.py','--matched-stationary-shadow'])
    args=module.arguments(['--camera-script','camera.py','--enable-preset-ab','--external-consumers-quiesced',
                          '--output','/tmp/not-executed'])
    assert not args.matched_stationary_shadow


def test_shadow_budget_includes_every_complete_full_resolution_array_and_reduces_count():
    profile=dict(native_depth_intrinsics=dict(width=640,height=480),color_intrinsics=dict(width=640,height=480),
                 geometry_intrinsics=dict(width=653,height=489))
    actual=module.shadow_capture_budget(profile,8,module.MAX_BYTES)
    assert actual['samples_per_phase']==8 and actual['maximum_total_bytes']<module.MAX_BYTES
    assert actual['maximum_pair_bytes']<8*module.MIB
    small=module.shadow_capture_budget(profile,8,70_000_000)
    assert small['samples_per_phase']<8 and small['maximum_total_bytes']<=70_000_000
    with pytest.raises(ValueError):module.shadow_capture_budget(profile,8,module.MIB)
    profile['geometry_intrinsics']=dict(width=4096,height=4096)
    with pytest.raises(ValueError,match='MiB'):module.shadow_capture_budget(profile,8,module.MAX_BYTES)


def test_matched_shadow_runs_each_real_read_in_all_modes_and_never_replaces_output(shadow_setup):
    s=shadow_setup;obs=shadow_observer(s);rows=[]
    for _ in range(420):
        result,new=s.tick();rows.extend(new)
        assert result is s.cam.last_result
        if obs.closed:break
    assert [r['phase'] for r in rows]==['A1','B','A2']
    assert obs.closed and obs.restore['succeeded'] and s.cam.read==s.original
    for row in rows:
        audit=row['matched_stationary_shadow']
        assert audit['active'] and audit['encoder']['qualified'] and audit['visual']['qualified']
        assert row['complete'] and audit['phase']['reads']>=90
        assert audit['encoder']['source_window_sha256']==module.digest(audit['encoder']['source_window'])
        assert audit['color_source']==row['production_read_diagnostics']['raw_frames']['color']
        assert row['arrays']['matched_shadow_depth'][1,1]==row['arrays']['pre_stationary_depth'][1,1]
        assert row['arrays']['final_depth'][1,1]==row['arrays']['matched_shadow_depth'][1,1]+30
        assert row['runtime_profile']['mode_depth_preset']['mode']==('carton' if row['phase']=='B' else 'unknown')
    assert len({r['matched_stationary_shadow']['history_id'] for r in rows})==3
    assert all(t.options=={'filter_smooth_alpha':.4,'filter_smooth_delta':100.,'holes_fill':0.} for t in s.created)
    summary=obs.summary()['matched_stationary_shadow']
    assert len(summary['phases'])==3 and sum(p['reads'] for p in summary['phases'])>250
    assert not summary['output_replaced'] and not summary['qualifies_old_experiment']


@pytest.mark.parametrize('fault',['pair','depth_gap','encoder','visual_motion'])
def test_shadow_revocation_resets_history_and_requires_new_real_stationarity(shadow_setup,fault):
    s=shadow_setup;obs=shadow_observer(s,samples_per_phase=8)
    for _ in range(40):s.tick()
    assert obs.shadow.last['active'];old_history=obs.shadow.history_id
    if fault=='pair':s.cam.complete=False
    elif fault=='depth_gap':s.cam.depth_source_skip=True
    elif fault=='encoder':s.cam.encoder_moves=True
    else:s.cam.pipeline.frames.color.data=np.roll(s.rgb,4,axis=1)
    s.tick()
    assert obs.shadow.temporal is None
    s.cam.complete=True;s.cam.depth_source_skip=False;s.cam.encoder_moves=False
    s.cam.pipeline.frames.color.data=s.rgb
    active=[]
    for _ in range(42):
        s.tick()
        if obs.shadow.last and obs.shadow.last['active']:active.append(copy.deepcopy(obs.shadow.last))
    assert active and active[0]['first_history_frame'] and active[0]['history_id']>old_history


@pytest.mark.parametrize('fault',['shadow_sdk_raises','shadow_fills_holes'])
def test_shadow_failure_isolated_from_camera_read_and_restores_hooks(shadow_setup,fault):
    s=shadow_setup;obs=shadow_observer(s)
    setattr(s.cam,fault,True)
    for _ in range(45):
        result,_=s.tick()
        assert result is s.cam.last_result
        if obs.closed:break
    assert obs.closed and obs.close_reason=='shadow_error' and obs.restore['succeeded']
    assert s.cam.read==s.original and obs.counts['original_read_errors']==0
    assert s.cam._filter_stationary_depth.__func__.__name__=='production_stationary'


def test_shadow_private_copy_isolates_even_a_filter_that_writes_its_input(shadow_setup):
    s=shadow_setup;obs=shadow_observer(s)
    s.cam.shadow_mutates_private=True
    for _ in range(45):s.tick()
    assert not obs.closed and obs.shadow.last['active']
    assert s.cam.production_input[1,1]==2430 and s.cam.last_result[1][1,1]==2460
    assert obs.shadow.temporal.previous[1,1]==2430


def test_shadow_original_read_exception_still_propagates_and_resets(shadow_setup):
    s=shadow_setup;obs=shadow_observer(s)
    for _ in range(40):s.tick()
    s.cam.raise_original=True
    with pytest.raises(RuntimeError,match='production failure'):s.tick()
    assert obs.closed and obs.shadow.temporal is None and obs.counts['original_read_errors']==1


def test_shadow_encoder_snapshot_is_real_bounded_and_not_replaced_by_later_feedback(shadow_setup):
    s=shadow_setup;obs=shadow_observer(s)
    for _ in range(40):s.tick()
    record=copy.deepcopy(obs.shadow.last['encoder'])
    assert record['qualified'] and all(len(rows)<=24 for rows in record['source_window'].values())
    s.cam.encoder_moves=True;s.tick()
    assert not obs.shadow.last['encoder']['qualified']
    assert module.digest(record['source_window'])==record['source_window_sha256']
    assert record['source_window']!=obs.shadow.last['encoder']['source_window']


def test_shadow_evidence_owner_change_does_not_prevent_restoring_original_camera(shadow_setup):
    s=shadow_setup;obs=shadow_observer(s)
    for _ in range(110):s.tick()
    assert obs.phase==1 and s.ab.policy.mode=='carton'
    s.cam.stationary_depth_provider=lambda:False
    s.tick()
    assert obs.closed and obs.close_reason=='shadow_evidence_owner_changed'
    assert obs.restore['succeeded'] and s.ab.policy.mode=='unknown'


def test_selected_shadow_array_and_evidence_survive_bounded_writer(shadow_setup):
    s=shadow_setup;obs=shadow_observer(s);row=None
    for _ in range(110):
        _,rows=s.tick()
        if rows:row=rows[0];break
    assert row is not None and row['matched_stationary_shadow']['active']
    obs.queue.put_nowait(row);obs.shutdown();obs._writer()
    with np.load(obs.output/'sample_01.npz',allow_pickle=False) as z:
        assert 'matched_shadow_depth' in z and z['matched_shadow_depth'].shape==(96,96)
    saved=json.loads((obs.output/'sample_01.json').read_text())
    assert saved['matched_stationary_shadow']['encoder']['source_window']
    assert not saved['matched_stationary_shadow']['geometry_authority']
    assert sum(p.stat().st_size for p in obs.output.iterdir())<obs.maximum_bytes


@pytest.mark.skipif(module.os.environ.get('ALICIA_OFFLINE_SDK_COPY_TEST')!='1',
                    reason='Explicit opt-in for isolated SDK software-frame backend initialization')
def test_actual_sdk_private_copy_preserves_frame_and_temporal_results(monkeypatch):
    # Software-only source: no physical context enumeration or pipeline. The
    # SDK backend may initialize udev internally, hence this separate opt-in.
    monkeypatch.syspath_prepend(str(WORK/'.runtime/realsense-2.56.5'))
    import pyrealsense2 as rs
    assert rs.__version__=='2.56.5'
    def forbidden(*args,**kwargs):raise AssertionError('no physical SDK entry is permitted')
    monkeypatch.setattr(rs,'context',forbidden);monkeypatch.setattr(rs,'pipeline',forbidden)
    device=rs.software_device();sensor=device.add_sensor('isolated_memory_test')
    stream=rs.video_stream();stream.type=rs.stream.depth;stream.index=0;stream.uid=1
    stream.width=16;stream.height=12;stream.fps=30;stream.bpp=2;stream.fmt=rs.format.z16
    k=rs.intrinsics();k.width=16;k.height=12;k.fx=k.fy=10;k.ppx=8;k.ppy=6
    k.model=rs.distortion.none;k.coeffs=[0.]*5;stream.intrinsics=k
    profile=sensor.add_video_stream(stream).as_video_stream_profile()
    sensor.add_read_only_option(rs.option.depth_units,.0001)
    queue=rs.frame_queue(1);sensor.open(profile);sensor.start(queue)
    private=module.PrivateDepthCopy(rs);temporal=rs.temporal_filter()
    for option,value in ((rs.option.filter_smooth_alpha,.4),(rs.option.filter_smooth_delta,100.),(rs.option.holes_fill,0.)):
        temporal.set_option(option,value)
    try:
        for i in range(2):
            data=np.full((12,16),2000+i*50,np.uint16);data[0,0]=0
            f=rs.software_video_frame();f.pixels=data;f.bpp=2;f.stride=32
            f.timestamp=1000+i*1000./30;f.domain=rs.timestamp_domain.hardware_clock
            f.frame_number=100+i;f.profile=profile;f.depth_units=.0001
            sensor.on_video_frame(f);original=queue.wait_for_frame(1000)
            copied=private.copy(original)
            original_data=np.asanyarray(original.get_data());copied_data=np.asanyarray(copied.get_data())
            assert not np.shares_memory(original_data,copied_data)
            np.testing.assert_array_equal(original_data,copied_data)
            assert copied.get_frame_number()==original.get_frame_number()
            assert copied.get_timestamp()==original.get_timestamp()
            assert copied.as_depth_frame().get_units()==original.as_depth_frame().get_units()
            filtered=np.asanyarray(temporal.process(copied).get_data()).copy()
            assert filtered[0,0]==0 and np.all(filtered[1:]==(2000 if i==0 else 2020))
            copied_data[1,1]=777
            assert original_data[1,1]==2000+i*50
    finally:sensor.stop();sensor.close()
