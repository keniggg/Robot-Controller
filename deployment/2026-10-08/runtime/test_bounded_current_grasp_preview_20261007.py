import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

P = Path(__file__).with_name('capture_bounded_current_grasp_preview_20261007.py')
spec = importlib.util.spec_from_file_location('bounded_preview', P)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
SEL = dict(mode='unknown', strategy='observe_return', generation=21, stamp_ns=100)


class Stamp:
    def __init__(self, ns): self.ns = ns
    def to_nsec(self): return self.ns


class Header:
    __slots__ = ('stamp',)
    def __init__(self, ns): self.stamp = Stamp(ns)


class String:
    __slots__ = ('data',)
    _type = 'std_msgs/String'
    def __init__(self, data): self.data = data
    def serialize(self, stream): stream.write(self.data.encode())


class Plan:
    __slots__ = ('header', 'valid', 'plan_id', 'candidate_source', 'diagnostic')
    _type = 'test/Plan'
    def __init__(self, ns=300, valid=True, plan_id='p', source='tabletop_geometry'):
        self.header = Header(ns); self.valid = valid; self.plan_id = plan_id
        self.candidate_source = source; self.diagnostic = 'original diagnostic'
    def serialize(self, stream):
        stream.write(json.dumps(m.message_value(self)).encode())


class Clock:
    def __init__(self): self.now = 1.
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


def capture(tmp_path, monkeypatch, **kwargs):
    monkeypatch.setattr(m, 'EXT', tmp_path)
    out = tmp_path / 'output'; out.mkdir()
    return m.Capture(out, SEL, **kwargs)


def audit(tmp_path, **changes):
    report = dict(grasp_mode_selection=SEL, snapshot_stamp_ns=300, plan_id='p',
                  outcome={'preview_valid': True}, request_id=9, generation=1, target_epoch=7)
    report.update(changes)
    raw = json.dumps(report).encode()
    p = tmp_path / 'original.json'; p.write_bytes(raw)
    return dict(report_path=str(p), report_sha256=m.sha(raw))


def enqueue_pair(c, tmp_path):
    ref = audit(tmp_path)
    c.enqueue('gate_audit', String(json.dumps(ref)), 400, 2.)
    c.enqueue('preview', Plan(), 410, 2.01)


def test_exact_original_audit_and_plan_pair(tmp_path, monkeypatch):
    c = capture(tmp_path, monkeypatch); c.start_ns = 200
    enqueue_pair(c, tmp_path); c.drain()
    assert c.matched['preview']['plan_id'] == 'p'
    result = c.finish()
    copied = c.output / c.matched['audit']['name']
    assert copied.read_bytes() == (tmp_path / 'original.json').read_bytes()
    assert result['records'] == 2
    row = json.loads((c.output / 'messages.jsonl').read_text().splitlines()[0])
    assert row['raw_text'] == json.dumps(audit(tmp_path))


@pytest.mark.parametrize('changes,error', [
    ({'grasp_mode_selection': dict(SEL, generation=20)}, 'audit_selection_mismatch'),
    ({'snapshot_stamp_ns': 200}, 'audit_source_not_new'),
    ({'snapshot_stamp_ns': 199}, 'audit_source_not_new')])
def test_wrong_or_old_audit_never_pairs(tmp_path, monkeypatch, changes, error):
    c = capture(tmp_path, monkeypatch); c.start_ns = 200
    c.enqueue('preview', Plan(), 401, 2.)
    c.enqueue('gate_audit', String(json.dumps(audit(tmp_path, **changes))), 402, 2.)
    c.drain(); assert c.matched is None; assert c.audit_errors[error] == 1
    c.finish()


@pytest.mark.parametrize('changes', [{'plan_id': 'other'}, {'snapshot_stamp_ns': 301},
                                   {'outcome': {'preview_valid': False}}])
def test_nonmatching_valid_preview_is_not_terminal_success(tmp_path, monkeypatch, changes):
    c = capture(tmp_path, monkeypatch); c.start_ns = 200
    c.enqueue('preview', Plan(), 401, 2.)
    c.enqueue('gate_audit', String(json.dumps(audit(tmp_path, **changes))), 402, 2.)
    c.drain(); assert c.matched is None; c.finish()


def test_superseded_atomic_file_not_relabelled(tmp_path, monkeypatch):
    c = capture(tmp_path, monkeypatch); c.start_ns = 200
    ref = audit(tmp_path); (tmp_path / 'original.json').write_text('{}')
    c.enqueue('gate_audit', String(json.dumps(ref)), 400, 2.)
    c.drain(); assert c.audit_errors['audit_atomic_reference_superseded'] == 1
    assert not c.audits; c.finish()


def test_before_request_and_old_latched_preview_not_admitted(tmp_path, monkeypatch):
    c = capture(tmp_path, monkeypatch)
    c.enqueue('preview', Plan(), 400, 2.)
    c.start_ns = 200
    c.enqueue('preview', Plan(ns=199), 500, 3.)
    c.drain(); assert not c.previews and c.records == 1; c.finish()


def test_invalid_preview_kept_but_not_terminal(tmp_path, monkeypatch):
    c = capture(tmp_path, monkeypatch); c.start_ns = 200
    c.enqueue('preview', Plan(valid=False, source='stability_pending'), 400, 2.)
    c.drain(); assert c.records == 1 and c.terminal is None; c.finish()


def test_terminal_stamp_not_claimed_measured_source(tmp_path, monkeypatch):
    c = capture(tmp_path, monkeypatch); c.start_ns = 200
    c.enqueue('preview', Plan(valid=False, source='near_field_terminal'), 400, 2.)
    c.drain(); assert c.terminal['source_pairing_claimed'] is False; c.finish()


@pytest.mark.parametrize('owned', [True, False])
def test_stream_shutdown_only_if_started_here(tmp_path, monkeypatch, owned):
    c = capture(tmp_path, monkeypatch); calls = []; clock = Clock()
    def request(trigger):
        calls.append(trigger)
        if trigger: enqueue_pair(c, tmp_path)
        return SimpleNamespace(success=True, message=(m.STARTED if owned else m.RUNNING) if trigger else 'stopped')
    result = m.capture_loop(c, request=request, validate=lambda:None, clock=clock,
        sleep=clock.sleep, ros_now_ns=lambda:200, shutdown=lambda:False, seconds=60)
    assert calls == ([True, False] if owned else [True])
    assert result['reason'] == 'valid_preview_with_exact_audit'
    assert c.handle.closed


def test_timeout_after_invalid_still_stops_owned_stream(tmp_path, monkeypatch):
    c = capture(tmp_path, monkeypatch); calls=[]; clock=Clock()
    def request(trigger):
        calls.append(trigger)
        if trigger: c.enqueue('preview', Plan(valid=False), 400, 2.)
        return SimpleNamespace(success=True, message=m.STARTED if trigger else 'stopped')
    result=m.capture_loop(c, request=request, validate=lambda:None, clock=clock,
        sleep=clock.sleep, ros_now_ns=lambda:200, shutdown=lambda:False, seconds=60)
    assert result['reason']=='capture_timeout' and result['elapsed_sec']>=60
    assert calls==[True,False]


@pytest.mark.parametrize('failure', [RuntimeError('selection_changed'), KeyboardInterrupt()])
def test_error_after_start_always_seals_and_cleans_up(tmp_path, monkeypatch, failure):
    c=capture(tmp_path,monkeypatch); calls=[]; checks=[]
    def validate():
        checks.append(1)
        if len(checks)>1: raise failure
    def request(trigger):
        calls.append(trigger); return SimpleNamespace(success=True,message=m.STARTED if trigger else 'stopped')
    result=m.capture_loop(c,request=request,validate=validate,ros_now_ns=lambda:200,shutdown=lambda:False)
    assert result['reason']=='capture_error' and calls==[True,False] and c.handle.closed


def test_failed_start_never_stops_someone_else(tmp_path,monkeypatch):
    c=capture(tmp_path,monkeypatch); calls=[]
    def request(trigger):
        calls.append(trigger); return SimpleNamespace(success=False,message='remote disabled')
    result=m.capture_loop(c,request=request,validate=lambda:None,ros_now_ns=lambda:200,shutdown=lambda:False)
    assert calls==[True] and 'rejected' in result['error'] and c.handle.closed


def test_stop_error_explicit_and_artifact_still_closed(tmp_path,monkeypatch):
    c=capture(tmp_path,monkeypatch)
    def request(trigger):
        if not trigger: raise RuntimeError('stop RPC failed')
        return SimpleNamespace(success=True,message=m.STARTED)
    result=m.capture_loop(c,request=request,validate=lambda:None,ros_now_ns=lambda:200,shutdown=lambda:True)
    assert 'stop RPC failed' in result['cleanup_error'] and c.handle.closed


def test_count_and_byte_limits_explicit(tmp_path,monkeypatch):
    c=capture(tmp_path,monkeypatch,per_kind=1,limit=m.RESERVE+1100); c.start_ns=200
    c.enqueue('planner_status',String('ok'),400,2.)
    c.enqueue('planner_status',String('next'),401,2.)
    c.enqueue('metrics',String('x'*1200),402,2.)
    result=c.finish()
    assert result['dropped']['planner_status:count_limit']==1
    assert result['budget_reached'] and result['records']==1


@pytest.mark.parametrize('change',[{'generation':True},{'stamp_ns':1.5},{'stamp_ns':0},
                                  {'mode':'carton'},{'strategy':'direct'}])
def test_bad_expected_selection(change):
    with pytest.raises(ValueError): m.selection(dict(SEL,**change))


def test_latched_old_task_is_allowed_but_active_or_changed_selection_not():
    mode=dict(SEL,active=False,start_reserved=False,pending='',state='ready',receipt_monotonic=1)
    m.verify_idle(dict(active=False,stamp_ns=1),mode,SEL,SEL,True)
    for task,actual,connected in [(dict(active=True),SEL,True),
                                  (dict(active=False),dict(SEL,generation=22),True),
                                  (dict(active=False),SEL,False)]:
        with pytest.raises(ValueError):m.verify_idle(task,mode,SEL,actual,connected)
    with pytest.raises(ValueError,match='grasp_mode_publisher_disconnected'):
        m.verify_idle(dict(active=False),mode,SEL,SEL,True,False)


def test_both_change_only_topics_have_no_receipt_age_gate():
    assert 'mode_status_stale' not in P.read_text()
    assert "live.get('mode_mono'" not in P.read_text()


def test_pure_import_and_only_planning_service_in_source():
    text=P.read_text()
    assert "ServiceProxy('/grasp_6d/request_plan'" in text
    for forbidden in ("ServiceProxy('/grasp/start'", "ServiceProxy('/grasp_mode/select'",
                      "ServiceProxy('/grasp_6d/replan_execution'", 'subprocess.', '.Publisher('):
        assert forbidden not in text
