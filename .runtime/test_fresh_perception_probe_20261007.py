import importlib.util
import json
from pathlib import Path
import pytest
P = Path(__file__).with_name('run_fresh_perception_probe_20261007.py')
spec = importlib.util.spec_from_file_location('fresh_probe',P)
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
OLD = dict(mode='unknown',strategy='observe_return',generation=73,stamp_ns='100')
NEW = dict(OLD,generation=74,stamp_ns='200')
IDLE = dict(active=False,start_reserved=False,pending='',state='waiting_for_target')

def test_dynamic_ack():
    assert m.validate_ack(OLD, NEW)['generation']==74

@pytest.mark.parametrize('delta',[dict(generation=73),dict(generation=75),dict(stamp_ns='100'),dict(mode='carton'),dict(strategy='two_stage')])
def test_wrong_ack(delta):
    with pytest.raises(ValueError):m.validate_ack(OLD,dict(NEW,**delta))

@pytest.mark.parametrize('delta',[dict(active=True),dict(start_reserved=True),dict(pending='unknown'),dict(state='switching'),dict(state='error')])
def test_busy_mode(delta):
    with pytest.raises(ValueError):m.validate_idle({'active':False},dict(IDLE,**delta))

def test_busy_task():
    with pytest.raises(ValueError):m.validate_idle({'active':True},IDLE)

def test_idle():m.validate_idle({'active':False},IDLE)

def test_historical_origin_is_not_new_alignment(tmp_path,monkeypatch):
    monkeypatch.setattr(m,'EXT',tmp_path)
    origin=dict(confirmation_source={'kind':'user_message','text':'historic'},confirmed_ns='100')
    event=tmp_path/'event.json';event.write_text(json.dumps(origin))
    receipt=tmp_path/'receipt.json';value=dict(origin,kind='explicit_user_target_alignment',confirmation_id='actual:0',event_file=str(event),event_file_sha256=m.sha(event))
    receipt.write_text(json.dumps(value))
    actual=m.historical(receipt,'explicit_user_target_alignment')
    assert actual['current_alignment_inferred'] is False
    assert actual['original_confirmed_ns']=='100'
    event.write_text('{}')
    with pytest.raises(ValueError,match='historical_event_changed'):m.historical(receipt,'explicit_user_target_alignment')

def test_historical_forged_confirmation_rejected(tmp_path,monkeypatch):
    monkeypatch.setattr(m,'EXT',tmp_path)
    event=tmp_path/'event.json';event.write_text(json.dumps(dict(confirmed_ns='1',confirmation_source={'kind':'user_message','text':'old'})))
    receipt=tmp_path/'receipt.json';receipt.write_text(json.dumps(dict(kind='explicit_user_target_alignment',confirmed_ns='2',confirmation_source={'kind':'user_message','text':'new'},confirmation_id='actual:0',event_file=str(event),event_file_sha256=m.sha(event))))
    with pytest.raises(ValueError,match='historical_confirmation_mismatch'):m.historical(receipt,'explicit_user_target_alignment')


def test_unchanged_inactive_latched_state_is_not_a_heartbeat():
    m.validate_latched_task({'active':False,'stamp_ns':1,'receipt_monotonic':1},1)


def test_latched_task_disconnected_is_not_accepted():
    with pytest.raises(ValueError,match='publisher_disconnected'):
        m.validate_latched_task({'active':False},0)


def test_latched_task_active_rejected():
    with pytest.raises(ValueError,match='not_inactive'):
        m.validate_latched_task({'active':True},1)
