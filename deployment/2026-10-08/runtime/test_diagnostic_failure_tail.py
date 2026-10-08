import sys
from pathlib import Path
from types import SimpleNamespace
from threading import RLock
import pytest
sys.path.insert(0,str(Path(__file__).parent))
from diagnostic_failure_tail import record_failure_tail

class Stamp:
    def __init__(self,n):self.n=n
    def to_nsec(self):return self.n

def diagnostic():
    marker=object()
    d=SimpleNamespace(publish_count=2,publisher=SimpleNamespace(unregister=lambda:None),
        lock=RLock(),fault='held-axis violation',contract=marker,events=[],accepted=[],reference=[])
    d.archive_writer=SimpleNamespace(require_healthy=lambda:None)
    d.event=lambda kind,**kw:d.events.append((kind,kw))
    return d,marker

class Clock:
    now=0.
    def __call__(self):return self.now
    def sleep(self,n):self.now+=n

def test_failure_stays_failed_and_publisher_disconnects():
    d,contract=diagnostic();clock=Clock();disconnected=[]
    d.publisher.unregister=lambda:disconnected.append(True)
    def tick(n):
        clock.sleep(n);msg=SimpleNamespace(header=SimpleNamespace(stamp=Stamp(int(clock.now*1e9))))
        d.accepted.append(msg);d.reference.append(msg)
    result=record_failure_tail(d,'original',monotonic=clock,sleep=tick)
    assert result['status']=='passive_window_recorded'
    assert result['elapsed_seconds']==5.
    assert result['commands_during_tail']==0
    assert result['observed_latest_accepted_source_count']>1
    assert d.publisher is None and disconnected==[True]
    assert d.fault=='held-axis violation' and d.contract is contract and d.publish_count==2
    assert result['original_failure']=='original' and result['fault_cleared'] is False
    assert result['failed_command_completed'] is False
    assert [e[0] for e in d.events]==['passive_failure_tail_started','passive_failure_tail_finished']

def test_no_commands_no_tail_no_disconnect():
    d,_=diagnostic();d.publish_count=0;publisher=d.publisher
    result=record_failure_tail(d,'before command')
    assert result['status']=='not_needed_no_commands' and d.publisher is publisher and not d.events

def test_archive_failure_does_not_replace_original():
    d,_=diagnostic()
    def failed():raise RuntimeError('disk full')
    d.archive_writer.require_healthy=failed
    result=record_failure_tail(d,'held axis')
    assert result['status']=='passive_tail_incomplete' and result['original_failure']=='held axis'
    assert result['tail_failure']=='disk full' and d.publisher is None and d.fault=='held-axis violation'

def test_shutdown_stops_only_passive_wait():
    d,_=diagnostic();clock=Clock()
    result=record_failure_tail(d,'failure',monotonic=clock,sleep=clock.sleep,is_shutdown=lambda:True)
    assert result['status']=='interrupted_ros_shutdown' and result['elapsed_seconds']==0.
    assert d.publish_count==2

def test_tail_rejects_recreated_command_interface():
    d,_=diagnostic();clock=Clock()
    def bad_tick(n):clock.sleep(n);d.publisher=object()
    result=record_failure_tail(d,'failure',monotonic=clock,sleep=bad_tick)
    assert result['status']=='passive_tail_incomplete' and 'command interface' in result['tail_failure']
    assert result['original_failure']=='failure'

def test_unset_fault_latches_original_without_contract_reset():
    d,contract=diagnostic();d.fault=None;clock=Clock()
    result=record_failure_tail(d,'failed geometry',monotonic=clock,sleep=clock.sleep)
    assert d.fault=='failed geometry' and d.contract is contract and result['fault_cleared'] is False

def test_unregistration_failure_is_reported_without_motion():
    d,_=diagnostic()
    def failed():raise RuntimeError('unregister failed')
    d.publisher.unregister=failed
    result=record_failure_tail(d,'primary')
    assert result['status']=='passive_tail_incomplete' and result['tail_failure']=='unregister failed'
    assert result['commands_during_tail']==0 and result['original_failure']=='primary'
