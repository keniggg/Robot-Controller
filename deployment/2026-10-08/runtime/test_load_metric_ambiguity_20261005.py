"""No ROS, process signals or external disk writes in these transaction tests."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import pytest

spec=importlib.util.spec_from_file_location('metric_loader',Path(__file__).with_name('load_metric_ambiguity_20261005.py'))
loader=importlib.util.module_from_spec(spec);spec.loader.exec_module(loader)


class FakeRuntime:
    def __init__(self, failure=None):
        self.events=[];self.failure=failure;self.running=True;self.generation=2
    def event(self, name):
        self.events.append(name)
        if name==self.failure:raise RuntimeError(name)
    def verify(self):self.event('verify')
    def stop_producer(self):
        self.event('stop');self.running=False
    def select_new_generation(self):
        assert not self.running
        self.event('select');self.generation=3
    def start_producer(self):
        assert self.generation==3 and not self.running
        self.event('start');self.running=True
    def verify_final(self):self.event('final')


def test_new_producer_never_observes_retired_generation():
    io=FakeRuntime();loader.reload_transaction(io)
    assert io.events==['verify','stop','select','start','final']
    assert io.generation==3 and io.running


@pytest.mark.parametrize('failure',['verify','stop','select','start','final'])
def test_failure_never_restarts_or_retries_retired_selection(failure):
    io=FakeRuntime(failure)
    with pytest.raises(RuntimeError,match=failure):loader.reload_transaction(io)
    sequence=['verify','stop','select','start','final']
    assert io.events==sequence[:sequence.index(failure)+1]
    if failure=='select':assert not io.running and io.generation==2
    if failure=='start':assert not io.running and io.generation==3


def test_epoch_uses_header_itself_not_nested_message_header():
    header=SimpleNamespace(frame_id='sdk_reference_epoch',stamp=SimpleNamespace(to_nsec=lambda:123))
    assert loader.validated_epoch(header,'123')==123


@pytest.mark.parametrize('frame,value',[('other',123),('sdk_reference_epoch',124),('sdk_reference_epoch',0)])
def test_epoch_change_or_wrong_frame_rejected(frame,value):
    header=SimpleNamespace(frame_id=frame,stamp=SimpleNamespace(to_nsec=lambda:value))
    with pytest.raises(ValueError,match='control_epoch_changed'):loader.validated_epoch(header,'123')
