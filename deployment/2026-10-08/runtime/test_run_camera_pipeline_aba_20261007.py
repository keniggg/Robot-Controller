import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import signal
import subprocess
import io
import pytest

spec=importlib.util.spec_from_file_location('outer_aba',Path(__file__).with_name('run_camera_pipeline_aba_20261007.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class IO:
    def __init__(self, fault=None):
        self.fault=fault;self.events=[];self.gone=False
    def stage(self,name):
        self.events.append(name)
        if self.fault==name:raise RuntimeError(name)
    def prepare(self):self.stage('prepare')
    def stop_camera(self):
        self.stage('stop');self.gone=True
    def old_camera_exited(self):return self.gone
    def probe(self):self.stage('probe')
    def close_probe(self):self.stage('close')
    def restore(self):self.stage('restore')
    def record_error(self,stage,exc):self.events.append('error:'+stage)
    def finish(self):self.events.append('finish')


def test_normal_always_closes_owned_diagnostic_before_one_restore():
    io=IO();m.run_with_restore(io)
    assert io.events==['prepare','stop','probe','close','restore','finish']


@pytest.mark.parametrize('fault',['prepare','stop'])
def test_no_second_camera_when_old_owner_did_not_exit(fault):
    io=IO(fault)
    with pytest.raises(RuntimeError,match=fault):m.run_with_restore(io)
    assert 'probe' not in io.events and 'restore' not in io.events
    assert io.events[-1]=='finish'


def test_probe_failure_restores_camera_before_rethrowing():
    io=IO('probe')
    with pytest.raises(RuntimeError,match='probe'):m.run_with_restore(io)
    assert io.events[-3:]==['close','restore','finish']
    assert io.events.count('restore')==1


def test_close_failure_never_starts_second_pipeline():
    io=IO('close')
    with pytest.raises(RuntimeError,match='close'):m.run_with_restore(io)
    assert 'restore' not in io.events and io.events[-1]=='finish'


def test_restore_failure_is_not_retried_or_hidden():
    io=IO('restore')
    with pytest.raises(RuntimeError,match='restore'):m.run_with_restore(io)
    assert io.events.count('restore')==1 and io.events[-2:]==['error:restoration','finish']


def test_stop_that_exits_then_raises_still_restores():
    io=IO()
    def stopped():io.gone=True;raise RuntimeError('late stop error')
    io.stop_camera=stopped
    with pytest.raises(RuntimeError,match='late stop'):m.run_with_restore(io)
    assert io.events[-3:]==['close','restore','finish']


def test_child_already_exited_is_not_signaled(monkeypatch):
    monkeypatch.setattr(m.os,'kill',lambda *a:pytest.fail('signal after exit'))
    m.owned_probe_exit(NS(poll=lambda:1),{},NS(alive=lambda v:pytest.fail('identity after exit')))


def test_reused_diagnostic_pid_is_never_signaled(monkeypatch):
    monkeypatch.setattr(m.os,'kill',lambda *a:pytest.fail('reused pid signal'))
    with pytest.raises(RuntimeError,match='identity changed'):
        m.owned_probe_exit(NS(poll=lambda:None),{'pid':9},NS(alive=lambda v:False))


def test_only_owned_diagnostic_pid_can_receive_bounded_escalation(monkeypatch):
    calls=[];proc=NS(status=None)
    proc.poll=lambda:proc.status
    def wait(timeout):
        if calls[-1][1]==signal.SIGKILL:proc.status=-9;return
        raise subprocess.TimeoutExpired('diagnostic',timeout)
    proc.wait=wait
    monkeypatch.setattr(m.os,'kill',lambda p,s:calls.append((p,s)))
    m.owned_probe_exit(proc,{'pid':42},NS(alive=lambda v:True))
    assert calls==[(42,signal.SIGINT),(42,signal.SIGTERM),(42,signal.SIGKILL)]


def test_registered_other_camera_prevents_replacement():
    class Pending(RuntimeError):pass
    with pytest.raises(RuntimeError,match='another live camera'):
        m.camera_absent(None,NS(node_pid=lambda *a:91,CameraRegistrationPending=Pending))


@pytest.mark.parametrize('error',[ConnectionRefusedError,ConnectionResetError])
def test_dead_ros_registration_does_not_masquerade_as_live_camera(error):
    class Pending(RuntimeError):pass
    def missing(*args):raise error()
    assert m.camera_absent(None,NS(node_pid=missing,CameraRegistrationPending=Pending))


def test_console_does_not_block_when_no_bytes_available():
    def unavailable(n):raise BlockingIOError()
    output=io.BytesIO();state={'bytes':0}
    m.drain_console(NS(read=unavailable),output,state)
    assert state=={'bytes':0} and output.getvalue()==b''


def test_console_budget_fails_before_extra_bytes_written():
    output=io.BytesIO();state={'bytes':0}
    with pytest.raises(RuntimeError,match='128 KiB'):
        m.drain_console(io.BytesIO(b'x'*(m.MAX_PROBE_CONSOLE+1)),output,state)
    assert len(output.getvalue())==state['bytes']==m.MAX_PROBE_CONSOLE


def test_normal_console_bytes_are_preserved():
    output=io.BytesIO();state={'bytes':0}
    m.drain_console(io.BytesIO(b'diagnostic result\n'),output,state)
    assert output.getvalue()==b'diagnostic result\n' and state['bytes']==18


def test_pose_guard_error_cannot_skip_late_owned_camera_exit_and_restore():
    now=[0.]
    io=IO()
    def stopped_then_pose_guard_fails():raise RuntimeError('pose changed after SIGINT')
    io.stop_camera=stopped_then_pose_guard_fails
    io.old_camera_exited=lambda:m.stopped_camera_exited({},True,
        lambda owner:now[0]<.2,clock=lambda:now[0],sleep=lambda dt:now.__setitem__(0,now[0]+dt))
    with pytest.raises(RuntimeError,match='pose changed'):m.run_with_restore(io)
    assert io.events[-3:]==['close','restore','finish'] and now[0]>=.2


def test_unrequested_owner_is_not_waited_on_or_replaced():
    assert not m.stopped_camera_exited({},False,lambda owner:True,
        sleep=lambda t:pytest.fail('no stop was sent'))


def test_nonexiting_camera_never_permits_a_second_pipeline():
    now=[0.]
    assert not m.stopped_camera_exited({},True,lambda owner:True,
        clock=lambda:now[0],sleep=lambda dt:now.__setitem__(0,now[0]+dt),wait_seconds=.1)
