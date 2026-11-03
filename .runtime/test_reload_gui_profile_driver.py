"""Offline contracts only. Never initializes ROS or executes the loader."""
import importlib.util
import math
from pathlib import Path
import subprocess
import numpy as np
import pytest

PATH=Path(__file__).resolve().parent/'gen4_root_cause/reload_gui_profile_driver.py'
spec=importlib.util.spec_from_file_location('reload_profile_draft',PATH)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def reference():
    return [(-math.pi+w*2*math.pi/4096) for w in (820,2426,1961,1978,1763,2077)]+[.04975]


def test_import_is_nonexecuting_and_requires_explicit_execute(monkeypatch):
    monkeypatch.setattr(subprocess,'run',lambda *a,**k:pytest.fail('subprocess before explicit execution'))
    with pytest.raises(SystemExit):m.main(['--old-pid','9881','--old-parent-pid','9685'])


def test_exact_seven_sdk_words_and_existing_sync_tolerance():
    q=reference()
    assert m.words7(q,.05)==[820,2426,1961,1978,1763,2077,995]
    actual=q.copy();actual[2]-=.010737865515199507
    m.validate_sync(q,actual,.05)
    actual[2]=q[2]-.050001
    with pytest.raises(RuntimeError,match='sync tolerance'):m.validate_sync(q,actual,.05)


@pytest.mark.parametrize('bad',[None,[],[0]*6,[0]*6+[float('nan')]])
def test_invalid_reference_rejected(bad):
    with pytest.raises((RuntimeError,ValueError,TypeError)):m.words7(bad,.05)


def test_gripper_never_silently_rebased():
    q=reference();actual=q.copy();actual[6]-=.001
    with pytest.raises(RuntimeError,match='gripper moved'):m.validate_sync(q,actual,.05)


def test_exact_one_designated_new_epoch():
    p=m.OneEpochTransition(100)
    assert p.observe(100,m.NODE) is None
    with pytest.raises(RuntimeError):p.observe(101,m.NODE)
    p.allow_start(200)
    with pytest.raises(RuntimeError):p.observe(199,m.NODE)
    assert p.observe(200,m.NODE)==200
    assert p.observe(200,m.NODE)==200
    for ns in (100,201):
        with pytest.raises(RuntimeError,match='second'):p.observe(ns,m.NODE)
    with pytest.raises(RuntimeError):p.allow_start(300)


def test_unknown_epoch_publisher_never_allowed():
    p=m.OneEpochTransition(100);p.allow_start(200)
    with pytest.raises(RuntimeError,match='publisher'):p.observe(200,'/other')
    assert p.new_ns is None


def test_frozen_hold_sync_and_single_axis_profile_corridor():
    original=m.words7(reference(),.05)
    f=m.WireFence(original);f.check(original)
    f.awaiting_sync()
    with pytest.raises(RuntimeError,match='before explicit'):f.check(original)
    f.allow_sync();f.check(original)
    with pytest.raises(RuntimeError):f.allow_sync()
    probe=original.copy();probe[4]-=18;f.allow_probe(probe)
    for j5 in range(probe[4],original[4]+1):
        row=original.copy();row[4]=j5;f.check(row)
    for axis in range(7):
        row=original.copy();row[axis]+=1
        with pytest.raises(RuntimeError):f.check(row)
    row=probe.copy();row[4]-=1
    with pytest.raises(RuntimeError):f.check(row)


def test_probe_words_cannot_expand_other_axes_or_original_envelope():
    original=m.words7(reference(),.05)
    for axis,delta in ((0,1),(6,1),(4,-25),(4,1),(4,0)):
        f=m.WireFence(original);row=original.copy();row[axis]+=delta
        with pytest.raises(RuntimeError):f.allow_probe(row)


def test_systemd_process_is_independent_and_has_no_restart_or_torque_command(tmp_path):
    command=m.systemd_command('review-test',tmp_path,{'ROS_MASTER_URI':'http://localhost:11311','TOKEN':'do-not-copy','LD_LIBRARY_PATH':'/opt/ros/noetic/lib'})
    assert command[:3]==['systemd-run','--user','--collect']
    assert '--property=Restart=no' in command
    assert '--property=Type=exec' in command
    assert '__name:=alicia_d_driver_node' in command
    assert '--setenv=ROS_LOG_DIR='+str(tmp_path/'ros') in command
    assert not any('TOKEN' in arg or 'do-not-copy' in arg for arg in command)
    assert str(m.BINARY) in command
    assert not any('stop' in arg or 'disable' in arg for arg in command)


def test_process_identity_reads_only_current_process():
    import os
    identity=m.process_identity(os.getpid())
    assert identity['pid']==os.getpid()
    assert identity['ppid']==os.getppid()
    assert len(identity['exe_sha256'])==64
    assert m.same_process(identity)
    changed=dict(identity,start_ticks=identity['start_ticks']+1)
    assert not m.same_process(changed)


def test_return_locks_back_to_exact_original_words():
    original=m.words7(reference(),.05);f=m.WireFence(original)
    probe=original.copy();probe[4]-=18;f.allow_probe(probe);f.check(probe)
    f.restore_original_hold();f.check(original)
    with pytest.raises(RuntimeError):f.check(probe)


def test_exact_unknown_node_is_registration_pending_but_other_master_errors_are_not():
    from types import SimpleNamespace
    def fake(detail):return SimpleNamespace(get_master=lambda:SimpleNamespace(lookupNode=lambda node:(-1,detail,'')))
    with pytest.raises(m.DriverRegistrationPending):m.master_driver_pid(fake('unknown node [/alicia_d_driver_node]'))
    with pytest.raises(RuntimeError) as error:m.master_driver_pid(fake('master authorization failed'))
    assert not isinstance(error.value,m.DriverRegistrationPending)


def test_registration_poll_retries_only_transient_absence_and_never_hash_mismatch(monkeypatch):
    monkeypatch.setattr(m.subprocess,'check_output',lambda *a,**k:'76933\n')
    monkeypatch.setattr(m,'master_driver_pid',lambda r:(_ for _ in ()).throw(m.DriverRegistrationPending('not registered yet')))
    assert m.replacement_process_ready(None,'existing-unit',9881,'reviewed') is False
    monkeypatch.setattr(m,'master_driver_pid',lambda r:76933)
    monkeypatch.setattr(m,'process_identity',lambda pid:{'pid':pid,'exe_sha256':'different'})
    with pytest.raises(RuntimeError,match='binary hash differs'):
        m.replacement_process_ready(None,'existing-unit',9881,'reviewed')
    monkeypatch.setattr(m,'process_identity',lambda pid:{'pid':pid,'exe_sha256':'reviewed'})
    assert m.replacement_process_ready(None,'existing-unit',9881,'reviewed')['pid']==76933


def test_resume_startup_log_evidence_rejects_commands_or_second_enable(tmp_path):
    stdout=tmp_path/'driver.stdout.log';stderr=tmp_path/'driver.stderr.log'
    startup='Requesting positive SDK torque_on from startup; encoder confirmation remains pending.\n'
    stdout.write_text(startup);stderr.write_text('PENDING:POSITIVE_ENABLE_REQUESTED\n')
    assert m.resume_startup_log_proof(tmp_path)['startup_positive_enable_count']==1
    for extra in ('Received /joint_commands source=gui_direct_sync\n','Streaming SDK command (ok)\n',startup):
        stdout.write_text(startup+extra)
        with pytest.raises(RuntimeError):m.resume_startup_log_proof(tmp_path)


def test_resume_branch_contains_no_process_or_enable_mutation():
    import ast
    tree=ast.parse(PATH.read_text())
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    resume_branches=[n for n in ast.walk(main) if isinstance(n,ast.If) and isinstance(n.test,ast.Compare)
                     and isinstance(n.test.left,ast.Name) and n.test.left.id=='previous_session'
                     and any(isinstance(child,ast.Name) and child.id=='startup_log_proof' for child in ast.walk(n))]
    # This is the execution branch, not the initial argument-validation branch.
    branch=next(n for n in resume_branches if n.orelse)
    calls=[]
    for statement in branch.body:
        for node in ast.walk(statement):
            if isinstance(node,ast.Call):
                func=node.func
                calls.append(func.attr if isinstance(func,ast.Attribute) else func.id if isinstance(func,ast.Name) else '')
    assert not {'kill','set_param','run','systemd_command','publish'} & set(calls)
