import importlib.util
import ast
import hashlib
import json
import os
from pathlib import Path
import sys
import types

import pytest

WORK = Path(os.environ.get('GEN6_TEST_WORKTREE', Path(__file__).resolve().parents[3]))
RUNTIME = Path(os.environ.get('GEN10_TEST_LAUNCHER', WORK / '.runtime/run_reboot_realigned_grasp_20261010.py')).parent
sys.path.insert(0, str(RUNTIME))
import continuous_grasp_authorization_20261012 as authority
from test_launcher_selection_commit import harness


def save(path, value):
    path.write_text(json.dumps(value))
    return value


def ref(path):
    return dict(path=str(path), sha256=authority.digest(path))


@pytest.fixture
def evidence(tmp_path):
    owner = tmp_path / 'authorization'; owner.mkdir()
    session = tmp_path / 'capture'; session.mkdir()
    record = tmp_path / 'record'; record.mkdir()
    hands = save(owner / 'hands.json', dict(user_message='当前已对准且手已移开，允许执行一轮', message_id='actual-prior-platform-reply'))
    auth = save(owner / 'authorization.json', dict(version=1, authorization_id='continuous-1',
        scope='observe_return_until_physical_grasp_success', issued_ns=1_000_000_000,
        confirmation_source=dict(source='actual_user_message', text=authority.REQUEST_TEXT,
            received_at='2026-10-12T12:00:00Z', received_at_kind='locally_recorded_receipt_time', local_record_id='local-message-1', platform_message_id=None),
        original_hands_clear_receipt=ref(owner / 'hands.json')))
    save(owner / 'control.json', dict(version=1, authorization_id='continuous-1', state='active', revision=1))
    selection = dict(mode='unknown', strategy='observe_return', generation=8, stamp_ns=123)
    before = dict(stamp_ns=1_990_000_000, frame_id='sdk_measured',
        names=['Joint%d' % i for i in range(1,7)], positions=[0.] * 6)
    preflight = save(session / 'preflight.json', dict(receipt_ns=2_000_000_000,
        image_stamp_ns=1_980_000_000, epoch_ns=100, topics={'/alicia_d/accepted_joint_states':before}))
    ready = save(session / 'readiness.json', dict(passed=True, deployment_processes_unchanged=True,
        source_matches_operational_release=True, motion_commands_sent=False, temperature_queried=False,
        sample_count=20, epoch_ns=100, free_bytes=1000, min_free_bytes=100,
        recorded_ns=2_010_000_000, selection=selection))
    (session / 'scene.png').write_bytes(b'captured-scene-fixture')
    observation = save(session / 'operator_observation.json', dict(version=1, observation_id='image-1',
        new_user_confirmation_claimed=False, same_target_observed=True, target_visible=True,
        hands_clear_basis=dict(prior_explicit_user_reply_id=hands['message_id'],
            current_image_review_no_conflicting_hand_in_motion_area=True),
        image=dict(ref(session / 'scene.png'), source_stamp_ns=1_980_000_000),
        preflight=ref(session / 'preflight.json'), readiness=ref(session / 'readiness.json'),
        captured_ns=2_000_000_000, assessed_ns=2_020_000_000, valid_until_ns=10_000_000_000))
    def attempt():
        return authority.ContinuationAttempt(owner/'authorization.json', record, session,
            authority.REQUEST_TEXT, 2_030_000_000, storage=tmp_path)
    return types.SimpleNamespace(root=tmp_path, owner=owner, session=session, record=record,
        auth=auth, observation=observation, preflight=preflight, ready=ready, selection=selection, attempt=attempt)


def test_claim_is_consumed_and_preserves_actual_provenance(evidence):
    e=evidence
    with e.attempt() as attempt:
        fields=attempt.authorization_fields()
        assert fields['confirmation_source']['user_alignment_claimed'] is False
        assert fields['confirmation_source']['platform_message_id'] is None
        assert fields['confirmation_source']['text'] == authority.REQUEST_TEXT
        assert fields['new_user_confirmation_claimed'] is False
        assert authority.read(attempt.ledger_path)['consumed'] is True
        assert (e.record/'continuation_attempt_claim.json').is_file()
    assert attempt.lock is None


def test_exclusive_lock_denies_another_attempt_even_different_record(evidence):
    e=evidence
    with e.attempt():
        other=e.root/'other-record';other.mkdir()
        second=authority.ContinuationAttempt(e.owner/'authorization.json', other,e.session,
            authority.REQUEST_TEXT,2_030_000_000,storage=e.root)
        with pytest.raises(BlockingIOError):
            with second: pass
        assert not (other/'continuation_attempt_claim.json').exists()


def test_failed_attempt_is_not_refunded_and_cannot_reuse_observation(evidence):
    e=evidence
    with pytest.raises(RuntimeError):
        with e.attempt() as attempt: raise RuntimeError('before selection')
    assert authority.read(attempt.ledger_path)['status']=='launcher_error'
    assert authority.read(attempt.ledger_path)['authorization_not_refunded'] is True
    with pytest.raises(ValueError, match='ALREADY_CONSUMED'):
        with e.attempt(): pass


@pytest.mark.parametrize('state',['paused','revoked','completed'])
def test_pause_revoke_and_completion_block_new_and_unreleased_attempts(evidence,state):
    e=evidence
    with e.attempt() as attempt:
        authority.set_control(e.owner/'authorization.json',state)
        with pytest.raises(ValueError,match='PAUSED_REVOKED_OR_COMPLETED'):
            attempt.check_active(2_040_000_000)
    with pytest.raises(ValueError,match='PAUSED_REVOKED_OR_COMPLETED'): e.attempt()


def test_pause_can_resume_explicitly_but_revocation_cannot(evidence):
    path=evidence.owner/'authorization.json'
    authority.set_control(path,'paused');authority.set_control(path,'active')
    evidence.attempt()
    authority.set_control(path,'revoked')
    with pytest.raises(ValueError,match='CONTROL_TERMINAL'): authority.set_control(path,'active')


@pytest.mark.parametrize('change', ['hand_conflict','different_target','fake_new_confirmation','stale_capture','expired','bad_hash','fake_platform_id','stale_readiness'])
def test_observation_and_provenance_fail_closed(evidence,change):
    e=evidence
    if change=='hand_conflict': e.observation['hands_clear_basis']['current_image_review_no_conflicting_hand_in_motion_area']=False
    elif change=='different_target': e.observation['same_target_observed']=False
    elif change=='fake_new_confirmation': e.observation['new_user_confirmation_claimed']=True
    elif change=='stale_capture': e.observation['captured_ns']=1
    elif change=='expired': e.observation['valid_until_ns']=2_025_000_000
    elif change=='bad_hash': (e.session/'scene.png').write_bytes(b'changed')
    elif change=='fake_platform_id':
        e.auth['confirmation_source']['platform_message_id']='invented'
        save(e.owner/'authorization.json',e.auth)
    elif change=='stale_readiness':
        e.ready['source_matches_operational_release']=False
        save(e.session/'readiness.json',e.ready)
        e.observation['readiness']=ref(e.session/'readiness.json')
    save(e.session/'operator_observation.json',e.observation)
    with pytest.raises(ValueError): e.attempt()


def test_mutation_after_claim_is_detected_before_release(evidence):
    e=evidence
    with e.attempt() as attempt:
        (e.session/'scene.png').write_bytes(b'new pixels')
        with pytest.raises(ValueError,match='PINNED_EVIDENCE_CHANGED'): attempt.check_active(2_040_000_000)


def test_record_write_failure_does_not_erase_consumed_ledger(evidence,monkeypatch):
    e=evidence
    original=Path.open
    def interrupted(path,*args,**kwargs):
        if path.name=='continuation_attempt_claim.json':raise OSError('simulated write failure')
        return original(path,*args,**kwargs)
    monkeypatch.setattr(Path,'open',interrupted)
    with pytest.raises(OSError,match='write failure'):
        with e.attempt():pass
    claims=list((e.owner/'ledger').glob('*.json'))
    assert len(claims)==1 and authority.read(claims[0])['consumed'] is True
    with pytest.raises(ValueError,match='ALREADY_CONSUMED'):
        with e.attempt():pass


def next_observation(e):
    e.observation.update(observation_id='image-2',captured_ns=3_000_000_000,assessed_ns=3_020_000_000)
    e.preflight.update(receipt_ns=3_000_000_000,image_stamp_ns=2_980_000_000)
    e.preflight['topics']['/alicia_d/accepted_joint_states']['stamp_ns']=2_990_000_000
    save(e.session/'preflight.json',e.preflight)
    e.ready['recorded_ns']=3_010_000_000;save(e.session/'readiness.json',e.ready)
    e.observation.update(preflight=ref(e.session/'preflight.json'),readiness=ref(e.session/'readiness.json'))
    e.observation['image']['source_stamp_ns']=2_980_000_000
    save(e.session/'operator_observation.json',e.observation)
    record=e.root/'record-2';record.mkdir()
    return authority.ContinuationAttempt(e.owner/'authorization.json',record,e.session,
        authority.REQUEST_TEXT,3_030_000_000,storage=e.root)


def test_next_record_requires_prior_recorder_sealed(evidence):
    e=evidence
    with e.attempt(): pass
    second=next_observation(e)
    with pytest.raises(FileNotFoundError):
        with second: pass
    save(e.record/'manifest.json',dict(state='complete',recorder_closed=True))
    save(e.record/'recorder_outcome.json',dict(bag_closed=True))
    with second: assert second.attempt['record_dir'] != str(e.record)


@pytest.fixture
def runtime(evidence):
    e=evidence;attempt=e.attempt()
    guard=object.__new__(authority.RuntimeGuard)
    guard.lock=authority.threading.Lock();guard.attempt=attempt
    guard.received={key:authority.time.monotonic() for key in ('task','arm','hand')}
    now=[2_030_000_000]
    stamp=lambda value:types.SimpleNamespace(to_nsec=lambda:value)
    guard.rospy=types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda:stamp(now[0])),
        is_shutdown=lambda:False,get_param=lambda _name:e.selection)
    guard.latest=dict(q=types.SimpleNamespace(header=types.SimpleNamespace(frame_id='sdk_measured',stamp=stamp(2_025_000_000)),
        name=['Joint%d'%i for i in range(1,7)],position=[0.]*6),
        epoch=types.SimpleNamespace(stamp=stamp(100)),task=types.SimpleNamespace(active=False),
        mode=types.SimpleNamespace(data=json.dumps(dict(e.selection,active=False,start_reserved=False,pending=''))),
        arm=types.SimpleNamespace(status_list=[],header=types.SimpleNamespace(stamp=stamp(2_025_000_000))),
        hand=types.SimpleNamespace(status_list=[],header=types.SimpleNamespace(stamp=stamp(2_025_000_000))))
    return guard,now,stamp


@pytest.mark.parametrize('fault',['none','task','arm','hand','pose','epoch','stale','router','selection','stale_task','stale_arm','stale_hand','stale_action_source'])
def test_runtime_rechecks_all_idle_and_current_sdk_bindings(evidence,runtime,fault):
    guard,now,stamp=runtime
    if fault=='task':guard.latest['task'].active=True
    elif fault in ('arm','hand'):guard.latest[fault].status_list=[types.SimpleNamespace(status=1)]
    elif fault=='pose':guard.latest['q'].position[2]=.021
    elif fault=='epoch':guard.latest['epoch'].stamp=stamp(101)
    elif fault=='stale':now[0]+=201_000_000
    elif fault=='router':guard.latest['mode'].data=json.dumps(dict(evidence.selection,start_reserved=True))
    elif fault in ('stale_task','stale_arm','stale_hand'):guard.received[fault[6:]]-=3.
    elif fault=='stale_action_source':guard.latest['arm'].header.stamp=stamp(1)
    elif fault=='selection':guard.rospy.get_param=lambda _name:dict(evidence.selection,generation=9)
    if fault=='none':guard.check(evidence.selection)
    else:
        with pytest.raises(ValueError):guard.check(evidence.selection)


def test_control_revocation_at_release_does_not_kill_existing_task(evidence,runtime):
    guard,now,_=runtime
    authority.set_control(evidence.owner/'authorization.json','revoked')
    with pytest.raises(ValueError):guard.check(evidence.selection)


def test_validated_commit_status_handles_lagging_mode_subscriber(evidence,runtime):
    guard,_,_=runtime
    guard.latest['mode'].data=json.dumps(dict(evidence.selection,generation=7))
    with pytest.raises(ValueError,match='ROUTER_BUSY'):guard.check(evidence.selection)
    status=dict(evidence.selection,state='waiting_for_target',active=False,pending='',start_reserved=False)
    guard.check(evidence.selection,commit_status=status)
    with pytest.raises(ValueError,match='ROUTER_BUSY'):
        guard.check(evidence.selection,commit_status=dict(status,generation=9))
    with pytest.raises(ValueError,match='COMMIT_NOT_READY'):
        guard.check(evidence.selection,commit_status=dict(status,state='error'))


def test_idle_refresh_requests_current_task_latch_and_both_action_messages(evidence,runtime):
    guard,_,_=runtime
    guard.refresh_topics=[(key,'/'+key,object) for key in ('task','arm','hand')]
    guard.received={key:0. for key in ('task','arm','hand')}
    calls=[]
    def fresh(topic,kind,timeout):
        calls.append(topic)
        assert 0. < timeout <= 5.
        return guard.latest[topic[1:]]
    guard.rospy.wait_for_message=fresh
    guard.refresh_idle()
    guard.check(evidence.selection)
    assert calls==['/task','/arm','/hand']


def test_release_commit_records_pause_boundary_and_never_releases_twice(evidence,runtime):
    guard,now,_=runtime
    with evidence.attempt() as attempt:
        guard.attempt=attempt
        status=dict(evidence.selection,state='ready',active=False,pending='')
        commit=attempt.commit_release(guard,evidence.selection,status,now[0])
        assert commit['control_revision']==1
        assert commit['pause_after_commit_cancels_this_attempt'] is False
        assert authority.read(attempt.ledger_path)['release_commit']==commit
        # This completes immediately: the control lock is not held while the
        # existing runner may execute. The committed attempt is not cancelled.
        authority.set_control(evidence.owner/'authorization.json','paused')
        with pytest.raises(ValueError,match='RELEASE_ALREADY_COMMITTED'):
            attempt.commit_release(guard,evidence.selection,status,now[0])
        with pytest.raises(ValueError,match='PAUSED_REVOKED_OR_COMPLETED'):
            attempt.check_active(now[0])


@pytest.mark.parametrize('pause_at', ['none','before_select','after_commit'])
def test_actual_launcher_block_checks_control_before_select_and_release(evidence,harness,pause_at):
    e=evidence;h=harness;h.block()
    checks=[]
    with e.attempt() as attempt:
        def check(selection, **kwargs):
            checks.append(selection)
            attempt.check_active(2_040_000_000)
        live=types.SimpleNamespace(check=check,refresh_idle=lambda:None)
        if pause_at=='before_select':authority.set_control(e.owner/'authorization.json','paused')
        if pause_at=='after_commit':
            original=h.clock.step
            def pause_after_reset():
                original()
                authority.set_control(e.owner/'authorization.json','paused')
            h.clock.step=pause_after_reset
        if pause_at=='none':
            h.run(attempt,live)
            assert h.bus.runner_count == 1
            assert checks == [h.prior,h.selected]
        else:
            with pytest.raises(ValueError,match='PAUSED_REVOKED_OR_COMPLETED'):h.run(attempt,live)
            assert h.bus.runner_count == 0
            assert h.bus.select_count == (0 if pause_at=='before_select' else 1)
        assert h.bus.prewarm_closed


def test_legacy_main_ast_is_unchanged_when_continuation_option_is_absent():
    """Project only the optional branches, then compare every old statement."""
    candidate=ast.parse((RUNTIME/'run_reboot_realigned_grasp_20261010.py').read_text())
    class Legacy(ast.NodeTransformer):
        def known(self,node):
            if not isinstance(node,ast.Compare) or len(node.ops)!=1 or not isinstance(node.comparators[0],ast.Constant):return None
            if node.comparators[0].value is not None:return None
            left=node.left
            if ((isinstance(left,ast.Name) and left.id in ('continuation','runtime'))
                    or (isinstance(left,ast.Attribute) and left.attr=='continuation_authorization')):
                return isinstance(node.ops[0],ast.Is)
        def visit_If(self,node):
            known=self.known(node.test)
            if known is not None:
                return [self.visit(n) for n in (node.body if known else node.orelse)]
            return self.generic_visit(node)
        def visit_IfExp(self,node):
            known=self.known(node.test)
            return self.visit(node.body if known else node.orelse) if known is not None else self.generic_visit(node)
        def visit_With(self,node):
            if isinstance(node.items[0].context_expr,ast.Call) and isinstance(node.items[0].context_expr.func,ast.Name) and node.items[0].context_expr.func.id=='ExitStack':
                result=[]
                for child in node.body:
                    transformed=self.visit(child)
                    result.extend(transformed if isinstance(transformed,list) else [transformed] if transformed is not None else [])
                return result
            return self.generic_visit(node)
        def visit_Assign(self,node):
            if all(isinstance(n,ast.Name) and n.id in ('continuation','runtime') for n in node.targets):return None
            return self.generic_visit(node)
        def visit_Expr(self,node):
            if isinstance(node.value,ast.Call) and node.value.args and isinstance(node.value.args[0],ast.Constant) and node.value.args[0].value=='--continuation-authorization':return None
            return self.generic_visit(node)
    new=next(n for n in candidate.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    # SHA of the complete deployed legacy main AST before this extension.
    projected = ast.dump(Legacy().visit(new),include_attributes=False).encode()
    assert hashlib.sha256(projected).hexdigest() == 'd02da32918d442ebd6df34141728b89fed6ad3b1af0f109f79f3dae3638c5b97'
