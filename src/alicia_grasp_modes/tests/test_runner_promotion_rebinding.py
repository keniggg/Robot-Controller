"""Promotion follows a fresh exact plan, never a stale ID or borrowed audit."""
from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

from test_runner_observe_return import observe_evidence, runner_for
from test_mode_runner import module, working_volume_view


class Clock:
    def __init__(self): self.value = 0.
    def now(self): return self.value
    def sleep(self, seconds): self.value += seconds


def rebinding_context(monkeypatch, phase='observation_prefix'):
    selection, report, preview = observe_evidence(phase)
    if phase == 'initial_contact':
        report['observe_return']['initial_surface_stamp_ns'] = report['snapshot_stamp_ns']
    runner = runner_for(deepcopy(selection))
    clock = Clock()
    monkeypatch.setattr(module.time, 'monotonic', clock.now)
    monkeypatch.setattr(module.time, 'sleep', clock.sleep)
    monkeypatch.setattr(module.rospy, 'is_shutdown', lambda: False)
    monkeypatch.setattr(module.rospy, 'get_param', lambda name, default=None: selection)
    monkeypatch.setattr(module.rospy.Time, 'now', staticmethod(lambda: module.rospy.Time(23)))
    # Message-integrity implementation has its own full-wire tests. This
    # fixture uses simple pose objects so exercise every other real gate.
    monkeypatch.setattr(module, 'plan_id_matches_content', lambda p: p.plan_id in ('a'*24,'b'*24))
    runner.on_preview(preview)
    runner.committed_audits[preview.plan_id] = report
    replacement, own = deepcopy(preview), deepcopy(report)
    replacement.plan_id = own['plan_id'] = own['selected']['final_strict_plan_id'] = 'b'*24
    replacement.header.stamp.secs += 1
    own['snapshot_stamp_ns'] += 1000000000
    own['observe_return']['origin_snapshot_stamp_ns'] = own['snapshot_stamp_ns']
    if phase == 'observation_prefix':
        own['selected']['observation_view'] = working_volume_view(own['snapshot_stamp_ns'])
    if phase == 'initial_contact':
        own['observe_return']['initial_surface_stamp_ns'] = own['snapshot_stamp_ns']
    replacement.poses[0].position.x += .002
    own['selected']['execution_sequence']['stages'][0]['position_m'][0] += .002
    return runner, clock, preview, replacement, own, selection


@pytest.mark.parametrize('phase', ['observation_prefix', 'initial_contact'])
def test_promotion_replacement_uses_own_audit_current_plan_and_same_deadline(monkeypatch, phase):
    runner,clock,old,new,report,_ = rebinding_context(monkeypatch, phase)
    calls=[];active=['none']
    def current(_):
        calls.append(('current',active[0]))
        return NS(success=active[0]!='none', message='plan_id='+active[0])
    def promote(_):
        calls.append(('promote',))
        runner.on_preview(new)
        runner.committed_audits[new.plan_id]=report
        active[0]=new.plan_id
        return NS(success=True,message='queued')
    result=runner.wait_for_execution_authority(current,promote,old,old.plan_id,1.)
    assert result[1] == new and result[0].message == 'plan_id='+new.plan_id
    assert runner.preview == old  # attempt identity remains anchored
    assert calls.count(('promote',)) == 1
    assert clock.value < 1.


@pytest.mark.parametrize('fault',['missing_audit','borrowed_audit','changed_pose','changed_source',
    'foreign_selection','weak_strict','different_target','different_phase','different_model',
    'old_source','future_source','stale_source','invalid_hash','missing_target','changed_epoch'])
def test_replacement_cannot_borrow_or_change_authority(monkeypatch,fault):
    runner,clock,old,new,report,_ = rebinding_context(monkeypatch)
    if fault=='borrowed_audit':report=deepcopy(runner.committed_audits[old.plan_id])
    elif fault=='changed_pose':new.poses[0].position.z+=.01
    elif fault=='changed_source':report['snapshot_stamp_ns']+=1
    elif fault=='foreign_selection':report['grasp_mode_selection']['generation']+=1
    elif fault=='weak_strict':report['selected']['moveit']['collision_free']=False
    elif fault=='different_target':new.target_track_id=report['observe_return']['target_track_id']='other'
    elif fault=='different_phase':new.diagnostic='CONTACT_EXECUTION_PLAN'
    elif fault=='different_model':new.model_choice='carton_segment'
    elif fault=='old_source':new.header.stamp.secs=19
    elif fault=='future_source':new.header.stamp.secs=24
    elif fault=='stale_source':runner.plan_validity_sec=.5
    elif fault=='invalid_hash':new.plan_id='c'*24
    elif fault=='missing_target':new.target_track_id=''
    elif fault=='changed_epoch':report['target_epoch']=report['observe_return']['target_epoch']=8
    if fault!='missing_audit':runner.committed_audits[new.plan_id]=report
    runner.on_preview(new)
    promotions=[]
    def current(_):return NS(success=True,message='plan_id='+new.plan_id)
    def promote(_):promotions.append(True);return NS(success=True,message='queued')
    result=runner.wait_for_execution_authority(current,promote,old,old.plan_id,.3)
    assert result is None
    assert clock.value < .5
    assert len(promotions)<=1


def test_multiple_new_previews_do_not_restart_deadline_or_repeat_promotion(monkeypatch):
    runner,clock,old,new,report,_=rebinding_context(monkeypatch)
    calls=[]
    def current(_):
        runner.on_preview(new)
        runner.committed_audits[new.plan_id]=report
        return NS(success=False,message='no execution authority')
    def promote(_):calls.append(True);return NS(success=True,message='queued')
    assert runner.wait_for_execution_authority(current,promote,old,old.plan_id,.3) is None
    assert .3<=clock.value<.5 and calls==[True]


def test_slow_current_plan_cannot_grant_authority_after_original_deadline(monkeypatch):
    runner,clock,old,new,report,_=rebinding_context(monkeypatch)
    def current(_):
        clock.value=1.
        return NS(success=True,message='plan_id='+old.plan_id)
    assert runner.wait_for_execution_authority(current,lambda _:pytest.fail('late promotion'),old,old.plan_id,.3) is None


def test_run_passes_replacement_plan_id_to_all_final_checks_and_start(monkeypatch):
    runner,clock,old,new,report,selection=rebinding_context(monkeypatch)
    calls=[]
    monkeypatch.setattr(module.rospy,'init_node',lambda *a,**kw:None)
    monkeypatch.setattr(module.rospy,'wait_for_service',lambda *a,**kw:None)
    monkeypatch.setattr(module.rospy,'Subscriber',lambda *a,**kw:None)
    monkeypatch.setattr(module.rospy.Time,'now',staticmethod(lambda:module.rospy.Time(20)))
    monkeypatch.setattr(module.rospy,'get_param',lambda name,default=None:
        120. if name=='/grasp_6d/plan_validity_sec' else selection)
    runner.wait_for_preview=lambda _:old
    runner.committed_audits[new.plan_id]=report
    runner.wait_for_execution_authority=lambda *a:(NS(success=True),new)
    def inference(flag):calls.append(('inference',flag));return NS(success=True,message='ok')
    def current(flag):calls.append(('current',new.plan_id));return NS(success=True,message='plan_id='+new.plan_id)
    def start(flag,plan_id):calls.append(('start',plan_id));return NS(success=True,message='done')
    services={'/grasp_6d/request_plan':inference,'/grasp_6d/replan_execution':lambda _:None,
        '/grasp/current_plan':current,'/grasp/start':start}
    monkeypatch.setattr(module.rospy,'ServiceProxy',lambda name,*a:services[name])
    assert runner.run()==0
    assert ('start',new.plan_id) in calls and ('start',old.plan_id) not in calls
    assert calls[-1]==('inference',False)


@pytest.mark.parametrize('fault', ['old_initial_surface', 'old_origin', 'changed_epoch'])
def test_fresh_initial_contact_cannot_reuse_old_surface_or_target_epoch(monkeypatch, fault):
    runner, clock, old, new, report, _ = rebinding_context(monkeypatch, 'initial_contact')
    previous = runner.committed_audits[old.plan_id]
    if fault == 'old_initial_surface':
        report['observe_return']['initial_surface_stamp_ns'] = previous['snapshot_stamp_ns']
    elif fault == 'old_origin':
        report['observe_return']['origin_snapshot_stamp_ns'] = previous['snapshot_stamp_ns']
    else:
        report['target_epoch'] = report['observe_return']['target_epoch'] = 8
    runner.on_preview(new)
    runner.committed_audits[new.plan_id] = report
    current = lambda _: NS(success=True, message='plan_id=' + new.plan_id)
    promote = lambda _: NS(success=True, message='queued')
    assert runner.wait_for_execution_authority(current, promote, old, old.plan_id, .3) is None
    assert .3 <= clock.value < .5


def test_observation_prefix_replacement_keeps_same_initial_surface(monkeypatch):
    runner, _, old, new, report, _ = rebinding_context(monkeypatch)
    report['observe_return']['initial_surface_stamp_ns'] = report['snapshot_stamp_ns']
    assert not runner.audit_error(report)  # Valid in isolation, wrong for this attempt.
    runner.on_preview(new)
    runner.committed_audits[new.plan_id] = report
    current = lambda _: NS(success=True, message='plan_id=' + new.plan_id)
    promote = lambda _: NS(success=True, message='queued')
    assert runner.wait_for_execution_authority(current, promote, old, old.plan_id, .3) is None
