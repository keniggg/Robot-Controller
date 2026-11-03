"""In-memory candidate only; original callbacks, fixtures and gates, no ROS I/O."""
from pathlib import Path
from copy import deepcopy
from types import MethodType
import ast
import hashlib
import itertools
import json
import os
import sys
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
D = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
O = D/'verification/gen41_private_wait_memory_focus_r11'
O.mkdir(exist_ok=False)
for name in ('ros','tmp'): (O/name).mkdir()
os.environ.update(PYTHONDONTWRITEBYTECODE='1', ROS_MASTER_URI='http://127.0.0.1:1',
                  ROS_LOG_DIR=str(O/'ros'), TMPDIR=str(O/'tmp'),ROS_PACKAGE_PATH=str(W/'src'))
os.environ['OFFLINE_BOUNDARY_LOG']=str(O/'boundary_attempts.jsonl')
bootstrap=D/'verification/gen40_combined_r1/bootstrap/sitecustomize.py'
exec(compile(bootstrap.read_bytes(),str(bootstrap),'exec'),{'__name__':'gen41_memory_boundary'})
for name in ('devel/lib/python3/dist-packages','src/alicia_flexible_grasp_supervisor/src',
             'src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/tests',
             'src/alicia_grasp_modes/tests'):
    sys.path.insert(0,str(W/name))

import numpy as np
import pytest
from _pytest.monkeypatch import MonkeyPatch
from test_observation_measurement_remote import context, at
from test_station_compute_wait_remote import station, row_at, clock, feedback
from test_station_compute_work_handoff import submitted, rgbd
from test_gen11_contact_remote_receipt import contact_plan
from test_unknown_pending_identity import deliver
from alicia_grasp_modes import station_compute_wait_remote as impl
from alicia_grasp_modes.feedback_bracket_gap import validate_station_feedback_gap
from alicia_grasp_modes.unknown_pending_identity import reference_binding

SOURCE=W/'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py'
source_bytes=SOURCE.read_bytes()
release_bytes=(D/'verification/gen40_release_manifest.json').read_bytes()
release=json.loads(release_bytes)
sha=lambda b:hashlib.sha256(b).hexdigest()
assert len(release['source_sha256'])==1029
assert all(sha((W/n).read_bytes())==h for n,h in release['source_sha256'].items())

# The candidate changes two in-memory expressions only. The real same-source
# mask/object/status/parent/feedback/recovery/context checks remain in place.
tree=ast.parse(source_bytes)
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='StationComputeWaitRemoteMixin')
quarantine=deepcopy(next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_station_compute_quarantine'))
quarantine.args.kwonlyargs.append(ast.arg(arg='preserve_unpublished_work'))
quarantine.args.kw_defaults.append(ast.Constant(False))
changes=[]
for n in ast.walk(quarantine):
    if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='soft' for t in n.targets):
        assert isinstance(n.value,ast.Call) and isinstance(n.value.args[0],ast.BoolOp)
        assert isinstance(n.value.args[0].values[0],ast.Name) and n.value.args[0].values[0].id=='clock_only'
        n.value.args[0].values[0]=ast.BoolOp(op=ast.Or(),values=[ast.Name('clock_only',ast.Load()),ast.Name('preserve_unpublished_work',ast.Load())])
        changes.append('quarantine private retention independent of truthful clock_only wire flag')
join=deepcopy(next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_station_compute_join_locked'))
class GapRecovery(ast.NodeTransformer):
    def visit_Call(self,n):
        # Extracted method retains the original class's exact super MRO.
        if isinstance(n.func,ast.Name) and n.func.id=='super' and not n.args and not n.keywords:
            n.args=[ast.Name('StationComputeWaitRemoteMixin',ast.Load()),ast.Name('self',ast.Load())]
        return self.generic_visit(n)
    def visit_Assign(self,n):
        if len(n.targets)==2 and all(isinstance(t,ast.Name) for t in n.targets) and {t.id for t in n.targets}=={'recovered','hard_recovery'}:
            assert isinstance(n.value,ast.Constant) and n.value.value is True
            changes.append('recovered typed gap does not force hard cutoff when private quarantine retained')
            return [ast.Assign(targets=[ast.Name('recovered',ast.Store())],value=ast.Constant(True)),
                    ast.Assign(targets=[ast.Name('hard_recovery',ast.Store())],value=ast.UnaryOp(ast.Not(),ast.Attribute(ast.Name('self',ast.Load()),'_station_compute_soft_wait',ast.Load())))]
        return self.generic_visit(n)
join=GapRecovery().visit(join)
mod=ast.fix_missing_locations(ast.Module(body=[quarantine,join],type_ignores=[]))
ns=dict(vars(impl));exec(compile(mod,'<memory-only-gen41-candidate>','exec'),ns)
assert len(changes)==2

ACTUAL_PARENT=1792224864862662315
ACTUAL_GAP=1792224865330518007
ACTUAL_PENDING_RECEIVED=1792224865624195575
ACTUAL_READY_SOURCE=1792224865779169321
ACTUAL_READY_RECEIVED=1792224866145929098
ACTUAL_BEFORE=1792224865305582708
ACTUAL_AFTER=1792224865585654750

def make():
    mp=MonkeyPatch()
    c=station.__wrapped__(context.__wrapped__(mp),mp)
    t,row,ticket=submitted(c)
    n=c.node;plan=contact_plan(n,ticket)
    n._station_compute_quarantine=MethodType(ns['_station_compute_quarantine'],n)
    n._station_compute_join_locked=MethodType(ns['_station_compute_join_locked'],n)
    original_quarantine=n._station_compute_quarantine
    state=dict(staged={},verified={},eligible=set(),events=[],rejected=False)
    original=dict(status=n._station_compute_status_cb,mask=n.mask_cb,object=n.object_cb)
    # Only a fully validated discarded-source receipt can select this flag.
    def quarantine_route(self,source_ns=None,*,clock_only=False):
        keep=source_ns in state['eligible']
        return original_quarantine(source_ns,clock_only=clock_only,preserve_unpublished_work=keep)
    n._station_compute_quarantine=MethodType(quarantine_route,n)
    parent=deepcopy(n._station_compute_reference['contract'])
    generation=n._capture_geometry_generation()
    def retire(reason):
        state['rejected']=True
        n._station_compute_retire('memory pending fan-in: '+reason)
    def hold():
        # Historical evidence only while a typed source is unresolved. No
        # READY enrollment/publication/clock renewal occurs in this branch.
        original_quarantine(clock_only=False,preserve_unpublished_work=True)
    def typed_gap(status,obj,mask):
        ref=n._station_compute_reference
        n._station_compute_context(ref['contract'])
        now=n._ros_source_clock_ns();source_ns=status['source_stamp_ns']
        if (status.get('state')!='processing_deadline_pending'
                or status.get('detail')!='station_accepted_feedback_bracket_pending_revalidation'
                or status.get('target_lost') is not False or status.get('first_loss') is not None
                or reference_binding(status,n._mode_selection)!=ref['contract']['binding']['reference_binding']
                or obj.detected is not False or obj.header.stamp.to_nsec()!=source_ns
                or obj.header.frame_id!='base_link' or obj.label!='unknown_small'
                or mask.header.stamp.to_nsec()!=source_ns
                or mask.header.frame_id!=ref['mask']['frame_id']):
            raise ValueError('pending source/topic identity contradiction')
        pixels=n.bridge.imgmsg_to_cv2(mask,desired_encoding='mono8')
        if list(pixels.shape)!=ref['mask']['shape'] or np.any(pixels):
            raise ValueError('discarded gap mask is not matching empty image')
        impl.validate_recovery_parent(ref['contract'],parent=status.get('station_compute_wait'),
            chain=status.get('station_compute_recovery_chain'),binding=ref['contract']['binding'],now_ns=now)
        stamps=validate_station_feedback_gap(status.get('station_feedback_gap'),
            contract=status['station_compute_wait'],now_ns=now)
        if source_ns!=stamps[-1] or status.get('discarded_source_stamp_ns')!=source_ns:
            raise ValueError('pending source/receipt mismatch')
        if any(source_ns in p['source_stamps_ns'] for p in n._station_compute_work.values()):
            raise ValueError('original immutable input cannot become discarded')
        return stamps
    def route_locked(kind,value):
        # Prototype cache policy, not a production edit. All target/control/
        # phase/feedback ownership gates are invoked on every held input.
        source_ns=value['source_stamp_ns'] if kind=='status' else value.header.stamp.to_nsec()
        if kind=='status' and value.get('state')=='ready' and value.get('station_feedback_gap_recovery') is not None:
            # A recovery READY can be the first notification that any older
            # frame was discarded. Enroll its unresolved source obligations
            # before handing READY to the original join, never after it.
            try:
                ref=n._station_compute_reference
                n._station_compute_context(ref['contract'])
                now=n._ros_source_clock_ns();parent_gap=value.get('station_compute_recovery')
                if (value.get('target_lost') is not False or value.get('first_loss') is not None
                        or reference_binding(value,n._mode_selection)!=ref['contract']['binding']['reference_binding']):
                    raise ValueError('recovery READY changed current owner/reference')
                impl.validate_recovery_parent(ref['contract'],parent=parent_gap,
                    chain=value.get('station_compute_recovery_chain'),binding=ref['contract']['binding'],
                    now_ns=now,source_ns=source_ns)
                declared=validate_station_feedback_gap(value['station_feedback_gap_recovery'],
                    contract=parent_gap,now_ns=now,recovery_source_ns=source_ns)
                for discarded in declared:
                    if discarded not in state['verified']:
                        if discarded<=ticket.payload[0].stamp_ns:
                            raise ValueError('recovery READY discarded an original immutable input')
                        state['staged'].setdefault(discarded,{})
            except (ValueError,KeyError,TypeError,AttributeError) as exc:
                return retire(str(exc))
        if source_ns in state['verified']:
            try:
                n._station_compute_context(n._station_compute_reference['contract'])
                if state['verified'][source_ns].get(kind)!=value:
                    raise ValueError('confirmed discarded source changed')
                return
            except (ValueError,KeyError,TypeError,AttributeError) as exc:
                return retire(str(exc))
        if kind=='status' and (value.get('target_lost') is True or value.get('first_loss') is not None):
            return original[kind](value)
        if source_ns in state['staged'] or ((kind=='object' and value.detected is False)
                or (kind=='mask' and not np.any(n.bridge.imgmsg_to_cv2(value,desired_encoding='mono8')))
                or (kind=='status' and value.get('state')=='processing_deadline_pending')):
            try:
                ref=n._station_compute_reference
                if ref is None or n._station_compute_retired:
                    return original[kind](value)
                n._station_compute_context(ref['contract'])
                impl.validate_contract(ref['contract'],binding=ref['contract']['binding'],now_ns=n._ros_source_clock_ns())
                # Exact clock notice: existing registered source, no new failed
                # frame, no gap receipt. Keep original READY fact immutable.
                if (kind=='status' and value.get('detail')=='processing_deadline_pending'
                        and source_ns==ref['contract']['source_ns']
                        and value.get('discarded_source_stamp_ns') is None
                        and value.get('station_feedback_gap') is None and value.get('station_support_gap') is None):
                    impl.validate_recovery_parent(ref['contract'],parent=value.get('station_compute_wait'),
                        chain=value.get('station_compute_recovery_chain'),binding=ref['contract']['binding'],now_ns=n._ros_source_clock_ns())
                    if (reference_binding(value,n._mode_selection)!=ref['contract']['binding']['reference_binding']
                            or value.get('target_lost') is not False or value.get('first_loss') is not None
                            or n._ros_source_clock_ns()<ref['contract']['geometry_deadline_ns']):
                        raise ValueError('clock notification does not match expired registered owner')
                    original_quarantine(clock_only=True)
                    state['events'].append('typed clock retained original private work; no READY overwrite')
                    return
                if source_ns<=ticket.payload[0].stamp_ns:
                    return original[kind](value)  # Original-input contradiction remains hard.
                cached=state['staged'].setdefault(source_ns,{})
                if kind in cached and cached[kind]!=value:
                    raise ValueError('same source/topic changed')
                cached[kind]=deepcopy(value)
                hold()
                if set(cached)=={'mask','object','status'}:
                    stamps=typed_gap(cached['status'],cached['object'],cached['mask'])
                    state['eligible'].update(stamps)
                    # Canonical fan-in uses the real callback implementations;
                    # no synthetic READY or qualified failed frame is produced.
                    for name in ('status','mask','object'): original[name](deepcopy(cached[name]))
                    state['verified'][source_ns]=deepcopy(cached)
                    del state['staged'][source_ns]
                    state['events'].append('fully paired rejected-gap source retained as nonmetric tombstone')
                    if not state['staged']:
                        for ready_kind,ready_value in state.pop('held_ready',[]):
                            original[ready_kind](ready_value)
                        state['events'].append('held newer READY released only after old gap receipt completed')
                return
            except (ValueError,KeyError,TypeError,AttributeError) as exc:
                return retire(str(exc))
        if state['staged'] and kind=='status' and value.get('state')=='ready':
            # A newer READY can arrive before an older gap's receipt. It is
            # withheld as current authority, not used to destroy history.
            state.setdefault('held_ready',[]).append((kind,deepcopy(value)))
            hold();return
        return original[kind](value)
    def route(kind,value):
        with n._mode_control_lock,n._stream_condition,n._geometry_state_guard():
            return route_locked(kind,value)
    return c,mp,ticket,plan,parent,generation,state,route,original

def gap_row(c,parent):
    source=parent['source_ns']+(ACTUAL_GAP-ACTUAL_PARENT)
    before=parent['source_ns']+(ACTUAL_BEFORE-ACTUAL_PARENT)
    after=parent['source_ns']+(ACTUAL_AFTER-ACTUAL_PARENT)
    now=parent['source_ns']+(ACTUAL_PENDING_RECEIVED-ACTUAL_PARENT)
    feedback(c,before);feedback(c,after);clock(c,now)
    row=row_at(c,source)
    row['object'].detected=False
    row['mask'].data=bytes(len(row['mask'].data))
    evidence=dict(accepted_before=[before,[0.]*6],accepted_after=[after,[0.]*6],
        bracket_qualified=False,maximum_span_ns=200_000_000,span_ns=ACTUAL_AFTER-ACTUAL_BEFORE)
    gap=dict(discarded_source_stamp_ns=source,discarded_source_stamps_ns=[source],
        feedback_gap_evidence=deepcopy(evidence),discarded_source_evidence=[dict(source_stamp_ns=source,feedback_gap_evidence=deepcopy(evidence))],
        source_qualified=False,identity_verified=False,metric_geometry_authority=False,motion_authorized=False,reference_refreshed=False)
    row['status'].update(state='processing_deadline_pending',detail='station_accepted_feedback_bracket_pending_revalidation',
        target_lost=False,first_loss=None,discarded_source_stamp_ns=source,
        station_compute_wait=deepcopy(parent),station_compute_recovery_chain=[deepcopy(parent)],station_feedback_gap=gap)
    return row

def fresh_after_gap(c,parent,gap):
    source=parent['source_ns']+(ACTUAL_READY_SOURCE-ACTUAL_PARENT)
    now=parent['source_ns']+(ACTUAL_READY_RECEIVED-ACTUAL_PARENT)
    feedback(c,source-10_000_000);feedback(c,source+10_000_000);clock(c,now)
    row=row_at(c,source,recovery=parent)
    row['status']['station_feedback_gap_recovery']=deepcopy(gap['status']['station_feedback_gap'])
    rgbd(c,row)
    return row

rows=[];negative_rows=[];early_ready_rows=[];zero_fragment_rows=[]
try:
    for order in itertools.permutations(('status','object','mask')):
        c,mp,ticket,plan,parent,generation,state,route,original=make();n=c.node
        gap=gap_row(c,parent)
        for kind in order:
            route(kind,deepcopy(gap[kind]));route(kind,deepcopy(gap[kind]))
            assert not n.latest_object_geometry.valid and not n.latest_preview_rich_plan.valid
            published,code=n._publish_plan_pair_if_current(plan,generation)
            assert published is False and code=='TARGET_PROCESSING_PENDING'
        assert not state['staged'] and not state['rejected'], state
        assert n._station_compute_work_certificate(ticket) and n._capture_geometry_generation()==generation
        assert n._station_compute_pending and n._station_compute_soft_wait
        assert gap['status']['station_feedback_gap']['source_qualified'] is False
        recovered=fresh_after_gap(c,parent,gap)
        deliver(n,recovered)
        assert not n._station_compute_pending and n._station_compute_work_certificate(ticket)
        assert n._capture_geometry_generation()==generation
        receipt=n._station_contact_computation_receipt(plan,ticket)
        assert receipt['motion_authorized'] is False and receipt['metric_geometry_authority'] is False
        assert receipt['plan_source_ns']==ticket.payload[0].stamp_ns
        assert receipt['current']['contract']['source_ns']==recovered['status']['source_stamp_ns']
        # The subsequent actual idle pattern is also pure age of the joined
        # registered source. Source stamps and original deadlines are unchanged.
        p2=deepcopy(n._station_compute_reference['contract'])
        clock(c,p2['geometry_deadline_ns']+86_252_000)
        idle=deepcopy(recovered['status']);idle.update(state='processing_deadline_pending',detail='processing_deadline_pending',
            target_lost=False,first_loss=None,discarded_source_stamp_ns=None)
        idle.pop('station_feedback_gap_recovery',None)
        route('status',idle)
        assert n._station_compute_work_certificate(ticket) and n._station_compute_pending
        assert n._capture_geometry_generation()==generation
        assert not n.latest_object_geometry.valid
        rows.append(dict(order=order,duplicates=True,gap_span_ns=280072042,maximum_span_ns=200000000,
            failed_source_qualified=False,ready_receive_delta_ns=ACTUAL_READY_RECEIVED-ACTUAL_PENDING_RECEIVED,
            original_plan_source_ns=ticket.payload[0].stamp_ns,original_phase_deadline_ns=parent['phase_deadline_ns'],
            private_work_retained=True,pending_publication_blocked=True,fresh_complete_ready_receipt_passed=True,
            fresh_receipt_motion_authorized=False,pure_idle_retained=True,events=state['events']))
        mp.undo()
    # A READY status can overtake the old gap receipt. Both complete sources
    # remain distinct; no authority is granted until the older gap is explained.
    c,mp,ticket,plan,parent,generation,state,route,original=make();n=c.node
    gap=gap_row(c,parent)
    route('object',deepcopy(gap['object']));route('mask',deepcopy(gap['mask']))
    recovered=fresh_after_gap(c,parent,gap)
    for kind in ('mask','object','status'):route(kind,deepcopy(recovered[kind]))
    assert n._station_compute_pending and n._station_compute_work_certificate(ticket)
    assert state['held_ready'] and state['staged']
    published,code=n._publish_plan_pair_if_current(plan,generation)
    assert not published and code=='TARGET_PROCESSING_PENDING'
    route('status',deepcopy(gap['status']))
    assert not state['staged'] and not state.get('held_ready') and not state['rejected']
    assert not n._station_compute_pending and n._station_compute_work_certificate(ticket)
    assert n._capture_geometry_generation()==generation
    early_ready_rows.append(dict(ready_overtook_gap_receipt=True,publication_blocked_until_receipt=True,
        original_private_work_retained=True,fresh_full_join_after_gap_completed=True))
    mp.undo()
    for ready_order in itertools.permutations(('status','object','mask')):
        for gap_order in itertools.permutations(('status','object','mask')):
            c,mp,ticket,plan,parent,generation,state,route,original=make();n=c.node
            gap=gap_row(c,parent)
            recovered=fresh_after_gap(c,parent,gap)
            for kind in ready_order:
                route(kind,deepcopy(recovered[kind]));route(kind,deepcopy(recovered[kind]))
            assert state['staged'] and state['held_ready']
            assert n._station_compute_pending and n._station_compute_work_certificate(ticket)
            assert n._capture_geometry_generation()==generation
            published,code=n._publish_plan_pair_if_current(plan,generation)
            assert not published and code=='TARGET_PROCESSING_PENDING'
            for kind in gap_order:
                route(kind,deepcopy(gap[kind]));route(kind,deepcopy(gap[kind]))
            assert not state['staged'] and not state.get('held_ready') and not state['rejected']
            assert not n._station_compute_pending and n._station_compute_work_certificate(ticket)
            assert n._capture_geometry_generation()==generation
            zero_fragment_rows.append(dict(ready_order=ready_order,gap_order=gap_order,duplicates=True,
                ready_first_declared_old_gap=True,private_work_never_destroyed=True,
                publication_blocked_before_old_receipt=True,full_fresh_join_only_after_gap_complete=True))
            mp.undo()
    for fault in ('missing_receipt','changed_parent','support_reference_changed','owner_selection',
                  'control_epoch','manual_takeover','measured_pose_changed','true_target_loss',
                  'original_input_mask_changed','discarded_source_became_true','identity_deadline'):
        c,mp,ticket,plan,parent,generation,state,route,original=make();n=c.node
        gap=gap_row(c,parent)
        valid_gap=deepcopy(gap)
        if fault=='missing_receipt':gap['status'].pop('station_feedback_gap')
        elif fault=='changed_parent':gap['status']['station_compute_wait']['binding']['config_sha256']='e'*64
        elif fault=='support_reference_changed':gap['status']['reference_support_plane_base'][3]+=.01
        for kind in ('status','mask','object'):
            if not state['rejected']:route(kind,deepcopy(gap[kind]))
        if fault=='owner_selection':
            from alicia_grasp_modes.selection import SELECTION_PARAM
            c.params[SELECTION_PARAM]={}
            n._station_compute_poll()
        elif fault=='control_epoch':n._measurement_remote_epoch_ns+=1;n._station_compute_poll()
        elif fault=='manual_takeover':c.params['/gui/joint_direct_mode']=True;n._station_compute_poll()
        elif fault=='measured_pose_changed':
            feedback(c,n._ros_source_clock_ns(),q=[.02,0,0,0,0,0]);n._station_compute_poll()
        elif fault=='true_target_loss':
            status=deepcopy(gap['status']);status.update(target_lost=True,first_loss={'reason':'explicit real loss'})
            route('status',status)
        elif fault=='original_input_mask_changed':
            input_ns=ticket.payload[0].sample_stamp_ns[0]
            bad=deepcopy(n._station_compute_masks[input_ns][2])
            pixels=n.bridge.imgmsg_to_cv2(bad,desired_encoding='mono8').copy();pixels.flat[0]^=255
            changed=n.bridge.cv2_to_imgmsg(pixels,encoding='mono8');changed.header=deepcopy(bad.header)
            original['mask'](changed)
        elif fault=='discarded_source_became_true':
            bad=deepcopy(gap['object']);bad.detected=True;route('object',bad)
        elif fault=='identity_deadline':clock(c,parent['identity_deadline_ns']+1_000_000);n._station_compute_poll()
        assert not n._station_compute_work_certificate(ticket),fault
        assert not n.latest_object_geometry.valid,fault
        assert not bool(getattr(n.latest_preview_rich_plan,'valid',False)),fault
        # A later measured source never recreates the destroyed original work.
        if fault in ('missing_receipt','changed_parent','support_reference_changed','true_target_loss'):
            recovered=fresh_after_gap(c,parent,valid_gap)
            deliver(n,recovered)
            assert not n._station_compute_work_certificate(ticket),fault
        negative_rows.append(dict(fault=fault,original_private_work_destroyed=True,
            public_geometry_invalid=True,public_preview_invalid=True,original_ticket_never_recreated=True))
        mp.undo()
    assert all(sha((W/n).read_bytes())==h for n,h in release['source_sha256'].items())
    assert SOURCE.read_bytes()==source_bytes
    attempts=(O/'boundary_attempts.jsonl').read_text().splitlines() if (O/'boundary_attempts.jsonl').exists() else []
    assert not attempts
    result=dict(schema_version=1,passed=True,production_or_old_tests_edited=False,source_count=1029,source_unchanged=True,
        kind='memory_only_candidate_with_real_remote_callbacks_and_existing_synthetic_station_fixture',
        full_software_or_live_end_to_end=False,motion_or_ROS_contacted=False,physical_grasp_verified=False,
        source_sha256=sha(source_bytes),in_memory_AST_changes=changes,cases=rows,
        newer_ready_before_old_receipt=early_ready_rows,hard_negative_cases=negative_rows,
        zero_fragment_ready_first_cases=zero_fragment_rows,
        actual_record_adaptation='Exact G40 280072042 ns bracket and .521733523 s recovery timing mapped to synthetic fixture owner/source; no real frame is qualified or interpolated.',
        limits=['Fan-in router itself is a prototype, not deployed code.',
            'This validates private evidence/receipt state, not current physical geometry, FK, a feasible path or movement.',
            'A fresh receipt is non-authorizing; all original final geometry/SDK/path checks remain required.'],external_boundary_attempts=attempts)
except BaseException as exc:
    import traceback
    result=dict(passed=False,kind='memory_only_candidate_probe_failure',error=repr(exc),traceback=traceback.format_exc(),
        completed_cases=rows,completed_negative_cases=negative_rows,completed_early_ready_cases=early_ready_rows,
        completed_zero_fragment_ready_first_cases=zero_fragment_rows,
        production_or_old_tests_edited=False,motion_or_ROS_contacted=False)
    raise
finally:
    data=(json.dumps(result,ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)+'\n').encode()
    (O/'result.json').write_bytes(data)
    print(json.dumps(dict(output=str(O),passed=result['passed'],result_sha256=sha(data)),ensure_ascii=False))
