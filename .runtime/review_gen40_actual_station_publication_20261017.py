"""Read-only GEN40 station-publication contract diagnostic; no ROS imports."""
from pathlib import Path
from types import SimpleNamespace as NS
from contextlib import nullcontext
from copy import deepcopy
import ast
import builtins
import hashlib
import importlib.util
import json
import time

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
D = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen36_restart_20261017T012037Z')
O = D / 'verification/gen40_actual_attempt1_publication_review_r2'
REMOTE = W / 'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_remote.py'
PURE = W / 'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_reacquisition.py'
PRODUCER = W / 'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py'
BASE = W / 'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
MODE = W / 'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
LOG = D / 'gen40_runtime/compute_deployment_r1/consumers_ros/595e69d8-c9d0-11f1-9e20-e9c5a490651c/remote_grasp6d_node-2.log'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def write(name, value):
    p = O / name
    assert not p.exists(), p
    data = (json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + '\n').encode()
    p.write_bytes(data)
    assert p.read_bytes() == data
    return sha(data)


def main():
    O.mkdir(exist_ok=False)
    release_bytes = (D / 'verification/gen40_release_manifest.json').read_bytes()
    release = json.loads(release_bytes)
    before = {name: sha((W / name).read_bytes()) for name in release['source_sha256']}
    assert before == release['source_sha256'] and len(before) == 1029
    source_bytes = {str(p): p.read_bytes() for p in (REMOTE, PURE, PRODUCER, BASE, MODE)}
    spec = importlib.util.spec_from_file_location('gen40_diagnostic_reacquisition', PURE)
    pure = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pure)  # Pure module: copy + math, no ROS/device/network.
    tree = ast.parse(source_bytes[str(REMOTE)])
    mixin = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'StationComputeWaitRemoteMixin')
    names = {'_station_compute_status_cb', '_station_compute_quarantine',
             '_station_compute_ticket_code_geometry_locked'}
    funcs = [n for n in mixin.body if isinstance(n, ast.FunctionDef) and n.name in names]
    namespace = dict(deepcopy=deepcopy, json=json, CODE='TARGET_PROCESSING_PENDING',
                     reference_binding=lambda status, selection: status['reference_binding'],
                     validate_recovery_parent=pure.validate_recovery_parent)
    exec(compile(ast.Module(body=funcs, type_ignores=[]), str(REMOTE), 'exec'), namespace)
    producer_tree = ast.parse(source_bytes[str(PRODUCER)])
    producer_class = next(n for n in producer_tree.body if isinstance(n, ast.ClassDef)
                          and n.name == 'StationComputeWaitProducerMixin')
    producer_names = {'_station_compute_idle_wait', '_station_compute_status'}
    producer_funcs = [n for n in producer_class.body if isinstance(n, ast.FunctionDef)
                      and n.name in producer_names]
    def import_constant_only(name, *args, **kwargs):
        assert name == 'observe_return_measurement', name
        return NS(STATION_REQUALIFICATION='observation_station_requalification')
    producer_namespace = dict(deepcopy=deepcopy, time=time,
        rospy=NS(Time=NS(now=lambda: NS(to_nsec=lambda: 11_000_000_000))),
        __builtins__={**vars(builtins), '__import__':import_constant_only})
    exec(compile(ast.Module(body=producer_funcs, type_ignores=[]), str(PRODUCER), 'exec'), producer_namespace)

    class Message:
        def __init__(self):
            self.header = NS(stamp=None)

    class Node:
        def __init__(self):
            binding = dict(selection=dict(mode='unknown', strategy='observe_return', generation=5,
                stamp_ns=1_000_000_000), lease_id='lease', session_id='session', view_plan_id='view',
                view_index=1, target_epoch=4, target_track_id='g1-t4', control_epoch_ns=2_000_000_000,
                phase_id=4, phase_started_ns=8_000_000_000, phase_reference_source_ns=9_000_000_000,
                reference_binding='source-bound-reference', source_id='a'*64,
                camera_configuration_sha256='b'*64, config_sha256='c'*64, station_q_rad=[0.]*6)
            contract = pure.build_contract(source_ns=10_000_000_000, maximum_frame_age_sec=.8,
                session_deadline_ns=100_000_000_000, phase_deadline_ns=98_000_000_000, binding=binding)
            self._mode_selection = binding['selection']
            self._stream_condition = NS(__enter__=lambda _: None)
            self._stream_condition = nullcontext()
            # nullcontext itself has no notify_all; this wrapper provides both.
            self._stream_condition = self._mode_control_lock = NS()
            self._mode_control_lock = nullcontext()
            self.input_lock = nullcontext()
            self._stream_condition = type('Condition', (), dict(
                __enter__=lambda s: s, __exit__=lambda s,*a: False, notify_all=lambda s: None))()
            self._station_compute_reference = dict(contract=contract, object='object', mask='mask')
            self._station_compute_registration = dict(contract=deepcopy(contract),
                monotonic_deadline=time.monotonic()+4., ready_source=object(), recovery_chain=[])
            self._station_contact_history = self._station_requalification_hold = None
            self._station_supplement_proposal = None
            self._station_compute_status = {}
            self._station_compute_work = {26: dict(source_witnesses='retained immutable inputs')}
            self._station_compute_ready_history = {10_000_000_000: 'original READY'}
            self._station_compute_work_published = self._station_compute_publication_started = False
            self._station_compute_pending = self._station_compute_soft_wait = False
            self._station_compute_retired = False
            self._station_compute_cutoff_ns = self._station_compute_soft_cutoff_ns = 0
            self.near_field_planning_snapshot_frames = 3
            self.last_submitted_stamp_ns = 10_000_000_000
            self._pending_request_id = None
            self.target_instance_epoch = 4
            self._geometry_invalidation_generation = 0
            self._near_field_request_plan_cache = {'strict-success': 'evidence'}
            self._candidate_plan_metrics = {'candidate': 'metrics'}
            self._direct_near_field_submission_generation = 7
            self.events = []
            self.inference_coordinator = NS(reset_target_epoch=lambda epoch: self.events.append('coordinator reset'))
            self._observe_return_original = NS(geometry_estimate_to_message=lambda *a,**k: Message(),
                Grasp6DPlan=Message, PoseArray=Message,
                RemoteGrasp6DNode=NS(_publish_preview_plan=lambda *a,**k: None))
            self.geometry_pub = object()
        def _geometry_state_guard(self): return nullcontext()
        def _station_gap_late_status(self, status): return False
        def _station_contact_source_changed(self, kind, ns, status): return False
        def _station_requalification_withdraw_notice(self, status): return False
        def _unknown_identity_cache(self, cache, ns, status): cache[ns] = status
        def _station_compute_admit_ready_before_tick(self, ns): pass
        def _station_compute_tick_locked(self): pass
        def _station_compute_join_locked(self): pass
        def _ros_source_clock_ns(self): return 11_000_000_000
        def _station_compute_support_gap_receipt(self, status): pass
        def _configured_near_field_strategy(self): return 'single_snapshot_direct'
        def _clear_geometry_cache(self): self.events.append('geometry cache cleared')
        def _invalid_geometry_estimate(self, *a): return Message()
        def _safe_time(self): return None
        def _current_stream_target_identity(self): return NS(epoch=4, track_id='g1-t4')
        def _publish_invalidation_safely(self, *a): self.events.append('public geometry unavailable')
        def _station_compute_work_certificate(self, ticket): return ticket.request_id in self._station_compute_work
        def _station_compute_retire(self, reason):
            self._station_compute_retired = True
            self._station_compute_work.clear()
            self.events.append('retired: '+reason)
        def producer_scope(self, registration):
            assert registration is self._station_compute_registration
            pure.validate_contract(registration['contract'],
                binding=registration['contract']['binding'], now_ns=self._ros_source_clock_ns())
            return 'assumed-current fixed owner/reference/feedback'
        def status(self, state, detail, source):
            assert source is self._station_compute_registration['ready_source']
            contract=self._station_compute_registration['contract']
            payload=dict(generation=5, selection_stamp_ns=1_000_000_000, mode='unknown',
                source_stamp_ns=contract['source_ns'], state=state, detail=detail,
                target_lost=False, first_loss=None,
                reference_binding=contract['binding']['reference_binding'])
            producer_namespace['_station_compute_status'](self,payload,source)
            self.events.append('real producer idle status delivered')
            self._station_compute_status_cb(payload)

    for name in names:
        setattr(Node, name, namespace[name])
    ticket = NS(request_id=26, payload=[NS(sample_stamp_ns=(10_000_000_000,), stamp_ns=10_000_000_000)])
    rows = []
    for case in ('local_clock_only', 'same_registered_source_wire_clock_notice',
                 'actual_producer_idle_to_remote', 'real_target_loss'):
        node = Node()
        contract = node._station_compute_reference['contract']
        if case == 'local_clock_only':
            node._station_compute_quarantine(clock_only=True)
        elif case == 'actual_producer_idle_to_remote':
            # Producer scope is intentionally inert except for real immutable
            # contract clocks; all target/feedback/owner checks assume passing.
            node._station_compute_scope=node.producer_scope
            assert producer_namespace['_station_compute_idle_wait'](node) is True
        else:
            status = dict(generation=5, selection_stamp_ns=1_000_000_000, mode='unknown',
                source_stamp_ns=contract['source_ns'], state='processing_deadline_pending',
                detail='processing_deadline_pending', target_lost=case == 'real_target_loss', first_loss=None,
                station_compute_wait=deepcopy(contract), station_compute_recovery_chain=[deepcopy(contract)],
                reference_binding=contract['binding']['reference_binding'])
            node._station_compute_status_cb(status)
        code = node._station_compute_ticket_code_geometry_locked(ticket)
        row = dict(case=case, private_work_retained=26 in node._station_compute_work,
            pending=node._station_compute_pending, soft_wait=node._station_compute_soft_wait,
            cutoff_ns=node._station_compute_cutoff_ns, invalidation_generation=node._geometry_invalidation_generation,
            ticket_code=code, events=node.events)
        rows.append(row)
    assert rows[0]['private_work_retained'] and rows[0]['ticket_code'] == '' and rows[0]['soft_wait']
    assert not rows[1]['private_work_retained'] and rows[1]['ticket_code'] == 'TARGET_PROCESSING_PENDING'
    assert rows[1]['invalidation_generation'] == 1
    assert not rows[2]['private_work_retained'] and rows[2]['ticket_code'] == 'TARGET_PROCESSING_PENDING'
    assert rows[2]['invalidation_generation'] == 1
    assert not rows[3]['private_work_retained'] and rows[3]['ticket_code'] == 'TARGET_LOST'

    log_bytes = LOG.read_bytes()
    log_events = []
    for number, line in enumerate(log_bytes.decode().splitlines(), 1):
        markers = ('remote 6D pipeline metrics: ', 'remote 6D contact precheck delivery: ')
        for marker in markers:
            if marker not in line:
                continue
            payload = json.loads(line.split(marker, 1)[1])
            wanted = payload.get('request_id') in (25,26,27) if 'pipeline metrics' in marker else (
                payload.get('source_stamp_ns') == 1792224801385555982)
            if wanted:
                log_events.append(dict(line_number=number, original_line_sha256=sha(line.encode()),
                    prefix=line.split(marker,1)[0], kind=marker, payload=payload))
    log_h = write('selected_actual_log_events.json', dict(source=str(LOG),
        source_bytes_at_read=len(log_bytes), source_sha256_at_read=sha(log_bytes),
        source_may_remain_process_log=True, events=log_events))
    witness_h = write('pending_role_counterexample.json', dict(
        kind='actual_AST_methods_with_inert_io_and_assumed_current_context',
        full_end_to_end_test=False, actual_attempt_reason_not_inferred_from_this_witness=True,
        original_contract_validation='real pure build_contract/validate_contract/validate_recovery_parent',
        producer_to_remote='real idle_wait + pending status formatter + remote status callback + quarantine + ticket guard AST',
        production_imports='only producer local constant import is inert; no ROS module import',
        context_source_join_feedback_io='explicit inert stubs; all assumed passing',
        no_hardware_or_ROS=True, cases=rows))
    after = {name: sha((W / name).read_bytes()) for name in before}
    assert after == before
    for p, data in source_bytes.items(): assert Path(p).read_bytes() == data
    result = dict(schema_version=1, kind='gen40_actual_station_publication_readonly_diagnostic',
        recorded_ns=time.time_ns(), source_count=1029, all_deployed_sources_unchanged=True,
        deployed_release_sha256=sha(release_bytes), reviewed_source_sha256={p:sha(b) for p,b in source_bytes.items()},
        artifacts_sha256={'pending_role_counterexample.json':witness_h,'selected_actual_log_events.json':log_h},
        static_findings=[
            'A timer-only geometry expiration permits certified private computation through soft quarantine.',
            'Producer idle wait emits processing_deadline_pending for its existing registered source after geometry age alone.',
            'Remote status callback applies hard quarantine to that generic pending notice, deleting private work and incrementing geometry invalidation.',
            'Identical accepted service plan and wire digest are rechecked during selection and preview construction; diagnostic success is not motion authority.',
        ],
        actual_cause_pending='Await independently indexed producer status / receipt chronology; accepted RPC alone does not prove a fresh publishable plan.',
        no_motion_authorized=True, no_ROS_or_hardware=True, production_or_tests_edited=False,
        physical_grasp_verified=False, release_review_complete=False)
    rh = write('result.json', result)
    sh = write('seal.json',dict(schema_version=1, artifacts_sha256={'result.json':rh,**result['artifacts_sha256']},
        source_unchanged=True, production_or_tests_edited=False, no_ROS_or_hardware=True))
    print(json.dumps(dict(result_sha256=rh,seal_sha256=sh,cases=rows)))


if __name__ == '__main__':
    main()
