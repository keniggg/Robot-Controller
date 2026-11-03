#!/usr/bin/env python3
"""Replay sealed gen13 pending identity receipts offline in recorded order.

Uses real serialized detector status, ObjectPose and masks, and recorded bag
receipt time as the ROS source clock. No ROS master, publishers, subscribers,
inference server, motion client or robot device is created. Runtime callbacks
are exercised on the existing non-network test fixture; that fixture's in
memory publishers only collect messages. This is not an execution simulation.
"""
from __future__ import annotations

import argparse
import base64
from collections import Counter
from copy import deepcopy
import hashlib
import io
import itertools
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import zlib

import rosbag
import roslib.message


ROOT = Path(__file__).resolve().parents[1]
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD = EXT / 'grasp_records/20261006T081830.262906Z_86c9846b27134c9f9e22b0ae99a37e1f'
UNTIL_NS = 1791274803353483200
TOPICS = ('/perception/unknown/detector_status', '/perception/object', '/perception/object_mask')
MODULE_FILES = (
    'src/alicia_grasp_modes/src/alicia_grasp_modes/unknown_pending_identity.py',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_remote.py',
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py',
    'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_handoff.py',
    'src/alicia_grasp_modes/tests/test_remote_modes.py',
)
INITIAL_AUDIT = RECORD / ('keyframes_audits/'
    '96ad196c6093a28a36a730ab13a21d42d921c581c97cbe00d3ba324e14594890.json')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def extract(destination):
    manifest = json.loads((RECORD / 'manifest.json').read_text())
    assert manifest['state'] == 'complete' and manifest['recorder_closed'] is True
    assert Path('/media/zhuyupei/SanStand').is_mount()
    counts = Counter()
    digest = hashlib.sha256()
    with (destination / 'input_events.jsonl').open('x') as out:
        with rosbag.Bag(str(RECORD / 'keyframes.bag')) as bag:
            for topic, message, receipt in bag.read_messages(topics=list(TOPICS)):
                ns = receipt.to_nsec()
                if ns > UNTIL_NS:
                    break
                serialized = io.BytesIO()
                message.serialize(serialized)
                raw = serialized.getvalue()
                row = dict(topic=topic, receipt_ns=ns, message_type=message._type,
                    md5sum=message._md5sum, raw_sha256=hashlib.sha256(raw).hexdigest(),
                    compressed_ros_message=base64.b64encode(zlib.compress(raw)).decode('ascii'))
                line = json.dumps(row, separators=(',', ':'), sort_keys=True) + '\n'
                out.write(line)
                digest.update(line.encode())
                counts[topic] += 1
    bag_entry = next(f for f in manifest['files'] if f['path'] == 'keyframes.bag')
    report = dict(record=str(RECORD), manifest_sha256=sha(RECORD / 'manifest.json'),
        recorder_closed=True, bag_sha256_from_sealed_manifest=bag_entry['sha256'],
        bag_full_hash_recomputed=False, cutoff_receipt_ns=UNTIL_NS,
        input_events_sha256=digest.hexdigest(), counts=dict(counts),
        ordering='rosbag receipt timestamp, original iterator tie order',
        raw_messages='unaltered serialized ROS message payloads, zlib/base64 encoded')
    (destination / 'extraction.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def load_events(path):
    events = []
    last = 0
    for line in path.read_text().splitlines():
        row = json.loads(line)
        raw = zlib.decompress(base64.b64decode(row.pop('compressed_ros_message')))
        assert hashlib.sha256(raw).hexdigest() == row['raw_sha256']
        cls = roslib.message.get_message_class(row['message_type'])
        assert cls is not None and cls._md5sum == row['md5sum'], row['message_type']
        message = cls().deserialize(raw)
        assert row['receipt_ns'] >= last
        last = row['receipt_ns']
        row['message'] = message
        events.append(row)
    return events


def runtime_fixture(selection, clock):
    # Project test fixtures construct __new__ nodes without starting ROS IO.
    # Put this checkout before every other Python package path.
    for path in (
        ROOT / 'src/alicia_grasp_modes/tests',
        ROOT / 'src/alicia_grasp_modes/src',
        ROOT / 'src/alicia_flexible_grasp_supervisor/src',
    ):
        sys.path.insert(0, str(path))
    from _pytest.monkeypatch import MonkeyPatch
    from cv_bridge import CvBridge
    import test_remote_modes as fixture

    patches = MonkeyPatch()
    patches.setattr(fixture.module.rospy.Time, 'now',
        staticmethod(lambda: fixture.module.rospy.Time(clock[0] // 10**9, clock[0] % 10**9)))
    node, params = fixture.configured.__wrapped__(patches)
    node._ros_source_clock_ns = lambda: clock[0]
    node._stream_source_clock = lambda: clock[0] / 1e9
    node.inference_coordinator._clock = node._stream_source_clock
    params[fixture.module.SELECTION_PARAM] = dict(selection)
    params['/unknown_perception'] = dict(max_frame_age_sec=.8)
    assert node.reset_mode_cb(None).success
    node.bridge = CvBridge()
    # Exact real runner interval; no reset or renewal is allowed by callbacks.
    node.near_field_direct_timeout_sec = (UNTIL_NS - clock[0]) / 1e9
    assert node.start_streaming()
    original_identity = json.loads(INITIAL_AUDIT.read_text())['observe_return']
    node.target_instance_epoch = original_identity['target_epoch']
    assert node._current_stream_target_identity().track_id == original_identity['target_track_id']
    node.inference_coordinator.reset_target_epoch(node.target_instance_epoch)
    return node, params, patches, fixture


def run_callbacks(events, phase):
    first_status = next(json.loads(e['message'].data) for e in events if e['topic'] == TOPICS[0])
    selection = dict(mode='unknown', strategy='observe_return', generation=first_status['generation'],
        stamp_ns=first_status['selection_stamp_ns'])
    clock = [events[0]['receipt_ns']]
    node, params, patches, fixture = runtime_fixture(selection, clock)
    initial_identity = node._current_stream_target_identity()
    node._observe_return_phase = phase
    deadline = node._near_field_phase_deadline_sec
    node._observe_return_initial_contact_deadline_ns = round(deadline * 1e9)
    original_deadline = node._observe_return_initial_contact_deadline_ns
    old_reference = {'frozen_source_id': 'offline sentinel, no geometry or motion authorization'}
    node._observe_return_trigger_evidence = deepcopy(old_reference)
    transitions, failures, retirement = [], [], []
    pending_events, recoveries, invalidations, old_ticket_rejections = 0, 0, 0, 0
    new_source_admissions = 0
    callback_wall_ms = {topic: [] for topic in TOPICS}
    previous_pending = None
    previous_geometry_generation = node._capture_geometry_generation()
    pending_clock_max = 0.
    last_ns = 0
    try:
        for index, event in enumerate(events):
            clock[0] = event['receipt_ns']
            # Tick occurs at every recorded event; the periodic no-message
            # timeout is checked independently in the mutation scenarios.
            if hasattr(node, '_unknown_identity_poll'):
                node._unknown_identity_poll()
            topic, message = event['topic'], event['message']
            before_pending = deepcopy(getattr(node, '_unknown_identity_pending', None))
            before_identity = node._current_stream_target_identity()
            before_geometry = node._capture_geometry_generation()
            callback_start = time.perf_counter_ns()
            if topic == TOPICS[0]:
                node._silhouette_source_status_cb(message)
            elif topic == TOPICS[1]:
                node.object_cb(message)
            else:
                node.mask_cb(message)
            callback_elapsed_ms = (time.perf_counter_ns()-callback_start)/1e6
            callback_wall_ms[topic].append(callback_elapsed_ms)
            pending = getattr(node, '_unknown_identity_pending', None)
            identity = node._current_stream_target_identity()
            if identity != before_identity:
                retirement.append(dict(event_index=index, receipt_ns=clock[0],
                    source_stamp_ns=(json.loads(message.data).get('source_stamp_ns') if topic == TOPICS[0]
                        else message.header.stamp.to_nsec()), topic=topic,
                    reason=getattr(node, '_last_geometry_invalidation_code', ''),
                    diagnostic=getattr(getattr(node, 'latest_object_geometry', None), 'failure_reason', '')))
            if topic == TOPICS[1] and not message.detected:
                pending_events += 1
                if pending is None:
                    failures.append(dict(code='REAL_PENDING_NOT_QUARANTINED', event_index=index))
                if node._capture_geometry_generation() <= before_geometry:
                    failures.append(dict(code='FALSE_DID_NOT_INVALIDATE_GEOMETRY', event_index=index))
                if getattr(getattr(node, 'latest_object_geometry', None), 'valid', False):
                    failures.append(dict(code='PENDING_GEOMETRY_VALID', event_index=index))
            if pending is not None:
                age = (clock[0] - pending['source_ns']) / 1e9
                pending_clock_max = max(pending_clock_max, age)
                if age >= .8 or clock[0] >= pending['deadline_ns']:
                    failures.append(dict(code='PENDING_EXCEEDS_ORIGINAL_DEADLINE', event_index=index, age=age))
                ns = pending['source_ns']
                ticket = type('OldTicket', (), dict(generation=node._stream_generation,
                    target_epoch=identity.epoch, payload=(type('OldSnapshot', (), dict(stamp_ns=ns))(),),
                    snapshot_stamp_sec=ns/1e9))()
                if not node._stream_ticket_cancellation_code_locked(ticket):
                    failures.append(dict(code='PENDING_OLD_TICKET_VALID', event_index=index))
                else:
                    old_ticket_rejections += 1
                # The public submit gate must reject before touching incomplete
                # payload fields, including submissions without an expected gen.
                candidate = type('OldSnapshot', (), dict(stamp_ns=ns, stamp_sec=ns/1e9,
                    target_identity=identity, target_epoch=identity.epoch))()
                if node.submit_stream_snapshot(candidate):
                    failures.append(dict(code='PENDING_SUBMISSION_ACCEPTED', event_index=index))
            if before_pending is not None and pending is None and identity == before_identity:
                recoveries += 1
                obj = node.latest_object
                current_ns = obj.header.stamp.to_nsec()
                exact_status = node._unknown_identity_status.get(current_ns)
                if not obj.detected or exact_status is None:
                    failures.append(dict(code='RECOVERY_NOT_EXACT_SOURCE', event_index=index))
                else:
                    qual = exact_status.get('source_qualification', {})
                    if qual.get('state') != 'recovered_unique_measured_frame' or not qual.get('measurement_republished'):
                        failures.append(dict(code='RECOVERY_NOT_COMMITTED', event_index=index))
                age = (clock[0] - before_pending['source_ns'])/1e9
                transitions.append(dict(kind='recovered', event_index=index, receipt_ns=clock[0],
                    source_stamp_ns=current_ns, reference_source_stamp_ns=before_pending['source_ns'],
                    elapsed_since_reference_sec=age,
                    replay_callback_wall_ms=callback_elapsed_ms,
                    source_cutoff_ns=node._unknown_identity_cancelled_through_ns,
                    target_epoch=identity.epoch, target_track_id=identity.track_id,
                    geometry_generation=node._capture_geometry_generation()))
                for old_ns in (before_pending['source_ns'], node._unknown_identity_cancelled_through_ns):
                    old = type('OldSnapshot', (), dict(stamp_ns=old_ns, stamp_sec=old_ns/1e9,
                        target_identity=identity, target_epoch=identity.epoch))()
                    if node.submit_stream_snapshot(old):
                        failures.append(dict(code='OLD_SOURCE_SUBMITTED_AFTER_RECOVERY', event_index=index))
                # Probe only the mode/source admission wrapper. The base is
                # intercepted so this positive check cannot submit any job.
                prior_submit = fixture.base.RemoteGrasp6DNode.submit_stream_snapshot
                fixture.base.RemoteGrasp6DNode.submit_stream_snapshot = lambda self, snap, **kw: True
                try:
                    current = NS(stamp_ns=current_ns, stamp_sec=current_ns/1e9,
                        sample_stamp_ns=(current_ns,), target_identity=identity, target_epoch=identity.epoch)
                    if node.submit_stream_snapshot(current):
                        new_source_admissions += 1
                    else:
                        failures.append(dict(code='NEW_RECOVERY_SOURCE_NOT_ADMITTED', event_index=index))
                    mixed = NS(stamp_ns=current_ns, stamp_sec=current_ns/1e9,
                        sample_stamp_ns=(before_pending['source_ns'], current_ns),
                        target_identity=identity, target_epoch=identity.epoch)
                    if node.submit_stream_snapshot(mixed):
                        failures.append(dict(code='MIXED_OLD_NEW_SAMPLES_ADMITTED', event_index=index))
                finally:
                    fixture.base.RemoteGrasp6DNode.submit_stream_snapshot = prior_submit
            if before_pending is None and pending is not None:
                transitions.append(dict(kind='pending', event_index=index, receipt_ns=clock[0],
                    reference_source_stamp_ns=pending['source_ns'], deadline_ns=pending['deadline_ns'],
                    target_epoch=identity.epoch, target_track_id=identity.track_id,
                    geometry_generation=node._capture_geometry_generation()))
            if (node._near_field_phase_deadline_sec != deadline
                    or node._observe_return_initial_contact_deadline_ns != original_deadline):
                failures.append(dict(code='PLANNING_DEADLINE_CHANGED', event_index=index))
            if node._mode_selection != selection:
                failures.append(dict(code='SELECTION_CHANGED', event_index=index))
            last_ns = clock[0]
        return dict(phase=phase, event_count=len(events), false_object_count=pending_events,
            recovered_intervals=recoveries, pending_receipt_age_max_sec=pending_clock_max,
            new_source_wrapper_admissions=new_source_admissions,
            old_ticket_rejection_checks=old_ticket_rejections,
            callback_wall_ms={topic: summarize_wall(values) for topic, values in callback_wall_ms.items()},
            callback_wall_scope='local offline fixture; ROS parameter calls are in-memory stubs; fixed source clock does not advance with replay CPU time',
            initial_identity=dict(epoch=initial_identity.epoch, track_id=initial_identity.track_id),
            final_identity=dict(epoch=node._current_stream_target_identity().epoch,
                track_id=node._current_stream_target_identity().track_id),
            identity_retirements=retirement, failures=failures, transitions=transitions,
            original_deadline_preserved=node._near_field_phase_deadline_sec == deadline,
            original_failure_evidence_preserved=node._observe_return_trigger_evidence == old_reference,
            source_clock='integer bag receipt timestamp at each event',
            planning_deadline_scope='fixed runner interval end for conservation check, not rederived original runtime planning budget',
            planning_submission_scope='negative gates and intercepted new-source wrapper admission only; no inference or motion')
    finally:
        patches.undo()


def summarize_wall(values):
    ordered = sorted(values)
    if not ordered:
        return dict(count=0)
    return dict(count=len(ordered), minimum=ordered[0], maximum=ordered[-1],
        mean=sum(ordered)/len(ordered),
        p50=ordered[round(.50*(len(ordered)-1))],
        p95=ordered[round(.95*(len(ordered)-1))],
        p99=ordered[round(.99*(len(ordered)-1))])


def validate_mutated_receipts(events):
    """Counterfactual receipt mutations, explicitly separate from real replay."""
    from alicia_grasp_modes.unknown_pending_identity import reference_binding, validate_support_receipt
    statuses = [json.loads(e['message'].data) for e in events if e['topic'] == TOPICS[0]]
    pending = next(s for s in statuses if (s.get('source_qualification') or {}).get('state') == 'pending_current_support')
    recovered = next(s for s in statuses if (s.get('source_qualification') or {}).get('state') == 'recovered_unique_measured_frame')
    q = pending['source_qualification']
    selection = dict(mode='unknown', strategy='observe_return', generation=pending['generation'],
        stamp_ns=pending['selection_stamp_ns'])
    clock = [events[0]['receipt_ns']]
    node, _, patches, _ = runtime_fixture(selection, clock)
    try:
        for event in events:
            deliver_event(node, event, clock)
            if node._unknown_identity_pending is not None and not node._unknown_identity_pending['unpaired']:
                reference = deepcopy(node._unknown_identity_pending)
                break
        else:
            raise AssertionError('real pending reference unavailable')
    finally:
        patches.undo()
    receipt = next(e['receipt_ns'] for e in events if e['topic'] == TOPICS[0]
        and json.loads(e['message'].data)['source_stamp_ns'] == recovered['source_stamp_ns'])
    # Negative cases are meaningful only if their unmodified control passes
    # the exact same production contract and frozen reference.
    assert validate_support_receipt(recovered, reference, receipt, recovery=True) == q['source_id']
    cases = {
        'source_id_changed': lambda s: s['source_qualification'].update(source_id='f'*64),
        'support_audit_missing': lambda s: s['source_qualification'].update(support_audit=None),
        'support_ratio_below_original_gate': lambda s: s['source_qualification']['support_audit'].update(support_current_audit_inlier_ratio=.699),
        'support_residual_above_original_gate': lambda s: s['source_qualification']['support_audit'].update(support_current_audit_residual_m=.002001),
        'support_displacement_above_original_gate': lambda s: s['source_qualification']['support_audit'].update(support_reference_max_distance_m=.004001),
        'support_nonfinite': lambda s: s['source_qualification']['support_audit'].update(support_current_audit_residual_m=float('nan')),
        'rgb_iou_below_original_gate': lambda s: s['source_qualification']['rgb_identity']['candidates'][0].update(mask_iou=.849),
        'rgb_nonfinite': lambda s: s['source_qualification']['rgb_identity']['candidates'][0].update(colour_distance=float('nan')),
        'not_committed': lambda s: s['source_qualification'].update(state='qualified_unique_measured_recovery',measurement_republished=False),
        'wrong_exact_source': lambda s: s.update(depth_stamp_ns=s['source_stamp_ns']+1),
        'anchor_changed': lambda s: s['anchor_position_base'].__setitem__(0,s['anchor_position_base'][0]+.01),
        'authority_claimed': lambda s: s['source_qualification'].update(motion_authorized=True),
    }
    rows = []
    for name, mutate in cases.items():
        trial = deepcopy(recovered)
        mutate(trial)
        try:
            validate_support_receipt(trial, reference, receipt, recovery=True)
            rejected, error = False, ''
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            rejected, error = True, str(exc)
        rows.append(dict(case=name, rejected=rejected, diagnostic=error))
    for delay in (0, 1):
        now = reference['deadline_ns']+delay
        try:
            validate_support_receipt(recovered, reference, now, recovery=True)
            rejected, error = False, ''
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            rejected, error = True, str(exc)
        rows.append(dict(case='deadline_plus_%dns' % delay, rejected=rejected, diagnostic=error))
    return dict(scope='counterfactual mutation of one real recovery receipt; not recorded events',
        unmodified_positive_control_accepted=True, reference_source_ns=reference['source_ns'],
        source_id=reference['source_id'], support_policy=reference.get('support_policy'),
        cases=rows, passed=all(row['rejected'] for row in rows))


def message_source(event):
    return (json.loads(event['message'].data).get('source_stamp_ns') if event['topic'] == TOPICS[0]
        else event['message'].header.stamp.to_nsec())


def deliver_event(node, event, clock, *, poll=True):
    clock[0] = event['receipt_ns']
    if poll:
        node._unknown_identity_poll()
    if event['topic'] == TOPICS[0]:
        node._silhouette_source_status_cb(event['message'])
    elif event['topic'] == TOPICS[1]:
        node.object_cb(event['message'])
    else:
        node.mask_cb(event['message'])


def validate_callback_scenarios(events):
    """Independently vary delivery ordering/expiry using unchanged payloads."""
    statuses = [json.loads(e['message'].data) for e in events if e['topic'] == TOPICS[0]]
    pending_status = next(s for s in statuses if (s.get('source_qualification') or {}).get('state') == 'pending_current_support')
    recovery_status = next(s for s in statuses if (s.get('source_qualification') or {}).get('state') == 'recovered_unique_measured_frame')
    pending_ns, recovered_ns = pending_status['source_stamp_ns'], recovery_status['source_stamp_ns']
    selection = dict(mode='unknown', strategy='observe_return', generation=pending_status['generation'],
        stamp_ns=pending_status['selection_stamp_ns'])
    triples = {}
    for name, ns in (('pending', pending_ns), ('recovery', recovered_ns)):
        indices = [i for i, e in enumerate(events) if message_source(e) == ns]
        assert len(indices) == 3, (name, indices)
        triples[name] = indices
    rows = []
    for changed in ('pending', 'recovery'):
        indices = triples[changed]
        payloads = {events[i]['topic']: events[i] for i in indices}
        for order in itertools.permutations(TOPICS):
            trial = list(events[:max(triples['recovery'])+1])
            for index, topic in zip(indices, order):
                trial[index] = dict(payloads[topic], receipt_ns=events[index]['receipt_ns'])
            clock = [trial[0]['receipt_ns']]
            node, _, patches, _ = runtime_fixture(selection, clock)
            identity = node._current_stream_target_identity()
            violations = []
            try:
                for event in trial:
                    deliver_event(node, event, clock)
                    if node._unknown_identity_pending is not None:
                        if getattr(getattr(node, 'latest_object_geometry', None), 'valid', False):
                            violations.append('pending has geometry')
                passed = (node._current_stream_target_identity() == identity
                    and node._unknown_identity_pending is None and node.latest_object.detected
                    and node.latest_object.header.stamp.to_nsec() == recovered_ns and not violations)
                rows.append(dict(case=changed+'_delivery_order', order=list(order), passed=passed,
                    failures=violations))
            finally:
                patches.undo()
    for timeout_kind in ('no_new_messages', 'expiry_in_true_callback'):
        clock = [events[0]['receipt_ns']]
        node, _, patches, _ = runtime_fixture(selection, clock)
        original_identity = node._current_stream_target_identity()
        try:
            for event in events[:max(triples['pending'])+1]:
                deliver_event(node, event, clock)
            reference = deepcopy(node._unknown_identity_pending)
            assert reference is not None
            clock[0] = reference['deadline_ns']+1
            if timeout_kind == 'no_new_messages':
                node._unknown_identity_poll()
            else:
                obj = next(events[i]['message'] for i in triples['recovery'] if events[i]['topic'] == TOPICS[1])
                node.object_cb(obj)
            # Deliver the previously genuine recovery after expiration. Its
            # source bytes are unchanged; only arrival is deliberately late.
            for offset, index in enumerate(triples['recovery'], 1):
                event = dict(events[index], receipt_ns=reference['deadline_ns']+offset+1)
                deliver_event(node, event, clock)
            current = node._current_stream_target_identity()
            late = NS(stamp_ns=recovered_ns, stamp_sec=recovered_ns/1e9,
                target_identity=current, target_epoch=current.epoch, sample_stamp_ns=(recovered_ns,))
            admitted = node.submit_stream_snapshot(late)
            passed = (current != original_identity and node._unknown_identity_pending is None
                and node._unknown_identity_reference is None and not node.latest_object.detected
                and not admitted and not getattr(node.latest_object_geometry, 'valid', False))
            rows.append(dict(case=timeout_kind, passed=passed, same_identity=current == original_identity,
                late_true_detected=node.latest_object.detected, late_source_admitted=admitted,
                original_deadline_ns=reference['deadline_ns']))
        finally:
            patches.undo()
    for critical_section in ('receipt_validation', 'base_object_accept'):
        clock = [events[0]['receipt_ns']]
        node, _, patches, fixture = runtime_fixture(selection, clock)
        original_identity = node._current_stream_target_identity()
        injected = []
        try:
            for event in events[:max(triples['pending'])+1]:
                deliver_event(node, event, clock)
            deadline = node._unknown_identity_pending['deadline_ns']
            if critical_section == 'receipt_validation':
                import alicia_grasp_modes.unknown_pending_identity as contract
                original_validation = contract.validate_support_receipt

                def advance_inside_validation(*args, **kwargs):
                    result = original_validation(*args, **kwargs)
                    if kwargs.get('recovery'):
                        clock[0] = deadline+1
                        injected.append(True)
                    return result

                patches.setattr(contract, 'validate_support_receipt', advance_inside_validation)
            else:
                original_cb = fixture.base.RemoteGrasp6DNode.object_cb

                def advance_inside_base(self, msg):
                    result = original_cb(self, msg)
                    if msg.detected and msg.header.stamp.to_nsec() == recovered_ns:
                        clock[0] = deadline+1
                        injected.append(True)
                    return result

                patches.setattr(fixture.base.RemoteGrasp6DNode, 'object_cb', advance_inside_base)
            for event in events[max(triples['pending'])+1:max(triples['recovery'])+1]:
                deliver_event(node, event, clock)
            current = node._current_stream_target_identity()
            passed = (bool(injected) and current != original_identity
                and node._unknown_identity_pending is None and node._unknown_identity_reference is None
                and not node.latest_object.detected)
            rows.append(dict(case='expiry_during_'+critical_section, passed=passed,
                time_advance_injected=bool(injected), same_identity=current == original_identity,
                late_true_detected=node.latest_object.detected))
        finally:
            patches.undo()
    return dict(scope='counterfactual callback schedules, unchanged real source payloads',
        cases=rows, passed=all(row['passed'] for row in rows))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--extract-only', action='store_true')
    parser.add_argument('--reuse-extracted', action='store_true')
    parser.add_argument('--report-name', default='replay_final.json')
    args = parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert EXT in args.output.resolve().parents
    if args.reuse_extracted:
        extraction = json.loads((args.output / 'extraction.json').read_text())
        assert sha(args.output / 'input_events.jsonl') == extraction['input_events_sha256']
    else:
        args.output.mkdir(parents=True, exist_ok=False)
        extraction = extract(args.output)
    if args.extract_only:
        print(json.dumps(extraction, ensure_ascii=False))
        return
    frozen = {name: sha(ROOT/name) for name in MODULE_FILES}
    events = load_events(args.output / 'input_events.jsonl')
    phases = [run_callbacks(events, phase) for phase in ('initial_contact', 'observation_prefix')]
    mutations = validate_mutated_receipts(events)
    callback_scenarios = validate_callback_scenarios(events)
    assert frozen == {name: sha(ROOT/name) for name in MODULE_FILES}, 'source changed during replay'
    report = dict(scope='offline real-message callback replay; no images fitted, no inference, no hardware',
        limitations=['Bag receipt order is the recorder connection order, not proof of original remote callback scheduling.',
            'The test fixture uses in-memory publishers and no ROS master.',
            'No successful planning/grasp or geometry-fusion accuracy is claimed.'],
        extraction=extraction, source_sha256=frozen, script_sha256=sha(__file__), phases=phases,
        initial_identity_evidence=dict(path=str(INITIAL_AUDIT), sha256=sha(INITIAL_AUDIT)),
        mutated_receipts=mutations, callback_scenarios=callback_scenarios,
        supersedes='replay_mutation_baseline.json mutation cases lacked a positive control and are not acceptance evidence')
    report['passed'] = all(not p['failures'] and not p['identity_retirements']
        and p['false_object_count'] == 12 and p['recovered_intervals'] == 10
        and p['new_source_wrapper_admissions'] == 10
        and p['initial_identity'] == p['final_identity'] for p in phases) and mutations['passed'] and callback_scenarios['passed']
    assert Path(args.report_name).name == args.report_name
    output = args.output / args.report_name
    with output.open('x') as fp:
        fp.write(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    if args.report_name == 'replay_final.json':
        manifest = dict(schema=1, passed=report['passed'],
            scope=report['scope'], argv=sys.argv, python=sys.version,
            input_events=dict(path=str(args.output/'input_events.jsonl'),
                sha256=extraction['input_events_sha256'], count=len(events)),
            report=dict(path=str(output), sha256=sha(output)),
            production_and_fixture_sha256=frozen,
            replay_script=dict(path=str(Path(__file__).resolve()), sha256=sha(__file__)),
            suites=dict(real_receipt_phase_replays=len(phases),
                counterfactual_receipt_cases=len(mutations['cases']),
                counterfactual_callback_cases=len(callback_scenarios['cases'])),
            source_clock='recorded receipt in integer nanoseconds',
            robot_commands=0, ros_publishers_created=0, model_inference_requests=0,
            limitations=report['limitations'])
        with (args.output/'validation_manifest.json').open('x') as fp:
            fp.write(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(output=str(output), sha256=sha(output), passed=report['passed'],
        mutated_receipts=mutations,
        callback_scenarios=callback_scenarios,
        phases=[{k:v for k,v in p.items() if k not in ('transitions',)} for p in phases]), ensure_ascii=False))


if __name__ == '__main__':
    main()
