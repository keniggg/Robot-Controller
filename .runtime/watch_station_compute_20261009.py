#!/usr/bin/env python3
"""Bounded, read-only station compute protocol/timing observer.

No services, publishing, parameter changes, image access or temperature access.
Only the selected generation's detector status is retained. State/preview rows
are attributed to the current selection, not claimed to carry a generation.
"""
import argparse
import json
import threading
import time
from collections import Counter
from pathlib import Path


MOUNT = Path('/media/zhuyupei/SanStand')
STORAGE = MOUNT / '实验记录/Robot-Controller'


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--generation', type=int, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=600.)
    args = parser.parse_args()
    if args.generation < 0 or not 0. < args.duration <= 3600.:
        parser.error('generation must be nonnegative and duration in (0, 3600] seconds')
    target = args.output_dir.resolve()
    if not MOUNT.is_mount() or STORAGE.resolve() not in target.parents:
        parser.error('output must be within the mounted SanStand experiment storage')
    args.output_dir = target
    return args


def main():
    args = arguments()
    import rospy
    from std_msgs.msg import String
    from alicia_flexible_grasp_supervisor.msg import GraspState, Grasp6DPlan

    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / ('station_compute_%d_%d.jsonl' % (args.generation, time.time_ns()))
    lock = threading.RLock()
    counters = Counter()
    transitions = {}
    selected = [None]
    stopped = [None]
    current = dict(detector_state=None, detector_detail=None, current_age_sec=None,
                   task_state=None, preview_diagnostic=None)
    started = time.monotonic()
    stream = output.open('x', buffering=1)

    def write(kind, values, signature=None, summary=None):
        with lock:
            if stopped[0] is not None:
                return
            row = dict(kind=kind, generation=args.generation,
                       received_ros_ns=rospy.Time.now().to_nsec(),
                       received_monotonic_sec=time.monotonic(), **values)
            try:
                stream.write(json.dumps(row, sort_keys=True, allow_nan=False)+'\n')
                counters[kind] += 1
            except (OSError, ValueError, TypeError) as exc:
                stopped[0] = 'recording_failed: '+str(exc)
                print(json.dumps(dict(error=stopped[0], output=str(output))), flush=True)
                return
            if signature is not None and transitions.get(kind) != signature:
                transitions[kind] = signature
                counters['transitions'] += 1
                print(json.dumps(dict(transition=kind, generation=args.generation,
                    received_ros_ns=row['received_ros_ns'], **(summary or {})), sort_keys=True), flush=True)

    def selection(message):
        try:
            value = json.loads(message.data)
            generation = value.get('generation')
            if type(generation) is not int:
                return
            with lock:
                selected[0] = dict(generation=generation, stamp_ns=value.get('stamp_ns'))
            if generation == args.generation:
                write('selection', dict(selection={k:value.get(k) for k in
                    ('mode', 'strategy', 'generation', 'stamp_ns')}),
                    signature=(generation, value.get('stamp_ns')), summary=dict(selection_stamp_ns=value.get('stamp_ns')))
        except (ValueError, TypeError, AttributeError):
            with lock:
                counters['invalid_selection_json'] += 1

    def detector(message):
        try:
            status = json.loads(message.data)
            if status.get('generation') != args.generation:
                return
            timing = status.get('frame_timing') or {}
            age = timing.get('failure_source_age_sec', timing.get('status_source_age_sec'))
            contract = status.get('station_compute_wait') or {}
            binding = contract.get('binding') or {}
            row = {key:status.get(key) for key in (
                'selection_stamp_ns', 'source_stamp_ns', 'state', 'detail', 'target_locked',
                'target_lost', 'first_loss', 'station_compute_wait', 'station_compute_recovery',
                'station_compute_recovery_chain', 'discarded_source_stamp_ns', 'station_compute_mask')}
            row['frame_timing'] = timing
            with lock:
                current.update(detector_state=status.get('state'), detector_detail=status.get('detail'),
                               current_age_sec=age)
                counters['current_age_gt_0.6'] += int(isinstance(age, (float, int)) and age > .6)
                counters['pending_frames'] += int(status.get('state') == 'processing_deadline_pending')
                counters['recovery_frames'] += int(status.get('station_compute_recovery') is not None)
            # Contract source changes every successful frame. Keep these in
            # the file without flooding stdout with normal READY transitions.
            signature = (status.get('state'), status.get('detail'), status.get('target_lost'),
                         bool(contract), binding.get('lease_id'), binding.get('view_index'),
                         (status.get('first_loss') or {}).get('source_stamp_ns'))
            write('detector', row, signature, dict(state=status.get('state'), detail=status.get('detail'),
                source_ns=status.get('source_stamp_ns'), current_age_sec=age,
                contract_source_ns=contract.get('source_ns'), identity_deadline_ns=contract.get('identity_deadline_ns'),
                view_index=binding.get('view_index'), target_lost=status.get('target_lost')))
        except (ValueError, TypeError, AttributeError):
            with lock:
                counters['invalid_detector_json'] += 1

    def selected_scope():
        with lock:
            return selected[0] is not None and selected[0]['generation'] == args.generation

    def task(message):
        if not selected_scope():
            return
        row = dict(source_stamp_ns=message.header.stamp.to_nsec(), state=message.state,
                   stage=message.stage, active=message.active, success=message.success,
                   message=message.message, attribution='current_selection_no_generation_on_message')
        signature = (message.state, message.active, message.success, message.message)
        with lock:
            current['task_state'] = message.state
            changed = transitions.get('task') != signature
        if changed:
            write('task', row, signature, row)

    def preview(message):
        if not selected_scope():
            return
        row = dict(source_stamp_ns=message.header.stamp.to_nsec(), valid=message.valid,
            plan_id=message.plan_id, diagnostic=message.diagnostic, candidate_source=message.candidate_source,
            target_track_id=message.target_track_id, attribution='current_selection_no_generation_on_message')
        signature = (message.valid, message.plan_id, message.diagnostic, message.target_track_id)
        with lock:
            current['preview_diagnostic'] = message.diagnostic
            changed = transitions.get('preview') != signature
        if changed:
            write('preview', row, signature, row)

    def initial_preflight(kind, message):
        try:
            payload = json.loads(message.data)
            binding = payload.get('selection')
            if binding is not None:
                if binding.get('generation') != args.generation:
                    return
                attribution = 'message_selection'
            elif selected_scope():
                attribution = 'current_selection_no_generation_on_message'
            else:
                return
            # Keep the complete contract on disk. Only state changes and IDs
            # go to the terminal, so full READY references cannot flood it.
            summary = {key: payload.get(key) for key in (
                'token_id', 'attempt_id', 'request_id', 'action', 'state',
                'accepted', 'reason', 'physical_action_submitted')}
            write(kind, dict(payload=payload, attribution=attribution),
                  json.dumps(summary, sort_keys=True), summary)
        except (ValueError, TypeError, AttributeError):
            with lock:
                counters['invalid_initial_preflight_json'] += 1

    rospy.init_node('readonly_station_compute_watch', anonymous=True, disable_rosout=True)
    subscriptions = [
        rospy.Subscriber('/grasp_mode/selection', String, selection, queue_size=4),
        rospy.Subscriber('/perception/unknown/detector_status', String, detector, queue_size=32),
        rospy.Subscriber('/grasp/state', GraspState, task, queue_size=16),
        rospy.Subscriber('/grasp_6d/preview_plan_enriched', Grasp6DPlan, preview, queue_size=16)]
    for suffix in ('request', 'ack', 'attempt'):
        subscriptions.append(rospy.Subscriber(
            '/grasp_mode/initial_preflight_' + suffix, String,
            lambda msg, kind='initial_preflight_' + suffix: initial_preflight(kind, msg),
            queue_size=32))
    write('observer_start', dict(duration_sec=args.duration, disabled_rosout=True, output=str(output)))
    print(json.dumps(dict(observer_ready=True, generation=args.generation, output=str(output))), flush=True)
    last_summary = started
    try:
        while not rospy.is_shutdown() and time.monotonic()-started < args.duration:
            if stopped[0] is not None:
                break
            now = time.monotonic()
            if now-last_summary >= 15.:
                if not MOUNT.is_mount():
                    stopped[0] = 'SanStand mount unavailable; no fallback'
                    break
                with lock:
                    summary = dict(elapsed_sec=now-started, counts=dict(counters), current=dict(current))
                write('summary', summary)
                print(json.dumps(dict(summary=True, generation=args.generation, **summary)), flush=True)
                last_summary = now
            time.sleep(.1)
    finally:
        for subscription in subscriptions:
            subscription.unregister()
        with lock:
            write('observer_end', dict(reason=stopped[0] or 'duration_or_shutdown', counts=dict(counters)))
            stream.close()
        print(json.dumps(dict(observer_finished=True, reason=stopped[0], output=str(output), counts=dict(counters))), flush=True)


if __name__ == '__main__':
    main()
