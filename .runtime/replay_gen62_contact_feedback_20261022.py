"""Finite, guarded replay of recorded contact feedback; no ROS I/O."""
import ast
from collections import deque
from copy import deepcopy
import gc
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import statistics
import struct
import sys
import threading
import time
from types import SimpleNamespace as NS

sys.dont_write_bytecode = True
_u = os.uname()
platform._uname_cache = platform.uname_result(
    _u.sysname, _u.nodename, _u.release, _u.version, _u.machine, '')
platform.architecture = lambda *a, **kw: (str(struct.calcsize('P')*8)+'bit', 'ELF')
DENIED = []
def guard(event, args):
    if event.startswith('socket.') or event in (
            'subprocess.Popen', 'os.system', 'os.posix_spawn', 'os.fork', 'os.exec'):
        DENIED.append(event)
        raise RuntimeError('offline replay forbids '+event)
sys.addaudithook(guard)

W = Path(__file__).resolve().parents[1]
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
E = N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1'
D = E/'contact_handoff_diagnosis_r1'
OUT = E/'contact_feedback_fix_r1/real_wire_replay_r1'
SOURCE = W/'src/alicia_grasp_modes/src/alicia_grasp_modes/contact_station_task.py'
sys.path[:0] = [str(W/'src/alicia_grasp_modes/src'),
               str(W/'src/alicia_flexible_grasp_supervisor/src'),
               str(W/'devel/lib/python3/dist-packages')]
import genpy
import rospy
from roslib.message import get_message_class
from sensor_msgs.msg import JointState
from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
from alicia_grasp_modes.contact_station_task import ContactStationTaskMixin
import alicia_grasp_modes.contact_station_task as production


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(name, value):
    raw = (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    path = OUT/name
    with path.open('xb') as target:
        target.write(raw)
        target.flush()
        os.fsync(target.fileno())
    assert path.read_bytes() == raw


def restore(message, values):
    for name, kind in zip(message.__slots__, message._slot_types):
        value = values[name]
        if kind in ('time', 'duration'):
            cls = genpy.Time if kind == 'time' else genpy.Duration
            value = cls(value['ns']//10**9, value['ns'] % 10**9)
        elif kind.endswith('[]') and '/' in kind:
            cls = get_message_class(kind[:-2])
            value = [restore(cls(), item) for item in value]
        elif '/' in kind:
            value = restore(get_message_class(kind)(), value)
        setattr(message, name, value)
    return message


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert E.stat().st_dev == Path('/media/zhuyupei/SanStand').stat().st_dev
    release = N/'verification/gen61_release_manifest.json'
    assert sha(release) == 'f233ba3a41dca9632a13e084b24c5a0c434c70ba0c28f38aeba369ecfb3c17e9'
    assert sha(SOURCE) == json.loads(release.read_bytes())['source_sha256'][str(SOURCE.relative_to(W))]
    inputs = {str(p): sha(p) for p in (release, SOURCE, D/'wire_result.json', D/'selected_wire.jsonl')}
    OUT.mkdir(exist_ok=False)
    wire = [json.loads(line) for line in (D/'selected_wire.jsonl').open()]
    request = next(r['message']['parsed'] for r in wire
        if r['topic'] == '/grasp_mode/contact_station_retirement_request')
    measurement = next(r['message']['parsed'] for r in wire
        if r['topic'] == '/grasp_mode/observe_return_measurement_request'
        and r['message']['parsed']['state'] == 'qualified')
    plan_data = next(r['message'] for r in wire
        if r['topic'] in ('/grasp_6d/preview_plan_enriched', '/grasp_6d/plan_enriched')
        and r['message']['plan_id'] == request['plan_id'])
    plan = restore(Grasp6DPlan(), plan_data)
    assert production.committed_plan_digest(plan) == request['plan_sha256']
    assert production.measurement_digest(measurement) == request['measurement_sha256']
    rows = [(r['bag_receipt_ns'], restore(JointState(), r['message'])) for r in wire
            if r['topic'] == '/alicia_d/accepted_joint_states']
    failure = json.loads((D/'wire_result.json').read_bytes())['failure_ns']
    clock = [request['stamp_ns']]
    production.rospy.Time.now = lambda: genpy.Time(clock[0]//10**9, clock[0] % 10**9)
    node = ContactStationTaskMixin()
    node._following_evidence_lock = threading.RLock()
    node._following_epoch_ns = request['control_epoch_ns']
    node._observation_measurement_request = deepcopy(measurement)
    node._observe_return_live = lambda: None
    validation_calls = []
    node._validate_bound_plan_locked = lambda *a: validation_calls.append(clock[0]) or NS(ok=True)
    events = []
    gc_start = {}
    def gc_event(phase, info):
        generation = info['generation']
        if phase == 'start':
            gc_start[generation] = time.perf_counter_ns()
        else:
            events.append(dict(generation=generation, elapsed_ns=time.perf_counter_ns()-gc_start[generation],
                               collected=info['collected'], uncollectable=info['uncollectable']))
    gc.callbacks.append(gc_event)
    def history(now):
        return deque((deepcopy(row) for receipt, row in rows if receipt <= now), maxlen=128)
    node._following_accepted_history = history(clock[0])
    start, reference = node._contact_station_feedback(request['control_epoch_ns'])
    timing = []
    for _ in range(20):
        before, cpu = time.perf_counter_ns(), time.thread_time_ns()
        result = node._contact_station_feedback(request['control_epoch_ns'], start, reference)
        timing.append(dict(wall_ns=time.perf_counter_ns()-before, cpu_ns=time.thread_time_ns()-cpu,
                           latest_source_ns=result[0], age_ns=clock[0]-result[0]))
    live_calls = []
    # Recorder arrivals are an explicit delivery model, never proof of the
    # actual task's callback schedule. No participant ACK is fabricated here.
    for now in (request['stamp_ns']+20_000_000, request['deadline_ns']-1, failure):
        clock[0] = now
        node._following_accepted_history = history(now)
        before = time.perf_counter_ns()
        try:
            node._contact_station_live(plan, request, {}, start, reference)
            outcome = 'accepted_local_checks_with_stubbed_bound_validator'
        except ValueError as exc:
            outcome = str(exc)
        live_calls.append(dict(now_ns=now, outcome=outcome,
            latest_local_source_ns=node._following_accepted_history[-1].header.stamp.to_nsec(),
            elapsed_ns=time.perf_counter_ns()-before))
    # Precisely model a pause after copying the local history. The observed
    # 2.239 s duration is applied as a controlled schedule, not claimed observed
    # inside this function. This must reproduce the error without changing rows.
    clock[0] = request['stamp_ns']+20_000_000
    node._following_accepted_history = history(clock[0])
    original_copy = production.deepcopy
    delays = []
    def delayed_copy(value):
        copied = original_copy(value)
        if isinstance(value, list) and value and isinstance(value[0], JointState):
            before = clock[0]
            clock[0] = failure
            delays.append(dict(from_ns=before, to_ns=clock[0], point='after_history_copy'))
        return copied
    production.deepcopy = delayed_copy
    try:
        node._contact_station_live(plan, request, {}, start, reference)
        controlled = 'unexpected_pass'
    except ValueError as exc:
        controlled = str(exc)
    finally:
        production.deepcopy = original_copy
    assert controlled == 'CONTACT_STATION_FRESH_FEEDBACK_REQUIRED'
    gc.collect()
    gc.callbacks.remove(gc_event)
    summary = dict(kind='GEN62_guarded_real_wire_feedback_replay', passed=True,
        actual_local_cache_or_scheduler_record_available=False,
        actual_production_failure_mechanism_reproduced=False,
        original_error_reproduced_under_controlled_post_snapshot_pause=True,
        inputs_sha256=inputs, helper_sha256=sha(__file__), request=request,
        measurement_sha256=production.measurement_digest(measurement),
        plan_sha256=production.committed_plan_digest(plan),
        samples=len(rows), local_history_size=128,
        normal_feedback_benchmark=dict(iterations=len(timing),
            median_wall_ns=statistics.median(row['wall_ns'] for row in timing),
            maximum_wall_ns=max(row['wall_ns'] for row in timing), samples=timing),
        replay_with_bag_receipt_as_assumed_local_delivery=live_calls,
        controlled_pause=dict(outcome=controlled, delays=delays),
        gc_events=events, forbidden_boundary_attempts=DENIED,
        production_or_old_candidate_modified=False, ROS_initialized=False,
        physical_grasp_success_verified=False,
        limitation='Frozen wire proves driver/recorder continuity; it cannot reconstruct task-local locks, GC, cache or scheduling. Bound validator is stubbed only for timing isolation; this is no execution validation.')
    assert not DENIED
    assert all(sha(path) == digest for path, digest in inputs.items())
    save('result.json', summary)
    print(json.dumps(dict(result=str(OUT/'result.json'), sha256=sha(OUT/'result.json'),
        normal_maximum_ns=summary['normal_feedback_benchmark']['maximum_wall_ns'],
        controlled=controlled, actual_mechanism_reproduced=False), ensure_ascii=False))


if __name__ == '__main__':
    main()
