"""Execute actual old/new replay clock functions against every SDK boundary.

This proves a harness mechanism only. It is not production timing evidence.
"""
import ast
from collections import deque
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

W = Path(__file__).resolve().parents[1]
F = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z/verification/gen14_arrival_independent')
OUT = F/'sdk_ingress_independent'
OLD = W/'.runtime/replay_gen14_three_owner_qualification_v5_repeated_sdk_r1_20261013.py'
NEW = W/'.runtime/replay_gen14_three_owner_qualification_v5_20261013.py'
FIXTURE = F/'full_arrival_frames/fixture.json'
PRODUCER = W/'.runtime/gen14_arrival_repair_20261013/src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'
JOINTS = W/'.runtime/gen14_arrival_repair_20261013/src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_observation_source.py'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
record = lambda p: dict(path=str(p), sha256=sha(p))
read = lambda p: json.loads(Path(p).read_text())


def function(path, name):
    found = [node for node in ast.walk(ast.parse(path.read_text()))
             if isinstance(node, ast.FunctionDef) and node.name == name]
    assert len(found) == 1
    return found[0]


def maxlen():
    found = []
    for node in ast.walk(ast.parse(PRODUCER.read_text())):
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Attribute)
                and t.attr == '_rgb_interest_feedback' for t in node.targets):
            assert isinstance(node.value, ast.Call) and node.value.func.id == 'deque'
            found.append(next(ast.literal_eval(k.value) for k in node.value.keywords if k.arg == 'maxlen'))
    assert found == [128]
    return found[0]


class CountedDeque(deque):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.append_count = self.clear_count = 0

    def append(self, value):
        self.append_count += 1
        super().append(value)

    def clear(self):
        self.clear_count += 1
        super().clear()


def context(path, samples, names, capacity):
    now = [0]
    node = SimpleNamespace(_rgb_interest_feedback=CountedDeque(maxlen=capacity))
    namespace = dict(np=np, samples=samples, module=SimpleNamespace(JOINT_NAMES=names),
        node=node, raw_clock=lambda: now[0], dynamic=[], tf_index=[0], feed_images=[False],
        remote_box=[None], remote_input_busy=[False], remote_input_events=[],
        remote_input_index=[0], remote_input_journal=[], last_clock_return=[0], clock_return_count=[0],
        sdk_ingress_index=[0], sdk_ingress_journal=[])
    nodes = [function(path, 'clock')]
    if path == NEW:
        nodes.insert(0, function(path, 'sdk_value'))
    module = ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[]))
    exec(compile(module, str(path)+'::<actual clock body SDK isolation>', 'exec'), namespace)
    return now, node._rgb_interest_feedback, namespace


def equal(a, b):
    assert a.maxlen == b.maxlen and len(a) == len(b)
    for (stamp_a, q_a), (stamp_b, q_b) in zip(a, b):
        assert stamp_a == stamp_b and q_a.dtype == q_b.dtype == np.dtype(float)
        assert np.array_equal(q_a, q_b)


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()
    paths = [OLD, NEW, FIXTURE, PRODUCER, JOINTS, Path(__file__).resolve()]
    before = {str(p): sha(p) for p in paths}
    samples = read(FIXTURE)['sdk']
    assert len(samples) == 531
    assert [s['received_ns'] for s in samples] == sorted(s['received_ns'] for s in samples)
    joint_assignment = next(n for n in ast.parse(JOINTS.read_text()).body
        if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'JOINT_NAMES' for t in n.targets))
    names = eval(compile(ast.Expression(joint_assignment.value), str(JOINTS), 'eval'))
    assert names == tuple('Joint%d' % n for n in range(1, 7))
    capacity = maxlen()
    boundaries = {s['received_ns']+offset for s in samples for offset in (-1, 0)}
    recorded = set()
    previous_reports = []
    clock_count = 0
    for p in sorted(F.glob('three_owner_v5_phase*_r1/result.json')):
        data = read(p); previous_reports.append(record(p))
        assert data['source_unchanged']
        clock_count += data['clock_monotonicity']['return_count']
        recorded.add(data['clock_monotonicity']['last_return_ns'])
        for rows in ('rows', 'event_journal', 'function_profile', 'task_waits', 'remote_input_journal'):
            for row in data[rows]:
                for key in ('start_ns', 'completion_ns', 'processing_start_ns', 'replay_callback_ns', 'due_ns', 'now_ns'):
                    if type(row.get(key)) is int:
                        recorded.add(row[key])
    assert len(previous_reports) == 3 and len(boundaries) == 1062
    points = sorted(boundaries | recorded)
    old_now, old_queue, old = context(OLD, samples, names, capacity)
    new_now, new_queue, new = context(NEW, samples, names, capacity)
    states = hashlib.sha256()
    for at in points:
        old_now[0] = new_now[0] = at
        assert old['clock']() == new['clock']() == at
        equal(old_queue, new_queue)
        states.update(str(at).encode())
        for stamp_ns, values in new_queue:
            states.update(str(stamp_ns).encode()); states.update(values.tobytes())
        # Repeated clock reads without an arrival add no incremental entry.
        count = new_queue.append_count
        assert old['clock']() == new['clock']() == at
        equal(old_queue, new_queue)
        assert new_queue.append_count == count
    assert new_queue.clear_count == 0 and new_queue.append_count == len(samples)
    assert new['sdk_ingress_index'] == [len(samples)]
    assert len(new['sdk_ingress_journal']) == len(samples)
    assert [x['original_received_ns'] for x in new['sdk_ingress_journal']] == [s['received_ns'] for s in samples]

    # Reentrant clock while a real harness callback is being dispatched may
    # cross an SDK arrival; both actual clock bodies must retain the same queue.
    reentrant = []
    for path in (OLD, NEW):
        now, queue, ns = context(path, samples, names, capacity)
        at, nested_at = samples[130]['received_ns'], samples[131]['received_ns']
        def callback(message, *, now=now, ns=ns):
            now[0] = nested_at
            ns['clock']()
        ns['remote_box'][0] = SimpleNamespace(joint_cb=callback)
        ns['remote_input_events'] = [(at, 'joint', SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(to_nsec=lambda: samples[130]['stamp_ns']))))]
        now[0] = at
        assert ns['clock']() == nested_at
        assert len(ns['remote_input_journal']) == 1
        reentrant.append(queue)
    equal(*reentrant)
    assert {str(p): sha(p) for p in paths} == before
    OUT.mkdir()
    (OUT/'comparison_times_ns.json').write_text(json.dumps(points)+'\n')
    result = dict(passed=True, source_sha256=before, source_unchanged=True,
        actual_old_new_clock_functions_executed=True, SDK_samples=len(samples), maxlen=capacity,
        exact_receive_boundary_checks=len(boundaries), additional_persisted_clock_times=len(recorded-boundaries),
        total_distinct_comparison_times=len(points), comparisons_including_repeated_reads=2*len(points),
        checked_state_digest=states.hexdigest(), every_stamp_dtype_and_position_array_equal=True,
        no_new_queue_clear=True, incremental_appends=new_queue.append_count,
        old_reconstruction_appends=old_queue.append_count,
        reentrant_clock_crossing_next_SDK_arrival_equal=True,
        historical_total_clock_return_count=clock_count,
        historical_1488_exact_clock_series_available=False,
        previous_failed_reports=previous_reports, comparison_times=record(OUT/'comparison_times_ns.json'),
        scope='Only harness SDK input queue equivalence; no CPU or physical success claim.',
        proof='The deque is constant between successive original received_ns values. Testing received_ns-1 and received_ns covers every step for every nondecreasing clock, including old unstored clock samples. Actual old/new clock bodies execute with unrelated TF/image/remote ingress inert; a separate reentrant remote callback case crosses one SDK boundary.',
        limits=['Old phase0 records 1488 clock returns but not their individual timestamps; they are not reconstructed or claimed as an observed trace.',
                'This fixed-epoch replay contains no external feedback clear/reset between clock calls. Epoch-change production behavior is covered separately and is not altered by the harness.',
                'No production source, original SDK timestamp/position, receive threshold, deque length, freshness check or deadline is changed. Original r1 failures remain preserved.'],
        no_ros_master=True, motion_commands_sent=False, temperature_queried=False)
    (OUT/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('passed','total_distinct_comparison_times','comparisons_including_repeated_reads','incremental_appends','old_reconstruction_appends','source_unchanged')}))


if __name__ == '__main__':
    main()
