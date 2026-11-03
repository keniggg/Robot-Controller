#!/usr/bin/env python3
"""Offline historical replay only. No HTTP, ROS, timestamp rewrite or authority."""
import sys
sys.dont_write_bytecode = True
import copy
import datetime
import hashlib
import importlib.util
import json
import pathlib
import time

W = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('frozen_candidate_wrapper', W / '.runtime/candidate_simulation/run_candidate.py')
wrapper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wrapper)
REQUESTS = wrapper.ARCHIVE / 'candidate_simulation_full_start_20261006T024226Z_aczfr1vf/service/requests'
PLAN = '6dd3259ea6c0f144e2d0d12f'


def difference(a, b, path=''):
    if type(a) is not type(b):
        return [dict(path=path, first=a, second=b)]
    if isinstance(a, dict):
        result = []
        for key in sorted(a.keys() | b.keys()):
            if key not in a or key not in b:
                result.append(dict(path=path + '.' + key, first=a.get(key), second=b.get(key)))
            else:
                result.extend(difference(a[key], b[key], path + '.' + key))
        return result
    if isinstance(a, list):
        if len(a) != len(b):
            return [dict(path=path, first=a, second=b)]
        result = []
        for i, (x, y) in enumerate(zip(a, b)):
            result.extend(difference(x, y, path + '[%d]' % i))
        return result
    return [] if a == b else [dict(path=path, first=a, second=b)]


def main():
    output = wrapper.ARCHIVE / ('frozen_candidate_replay_' + datetime.datetime.utcnow().strftime('%Y%m%dT%H%M%SZ'))
    wrapper.storage_guard(output).mkdir()
    paths = []
    for path in sorted(REQUESTS.glob('*/request.raw.json')):
        if json.loads(path.read_text()).get('plan_id') == PLAN:
            paths.append(path)
    assert len(paths) == 2, paths
    payloads = [json.loads(path.read_text()) for path in paths]
    pins = wrapper.verify_pins()
    engine = wrapper.load_engine(pins)
    report = dict(scope='historical offline diagnostics only; no live validity or execution authority',
                  timestamp_policy='validate against original snapshot_stamp_sec; input timestamps unchanged',
                  script_sha256=wrapper.sha(__file__), pins_sha256=wrapper.sha(wrapper.HERE / 'pins.json'),
                  requests=[dict(path=str(p), sha256=wrapper.sha(p)) for p in paths],
                  request_differences=difference(*payloads), runs=[])
    wrapper.write_json(output / 'report.json', report)
    print(json.dumps(dict(output=str(output), request_differences=report['request_differences'])), flush=True)
    for group, order in enumerate(((0, 1, 0, 1), (1, 0))):
        backend, compilation = wrapper.build_backend(engine, pins, pins['model_xml'])
        wrapper.write_json(output / ('compilation_%d.json' % group), compilation)
        for position, which in enumerate(order):
            raw = copy.deepcopy(payloads[which])
            checked = engine._validate_v3_payload(raw, now_sec=raw['snapshot_stamp_sec'], max_snapshot_age_sec=120.)
            started = time.monotonic()
            result = backend._simulate_validated_grasp(checked, PLAN)
            normalized = engine._normalize_http_simulation_result(checked, result, backend)
            name = 'group%d_step%d_%s' % (group, position, 'screening' if which == 0 else 'execution')
            wrapper.write_json(output / (name + '_raw_result.json'), result)
            wrapper.write_json(output / (name + '_normalized.json'), normalized)
            original = json.loads(paths[which].with_name('response.json').read_text())
            row = dict(name=name, request_index=which, elapsed_sec=time.monotonic() - started,
                       source_unmodified=(raw == payloads[which]),
                       normalized_sha256=wrapper.sha(output / (name + '_normalized.json')),
                       original_response_differences=difference(original, normalized),
                       verdict={k: normalized.get(k) for k in ('score','simulation_ok','collision_free','contact_success','lift_success','failure_code','failure_reason')})
            report['runs'].append(row)
            wrapper.verify_pins()
            wrapper.write_json(output / 'report.json', report)
            print(json.dumps(row), flush=True)
    report['complete'] = True
    report['all_matching_original_responses'] = all(not r['original_response_differences'] for r in report['runs'])
    wrapper.write_json(output / 'report.json', report)
    print(json.dumps(dict(complete=True, output=str(output), all_matching_original_responses=report['all_matching_original_responses'])), flush=True)


if __name__ == '__main__':
    main()
