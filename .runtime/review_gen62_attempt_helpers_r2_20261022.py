"""Offline exact-diff and pure-contract review; never calls a live entrypoint."""
import ast
from copy import deepcopy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import struct
import sys
import time
import traceback

sys.dont_write_bytecode = True
u = os.uname()
platform._uname_cache = platform.uname_result(u.sysname, u.nodename, u.release, u.version, u.machine, '')
platform.architecture = lambda *a, **kw: (str(struct.calcsize('P')*8)+'bit', 'ELF')
denied = []
def guard(event, args):
    if event.startswith('socket.') or event in ('subprocess.Popen', 'os.system', 'os.posix_spawn', 'os.fork', 'os.exec'):
        denied.append(event)
        raise RuntimeError('offline attempt review forbids '+event)
sys.addaudithook(guard)

W = Path(__file__).resolve().parents[1]
N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z')
E61 = N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1'
F = E61/'contact_feedback_fix_r1'
OUT = F/'attempt_helpers_review_r2'
EXPECTED = {
    'gen62_compute_attempt_common_20261022.py': '7dc87bf19147ebfa24d242203d79ff95542e2f809a0eae9f788c2ddd698146d7',
    'gen62_compute_attempt_capture_20261022.py': '258ef54fa6e66bfd7a8f98a7cf5ade7f4e8fa5d5d036d4d24471e9b198516328',
    'gen62_compute_attempt_receipt_20261022.py': '73586e5a599251c45492caf1fde144bdc66cfd93eede6ab4e92969070d743e5b',
    'gen62_compute_attempt_entry_20261022.py': 'bf12e9a66d663618c3a98b54c63a642549d090a05156499107b64615b795130c',
    'gen62_compute_attempt_gate_20261022.py': '2cd5a70f5c700b09b72784e1f5049c0c20c9c3fcdddea0263da539e43dd9362c',
    'write_gen62_accepted_release_20261022.py': '3d4f0e94b4ca81875c69374fc860e114665bfef8d1266a4f41d9a6cb4a5c4e6a',
}

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def load(name):
    path = W/'.runtime'/name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[path.stem] = mod
    spec.loader.exec_module(mod)
    return mod

def trace_target(calls):
    calls.append('before')
    calls.append('boundary')
    calls.append('after')
    return 'done'

def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    assert F.stat().st_dev == Path('/media/zhuyupei/SanStand').stat().st_dev
    OUT.mkdir(exist_ok=False)
    result = dict(kind='GEN62_attempt_helpers_offline_independent_review', passed=False,
        helper_sha256={str(W/'.runtime'/name): digest for name, digest in EXPECTED.items()},
        no_ROS_access=True, no_live_process_probe=True, no_control_or_motion=True,
        no_source_or_reviewed_helper_modified=True, no_temperature_query=True,
        actual_GEN62_deployment_or_scene_accepted_by_this_review=False,
        physical_grasp_success_verified=False, started_ns=time.time_ns())
    try:
        assert all(sha(W/'.runtime'/name) == digest for name, digest in EXPECTED.items())
        allowed = {}
        for name in EXPECTED:
            old_name = name.replace('gen62', 'gen61')
            old = (W/'.runtime'/old_name).read_text()
            expected = old.replace('GEN61', 'GEN62').replace('gen61', 'gen62')
            edits = [('GEN61', 'GEN62'), ('gen61', 'gen62')]
            if 'attempt_common' in name:
                extra = [
                    ("S = E61 / 'actual_attempt_r1'", "E62 = E61 / 'contact_feedback_fix_r1'\nS = E62 / 'actual_attempt_r1'"),
                    ('8beaf2eeabac1d86f217959524083c09b927969f0be4f22646d6c306e6865754', '8fe6865ef453dad1808faf32f01da1b5bac0493755a4e934fdb63bc76114d995'),
                    ("REQUEST = N / 'gen57_resume_r3/user_resume_request.json'", "REQUEST = E61 / 'user_resume_r1/user_resume_request.json'"),
                    ('89128addf34ef53109378f3d08f50d927131c206f7e43a62ad341d6aa6396023', '87ed176de63a7612d41d9b1a314421a2033b867ad76f8a05cc84a7c1421b3f15'),
                    ("CONTINUE_REQUEST = N / 'gen58_resume_r1/user_continue_request.json'", 'CONTINUE_REQUEST = REQUEST'),
                    ("CONTINUE_REQUEST_SHA = '55dd4a17729108de8964bb72692d7ee0a326c14715cf5b29e354cc659d533281'", 'CONTINUE_REQUEST_SHA = REQUEST_SHA'),
                    ("summary['tests'] > 10044", "summary['tests'] == 10213"),
                    ("deployment_path == E61/'runtime_reload_r1/acceptance/result.json'", "deployment_path == E62/'runtime_reload_r1/acceptance/result.json'"),
                    ("runtime_context['previous']['full_runtime_nodes']", "runtime_context['nodes']"),
                    ("control['revision'] == 67", "control['revision'] == 69"),
                    ("runtime_context['previous']['control_snapshot'] == control", "runtime_base.recheck(runtime_context) == control"),
                ]
            elif 'attempt_receipt' in name:
                extra = [('active67', 'active69'), ("control['revision'] == 67", "control['revision'] == 69"),
                         ('control_revision=67', 'control_revision=69')]
            elif 'attempt_entry' in name:
                extra = [('ce19062251dd265247cf109eadad8d868beeee98fb7d9bf79e703ed8f8258466', EXPECTED['gen62_compute_attempt_capture_20261022.py']),
                         ('6e44829192503b911748ee39615dbe17b30ca9cc5a0b732d53970f8bd6fa84f2', EXPECTED['gen62_compute_attempt_receipt_20261022.py']),
                         ("['control_revision'] == 67", "['control_revision'] == 69")]
            elif name.startswith('write_'):
                extra = [("E=N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1'", "E=N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1'")]
            else:
                extra = []
            for before, after in extra:
                assert before in expected, (name, before)
                expected = expected.replace(before, after)
            actual = (W/'.runtime'/name).read_text()
            assert expected == actual, 'difference outside explicit allowed set: '+name
            ast.parse(actual)
            allowed[name] = dict(original_path=str(W/'.runtime'/old_name), original_sha256=sha(W/'.runtime'/old_name),
                                 exact_bytes_after_allowed_replacements=True, replacements=edits+extra)
        result['exact_diff_review'] = allowed
        sys.path[:0] = [str(W/'.runtime'), str(W/'src/alicia_grasp_modes/src')]
        common = load('gen62_compute_attempt_common_20261022.py')
        capture = load('gen62_compute_attempt_capture_20261022.py')
        receipt = load('gen62_compute_attempt_receipt_20261022.py')
        entry = load('gen62_compute_attempt_entry_20261022.py')
        gate = load('gen62_compute_attempt_gate_20261022.py')
        writer = load('write_gen62_accepted_release_20261022.py')
        assert sha(entry.COLLECTOR) == entry.COLLECTOR_SHA == EXPECTED[entry.COLLECTOR.name]
        assert sha(entry.WRITER) == entry.WRITER_SHA == EXPECTED[entry.WRITER.name]
        pins = {name: dict(path=str(getattr(entry, name)), sha256=getattr(entry, name+'_SHA'))
                for name in ('ORIGINAL', 'CONTINUATION', 'PREWARM', 'RECORD_WRAPPER', 'RECORDER', 'ARCHIVE')}
        assert all(sha(row['path']) == row['sha256'] for row in pins.values())
        assert entry.original_boundary() == 121
        assert common.S == F/'actual_attempt_r1'
        assert common.CONTROL_SHA == sha(common.CONTROL)
        assert common.REQUEST == common.CONTINUE_REQUEST and common.REQUEST_SHA == common.CONTINUE_REQUEST_SHA == sha(common.REQUEST)
        ctl, req = common.read(common.CONTROL), common.read(common.REQUEST)
        assert ctl['state'] == 'active' and ctl['revision'] == 69
        assert req['source'] == 'actual_user_message' and req['text'] == '继续处理'
        assert req['new_hands_clear_confirmation_claimed'] is False
        result['fixed_original_helpers'] = pins
        result['original_mode_boundary_line'] = 121
        result['current_control_request'] = dict(control=common.ref(common.CONTROL), request=common.ref(common.REQUEST))
        # Reuse the real prior 17/3/14 topology as a pure schema fixture. Only
        # fixture generation/accepted reference fields change; no new runtime
        # acceptance, release, scene or controller permission is manufactured.
        r = common.read(N/'verification/gen61_release_manifest.json')
        d = common.read(E61/'runtime_reload_r1/acceptance/result.json')
        b, t = common.read(d['bundle']), common.read(d['reload'])
        r.update(generation='GEN62', test_summary=dict(tests=10213, failures=0, errors=0, skipped=0))
        d.update(generation='GEN62', kind='GEN62_three_compute_acceptance', epoch_ns=1792661888995982113)
        t['kind'] = 'GEN62_three_compute_reload'
        b['epoch_ns'] = d['epoch_ns']
        selected = dict(mode='unknown', strategy='observe_return', generation=5, stamp_ns=1792661288399203300)
        d['selection'], b['selection'] = deepcopy(selected), deepcopy(selected)
        assert common.validate_acceptance_values(r, d, b, t) == d['full_runtime_nodes']
        cases = [
            ('old_generation', lambda r,d,b,t:r.update(generation='GEN61')),
            ('not_reviewed', lambda r,d,b,t:r.update(reviewed=False)),
            ('partial_tests', lambda r,d,b,t:r['test_summary'].update(tests=10149)),
            ('test_skipped', lambda r,d,b,t:r['test_summary'].update(skipped=1)),
            ('source_count', lambda r,d,b,t:r.update(source_count=0)),
            ('source_map', lambda r,d,b,t:d['source_sha256'].update(unexpected='a'*64)),
            ('bundle_not_ready', lambda r,d,b,t:b.update(deployment_ready=False)),
            ('wrong_deployment_kind', lambda r,d,b,t:d.update(kind='GEN61_three_compute_acceptance')),
            ('full_restart', lambda r,d,b,t:d.update(full_restart=True)),
            ('only_two_replacements', lambda r,d,b,t:d.update(replaced_count=2)),
            ('lost_preserved', lambda r,d,b,t:d.update(preserved_count=13)),
            ('missing_runtime_node', lambda r,d,b,t:d['full_runtime_nodes'].pop('/grasp_task_node')),
            ('target_identity', lambda r,d,b,t:d['targets']['/grasp_task_node'].update(pid=-1)),
            ('preserved_identity', lambda r,d,b,t:d['preserved']['/alicia_d_driver_node'].update(pid=-1)),
            ('acceptance_graph_changed', lambda r,d,b,t:d['after']['node_identities']['/grasp_task_node'].update(pid=-1)),
            ('transition_failed', lambda r,d,b,t:t.update(passed=False)),
            ('missing_unit', lambda r,d,b,t:t['units'].pop('/remote_grasp6d_node')),
            ('service_changed', lambda r,d,b,t:d['after']['services'].update(unexpected='owner')),
            ('epoch_changed', lambda r,d,b,t:d.update(epoch_ns=d['epoch_ns']+1)),
            ('selection_changed', lambda r,d,b,t:d['selection'].update(generation=4)),
            ('manual_false', lambda r,d,b,t:d.update(manual_mode=False)),
            ('task_active', lambda r,d,b,t:d.update(task_active=True)),
            ('action_active', lambda r,d,b,t:d.update(actions_active=True)),
            ('unconfirmed', lambda r,d,b,t:d.update(actuation_status='UNCONFIRMED')),
            ('gui_ack_missing', lambda r,d,b,t:d.update(gui_owning_manual_ack=False)),
            ('acceptance_grants_attempt', lambda r,d,b,t:d.update(automatic_attempt_authorized=True)),
            ('no_fresh_scene_gate', lambda r,d,b,t:d.update(fresh_scene_and_dependency_gate_required_before_automatic_attempt=False)),
            ('memory_attestation_claimed', lambda r,d,b,t:d['source_process_proof']['/grasp_task_node'].update(memory_attestation=True)),
        ]
        negatives = []
        for label, mutate in cases:
            values = deepcopy((r, d, b, t))
            mutate(*values)
            try:
                common.validate_acceptance_values(*values)
            except RuntimeError as exc:
                negatives.append(dict(case=label, rejected=True, reason=str(exc)))
            else:
                raise AssertionError('pure contract accepted negative case '+label)
        result['pure_acceptance_fixture'] = dict(positive=1, negatives=negatives,
            synthetic_generation_reference_fields=True, not_actual_GEN62_acceptance=True)
        source = Path(__file__).resolve()
        tree = ast.parse(source.read_text())
        target = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'trace_target')
        target_line = target.body[1].lineno
        fixture_namespace = {}
        exec(compile(ast.Module(body=[target], type_ignores=[]), str(source), 'exec'), fixture_namespace)
        traced_target = fixture_namespace['trace_target']
        calls = []
        code = entry.run_with_line_gate(source, sha(source), target_line,
            lambda frame:calls.append('gate'), lambda:traced_target(calls))
        assert code == 'done' and calls == ['before', 'gate', 'boundary', 'after']
        calls = []
        def reject(frame):
            calls.append('gate_rejected')
            raise RuntimeError('review gate rejected')
        try:
            entry.run_with_line_gate(source, sha(source), target_line, reject, lambda:traced_target(calls))
        except RuntimeError as exc:
            assert str(exc) == 'review gate rejected'
        else:
            raise AssertionError('rejected gate allowed bound line')
        assert calls == ['before', 'gate_rejected'] and sys.gettrace() is None
        result['pure_line_gate_checks'] = dict(positive=1, rejected_boundary_prevents_next_line=True,
                                             original_runner_not_executed=True, tracer_restored=True)
        assert all(sha(W/'.runtime'/name) == digest for name, digest in EXPECTED.items())
        assert not denied
        result.update(passed=True, findings=[],
            unchanged_contracts=['scene age under 90 seconds', 'reviewed observation validity and control lock',
                '20 distinct ordered SDK samples with source/caller ownership and <=.004 stationary bounds',
                'manual/idle/actions/CONFIRMED gates and disabled temperature policy',
                'remote live health bound to current instance/config and final <=5 second freshness',
                '17 runtime instances / 3 replaced units / 14 preserved identities',
                'exact one recorder exception with parent PID/start ticks/argv/source ownership',
                'unchanged original runner and exact automatic-mode line 121',
                'post-fsync checks before returning to original mode line'],
            deferred_actual_checks=['Actual GEN62 release/acceptance/bundle/transition load_context closure after deployment.',
                'Fresh current scene, original runner, remote/SDK observations and recording performed only by root.'],
            known_operational_requirement='Launch the unchanged entry with the complete bound ROS environment; this review does not supply or test live prewarm.',
            forbidden_boundary_attempts=denied)
    except BaseException as exc:
        result.update(error=repr(exc), traceback=traceback.format_exc(), forbidden_boundary_attempts=denied)
    result.update(finished_ns=time.time_ns(), review_helper_sha256=sha(__file__))
    raw = (json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    with (OUT/'result.json').open('xb') as f:
        f.write(raw); f.flush(); os.fsync(f.fileno())
    assert (OUT/'result.json').read_bytes() == raw
    print(json.dumps(dict(passed=result['passed'], error=result.get('error'), result=str(OUT/'result.json'),
                         sha256=sha(OUT/'result.json')), ensure_ascii=False))
    return 0 if result['passed'] else 2

if __name__ == '__main__':
    sys.exit(main())
