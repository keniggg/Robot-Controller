"""Bounded offline GEN60 parameter-projection contracts; no live observations."""
import argparse
from copy import deepcopy
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
E = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1')
BEFORE = E / 'runtime_parameter_probe_r1/before.json'
AFTER = E / 'runtime_parameter_probe_r1/after.json'
PINS = {str(BEFORE): 'cdbf19d361ee411a14c6659b419d0ef0007a38c914d7d0cea79accb2f773a766',
        str(AFTER): '69b0eb31adc616bb97d51ad5bb9f157a6a98c24834deb74ef8df57099ead9405'}
c = None

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def load(path):
    return json.loads(Path(path).read_bytes())

def payload(tree):
    return json.loads(tree['camera_read_diagnostics']['payload_json'])

def change_payload(tree, operation):
    value = payload(tree)
    operation(value)
    tree['camera_read_diagnostics']['payload_json'] = json.dumps(value, allow_nan=False)

def history():
    return dict(diagnostic_only=True, authorizes_pair_or_motion=False,
                state='resumed_for_existing_gates', reason='next_complete_pair', history_epoch=1)

def add_history(tree, branch=None):
    def mutate(value):
        data = history()
        if branch is not None:
            branch(data)
        value['entries'][0]['evidence']['stationary_pair_history'] = data
    change_payload(tree, mutate)

class Contracts(unittest.TestCase):
    def setUp(self):
        self.raw = load(BEFORE)

    def project(self, raw):
        def get_param(key):
            self.assertEqual(key, '/')
            return deepcopy(raw)
        return c.parameters(SimpleNamespace(getParam=get_param))

    def assert_drift_rejected(self, mutate):
        before = self.project(self.raw)
        changed = deepcopy(self.raw)
        mutate(changed)
        with self.assertRaises((RuntimeError, ValueError)):
            c.same_parameters(before, self.project(changed))

    def test_real_two_snapshot_drift_is_only_diagnostic(self):
        original = deepcopy(self.raw)
        before, after = self.project(self.raw), self.project(load(AFTER))
        self.assertNotEqual(before['all_parameters_sha256'], after['all_parameters_sha256'])
        self.assertEqual(before['configuration_sha256'], after['configuration_sha256'])
        self.assertEqual(before['configuration_values'], after['configuration_values'])
        c.same_parameters(before, after)
        self.assertEqual(self.raw, original)
        self.assertEqual(before['parameter_values'], original)
        self.assertEqual(set(before['parameter_values']), set(before['configuration_values']))

    def test_known_rolling_entries_and_copy_errors_are_telemetry(self):
        changed = deepcopy(self.raw)
        change_payload(changed, lambda p: p.update(entries=[], copy_errors=9))
        c.same_parameters(self.project(self.raw), self.project(changed))

    def test_known_compact_history_branch_is_valid(self):
        changed = deepcopy(self.raw)
        add_history(changed)
        c.same_parameters(self.project(self.raw), self.project(changed))

    def test_unknown_configuration_is_preserved_when_stable(self):
        self.raw['camera']['future_policy'] = {'enabled': True}
        change_payload(self.raw, lambda p: p.update(future_policy={'value': 1}))
        projected = self.project(self.raw)['configuration_values']
        self.assertEqual(projected['camera']['future_policy'], {'enabled': True})
        self.assertEqual(projected['camera_read_diagnostics']['payload_json']['future_policy'], {'value': 1})

    def test_raw_and_comparison_evidence_are_written(self):
        first = c.parameters(SimpleNamespace(getParam=lambda _: deepcopy(self.raw)), OUTPUT, 'before')
        second = c.parameters(SimpleNamespace(getParam=lambda _: load(AFTER)), OUTPUT, 'after')
        c.same_parameters(first, second, OUTPUT, 'equal')
        self.assertEqual(load(first['raw_evidence']['path'])['parameter_values'], self.raw)
        self.assertEqual(sha(first['raw_evidence']['path']), first['raw_evidence']['sha256'])
        self.assertTrue(load(OUTPUT/'parameter_comparison_equal.json')['passed'])


def rejection(name, mutate):
    def test(self):
        self.assert_drift_rejected(mutate)
    test.__name__ = 'test_reject_' + name
    setattr(Contracts, test.__name__, test)

rejection('noncamera_control', lambda d: d['grasp'].update(require_actuation_confirmation=False))
rejection('noncamera_unknown', lambda d: d.update(future_control_policy=True))
rejection('camera_enabled', lambda d: d['camera']['runtime_profile']['stationary_temporal'].update(enabled=False))
rejection('camera_alpha', lambda d: d['camera']['runtime_profile']['stationary_temporal'].update(alpha=.5))
rejection('camera_unknown_null', lambda d: d['camera']['runtime_profile']['stationary_temporal'].update(future_setting=None))
rejection('camera_intrinsics', lambda d: d['camera']['runtime_profile']['color_intrinsics'].update(width=641))
rejection('camera_activation', lambda d: d['camera']['runtime_profile'].update(geometry_activation_stamp_ns='1'))
rejection('camera_frame_id', lambda d: d['camera'].update(frame_id='other_frame'))
rejection('unknown_diagnostic_envelope', lambda d: d['camera_read_diagnostics'].update(future_flag=None))
rejection('unknown_payload', lambda d: change_payload(d, lambda p: p.update(future_policy=None)))
rejection('unknown_row_null', lambda d: change_payload(d, lambda p: p['entries'][0].update(future_flag=None)))
rejection('unknown_evidence_null', lambda d: change_payload(d, lambda p: p['entries'][0]['evidence'].update(future_flag=None)))
rejection('unknown_history_null', lambda d: add_history(d, lambda h: h.update(future_flag=None)))
rejection('unknown_history_raw_null', lambda d: add_history(d, lambda h: h.update(last_complete_raw_frames={'color':{'present':True,'metadata_raw':{'future_null':None}}})))
rejection('unknown_history_evaluation_null', lambda d: add_history(d, lambda h: h.update(existing_gate_evaluation={'future_null':None})))
rejection('history_scalar_hides_unknown_null', lambda d: add_history(d, lambda h: h.update(reason={'future_null':None})))
rejection('history_evaluation_scalar_hides_unknown_null', lambda d: add_history(d, lambda h: h.update(existing_gate_evaluation={'regional_global_reset_reason':{'future_null':None}})))
rejection('history_region_scalar_hides_unknown_null', lambda d: add_history(d, lambda h: h.update(existing_gate_evaluation={'regions':[{'name':{'future_null':None}}]})))
rejection('regional_current_zero_not_preserved', lambda d: change_payload(d, lambda p: p['entries'][0]['evidence']['regional_stationary_evidence'].update(current_zero_pixels_preserved=False)))
rejection('unknown_host_null', lambda d: change_payload(d, lambda p: p['entries'][0]['evidence']['host_wait']['enter'].update(future_flag=None)))
rejection('unknown_raw_metadata', lambda d: change_payload(d, lambda p: p['entries'][0]['evidence']['raw_frames']['color']['metadata_raw'].update(future_flag=1)))
rejection('unknown_stage', lambda d: change_payload(d, lambda p: p['entries'][0]['evidence']['stages_sec'].update(future_stage=0.1)))
rejection('unknown_regional_null', lambda d: change_payload(d, lambda p: p['entries'][0]['evidence']['regional_stationary_evidence'].update(future_flag=None)))
rejection('regional_visual_policy', lambda d: change_payload(d, lambda p: p['entries'][0]['evidence']['regional_stationary_evidence']['regions'][0]['visual'].update(window_sec=.7)))
rejection('regional_roi', lambda d: change_payload(d, lambda p: p['entries'][0]['evidence']['regional_stationary_evidence']['regions'][0].update(bounds_xyxy=[0,0,1,1])))
rejection('history_authority', lambda d: add_history(d, lambda h: h.update(authorizes_pair_or_motion=True)))
rejection('shadow_authority', lambda d: change_payload(d, lambda p: p['entries'][0]['evidence']['regional_stationary_evidence']['nested_center_shadow'].update(motion_authorized=True)))
rejection('payload_geometry_generation', lambda d: change_payload(d, lambda p: p.update(geometry_generation=p['geometry_generation']+1)))
rejection('payload_capacity', lambda d: change_payload(d, lambda p: p.update(capacity=33)))
rejection('payload_authority', lambda d: change_payload(d, lambda p: p.update(motion_authorized=True)))
rejection('row_geometry_generation', lambda d: change_payload(d, lambda p: p['entries'][0].update(geometry_generation=999)))
rejection('row_frame_id', lambda d: change_payload(d, lambda p: p['entries'][0].update(frame_id='other_frame')))
rejection('row_activation', lambda d: change_payload(d, lambda p: p['entries'][0].update(geometry_activation_stamp_ns='1')))
rejection('busy_ledger_reserved', lambda d: d.update(rgb_acquisition={'ledger':{'phase':'reserved'}}))
rejection('busy_ledger_executing', lambda d: d.update(rgb_acquisition={'ledger':{'phase':'executing'}}))
rejection('duplicate_json_key', lambda d: d['camera_read_diagnostics'].update(payload_json='{"schema":1,' + d['camera_read_diagnostics']['payload_json'][1:]))
rejection('nonfinite_json', lambda d: d['camera_read_diagnostics'].update(payload_json='{"future_nan":NaN,' + d['camera_read_diagnostics']['payload_json'][1:]))
rejection('infinite_json', lambda d: d['camera_read_diagnostics'].update(payload_json='{"future_inf":Infinity,' + d['camera_read_diagnostics']['payload_json'][1:]))

def main():
    global c, OUTPUT
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    OUTPUT = args.output.resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount() and E in OUTPUT.parents
    assert all(sha(path) == digest for path, digest in PINS.items())
    OUTPUT.mkdir()
    # Reuse the existing small guard definition without importing its helper
    # modules before isolation is installed.
    guard_path = W / '.runtime/test_gen60_gateway_reload_contracts_20261022.py'
    import ast
    node = next(n for n in ast.parse(guard_path.read_bytes()).body if isinstance(n, ast.ClassDef) and n.name == 'OfflineGuard')
    namespace = {'Path':Path, 'os':os}
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(guard_path),'exec'),namespace)
    guard = namespace['OfflineGuard'](OUTPUT)
    sys.addaudithook(guard)
    sys.path.insert(0, str(W/'.runtime'))
    import gen60_gateway_reload_common_20261022 as common
    c = common
    before = c.helper_map()
    sources = dict(c.CAMERA_PINS)
    stream = io.StringIO()
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(Contracts)
    identities = [test.id() for test in suite]
    started = time.monotonic()
    run = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    elapsed = time.monotonic()-started
    (OUTPUT/'tests.log').write_text(stream.getvalue())
    unchanged = before == c.helper_map() and all(sha(p)==h for p,h in sources.items())
    passed = run.wasSuccessful() and unchanged and not guard.blocked
    result = dict(passed=passed, tests=run.testsRun, failures=len(run.failures), errors=len(run.errors),
        skipped=len(run.skipped), elapsed_sec=elapsed, case_identities=identities,
        helper_sha256=before, source_unchanged=unchanged, producer_sha256=sources,
        test_helper_sha256=sha(__file__), guard_source_sha256={str(guard_path):sha(guard_path)},
        input_sha256=PINS, blocked_operations=guard.blocked,
        scope='Real frozen parameter snapshots and independent configuration/unknown-key/geometry/ledger negatives; mock parameter reader only.',
        production_tests_repeated=False, ros_calls_performed=False, network_calls_performed=False,
        installation_executed=False, reload_executed=False, motion_commands_sent=False)
    c.save(OUTPUT/'result.json',result)
    print(json.dumps(dict(passed=passed,tests=run.testsRun,failures=len(run.failures),errors=len(run.errors),elapsed_sec=elapsed,result=str(OUTPUT/'result.json'),sha256=sha(OUTPUT/'result.json'))))
    if not passed:
        raise SystemExit(1)

if __name__ == '__main__':
    main()
