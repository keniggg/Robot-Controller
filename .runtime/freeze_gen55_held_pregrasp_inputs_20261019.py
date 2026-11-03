"""Freeze the eight actual GEN54 endpoint-selection inputs; no ROS imports/I/O."""
import gzip
import hashlib
import json
import math
from pathlib import Path

W = Path(__file__).resolve().parents[1]
D = W/'.runtime/gen55_candidate_r1'
V = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
O = V/'gen55_held_pregrasp_reuse_r1'
SOURCE = 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/held_pregrasp_selection.py'
FIXTURE = 'src/alicia_flexible_grasp_supervisor/tests/fixtures/gen55_g54_held_pregrasp_inputs.json.gz'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    baseline_path = V/'gen55_baseline_r1/result.json'
    assert sha(baseline_path.read_bytes()) == 'c74a26f5bb9de1356141dc8665fa32d5261af3357d434a184ab930c38a55212f'
    baseline = json.loads(baseline_path.read_bytes())
    original = (W/SOURCE).read_bytes()
    assert sha(original) == baseline['source_sha256'][SOURCE]
    delivery_path = V/'gen54_actual_resources_analysis_r1/contact_deliveries.json'
    assert sha(delivery_path.read_bytes()) == 'c6016cad5624e04947fab1fb34d781a60bd146a5250283e038e99f2abf982761'
    deliveries = json.loads(delivery_path.read_bytes())
    assert len(deliveries) == 9 and deliveries[-1]['value']['response'] is None
    model = None
    cases = []
    dependencies = {str(baseline_path): sha(baseline_path.read_bytes()),
                    str(delivery_path): sha(delivery_path.read_bytes()),
                    str(W/SOURCE): sha(original)}
    for index, delivery in enumerate(deliveries[:8]):
        value = delivery['value']; precheck = value['precheck']
        ref = precheck['contact_failure_replay']['reference']
        replay_path = Path(ref['store_root'])/ref['relative_path']
        compressed = replay_path.read_bytes()
        raw = gzip.decompress(compressed)
        assert sha(compressed) == ref['gzip_sha256'] and sha(raw) == ref['object_sha256']
        assert len(compressed) == ref['gzip_bytes'] and len(raw) == ref['uncompressed_bytes']
        replay = json.loads(raw)
        assert replay['following_sample'] == precheck['source_sample']
        assert sha(replay['model']['xml'].encode()) == replay['model']['sha256']
        if model is None:
            model = replay['model']
        assert replay['model'] == model
        held = precheck['held_pregrasp_selection']
        assert held['model_sha256'] == model['sha256']
        assert held['success'] is True and held['reselected'] is True
        grasp = replay['config']['grasp']
        position = grasp['unknown_measured_endpoint_position_tolerance_m']
        angle = math.radians(grasp['measured_endpoint_orientation_tolerance_deg'])
        assert position == .010 and angle == math.radians(5)
        cases.append(dict(index=index, diagnostic_call_id=value['diagnostic_call_id'],
            request_wall_ns=value['request_wall_ns'], recorded_wall_ns=value['recorded_wall_ns'],
            model_sha256=model['sha256'], sample=precheck['source_sample'],
            pregrasp_goal=held['fixed_pregrasp_goal'], approach_goal=held['fixed_approach_goal'],
            nominal_command=held['original_nominal_command_rad'],
            position_tolerance_m=position, orientation_tolerance_rad=angle,
            recorded_selection=held, recorded_precheck_success=precheck['success'],
            recorded_failure_code=precheck['failure_code'], replay_reference=ref))
        dependencies[str(replay_path)] = ref['gzip_sha256']
    fixture = dict(schema=1, kind='actual_GEN54_frozen_held_pregrasp_selection_inputs',
        execution_authorized=False, whole_contact_pipeline_replayed=False,
        future_hardware_prediction=False, model=model, cases=cases,
        baseline_source=dict(path=SOURCE, sha256=sha(original), text=original.decode()),
        dependencies=dependencies)
    fixture_bytes = gzip.compress((json.dumps(fixture, sort_keys=True, allow_nan=False)+'\n').encode(), mtime=0)
    with (D/FIXTURE).open('xb') as stream:
        stream.write(fixture_bytes)
    assert json.loads(gzip.decompress((D/FIXTURE).read_bytes())) == fixture
    O.mkdir()
    result = dict(passed=True, helper_sha256=sha(Path(__file__).read_bytes()),
        fixture_path=FIXTURE, fixture_sha256=sha(fixture_bytes), cases=len(cases),
        baseline_source_sha256=sha(original), model_sha256=model['sha256'], dependencies=dependencies,
        nominal_input_note='Recorded original_nominal_command_rad is the selector\'s already-quantized input; repeated sdk_counts is checked against the actual recorded report.',
        production_modified=False, ROS_initialized=False, hardware_access=False,
        temperature_queried=False, full_regression_run=False)
    with (O/'input_freeze.json').open('x') as stream:
        json.dump(result, stream, indent=2, sort_keys=True); stream.write('\n')
    print(json.dumps({key:result[key] for key in ('passed','fixture_sha256','cases','baseline_source_sha256','model_sha256')}))


if __name__ == '__main__':
    main()
