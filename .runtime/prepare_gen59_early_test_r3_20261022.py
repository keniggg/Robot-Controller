"""Prepare source-bound GEN58 fixture and preserve old early-validation invariants."""
from pathlib import Path
import json,hashlib
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen59_observation_pose_candidate_r3';N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1');I=N/'observation_pose_fix/offline_selection_r1/source_attempt3';F=C/'src/alicia_grasp_modes/tests/fixtures';O=N/'observation_pose_fix/full_r2_failure_review/r3_preparation';assert Path('/media/zhuyupei/SanStand').is_mount();O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();m=json.loads((I/'source.json').read_text());proof=json.loads((I/'reconstruction.json').read_text());work=json.loads((N/'observation_pose_fix/full_r2_failure_review/gen59_workclock/result.json').read_text());assert all(x['exact'] for x in proof['checks'].values());assert work['candidates'] and work['counters']['first_qualified']['calls']==43
modelrel='deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf';assert sha(C/modelrel)==m['model_sha256'];assert not (F/'gen59_fk_source.json').exists() and not (F/'gen59_fk_source.npz').exists();(F/'gen59_fk_source.npz').write_bytes((I/'source.npz').read_bytes())
meta=dict(schema=1,record='GEN58 sealed source1792639754162577867 for GEN59 opposite-band policy',source=m['source'],arrival_audit=m['arrival_audit'],current_joint_positions=m['current_joint_positions'],gaps=m['gaps'],joint_error_bounds_rad=m['joint_error_bounds_rad'],model_relative_path=modelrel,model_sha256=m['model_sha256'],opening_width_m=.05,arrays_sha256=sha(F/'gen59_fk_source.npz'),source_audit=m['source_audit_path'],source_audit_sha256=m['source_audit_sha256'],source_reconstruction_report=str(I/'reconstruction.json'),source_reconstruction_report_sha256=sha(I/'reconstruction.json'),all_audited_source_hashes_exact=True,first_real_qualified_probe=work['counters']['first_qualified']['q'],first_real_qualified_probe_source=str(N/'observation_pose_fix/full_r2_failure_review/gen59_workclock/result.json'),first_real_qualified_probe_source_sha256=sha(N/'observation_pose_fix/full_r2_failure_review/gen59_workclock/result.json'),work_clock_step_sec=.002,work_clock_budget_sec=2.,first_real_qualified_probe_calls=43,execution_authority=False,whole_arm_collision_checked=False,complete_path_checked=False)
(F/'gen59_fk_source.json').write_text(json.dumps(meta,indent=2,allow_nan=False)+'\n');metasha=sha(F/'gen59_fk_source.json')
p=C/'src/alicia_grasp_modes/tests/test_reachable_observation_early_qualified.py';before=p.read_text();s=before.replace('"""GEN14 exact measured source;','"""GEN58 exact measured source;',1)
old='FEASIBLE_PROBE = np.array([-1.9662889215842203, -.541789220533962,\n    1.1962718646211747, -.012362442272550192, -1.4279141953957244, .13385894906599669])'
new='FEASIBLE_PROBE = np.array('+repr(meta['first_real_qualified_probe'])+')';assert old in s;s=s.replace(old,new,1)
old="    metadata = json.loads((FIXTURE / 'gen14_fk_source.json').read_text())\n    arrays_path = FIXTURE / 'gen14_fk_source.npz'"
new="    # This mechanism suite retains its historical test identities. GEN14's\n    # original probe now fails the new evidence contract and has a separate\n    # rejection regression; these exact GEN58 inputs have a real qualified probe.\n    metadata_path = FIXTURE / 'gen59_fk_source.json'\n    assert hashlib.sha256(metadata_path.read_bytes()).hexdigest() == '"+metasha+"'\n    metadata = json.loads(metadata_path.read_text())\n    assert np.array_equal(FEASIBLE_PROBE, metadata['first_real_qualified_probe'])\n    arrays_path = FIXTURE / 'gen59_fk_source.npz'"
assert old in s;s=s.replace(old,new,1)
s=s.replace('def test_gen14_first_qualified_probe_survives_original_computation_budget(recorded, measured_work_clock):\n','def test_gen14_first_qualified_probe_survives_original_computation_budget(recorded, measured_work_clock):\n    """Historical test identity; exact GEN58 source, unchanged .002 work clock."""\n',1);p.write_text(s)
newtest=C/'src/alicia_grasp_modes/tests/test_observation_shared_band_gain.py';oldmeta=F/'gen14_fk_source.json';oldmeta_sha=sha(oldmeta)
addition='''\n\ndef test_original_gen14_probe_fails_retained_opposite_band_without_relaxing_gates():
    """Keep the original GEN14 data: its old feasible pose lacks new side gain."""
    from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
    root = Path(__file__).resolve().parents[3]
    folder = Path(__file__).parent / 'fixtures'
    metadata_path = folder / 'gen14_fk_source.json'
    assert hashlib.sha256(metadata_path.read_bytes()).hexdigest() == '__GEN14_SHA__'
    metadata = json.loads(metadata_path.read_text())
    arrays_path = folder / 'gen14_fk_source.npz'
    assert hashlib.sha256(arrays_path.read_bytes()).hexdigest() == metadata['arrays_sha256']
    with np.load(str(arrays_path)) as arrays:
        mount = arrays['tool_from_camera'].copy()
    fk = SerialUrdfFk((root / metadata['model_relative_path']).read_text(),
                      ['Joint%d' % i for i in range(1, 7)])
    assert fk.model_sha256 == metadata['model_sha256']
    original_probe = np.array([-1.9662889215842203, -.541789220533962,
        1.1962718646211747, -.012362442272550192, -1.4279141953957244, .13385894906599669])
    camera = (fk(original_probe) @ mount)[:3, 3]
    old_gains, bounded_gains = [], []
    for gap in metadata['gaps']:
        assert gap['side_measurement_basis'] == 'before_common_support_intersection'
        center, jaw = np.asarray(gap['target_center_base_m']), np.asarray(gap['jaw_axis_base'])
        initial = np.asarray(gap['initial_camera_origin_base_m']) - center
        proposed = camera - center
        for sign in gap['missing_side_signs']:
            span, audit = missing_side_observation_span(gap, sign)
            assert audit['observation_span_policy'] == 'retained_opposite_height_span_prediction_v1'
            no_authority(audit)
            delta = max(0., sign * float(jaw @ proposed) / np.linalg.norm(proposed)) - max(
                0., sign * float(jaw @ initial) / np.linalg.norm(initial))
            old_gains.append(gap['object_height_m'] * delta)
            bounded_gains.append(span * delta)
    assert max(old_gains) > .003
    assert max(bounded_gains) < .003
    result = missing_side_view_gain(metadata['gaps'], camera, .003)
    assert not result['ok']
    assert result['code'] == 'OBSERVE_RETURN_MISSING_SIDE_NOT_EXPOSED'
    no_authority(result)
'''.replace('__GEN14_SHA__',oldmeta_sha)
assert 'test_original_gen14_probe_fails_retained_opposite_band_without_relaxing_gates' not in newtest.read_text();newtest.write_text(newtest.read_text()+addition)
receipt=dict(early_test_sha256=sha(p),new_gain_test_sha256=sha(newtest),fixture_metadata_sha256=metasha,fixture_arrays_sha256=sha(F/'gen59_fk_source.npz'),unchanged_gen14_metadata_sha256=oldmeta_sha,unchanged_gen14_arrays_sha256=sha(F/'gen14_fk_source.npz'),modified_early_AST_nodes=['module docstring exact GEN14 to GEN58','FEASIBLE_PROBE assignment value','recorded function body','test_gen14_first_qualified_probe_survives_original_computation_budget function body docstring only'],retained_early_test_names=True,retained_work_clock_step=.002,retained_work_clock_budget=2.,retained_other_test_bodies=True,production_files_edited=False,helper_sha256=sha(Path(__file__)))
(O/'result.json').write_text(json.dumps(receipt,indent=2)+'\n');(O/'helper_source.py').write_bytes(Path(__file__).read_bytes());print(json.dumps(receipt,indent=2))
