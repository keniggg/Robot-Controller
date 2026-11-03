"""Actual failed supplementary-view source; no robot or server access."""
from copy import deepcopy
from pathlib import Path
import hashlib
import json

import numpy as np

from alicia_flexible_grasp.grasp.gripper_geometry import GripperGeometry
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk
from alicia_grasp_modes.reachable_observation_views import generate_reachable_observation_views


def load_recorded_scene(stem):
    root=Path(__file__).resolve().parents[3]
    fixtures=Path(__file__).with_name('fixtures')
    metadata=json.loads((fixtures/(stem+'.json')).read_text())
    archive=fixtures/(stem+'.npz')
    assert hashlib.sha256(archive.read_bytes()).hexdigest()==metadata['arrays_sha256']
    args=deepcopy(metadata['arguments'])
    source={k:deepcopy(v) for k,v in metadata.items() if k not in (
        'arguments','arrays_sha256','model_relative_path','robot_description_sha256',
        'provenance','arrival_projection_audit')}
    with np.load(archive) as arrays:
        source.update({k:arrays[k].copy() for k in ('points_base','corners_base','tool_from_camera')})
        source['arrival_projection']={k:arrays[k].copy() for k in ('prism_base','rgb_valid','invalid_integral')}
    source['arrival_projection']['audit']=deepcopy(metadata['arrival_projection_audit'])
    assert hashlib.sha256(source['points_base'].tobytes()).hexdigest()==source['audit']['support_points_sha256']
    fk=SerialUrdfFk((root/metadata['model_relative_path']).read_text(),['Joint%d'%i for i in range(1,7)])
    assert fk.model_sha256==metadata['robot_description_sha256']
    args.update(source=source,fk=fk,gripper=GripperGeometry(.05,.002,
        np.array([.0434,.0286,.06]),np.array([.1175,.155,.0774]),.003))
    return args


def test_recorded_missing_stronger_view_is_recovered_with_all_original_endpoint_checks():
    args=load_recorded_scene('supplement_view_seed_20261023')
    source, fk = args['source'], args['fk']
    assert len(source['points_base'])==94960
    # Regression runs may share CPU with another isolated worker. The live
    # two-second budget is separately timed on these identical frozen inputs.
    args['max_seconds']=3.
    before=deepcopy(source)
    rows,report=generate_reachable_observation_views(**args)
    assert report['status']=='complete',report
    stronger=[r for r in rows if r['endpoint_checks']['missing_side_view_gain']['predicted_new_side_gain_m']>.005]
    assert stronger,report
    assert len(rows)<=4 and report['tier_attempt_count']==4
    for row in rows:
        for gate in ('missing_side_view_gain','working_volume','observation_envelope',
                'conditional_endpoint_support','measured_support_projection',
                'optical_side_evidence','arrival_projection_endpoint','endpoint_joint_reserve'):
            assert row['endpoint_checks'][gate]['ok'],(gate,row['endpoint_checks'][gate])
        np.testing.assert_allclose(fk(row['q']),row['T_base_tool0'],rtol=0.,atol=1e-12)
        assert row['joint_max_delta_rad']==max(abs(a-b) for a,b in zip(row['q'],args['current_joint_positions']))
        assert row['binding']==source['binding']
        assert row['execution_authority'] is False and row['complete_path_checked'] is False
        assert row['whole_arm_collision_checked'] is False and row['actual_arrival_measurement_required'] is True
    for key in ('points_base','corners_base','tool_from_camera'):
        np.testing.assert_array_equal(source[key],before[key])
    for key in ('prism_base','rgb_valid','invalid_integral'):
        np.testing.assert_array_equal(source['arrival_projection'][key],before['arrival_projection'][key])


def test_safety_limited_recorded_refinement_keeps_useful_intermediate_views():
    args=load_recorded_scene('supplement_view_feasible_gain_20261023')
    args['max_seconds']=3.  # Same-input live two-second timing is recorded separately.
    before=deepcopy(args['source'])
    rows, report=generate_reachable_observation_views(**args)
    assert report['status']=='complete', report
    assert len(rows)<=4 and report['tier_attempt_count']==4
    baseline=[r for r in rows if r['exposure_target_noise_multiplier']==1.]
    refined=[r for r in rows if r['exposure_target_noise_multiplier']==2.]
    assert len(baseline)==2 and len(refined)==2
    # Old hard-target searches returned only the approximately 3 mm baselines.
    # The actual frozen geometry admits useful views below the 6.2 mm request.
    assert all(.005 < r['endpoint_checks']['missing_side_view_gain']['predicted_new_side_gain_m']
        < r['requested_new_side_gain_m'] for r in refined)
    for row in rows:
        for gate in ('missing_side_view_gain','working_volume','observation_envelope',
                'conditional_endpoint_support','measured_support_projection',
                'optical_side_evidence','arrival_projection_endpoint','endpoint_joint_reserve'):
            assert row['endpoint_checks'][gate]['ok'], (gate,row['endpoint_checks'][gate])
        np.testing.assert_allclose(args['fk'](row['q']),row['T_base_tool0'],rtol=0.,atol=1e-12)
        assert row['binding']==args['source']['binding']
        assert not row['execution_authority'] and not row['complete_path_checked']
        assert not row['whole_arm_collision_checked'] and row['actual_arrival_measurement_required']
    for key in ('points_base','corners_base','tool_from_camera'):
        np.testing.assert_array_equal(args['source'][key],before[key])
    for key in ('prism_base','rgb_valid','invalid_integral'):
        np.testing.assert_array_equal(args['source']['arrival_projection'][key],before['arrival_projection'][key])
