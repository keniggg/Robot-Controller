"""User-authorized 36 -> 24 image margin, using sealed GEN4 geometry only."""
import ast
import inspect
import json
from pathlib import Path
import re

import numpy as np
import pytest
import yaml

from alicia_flexible_grasp.grasp.observation_working_volume import (
    ObservationWorkingVolumeError, observation_obb_corners,
    validate_planned_observation_working_volume,
    validate_observation_working_volume,
)
from alicia_flexible_grasp.robot.observation_view_limits import (
    DEFAULT_OBSERVATION_VIEW_PARAMETERS, MINIMUM_OBSERVATION_IMAGE_MARGIN_PX,
    effective_observation_view_limits, read_observation_view_parameters,
)


@pytest.fixture
def recorded():
    return json.loads((Path(__file__).parent / 'fixtures/gen4_view_margin_policy.json').read_text())


def inputs(recorded, measured):
    camera = recorded['camera']
    return dict(target_corners_base=observation_obb_corners(
        recorded['geometry_center_base_m'], recorded['geometry_axes_base'], recorded['geometry_size_xyz_m']),
        intrinsics=camera['intrinsics'], image_width=camera['width'], image_height=camera['height'],
        base_from_camera=np.asarray(measured['base_from_camera']),
        source_stamp_ns=measured['source_ns'], camera_transform_stamp_ns=measured['source_ns'])


@pytest.mark.parametrize('index', [0, 1])
def test_gen4_exact_actual_31_px_passes_default24_but_explicit36_rejects(recorded, index):
    data = inputs(recorded, recorded['measured_source_frames'][index])
    result = validate_observation_working_volume(**data)
    assert result['ok'] and result['required_image_margin_px'] == 24.
    assert result['minimum_image_edge_distance_px'] == pytest.approx(31.139970754524143, abs=1e-9)
    assert result['certifies_grasp_success'] is False
    for margin in (32., 36., 48.):
        with pytest.raises(ObservationWorkingVolumeError, match='image margin') as caught:
            validate_observation_working_volume(**data, roi_margin_px=margin)
        assert caught.value.audit['required_image_margin_px'] == margin
    # Identical geometry in the planning API gets the identical margin result,
    # without confusing a future prediction with a timestamped measurement.
    transform = data.pop('base_from_camera')
    stamp = data.pop('source_stamp_ns')
    data.pop('camera_transform_stamp_ns')
    planned = validate_planned_observation_working_volume(**data,
        planned_base_from_camera=transform, snapshot_stamp_ns=stamp)
    assert planned['minimum_image_edge_distance_px'] == result['minimum_image_edge_distance_px']
    assert planned['actual_arrival_measurement_required'] is True
    assert 'source_stamp_ns' not in planned
    with pytest.raises(ObservationWorkingVolumeError, match='image margin'):
        validate_planned_observation_working_volume(**data,
            planned_base_from_camera=transform, snapshot_stamp_ns=stamp, roi_margin_px=36.)


def test_recorded_nominal59_px_retains_explicit36_and_newdefault(recorded):
    data = inputs(recorded, recorded['measured_source_frames'][0])
    data['base_from_camera'] = np.asarray(recorded['planned']['base_from_camera'])
    for margin in (24., 36.):
        result = validate_observation_working_volume(**data, roi_margin_px=margin)
        assert result['minimum_image_edge_distance_px'] == pytest.approx(59.540175527635796)


@pytest.mark.parametrize('fault', ['stamp', 'depth', 'shape', 'relaxed_margin'])
def test_margin_change_does_not_admit_bad_source_or_geometry(recorded, fault):
    data = inputs(recorded, recorded['measured_source_frames'][0])
    if fault == 'stamp': data['camera_transform_stamp_ns'] += 1000000
    elif fault == 'depth': data['base_from_camera'][:3, 3] += data['base_from_camera'][:3, 0] * .12
    elif fault == 'shape': data['target_corners_base'] = np.zeros((8, 3))
    else: data['roi_margin_px'] = 23.999
    with pytest.raises(ObservationWorkingVolumeError):
        validate_observation_working_volume(**data)


def test_all_public_view_defaults_and_config_use_same24_minimum():
    from alicia_flexible_grasp.robot.observation_view_guard import (
        validate_observation_target_trajectory, validate_observation_support_projection,
    )
    from alicia_grasp_modes.observation_arrival_projection import validate_arrival_projection
    from alicia_grasp_modes.observation_policy import observation_distance_preferences
    assert MINIMUM_OBSERVATION_IMAGE_MARGIN_PX == 24.
    assert DEFAULT_OBSERVATION_VIEW_PARAMETERS['observation_camera_visibility_margin_px'] == 24.
    for function in (validate_planned_observation_working_volume,
                     validate_observation_working_volume, validate_observation_target_trajectory,
                     validate_observation_support_projection, validate_arrival_projection,
                     observation_distance_preferences):
        assert inspect.signature(function).parameters['roi_margin_px'].default == 24.
    for margin in (24., 31., 36., 48.):
        assert effective_observation_view_limits({'camera_visibility_margin_px': margin})['roi_margin_px'] == margin
    config = yaml.safe_load((Path(__file__).parents[1] / 'config/grasp_params.yaml').read_text())
    assert config['grasp_6d']['remote']['camera_visibility_margin_px'] == 36
    assert config['grasp_6d']['remote']['observation_camera_visibility_margin_px'] == 24


def test_no_hidden_36_floor_or_default_in_observation_production_sources():
    # Scan the real runtime packages rather than relying on a second hard-coded
    # list of call sites. Explicit stricter test fixtures are intentionally omitted.
    root = Path(__file__).parents[3]
    found = []
    for package in ('alicia_flexible_grasp_supervisor', 'alicia_grasp_modes'):
        for folder in ('src', 'scripts'):
            for path in (root / 'src' / package / folder).rglob('*.py'):
                if path.name == 'remote_grasp6d_node.py':
                    continue  # Generic contact-camera policy intentionally remains 36px.
                source = path.read_text()
                ast.parse(source)
                for number, line in enumerate(source.splitlines(), 1):
                    code = line.split('#', 1)[0]
                    if re.search(r'\b36(?:\.|\b)', code) and 'margin' in code.lower():
                        found.append('%s:%s:%s' % (path.relative_to(root), number, code.strip()))
    assert not found, '\n'.join(found)


def test_runtime_observation_override_is_independent_of_generic_contact_setting():
    params = {'/grasp_6d/remote/camera_visibility_margin_px': 36.}
    requested = read_observation_view_parameters(params.get)
    assert set(requested) == set(DEFAULT_OBSERVATION_VIEW_PARAMETERS)
    assert effective_observation_view_limits(requested)['roi_margin_px'] == 24.
    params['/grasp_6d/remote/camera_visibility_margin_px'] = 50.
    assert read_observation_view_parameters(params.get) == requested
    params['/grasp_6d/remote/observation_camera_visibility_margin_px'] = 36.
    new_requested = read_observation_view_parameters(params.get)
    assert new_requested != requested
    assert effective_observation_view_limits(new_requested)['roi_margin_px'] == 36.
    assert params['/grasp_6d/remote/camera_visibility_margin_px'] == 50.
