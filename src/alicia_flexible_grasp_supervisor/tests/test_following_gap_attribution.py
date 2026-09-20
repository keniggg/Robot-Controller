import importlib.util
from pathlib import Path

import numpy as np
import pytest

from test_stationary_following import fixture


spec = importlib.util.spec_from_file_location('following_attribution_readonly',
    Path(__file__).resolve().parents[3] / 'tools/analyze_following_gap_bag_readonly.py')
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def test_joint_contributions_sum_to_true_fk_vector_not_sum_of_norms():
    fk, sdk, actual = fixture()
    sdk_q, actual_q = np.array(sdk[-1].position), np.array(actual[-1].position[:6])
    actual_q += np.array([.01, -.02, .03, -.01, .015, -.01])
    result = tool.displacement_attribution(fk, sdk_q, actual_q)
    error_mm = (fk(actual_q)[:3, 3]-fk(sdk_q)[:3, 3])*1000
    assert result['sum_vector_mm'] == pytest.approx(error_mm, abs=1e-12)
    assert sum(result['projection_along_total_error_mm'].values()) == pytest.approx(
        np.linalg.norm(error_mm), abs=1e-12)


def test_zero_gap_has_no_arbitrary_normalized_direction():
    fk, sdk, _ = fixture()
    result = tool.displacement_attribution(fk, sdk[-1].position, sdk[-1].position)
    assert result['sum_vector_mm'] == [0., 0., 0.]
    assert result['projection_along_total_error_mm'] is None


def test_single_joint_error_is_not_assigned_to_unmoved_joints():
    fk, sdk, actual = fixture()
    result = tool.displacement_attribution(fk, sdk[-1].position, actual[-1].position[:6])
    for joint, vector in result['joint_displacement_contributions_mm'].items():
        if joint != 'Joint2':
            assert vector == [0., 0., 0.]


def test_invalid_joint_shape_is_rejected():
    fk, _, _ = fixture()
    with pytest.raises(ValueError):
        tool.displacement_attribution(fk, [0.]*5, [0.]*6)
