import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from test_stationary_following import fixture, NAMES

spec = importlib.util.spec_from_file_location('bounded_probe',
    Path(__file__).resolve().parents[3] / 'tools/run_bounded_tracking_probe.py')
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def make_plan(start, end):
    return SimpleNamespace(joint_trajectory=SimpleNamespace(
        joint_names=list(NAMES), points=[SimpleNamespace(positions=start),
                                         SimpleNamespace(positions=end)]),
        multi_dof_joint_trajectory=SimpleNamespace(points=[]))


def test_multiple_joints_can_be_probed_together_with_no_single_axis_lock():
    fk, _, _ = fixture()
    sdk = np.zeros(6)
    end = np.array([0, 4, 4, 3, 3, -3])*tool.SDK_QUANTUM_RAD
    report = tool.validate_small_plan(make_plan(sdk, end), sdk, fk(end), fk)
    assert report['terminal_sdk_delta_counts'] == [0, 4, 4, 3, 3, -3]


@pytest.mark.parametrize('counts', [5, -5, 30])
def test_over_four_count_terminal_is_rejected(counts):
    fk, _, _ = fixture()
    sdk = np.zeros(6)
    end = sdk.copy(); end[1] = counts*tool.SDK_QUANTUM_RAD
    with pytest.raises(ValueError):
        tool.validate_small_plan(make_plan(sdk, end), sdk, fk(end), fk)


def test_zero_increment_does_not_masquerade_as_response_experiment():
    fk, _, _ = fixture()
    with pytest.raises(ValueError):
        tool.validate_small_plan(make_plan([0]*6, [0]*6), [0]*6, fk([0]*6), fk)


def test_large_start_branch_is_rejected_even_with_small_endpoint():
    fk, _, _ = fixture()
    with pytest.raises(ValueError):
        tool.validate_small_plan(make_plan([.03]*6, [.0015]*6), [0]*6, fk([.0015]*6), fk)


def test_other_display_target_cannot_be_accepted():
    fk, _, _ = fixture()
    with pytest.raises(ValueError):
        tool.validate_small_plan(make_plan([0]*6, [.0015]*6), [0]*6, fk([.1]*6), fk)


def test_shuffled_names_preserve_exact_target_mapping():
    fk, _, _ = fixture()
    end = np.arange(6)*.0005
    plan = make_plan([0]*6, list(end))
    expected = tool.validate_small_plan(plan, [0]*6, fk(end), fk)
    plan.joint_trajectory.joint_names.reverse()
    for p in plan.joint_trajectory.points:
        p.positions.reverse()
    assert tool.validate_small_plan(plan, [0]*6, fk(end), fk) == expected
