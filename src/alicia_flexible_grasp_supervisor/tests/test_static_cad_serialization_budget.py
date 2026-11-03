"""Actual monotonic budget boundary; no ROS graph or geometry bypass."""
import hashlib
import io
import time

import pytest

from alicia_flexible_grasp.robot import observation_path_guard as guard
from test_observation_path_guard import check, point, trajectory


@pytest.mark.parametrize('exhaust', [False, True], ids=['account_serialization', 'reject_late_serialization'])
def test_continuous_path_budget_includes_final_real_serialization(monkeypatch, exhaust):
    # Use the original geometry/FK/spline checks and a real ROS message.
    # Only serialization latency is injected; no collision verdict is mocked.
    plan = trajectory([
        point([0.], 0., [0.], [0.]),
        point([0.], 1., [0.], [0.]),
    ])
    budget = 1.0
    real_serialize = type(plan).serialize
    raw = io.BytesIO()
    real_serialize(plan, raw)
    expected_sha256 = hashlib.sha256(raw.getvalue()).hexdigest()
    serializations = []

    def delayed_real_serialization(message, stream):
        real_serialize(message, stream)
        if message is plan:
            entered = time.monotonic()
            # Real wall time, using the same monotonic clock as the validator.
            # The late branch crosses the unchanged local proof budget.
            time.sleep(budget + .03 if exhaust else .03)
            serializations.append((entered, time.monotonic()))

    monkeypatch.setattr(type(plan), 'serialize', delayed_real_serialization)
    if exhaust:
        with pytest.raises(guard.ObservationPathError, match='computation budget exhausted'):
            check(plan, max_seconds=budget, clock=time.monotonic)
    else:
        report = check(plan, max_seconds=budget, clock=time.monotonic)
        assert report['trajectory_sha256'] == expected_sha256
        assert report['certified_minimum_clearance_m'] >= .003
        assert serializations
        serialization_elapsed = serializations[0][1] - serializations[0][0]
        assert serialization_elapsed >= .03
        assert report['elapsed_sec'] >= serialization_elapsed
        assert report['elapsed_sec'] <= budget
    assert len(serializations) == 1
    if exhaust:
        assert serializations[0][1] - serializations[0][0] > budget
