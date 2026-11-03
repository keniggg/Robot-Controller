"""The user's eight-worker ceiling and resource/file bounds, without execution."""
import argparse
import importlib.util
from pathlib import Path

import pytest

path = Path(__file__).resolve().parents[2] / '.runtime/verify_integrated_failure_repair.py'
spec = importlib.util.spec_from_file_location('parallel_budget_runner', path)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.mark.parametrize('requested,cpus,files,expected', [
    (None, 12, 674, 8), (None, 24, 674, 8), (8, 12, 674, 8),
    (1, 12, 674, 1), (6, 12, 674, 6), (None, 6, 674, 6),
    (8, 3, 674, 3), (None, 12, 3, 3), (8, 12, 1, 1),
    (None, 1, 674, 1), (3, 12, 674, 3),
])
def test_user_limit_resource_budget_and_independent_scope(requested, cpus, files, expected):
    assert runner.allocate_jobs(requested, cpus, files) == expected


@pytest.mark.parametrize('requested,cpus,files', [
    (0, 12, 674), (-1, 12, 674), (9, 12, 674), (True, 12, 674),
    (1.5, 12, 674), (None, 0, 674), (None, 12, 0), (None, True, 674),
])
def test_invalid_budget_is_rejected(requested, cpus, files):
    with pytest.raises(ValueError):
        runner.allocate_jobs(requested, cpus, files)


@pytest.mark.parametrize('value', ['0', '-1', '9', '1.5', 'auto', ''])
def test_cli_rejects_invalid_job_count_before_any_storage_or_worker_access(value, monkeypatch):
    monkeypatch.setattr(runner, 'scope', lambda: pytest.fail('invalid CLI reached discovery'))
    monkeypatch.setattr(runner, 'run_file', lambda *a: pytest.fail('invalid CLI launched a worker'))
    with pytest.raises(SystemExit) as caught:
        runner.main(['--name', 'must_not_run', '--jobs', value])
    assert caught.value.code == 2


@pytest.mark.parametrize('value,expected', [('1', 1), ('8', 8)])
def test_cli_accepts_both_user_boundaries(value, expected):
    assert runner.parse_jobs(value) == expected
