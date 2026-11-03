"""Selected-disk observers retain identity refusal and historical input audits."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


W = Path(__file__).resolve().parents[2]


def load(name, relative):
    spec = importlib.util.spec_from_file_location(name, W/relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_input_audit_tracks_both_disks_and_excludes_own_generated_files(tmp_path):
    audit = load('two_disk_input_audit', '.runtime/integrated_import_audit.py')
    old, new, outside = (tmp_path/p for p in ('old', 'new', 'outside'))
    for p in (old, new, outside):
        p.mkdir()
    worker = new/'worker'
    worker.mkdir()
    paths = [old/'historical.json', new/'current.json', worker/'fixture.json', outside/'other.json']
    for p in paths:
        p.write_text('{}')
    audit._experiments = (old, new)
    audit._output = worker
    audit._input_events = worker/'input_events.jsonl'
    for p in paths:
        audit._record_input(str(p))
    rows = [json.loads(line) for line in audit._input_events.read_text().splitlines()]
    assert {row['path'] for row in rows} == {str(p) for p in paths[:2]}
    audit._record_input(str(paths[0]))
    assert len(audit._input_events.read_text().splitlines()) == 2
    paths[0].write_text('{"changed":true}')
    audit._record_input(str(paths[0]))
    assert len(audit._violations) == 1
    assert audit._violations[0]['external_input_changed']['after']['size'] != 2


@pytest.mark.parametrize('backend', ['ssd', 'sanstand'])
def test_sampler_rechecks_frozen_storage_and_same_device_before_writing(tmp_path, backend):
    sampler = load('selected_disk_sampler_'+backend, 'tools/diagnostics/sample_bound_process_resources.py')
    logs = tmp_path/'ros_log'
    logs.mkdir()
    output = logs/'samples'
    binding = {'backend': backend, 'mount_id': 7}
    calls = []

    def validate(path, *, subtree, expected):
        calls.append((path, subtree, expected))
        assert expected is binding
        return binding

    sampler.STORAGE = SimpleNamespace(mount=tmp_path, ros_log_root=logs, validate=validate)
    sampler.STORAGE_BINDING = binding
    device = sampler.storage(output)
    assert calls == [(output, logs, binding)]
    assert not output.exists()
    with pytest.raises(ValueError, match='device changed'):
        sampler.storage(output, device + 1)
    assert not output.exists()

    def changed(*args, **kwargs):
        raise RuntimeError('frozen storage identity changed')

    sampler.STORAGE.validate = changed
    with pytest.raises(RuntimeError, match='frozen storage identity changed'):
        sampler.storage(output, device)
    assert not output.exists()


def test_sampler_rejects_unselected_or_outside_log_root(tmp_path):
    sampler = load('missing_storage_sampler', 'tools/diagnostics/sample_bound_process_resources.py')
    with pytest.raises(ValueError, match='selection is missing'):
        sampler.storage(tmp_path/'samples')
    sampler.STORAGE = SimpleNamespace(mount=tmp_path, ros_log_root=tmp_path/'ros_log',
                                     validate=lambda *args, **kwargs: {})
    with pytest.raises(ValueError, match='below selected ros_log'):
        sampler.storage(tmp_path/'samples')
