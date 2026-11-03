"""Historical byte contracts only: no ROS, runtime imports or robot access."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

import pytest


SOURCE = Path(__file__).resolve().parents[1] / "runtime_evidence.py"
SPEC = importlib.util.spec_from_file_location("workflow_candidate_runtime_evidence", SOURCE)
evidence = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = evidence
SPEC.loader.exec_module(evidence)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def binding(tmp_path, raw=b'{"result": "sealed"}'):
    path = tmp_path / "closed.json"
    path.write_bytes(raw)
    return path, raw, evidence.ReleaseEvidence({path: digest(raw)}, current_paths=[])


def test_path_string_alias_and_separate_maps_read_once(tmp_path, monkeypatch):
    path, raw, _ = binding(tmp_path)
    alias = path.parent / "unused" / ".." / path.name
    pins = evidence.normalize_sha256({path: digest(raw)}, {str(alias): digest(raw)})
    assert list(pins) == [path]
    original = Path.read_bytes
    reads = []

    def counted_read(name):
        reads.append(name)
        return original(name)

    monkeypatch.setattr(Path, "read_bytes", counted_read)
    with evidence.ReleaseEvidence(pins, current_paths=[]).transaction() as transaction:
        transaction.verify_all()
        assert transaction.read_json(str(path)) == {"result": "sealed"}
        assert transaction.read_bytes(alias) == raw
        assert transaction.stats == dict(read_attempts=1, bytes_read=len(raw),
                                         sha256_computations=1, cache_hits=2)
    assert reads == [path]


def test_conflicting_path_types_and_maps_reject_before_read(tmp_path, monkeypatch):
    path = tmp_path / "closed.json"
    monkeypatch.setattr(Path, "read_bytes", lambda _: pytest.fail("must not read"))
    with pytest.raises(evidence.EvidenceError, match="EVIDENCE_PATH_CONFLICT"):
        evidence.normalize_sha256({path: "a" * 64}, {str(path): "b" * 64})
    with pytest.raises(evidence.EvidenceError, match="EVIDENCE_PATH_CONFLICT"):
        evidence.ReleaseEvidence({path: "a" * 64, str(path): "b" * 64}, current_paths=[])


@pytest.mark.parametrize("invalid", [None, True, "a" * 63, "g" * 64])
def test_invalid_digest_rejected(tmp_path, invalid):
    with pytest.raises(evidence.EvidenceError, match="EVIDENCE_SHA256_INVALID"):
        evidence.ReleaseEvidence({tmp_path / "closed.json": invalid}, current_paths=[])


def test_explicit_current_inputs_cannot_enter_historical_cache(tmp_path):
    path, raw, _ = binding(tmp_path)
    with pytest.raises(evidence.EvidenceError, match="CURRENT_INPUT_CANNOT_BE_CACHED"):
        evidence.ReleaseEvidence({path: digest(raw)}, current_paths=[str(path)])
    with pytest.raises(TypeError):
        evidence.ReleaseEvidence({path: digest(raw)})
    with pytest.raises(TypeError):
        evidence.ReleaseEvidence({path: digest(raw)}, current_paths=str(path))


def test_json_uses_verified_bytes_even_if_file_replaced_after_read(tmp_path, monkeypatch):
    path, raw, release = binding(tmp_path)
    original = Path.read_bytes

    def read_then_replace(name):
        data = original(name)
        name.write_bytes(b'{"result": "unverified"}')
        return data

    monkeypatch.setattr(Path, "read_bytes", read_then_replace)
    with release.transaction() as transaction:
        assert transaction.read_json(path) == {"result": "sealed"}
        assert transaction.read_bytes(path) == raw


def test_snapshot_is_not_freshness_and_new_transaction_rehashes_same_mtime(tmp_path):
    path, raw, release = binding(tmp_path)
    stamp = path.stat()
    with release.transaction() as transaction:
        assert transaction.read_bytes(path) == raw
        changed = b'{"result": "edited"}'
        assert len(changed) == len(raw)
        path.write_bytes(changed)
        os.utime(path, ns=(stamp.st_atime_ns, stamp.st_mtime_ns))
        assert transaction.read_bytes(path) == raw  # Historical bytes, not current file.
    with release.transaction() as transaction:
        with pytest.raises(evidence.EvidenceError, match="HASH_MISMATCH"):
            transaction.read_bytes(path)
        with pytest.raises(evidence.EvidenceError, match="TRANSACTION_CLOSED"):
            transaction.read_bytes(path)


def test_new_transaction_always_reads_again(tmp_path, monkeypatch):
    path, _, release = binding(tmp_path)
    original = Path.read_bytes
    reads = []

    def counted_read(name):
        reads.append(name)
        return original(name)

    monkeypatch.setattr(Path, "read_bytes", counted_read)
    for _ in range(2):
        with release.transaction() as transaction:
            transaction.verify_all()
    assert reads == [path, path]


def test_json_mutation_cannot_change_snapshot_or_other_consumer(tmp_path):
    path, _, release = binding(tmp_path)
    with release.transaction() as transaction:
        value = transaction.read_json(path)
        value["result"] = "caller mutation"
        assert transaction.read_json(path) == {"result": "sealed"}


def test_binding_map_is_copied_and_readonly(tmp_path):
    path, raw, _ = binding(tmp_path)
    pins = {path: digest(raw)}
    release = evidence.ReleaseEvidence(pins, current_paths=[])
    pins[path] = "0" * 64
    with pytest.raises(TypeError):
        release.historical_sha256[path] = "0" * 64
    with release.transaction() as transaction:
        assert transaction.read_bytes(path) == raw


@pytest.mark.parametrize("failure", ["missing", "undeclared", "bad_json"])
def test_failed_read_or_parse_closes_transaction(tmp_path, failure):
    raw = b"not JSON" if failure == "bad_json" else b'{"result": "sealed"}'
    path, _, release = binding(tmp_path, raw)
    if failure == "missing":
        path.unlink()
    with release.transaction() as transaction:
        with pytest.raises((OSError, evidence.EvidenceError, json.JSONDecodeError)):
            transaction.read_json(tmp_path / "undeclared.json" if failure == "undeclared" else path)
        with pytest.raises(evidence.EvidenceError, match="TRANSACTION_CLOSED"):
            transaction.verify_all()


def test_closed_transaction_cannot_be_reopened_or_used(tmp_path):
    path, _, release = binding(tmp_path)
    transaction = release.transaction()
    with transaction:
        transaction.verify_all()
    with pytest.raises(evidence.EvidenceError, match="TRANSACTION_CLOSED"):
        transaction.read_bytes(path)
    with pytest.raises(evidence.EvidenceError, match="TRANSACTION_CLOSED"):
        transaction.__enter__()
