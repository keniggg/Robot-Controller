"""Archive guarantees, exercised without ROS, hardware, or network access."""

import hashlib
import importlib.util
import json
from collections import namedtuple
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "grasp_archive.py"
SPEC = importlib.util.spec_from_file_location("grasp_archive", MODULE_PATH)
archive = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(archive)


@pytest.fixture
def config(tmp_path):
    value = archive.load_config(tmp_path / "unused_config.json")
    value.update(repo="example/grasp-recordings", min_free_bytes=0,
                 upload_enabled=True, delete_enabled=False)
    return value


class MemoryStore:
    """Remote survives worker recreation; corruption and outages are explicit."""

    def __init__(self):
        self.remote = {}
        self.uploads = []
        self.restores = []
        self.fail_uploads = 0
        self.fail_verification = False
        self.prepared = []

    def factory(self, config, manifest):
        return self

    def prepare_release(self, create=True):
        self.prepared.append(create)
        return {"id": 1, "html_url": "https://example.invalid/release"}

    def upload_file(self, record_id, source, relative_path):
        self.uploads.append((record_id, relative_path))
        if self.fail_uploads:
            self.fail_uploads -= 1
            raise OSError("temporary upload outage")
        payload = Path(source).read_bytes()
        key = record_id + "/" + relative_path
        self.remote[key] = payload
        return {"key": key, "size_bytes": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
                "download_url": "https://example.invalid/assets/" + key}

    def verify_file(self, entry):
        if self.fail_verification:
            return False
        payload = self.remote.get(entry["key"], b"")
        return (len(payload) == entry["size_bytes"] and
                hashlib.sha256(payload).hexdigest() == entry["sha256"])

    def restore_file(self, entry, dest):
        dest = Path(dest)
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(self.remote[entry["key"]])
        self.restores.append(dest)
        return dest


def save_manifest(record, manifest):
    (record / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")


def completed(root, config, index=0, result="unknown", payload=b"raw rosbag bytes"):
    record = archive.begin_record(root, metadata={"test_index": index}, config=config)
    (record / "capture.bag").write_bytes(payload)
    evidence = {"source": "manual", "note": "operator confirmed outcome"}
    archive.finalize_record(record, result=result, evidence=evidence,
                            failure_reason="test failure" if result == "failure" else "",
                            closed_files=["capture.bag"])
    manifest = archive.load_manifest(record)
    manifest["completed_at"] = "2026-09-22T00:00:%02d+00:00" % index
    save_manifest(record, manifest)
    return record


def release_retry_delay(record):
    manifest = archive.load_manifest(record)
    manifest["upload"]["next_retry_at"] = 0
    save_manifest(record, manifest)


def test_each_attempt_has_independent_timestamp_and_unique_id(tmp_path, config):
    first = archive.begin_record(tmp_path, metadata={"attempt": 1}, config=config)
    second = archive.begin_record(tmp_path, metadata={"attempt": 1}, config=config)
    assert first != second
    assert first.parent == second.parent == tmp_path
    for record in (first, second):
        manifest = archive.load_manifest(record)
        assert manifest["state"] == "recording"
        assert manifest["result"] == "unknown"
        assert manifest["started_at"]
        assert not manifest.get("completed_at")
        assert manifest["record_id"] == record.name


def test_normal_completion_and_attempt_name_do_not_imply_success(tmp_path, config):
    root = tmp_path / "successful_attempt_retry"
    root.mkdir()
    record = archive.begin_record(root, metadata={"exit_code": 0}, config=config)
    (record / "capture.bag").write_bytes(b"closed bag")
    manifest = archive.finalize_record(record, closed_files=["capture.bag"])
    assert manifest["result"] == "unknown"
    assert manifest["state"] == "complete"
    assert manifest["completed_at"]


@pytest.mark.parametrize("result", ["success", "failure", "unknown", "interrupted"])
def test_actual_or_explicitly_confirmed_result_is_preserved(tmp_path, config, result):
    record = completed(tmp_path, config, result=result)
    manifest = archive.load_manifest(record)
    assert manifest["result"] == result
    assert manifest["files"][0]["path"] == "capture.bag"
    assert manifest["files"][0]["size_bytes"] == len(b"raw rosbag bytes")
    assert manifest["files"][0]["sha256"] == hashlib.sha256(b"raw rosbag bytes").hexdigest()
    if result == "failure":
        assert manifest["failure_reason"] == "test failure"


def test_invalid_result_rejected(tmp_path, config):
    record = archive.begin_record(tmp_path, config=config)
    with pytest.raises(archive.ArchiveError):
        archive.mark_result(record, "normal_exit", "", {"source": "manual"})


def test_unregistered_files_are_not_implicitly_added(tmp_path, config):
    record = archive.begin_record(tmp_path, config=config)
    (record / "capture.bag").write_bytes(b"closed")
    (record / "still_open.log").write_bytes(b"do not archive yet")
    manifest = archive.finalize_record(record, closed_files=["capture.bag"])
    assert [entry["path"] for entry in manifest["files"]] == ["capture.bag"]


@pytest.mark.parametrize("bad_path", ["../outside.bag", "/tmp/outside.bag", ".git/config",
                                     "launch.sh", "source.py", "calibration.yaml"])
def test_manifest_cannot_register_escape_or_protected_files(tmp_path, config, bad_path):
    record = archive.begin_record(tmp_path, config=config)
    path = record / bad_path
    if not Path(bad_path).is_absolute():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"must survive")
    with pytest.raises((archive.ArchiveError, OSError)):
        archive.finalize_record(record, closed_files=[bad_path])
    if not Path(bad_path).is_absolute():
        assert path.read_bytes() == b"must survive"


def test_symlink_file_cannot_register_external_data(tmp_path, config):
    outside = tmp_path / "outside.bag"
    outside.write_bytes(b"external recording")
    record = archive.begin_record(tmp_path / "records", config=config)
    (record / "capture.bag").symlink_to(outside)
    with pytest.raises((archive.ArchiveError, OSError)):
        archive.finalize_record(record, closed_files=["capture.bag"])
    assert outside.read_bytes() == b"external recording"


def test_recording_never_enters_upload_queue(tmp_path, config):
    record = archive.begin_record(tmp_path, config=config)
    (record / "capture.bag").write_bytes(b"still recording")
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    assert store.uploads == []
    assert (record / "capture.bag").exists()


def test_closed_files_upload_and_retry_survive_new_worker(tmp_path, config):
    record = completed(tmp_path, config)
    store = MemoryStore()
    store.fail_uploads = 1
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    failed = archive.load_manifest(record)
    assert failed["upload"]["attempts"] >= 1
    assert failed["upload"]["error"]
    assert (record / "capture.bag").exists()
    release_retry_delay(record)
    archive.process_queue(tmp_path, config=dict(config), store_factory=store.factory)
    uploaded = archive.load_manifest(record)
    assert uploaded["upload"]["status"] == "verified"
    assert uploaded["files"][0]["upload"]["status"] == "verified"
    entry = uploaded["files"][0]["upload"]["entry"]
    restored = tmp_path / "restore" / "capture.bag"
    store.restore_file(entry, restored)
    assert restored.read_bytes() == (record / "capture.bag").read_bytes()
    uploads_before = len(store.uploads)
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    assert len(store.uploads) == uploads_before


def test_remote_verification_failure_keeps_local_record(tmp_path, config):
    records = [completed(tmp_path, config, index=i) for i in range(archive.RETAIN + 1)]
    store = MemoryStore()
    store.fail_verification = True
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    archive.cleanup(tmp_path, config=config, store_factory=store.factory, dry_run=False)
    for record in records:
        assert (record / "capture.bag").exists()
        assert archive.load_manifest(record)["upload"]["status"] != "verified"


def test_default_cleanup_is_preview_and_keeps_every_record(tmp_path, config):
    records = [completed(tmp_path, config, index=i) for i in range(archive.RETAIN + 2)]
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    archive.cleanup(tmp_path, config=config, store_factory=store.factory)
    assert all((record / "capture.bag").exists() for record in records)
    archive.cleanup(tmp_path, config=config, store_factory=store.factory, dry_run=False)
    assert all((record / "capture.bag").exists() for record in records)


def test_low_space_blocks_new_record_without_touching_existing_files(tmp_path, config, monkeypatch):
    usage = namedtuple("usage", "total used free")
    monkeypatch.setattr(archive.shutil, "disk_usage", lambda _: usage(1000, 999, 1))
    config["min_free_bytes"] = 100
    existing = tmp_path / "keep.log"
    existing.write_text("existing log")
    with pytest.raises(archive.InsufficientSpace):
        archive.preflight_space(tmp_path, config=config)
    with pytest.raises(archive.InsufficientSpace):
        archive.begin_record(tmp_path, config=config)
    assert existing.read_text() == "existing log"
    assert not list(tmp_path.glob("*/manifest.json"))


def test_preview_has_no_remote_or_deletion_side_effects(tmp_path, config):
    records = [completed(tmp_path, config, index=i, result="success" if i % 2 else "failure")
               for i in range(archive.RETAIN + 2)]
    before = {record.name: (record / "manifest.json").read_bytes() for record in records}
    report = archive.preview(tmp_path, config=config)
    assert {"records", "upload", "cleanup", "protected"}.issubset(report)
    for record in records:
        assert (record / "capture.bag").exists()
        assert (record / "manifest.json").read_bytes() == before[record.name]


def deletion_config(config):
    """Proof is test-local; production validation must perform a real round trip."""
    return dict(config, delete_enabled=True, deletion_validation={
        "repo": config["repo"], "retain_count": archive.RETAIN, "upload_verified": True,
        "restore_verified": True, "retention_verified": True})


def test_all_results_share_one_latest_five_completed_retention_set(tmp_path, config):
    outcomes = ["success", "failure", "unknown", "interrupted"] * 3
    assert archive.RETAIN == 5
    records = [completed(tmp_path, config, index=i, result=result)
               for i, result in enumerate(outcomes)]
    (records[0] / "unregistered.log").write_text("newer modification on older record")
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    preview = archive.preview(tmp_path, config=config)
    assert set(preview["protected"]) == {p.name for p in records[-archive.RETAIN:]}
    assert {item["record_id"] for item in preview["cleanup"]} == {p.name for p in records[:-archive.RETAIN]}
    assert all(archive.load_manifest(p)["upload"]["status"] == "verified" for p in records)
    result = archive.cleanup(tmp_path, config=deletion_config(config),
                             store_factory=store.factory, dry_run=False)
    assert set(result["deleted"]) == {p.name for p in records[:-archive.RETAIN]}
    for p in records[:-archive.RETAIN]:
        assert not (p / "capture.bag").exists()
        assert archive.load_manifest(p)["local_data_deleted"]
        assert (p / "manifest.json").exists()
        assert (p / "result_summary.json").exists()
        assert archive.load_manifest(p)["files"][0]["upload"]["entry"]["download_url"]
    assert (records[0] / "unregistered.log").exists()
    assert all((p / "capture.bag").exists() for p in records[-archive.RETAIN:])


def test_completion_time_takes_precedence_over_creation_time(tmp_path, config):
    records = [completed(tmp_path, config, index=i) for i in range(archive.RETAIN + 1)]
    first = archive.load_manifest(records[0])
    first["completed_at"] = "2026-09-22T01:00:00+00:00"
    save_manifest(records[0], first)
    protected = set(archive.preview(tmp_path, config=config)["protected"])
    assert records[0].name in protected
    assert records[1].name not in protected


@pytest.mark.parametrize("missing_proof", ["upload_verified", "restore_verified", "retention_verified"])
def test_deletion_needs_all_validation_evidence(tmp_path, config, missing_proof):
    records = [completed(tmp_path, config, index=i) for i in range(archive.RETAIN + 1)]
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    validated = deletion_config(config)
    validated["deletion_validation"].pop(missing_proof)
    archive.cleanup(tmp_path, config=validated, store_factory=store.factory, dry_run=False)
    assert all((p / "capture.bag").exists() for p in records)


def test_old_incomplete_and_upload_pending_records_survive(tmp_path, config):
    incomplete = archive.begin_record(tmp_path, config=config)
    (incomplete / "capture.bag").write_bytes(b"unclosed")
    state = archive.load_manifest(incomplete)
    state.update(state="incomplete", result="interrupted", completed_at="2026-01-01T00:00:00+00:00")
    save_manifest(incomplete, state)
    pending = completed(tmp_path, config, index=0)
    newer = [completed(tmp_path, config, index=i) for i in range(1, archive.RETAIN + 1)]
    archive.cleanup(tmp_path, config=deletion_config(config), store_factory=MemoryStore().factory, dry_run=False)
    assert all((p / "capture.bag").exists() for p in [incomplete, pending] + newer)


@pytest.mark.parametrize("mutation", ["rewrite", "symlink", "hardlink", "manifest_escape"])
def test_changes_after_upload_block_deletion(tmp_path, config, mutation):
    records = [completed(tmp_path / "records", config, index=i) for i in range(archive.RETAIN + 1)]
    store = MemoryStore()
    archive.process_queue(tmp_path / "records", config=config, store_factory=store.factory)
    oldest = records[0]
    original = oldest / "capture.bag"
    outside = tmp_path / "untouchable.bag"
    outside.write_bytes(original.read_bytes())
    if mutation == "rewrite":
        original.write_bytes(b"x" * original.stat().st_size)
    elif mutation == "symlink":
        original.unlink()
        original.symlink_to(outside)
    elif mutation == "hardlink":
        import os
        original.unlink()
        os.link(outside, original)
    elif mutation == "manifest_escape":
        state = archive.load_manifest(oldest)
        state["files"][0]["path"] = "../../untouchable.bag"
        save_manifest(oldest, state)
    report = archive.cleanup(tmp_path / "records", config=deletion_config(config),
                             store_factory=store.factory, dry_run=False)
    assert not report["deleted"]
    assert original.exists()
    assert outside.read_bytes() == b"raw rosbag bytes"


def test_unrelated_scripts_source_calibration_and_git_survive_cleanup(tmp_path, config):
    records = [completed(tmp_path, config, index=i) for i in range(archive.RETAIN + 1)]
    oldest = records[0]
    keep = ["start.sh", "calibration.yaml", "src/controller.cpp", ".git/config", "auxiliary.json"]
    for relative in keep:
        p = oldest / relative
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("must remain")
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    archive.cleanup(tmp_path, config=deletion_config(config), store_factory=store.factory, dry_run=False)
    assert not (oldest / "capture.bag").exists()
    assert all((oldest / relative).read_text() == "must remain" for relative in keep)


def test_local_data_is_kept_if_remote_corrupts_after_upload(tmp_path, config):
    records = [completed(tmp_path, config, index=i) for i in range(archive.RETAIN + 1)]
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    entry = archive.load_manifest(records[0])["files"][0]["upload"]["entry"]
    store.remote[entry["key"]] = b"corrupted remote"
    archive.cleanup(tmp_path, config=deletion_config(config), store_factory=store.factory, dry_run=False)
    assert all((p / "capture.bag").exists() for p in records)


def test_success_requires_actual_or_manual_result_evidence(tmp_path, config):
    record = archive.begin_record(tmp_path, config=config)
    (record / "capture.bag").write_bytes(b"closed")
    with pytest.raises(archive.ArchiveError):
        archive.finalize_record(record, result="success", closed_files=["capture.bag"])
    assert archive.load_manifest(record)["result"] == "unknown"


def test_restore_checks_original_size_and_hash(tmp_path, config):
    record = completed(tmp_path / "records", config)
    store = MemoryStore()
    archive.process_queue(tmp_path / "records", config=config, store_factory=store.factory)
    destination = tmp_path / "restored"
    report = archive.restore_manifest(record / "manifest.json", destination,
                                      config=config, store_factory=store.factory)
    assert report["verified"]
    assert (destination / "capture.bag").read_bytes() == (record / "capture.bag").read_bytes()
    assert (destination / "manifest.json").exists()
    entry = archive.load_manifest(record)["files"][0]["upload"]["entry"]
    store.remote[entry["key"]] = b"bad restoration"
    with pytest.raises(archive.ArchiveError):
        archive.restore_manifest(record / "manifest.json", tmp_path / "bad_restore",
                                 config=config, store_factory=store.factory)


def test_abandoned_record_is_interrupted_without_claiming_clean_close(tmp_path, config, monkeypatch):
    record = archive.begin_record(tmp_path, config=config)
    (record / "capture.bag").write_bytes(b"possibly unclosed")
    monkeypatch.setattr(archive, "process_alive", lambda _: False)
    archive.recover_abandoned(tmp_path)
    manifest = archive.load_manifest(record)
    assert manifest["result"] == "interrupted"
    assert manifest["state"] == "incomplete"
    assert not manifest["recorder_closed"]
    assert (record / "capture.bag").exists()
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    assert not store.uploads


def test_low_space_attempts_archive_before_blocking(tmp_path, config, monkeypatch):
    usage = namedtuple("usage", "total used free")
    calls = []
    monkeypatch.setattr(archive.shutil, "disk_usage", lambda _: usage(1000, 999, 1))
    monkeypatch.setattr(archive, "process_queue", lambda *a, **k: calls.append("archive"))
    monkeypatch.setattr(archive, "cleanup", lambda *a, **k: calls.append("cleanup"))
    config.update(deletion_config(config), min_free_bytes=100)
    with pytest.raises(archive.InsufficientSpace):
        archive.preflight_space(tmp_path, config=config)
    assert calls == ["cleanup", "archive", "cleanup"]


def test_same_size_mutation_during_remote_verification_blocks_unlink(tmp_path, config):
    records = [completed(tmp_path, config, index=i) for i in range(archive.RETAIN + 1)]
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    target = records[0] / "capture.bag"
    entry = archive.load_manifest(records[0])["files"][0]["upload"]["entry"]
    original_verify = store.verify_file

    def mutate_while_remote_verifies(remote_entry):
        if remote_entry["key"] == entry["key"]:
            target.write_bytes(b"x" * target.stat().st_size)
        return original_verify(remote_entry)

    store.verify_file = mutate_while_remote_verifies
    report = archive.cleanup(tmp_path, config=deletion_config(config), store_factory=store.factory, dry_run=False)
    assert not report["deleted"]
    assert target.exists()


def test_partial_upload_resumes_without_reuploading_verified_file(tmp_path, config):
    record = archive.begin_record(tmp_path, config=config)
    (record / "capture.bag").write_bytes(b"recording")
    (record / "task.log").write_text("necessary task log")
    archive.finalize_record(record, closed_files=["capture.bag", "task.log"])
    store = MemoryStore()
    original_upload = store.upload_file
    failed_once = []

    def fail_second_file_once(record_id, source, relative_path):
        if relative_path == "task.log" and not failed_once:
            failed_once.append(True)
            raise OSError("worker interrupted between files")
        return original_upload(record_id, source, relative_path)

    store.upload_file = fail_second_file_once
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    manifest = archive.load_manifest(record)
    assert manifest["files"][0]["upload"]["status"] == "verified"
    assert manifest["upload"]["status"] != "verified"
    release_retry_delay(record)
    archive.process_queue(tmp_path, config=dict(config), store_factory=store.factory)
    assert archive.load_manifest(record)["upload"]["status"] == "verified"
    assert store.uploads.count((record.name, "capture.bag")) == 1
    assert store.uploads.count((record.name, "task.log")) == 1


def test_local_mutation_before_upload_never_reaches_remote(tmp_path, config):
    record = completed(tmp_path, config)
    (record / "capture.bag").write_bytes(b"changed closed recording")
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    assert not store.uploads
    manifest = archive.load_manifest(record)
    assert manifest["upload"]["status"] != "verified"
    assert manifest["upload"]["error"]
    assert (record / "capture.bag").exists()


def test_manual_result_correction_after_local_deletion_republishes_summary(tmp_path, config):
    records = [completed(tmp_path, config, index=i) for i in range(archive.RETAIN + 1)]
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    archive.cleanup(tmp_path, config=deletion_config(config), store_factory=store.factory, dry_run=False)
    oldest = records[0]
    assert not (oldest / "capture.bag").exists()
    archive.mark_result(oldest, "failure", "operator verified empty gripper",
                        {"source": "manual_confirmation", "at": archive.utc_now()})
    previous_uploads = list(store.uploads)
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    manifest = archive.load_manifest(oldest)
    assert manifest["result"] == "failure"
    assert manifest["upload"]["status"] == "verified"
    assert store.uploads[len(previous_uploads):] == [(oldest.name, "archive_manifest.json")]
    remote_entry = manifest["remote_manifest"]["entry"]
    remote_manifest = json.loads(store.remote[remote_entry["key"]])
    assert remote_manifest["result"] == "failure"
    assert remote_manifest["failure_reason"] == "operator verified empty gripper"


@pytest.mark.parametrize("launcher", [False, True])
def test_confirmed_closed_record_recovers_after_owner_crash(tmp_path, config, monkeypatch, launcher):
    monkeypatch.syspath_prepend(str(MODULE_PATH.parent))
    metadata = {"launcher": "run_recorded_grasp.py"} if launcher else {}
    record = archive.begin_record(tmp_path, metadata=metadata, config=config)
    (record / "capture.bag").write_bytes(b"bag close completed")
    (record / "recorder_ready.json").write_text("{}")
    evidence = {"source": "/grasp/state", "message": {"state": "FAILED", "success": False}}
    outcome = {
        "bag_closed": True, "finished_at": archive.utc_now(),
        "closed_files": ["capture.bag"],
        "task_result": {"result": "failure", "reason": "actual grasp failed", "evidence": evidence},
    }
    (record / "recorder_outcome.json").write_text(json.dumps(outcome))
    if launcher:
        (record / "runner.log").write_text("GRASP_RESULT success=False message=actual grasp failed")
        (record / "recorder.log").write_text("recorder normally closed")
        (record / "command_result.json").write_text(json.dumps({
            "result": "failure", "failure_reason": "actual grasp failed",
            "evidence": evidence, "closed_logs": ["runner.log", "recorder.log"],
        }))
    monkeypatch.setattr(archive, "process_alive", lambda _: False)
    report = archive.recover_abandoned(tmp_path)
    manifest = archive.load_manifest(record)
    assert report == [{"record_id": record.name, "recovered_closed": True}]
    assert manifest["state"] == "complete"
    assert manifest["result"] == "failure"
    assert manifest["failure_reason"] == "actual grasp failed"
    assert manifest["recorder_closed"] is True
    assert manifest["upload"]["status"] == "pending"
    paths = {entry["path"] for entry in manifest["files"]}
    assert "capture.bag" in paths
    assert ("runner.log" in paths) is launcher
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    assert archive.load_manifest(record)["upload"]["status"] == "verified"


def test_closed_bag_without_all_wrapper_log_close_receipts_stays_incomplete(tmp_path, config, monkeypatch):
    record = archive.begin_record(tmp_path, metadata={"launcher": "run_recorded_grasp.py"}, config=config)
    (record / "capture.bag").write_bytes(b"bag is closed")
    (record / "runner.log").write_text("runner may still be writing")
    (record / "recorder.log").write_text("recorder may still be writing")
    (record / "recorder_ready.json").write_text("{}")
    (record / "recorder_outcome.json").write_text(json.dumps({
        "bag_closed": True, "finished_at": archive.utc_now(), "closed_files": ["capture.bag"],
    }))
    (record / "command_result.json").write_text(json.dumps({
        "result": "unknown", "evidence": {}, "closed_logs": ["recorder.log"],
    }))
    monkeypatch.setattr(archive, "process_alive", lambda _: False)
    archive.recover_abandoned(tmp_path)
    manifest = archive.load_manifest(record)
    assert manifest["state"] == "incomplete"
    assert manifest["result"] == "interrupted"
    assert manifest["upload"]["status"] == "blocked_unclosed"
    assert not manifest["recorder_closed"]
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    assert not store.uploads
    assert (record / "runner.log").read_text() == "runner may still be writing"
    assert (record / "capture.bag").read_bytes() == b"bag is closed"


@pytest.mark.parametrize("failure_point", ["manifest", "data"])
def test_validation_readback_failure_cannot_enable_automatic_deletion(tmp_path, config, monkeypatch, failure_point):
    record = completed(tmp_path, config)
    store = MemoryStore()
    archive.process_queue(tmp_path, config=config, store_factory=store.factory)
    archive.atomic_json(tmp_path / "archive_config.json", config)
    proof = deletion_config(config)["deletion_validation"]
    proof["validation_record_manifest"] = str(record / "manifest.json")
    proof_path = tmp_path / "validation.json"
    archive.atomic_json(proof_path, proof)
    verified_calls = []

    def reject_invalid_remote(entry, readback=False):
        assert readback is True
        verified_calls.append(entry["key"])
        return not (failure_point == "manifest" or entry["key"].endswith("/capture.bag"))

    store.verify_file = reject_invalid_remote
    monkeypatch.setattr(archive, "make_store", lambda cfg, manifest: store)
    monkeypatch.setattr(archive.sys, "argv", [
        "grasp_archive.py", "--root", str(tmp_path), "enable-deletion",
        "--validation-report", str(proof_path),
    ])
    with pytest.raises(archive.ArchiveError):
        archive.main()
    persisted = json.loads((tmp_path / "archive_config.json").read_text())
    assert persisted["delete_enabled"] is False
    assert persisted["deletion_validation"] is None
    assert verified_calls
    assert (record / "capture.bag").exists()


def test_low_space_reclaims_verified_older_data_before_long_upload(tmp_path, config, monkeypatch):
    free = [1]
    events = []
    config.update(min_free_bytes=100, delete_enabled=True)
    monkeypatch.setattr(archive.shutil, 'disk_usage',
                        lambda _: namedtuple('usage', 'free')(free[0]))
    def cleanup(*args, **kwargs):
        assert kwargs['dry_run'] is False
        events.append('cleanup')
        free[0] = 200
        return {'deleted': ['previously_verified_old_record'], 'failed': []}
    monkeypatch.setattr(archive, 'cleanup', cleanup)
    monkeypatch.setattr(archive, 'process_queue', lambda *args: events.append('upload'))
    assert archive.preflight_space(tmp_path, config)['recording_allowed'] is True
    assert events == ['cleanup']


def test_worker_reclaims_already_verified_records_before_upload(tmp_path, config, monkeypatch):
    events = []
    archive.atomic_json(tmp_path / 'archive_config.json', config)
    monkeypatch.setattr(archive, 'recover_abandoned', lambda *args: [])
    def cleanup(*args, **kwargs):
        events.append('cleanup')
        return {'dry_run': kwargs['dry_run']}
    def upload(*args, **kwargs):
        assert kwargs == {'max_records': 1}
        events.append('upload')
        return {'uploaded': []}
    monkeypatch.setattr(archive, 'cleanup', cleanup)
    monkeypatch.setattr(archive, 'process_queue', upload)
    monkeypatch.setattr(archive.sys, 'argv', [
        'grasp_archive.py', '--root', str(tmp_path), 'worker', '--once'])
    archive.main()
    assert events == ['cleanup', 'upload', 'cleanup']

def test_bounded_worker_uploads_by_completion_then_reclaims_before_next(tmp_path, config):
    store = MemoryStore()
    # Create in a different order, with mixed results: completion is authority.
    records = [completed(tmp_path, config, index=i,
                         result='success' if i % 2 else 'failure')
               for i in (5, 2, 6, 0, 4, 1, 3, 10, 8, 11, 7, 9)]
    by_index = {archive.load_manifest(p)['metadata']['test_index']: p for p in records}
    first = archive.process_queue(tmp_path, config, store.factory, max_records=1)
    assert first['uploaded'] == [by_index[0].name]
    cleaned = archive.cleanup(tmp_path, deletion_config(config), store.factory, dry_run=False)
    assert cleaned['deleted'] == [by_index[0].name]
    assert not (by_index[0] / 'capture.bag').exists()
    assert all((p / 'capture.bag').exists() for i, p in by_index.items() if i != 0)
    second = archive.process_queue(tmp_path, config, store.factory, max_records=1)
    assert second['uploaded'] == [by_index[1].name]


def test_bounded_worker_skips_retry_backoff_without_starving_next(tmp_path, config):
    store = MemoryStore()
    old = completed(tmp_path, config, index=0)
    new = completed(tmp_path, config, index=1)
    store.fail_uploads = 1
    first = archive.process_queue(tmp_path, config, store.factory, max_records=1)
    assert first['failed'][0]['record_id'] == old.name
    second = archive.process_queue(tmp_path, config, store.factory, max_records=1)
    assert second['uploaded'] == [new.name]
    assert archive.load_manifest(old)['upload']['status'] == 'retry'
    assert (old / 'capture.bag').exists()


@pytest.mark.parametrize('limit', [0, -1, True, 1.5])
def test_invalid_queue_batch_limit_fails_before_upload(tmp_path, config, limit):
    with pytest.raises(ValueError):
        archive.process_queue(tmp_path, config, max_records=limit)


def test_old_ten_record_validation_cannot_authorize_five_record_deletion(tmp_path, config):
    previous = deletion_config(config)
    previous['deletion_validation']['retain_count'] = 10
    assert not archive.deletion_authorized(tmp_path, previous)


def test_ten_record_inventory_is_reduced_to_latest_five_after_verified_upload(tmp_path, config):
    first = [completed(tmp_path, config, index=i) for i in range(5)]
    store = MemoryStore()
    archive.process_queue(tmp_path, config, store.factory)
    newer = [completed(tmp_path, config, index=i, result='failure') for i in range(5, 10)]
    archive.process_queue(tmp_path, config, store.factory)
    report = archive.cleanup(tmp_path, deletion_config(config), store.factory, dry_run=False)
    assert set(report['deleted']) == {p.name for p in first}
    assert len(archive.preview(tmp_path, config)['protected']) == 5
    assert all(not (p / 'capture.bag').exists() for p in first)
    assert all((p / 'capture.bag').exists() for p in newer)
    assert all(archive.load_manifest(p)['upload']['status'] == 'verified' for p in first + newer)


@pytest.mark.parametrize('old_count', [0, 4, 6, 10])
def test_config_requires_exactly_five_total_records(tmp_path, old_count):
    path = tmp_path / 'archive_config.json'
    archive.atomic_json(path, {'retain_count': old_count})
    with pytest.raises(archive.ArchiveError, match='exactly 5 total records'):
        archive.load_config(path)


@pytest.mark.parametrize('root_order', [(0, 1), (1, 0)])
def test_two_roots_retain_only_global_latest_five(tmp_path, config, root_order):
    roots = [tmp_path / 'active', tmp_path / 'legacy']
    cfg = dict(config, retention_roots=[str(p) for p in roots])
    mixed = ['success', 'failure', 'unknown', 'interrupted']
    attempts = [completed(roots[i % 2], cfg, index=i, result=mixed[i % 4])
                for i in range(12)]
    store = MemoryStore()
    for root in roots:
        archive.process_queue(root, cfg, store.factory)
    expected = {p.name for p in attempts[-5:]}
    for root in roots:
        preview = archive.preview(root, cfg)
        assert set(preview['protected']) == expected
        assert set(preview['retention_roots']) == {str(p) for p in roots}
        assert {item['record_id'] for item in preview['records']} == {
            p.name for p in attempts if p.parent == root}
    deleted = []
    for root_index in root_order:
        report = archive.cleanup(roots[root_index], deletion_config(cfg),
                                 store.factory, dry_run=False)
        assert not report['failed']
        deleted.extend(report['deleted'])
    assert set(deleted) == {p.name for p in attempts[:-5]}
    assert {p.name for p in attempts if (p / 'capture.bag').exists()} == expected
    assert all(archive.load_manifest(p)['upload']['status'] == 'verified' for p in attempts)


def test_retention_roots_default_remains_current_root_only(tmp_path, config):
    active, unrelated = tmp_path / 'active', tmp_path / 'unrelated'
    attempts = [completed(active, config, index=i) for i in range(7)]
    for i in range(7, 14):
        completed(unrelated, config, index=i)
    assert archive.preview(active, config)['retention_roots'] == [str(active)]
    assert set(archive.preview(active, config)['protected']) == {p.name for p in attempts[-5:]}


def test_retention_roots_always_include_current_root_and_deduplicate_aliases(tmp_path, config):
    active, legacy = tmp_path / 'active', tmp_path / 'legacy'
    attempts = [completed(active if i % 2 else legacy, config, index=i) for i in range(7)]
    cfg = dict(config, retention_roots=[str(legacy), str(legacy / '..' / 'legacy')])
    preview = archive.preview(active, cfg)
    assert preview['retention_roots'] == [str(active), str(legacy)]
    assert set(preview['protected']) == {p.name for p in attempts[-5:]}


@pytest.mark.parametrize('operation', ['preview', 'cleanup'])
def test_duplicate_record_id_across_roots_blocks_retention(tmp_path, config, operation):
    import shutil
    active, legacy = tmp_path / 'active', tmp_path / 'legacy'
    attempts = [completed(active, config, index=i) for i in range(7)]
    store = MemoryStore()
    archive.process_queue(active, config, store.factory)
    legacy.mkdir()
    shutil.copytree(attempts[0], legacy / attempts[0].name)
    cfg = dict(deletion_config(config), retention_roots=[str(active), str(legacy)])
    with pytest.raises(archive.ArchiveError, match='duplicate record_id across retention roots'):
        if operation == 'preview':
            archive.preview(active, cfg)
        else:
            archive.cleanup(active, cfg, store.factory, dry_run=False)
    assert all((p / 'capture.bag').exists() for p in attempts)
    assert (legacy / attempts[0].name / 'capture.bag').exists()


def test_incomplete_records_across_roots_do_not_consume_retention_slots(tmp_path, config):
    roots = [tmp_path / 'active', tmp_path / 'legacy']
    cfg = dict(config, retention_roots=[str(p) for p in roots])
    attempts = [completed(roots[i % 2], cfg, index=i) for i in range(7)]
    incomplete = []
    for root in roots:
        record = archive.begin_record(root, config=cfg)
        (record / 'capture.bag').write_bytes(b'unclosed recording')
        manifest = archive.load_manifest(record)
        manifest.update(state='incomplete', completed_at='2026-12-01T00:00:00+00:00')
        save_manifest(record, manifest)
        incomplete.append(record)
    store = MemoryStore()
    for root in roots:
        archive.process_queue(root, cfg, store.factory)
        archive.cleanup(root, deletion_config(cfg), store.factory, dry_run=False)
    assert all((p / 'capture.bag').exists() for p in incomplete)
    assert {p.name for p in attempts if (p / 'capture.bag').exists()} == {
        p.name for p in attempts[-5:]}


@pytest.mark.parametrize('bad_roots', [None, '/tmp/root', ['relative'], [''], [1], {}])
def test_retention_roots_reject_ambiguous_configuration(tmp_path, config, bad_roots):
    cfg = dict(config, retention_roots=bad_roots)
    archive.atomic_json(tmp_path / 'archive_config.json', cfg)
    with pytest.raises(archive.ArchiveError, match='list of absolute paths'):
        archive.load_config(tmp_path / 'archive_config.json')
    with pytest.raises(archive.ArchiveError, match='list of absolute paths'):
        archive.preview(tmp_path / 'other', cfg)


@pytest.fixture
def external_storage(tmp_path, monkeypatch):
    project = tmp_path / 'workspace' / '.worktrees' / 'active'
    mount = tmp_path / 'external'
    storage = dict(mount_point=str(mount), archive_root=str(mount / 'grasp_records'),
                   ros_log_root=str(mount / 'ros_log'))
    path = project / '.runtime' / 'experiment_storage.json'
    path.parent.mkdir(parents=True)
    archive.atomic_json(path, storage)
    monkeypatch.setattr(archive, 'PROJECT_ROOT', project)
    monkeypatch.setattr(archive, 'STORAGE_CONFIG_PATH', path)
    monkeypatch.delenv('GRASP_RECORD_ROOT', raising=False)
    monkeypatch.setattr(archive.os.path, 'ismount', lambda value: Path(value) == mount)
    return project, storage


def test_storage_config_maps_default_and_both_internal_aliases(external_storage):
    project, storage = external_storage
    assert archive.root_path() == Path(storage['archive_root'])
    assert archive.root_path(project / '.grasp_records') == Path(storage['archive_root'])
    old_ops = project.parent.parent / '.ros_log' / 'unknown_grasp_20260923'
    assert archive.root_path(old_ops) == Path(storage['ros_log_root']) / old_ops.name
    cfg = dict(retention_roots=[str(project / '.grasp_records'), str(old_ops)])
    assert archive.retention_roots(old_ops, cfg) == [
        Path(storage['ros_log_root']) / old_ops.name, Path(storage['archive_root'])]


def test_storage_environment_and_explicit_other_roots_take_priority(external_storage, tmp_path, monkeypatch):
    project, storage = external_storage
    custom = tmp_path / 'custom'
    explicit = tmp_path / 'explicit'
    monkeypatch.setenv('GRASP_RECORD_ROOT', str(custom))
    monkeypatch.setattr(archive.os.path, 'ismount', lambda _: False)
    assert archive.root_path() == custom
    assert archive.root_path(explicit) == explicit


@pytest.mark.parametrize('selection', ['default', 'canonical_alias', 'ops_alias', 'external_canonical', 'external_ops'])
def test_missing_external_mount_never_falls_back_or_creates_data(external_storage, monkeypatch, selection):
    project, storage = external_storage
    selected = {
        'default': None,
        'canonical_alias': project / '.grasp_records',
        'ops_alias': project.parent.parent / '.ros_log' / 'unknown_grasp_20260923',
        'external_canonical': Path(storage['archive_root']),
        'external_ops': Path(storage['ros_log_root']) / 'unknown_grasp_20260923',
    }[selection]
    monkeypatch.setattr(archive.os.path, 'ismount', lambda _: False)
    with pytest.raises(archive.ArchiveError, match='EXPERIMENT_STORAGE_UNAVAILABLE'):
        archive.root_path(selected)
    assert not (project / '.grasp_records').exists()
    assert not Path(storage['archive_root']).exists()


def test_no_storage_config_preserves_original_default(tmp_path, monkeypatch):
    monkeypatch.setattr(archive, 'PROJECT_ROOT', tmp_path)
    monkeypatch.setattr(archive, 'STORAGE_CONFIG_PATH', tmp_path / 'missing.json')
    monkeypatch.delenv('GRASP_RECORD_ROOT', raising=False)
    assert archive.root_path() == tmp_path / '.grasp_records'



def test_persisted_upload_disabled_never_constructs_store(tmp_path, config):
    record = completed(tmp_path, config)
    before = (record / 'manifest.json').read_bytes()
    disabled = dict(config, upload_enabled=False)
    (tmp_path / 'archive_config.json').write_text(json.dumps(disabled))
    def forbidden_store(*args):
        pytest.fail('disabled upload constructed a network store')
    report = archive.process_queue(tmp_path, store_factory=forbidden_store)
    assert report['upload_disabled'] is True
    assert report['uploaded'] == report['failed'] == []
    assert (record / 'manifest.json').read_bytes() == before
    assert (record / 'capture.bag').read_bytes() == b'raw rosbag bytes'
    assert archive.preview(tmp_path)['upload_enabled'] is False


@pytest.mark.parametrize('disabled_value', [False, None, 'false', 0])
def test_upload_override_without_explicit_true_fails_closed(tmp_path, config, disabled_value):
    record = completed(tmp_path, config)
    def forbidden_store(*args):
        pytest.fail('disabled upload constructed a network store')
    report = archive.process_queue(tmp_path, dict(config, upload_enabled=disabled_value), forbidden_store)
    assert report['upload_disabled'] is True
    assert archive.load_manifest(record)['upload']['status'] == 'pending'



def test_upload_missing_permission_defaults_to_disabled(tmp_path, config):
    record = completed(tmp_path, config)
    old = dict(config); old.pop('upload_enabled')
    (tmp_path / 'archive_config.json').write_text(json.dumps(old))
    assert archive.load_config(tmp_path / 'archive_config.json')['upload_enabled'] is False
    def forbidden_store(*args):
        pytest.fail('old config without explicit permission uploaded')
    assert archive.process_queue(tmp_path, store_factory=forbidden_store)['upload_disabled'] is True
    assert (record / 'capture.bag').exists()



def test_upload_explicit_true_preserves_existing_upload_behavior(tmp_path, config):
    record = completed(tmp_path, config)
    (tmp_path / 'archive_config.json').write_text(json.dumps(config))
    store = MemoryStore()
    report = archive.process_queue(tmp_path, store_factory=store.factory)
    assert report['uploaded'] == [record.name]
    assert store.prepared == [True]
    assert {relative for _, relative in store.uploads} == {'capture.bag', 'archive_manifest.json'}


@pytest.mark.parametrize('invalid', ['false', None, 0, 1])
def test_upload_config_requires_boolean(tmp_path, config, invalid):
    path = tmp_path / 'archive_config.json'
    path.write_text(json.dumps(dict(config, upload_enabled=invalid)))
    with pytest.raises(archive.ArchiveError, match='upload_enabled'):
        archive.load_config(path)



def test_disabled_upload_preserves_restore_and_query(tmp_path, config):
    root = tmp_path / 'records'
    record = completed(root, config)
    store = MemoryStore()
    archive.process_queue(root, config, store.factory)
    uploads_before = list(store.uploads)
    disabled = dict(config, upload_enabled=False, delete_enabled=False)
    destination = tmp_path / 'restored'
    result = archive.restore_manifest(record / 'archive_manifest.json', destination,
                                      disabled, store.factory)
    assert result['verified'] is True
    assert (destination / 'capture.bag').read_bytes() == b'raw rosbag bytes'
    assert store.uploads == uploads_before
    assert archive.preview(root, disabled)['records'][0]['upload'] == 'verified'
    assert archive.cleanup(root, disabled, store.factory, dry_run=False)['deleted'] == []
    assert (record / 'capture.bag').exists()



def test_disabled_worker_once_does_not_create_store(tmp_path, config, monkeypatch, capsys):
    record = completed(tmp_path, config)
    (tmp_path / 'archive_config.json').write_text(json.dumps(dict(config, upload_enabled=False)))
    monkeypatch.setattr(archive, 'make_store', lambda *args: pytest.fail('disabled worker accessed store'))
    monkeypatch.setattr(archive.sys, 'argv', ['grasp_archive.py', '--root', str(tmp_path), 'worker', '--once'])
    archive.main()
    report = json.loads(capsys.readouterr().out)
    assert report['upload_disabled'] is True
    assert report['uploaded'] == []
    assert (record / 'capture.bag').exists()



def test_low_disk_does_not_bypass_disabled_upload(tmp_path, config, monkeypatch):
    record = completed(tmp_path, config)
    usage = namedtuple('usage', 'total used free')(100, 99, 1)
    monkeypatch.setattr(archive.shutil, 'disk_usage', lambda root: usage)
    monkeypatch.setattr(archive, 'make_store', lambda *args: pytest.fail('low disk bypassed disabled upload'))
    with pytest.raises(archive.InsufficientSpace):
        archive.preflight_space(tmp_path, dict(config, upload_enabled=False, min_free_bytes=2))
    assert archive.load_manifest(record)['upload']['status'] == 'pending'
    assert (record / 'capture.bag').exists()
