"""One real continuation entry, using local fixtures and no ROS initialization."""

import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

TOOLS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS))
import continuous_grasp_authorization as authority


@pytest.fixture
def scene(tmp_path):
    owner, session, record = [tmp_path / name for name in ('owner', 'session', 'record')]
    for directory in (owner, session, record):
        directory.mkdir()

    def save(path, value):
        path.write_text(json.dumps(value))
        return value

    def reference(path):
        return dict(path=str(path), sha256=authority.digest(path))

    hands = owner / 'hands.json'
    save(hands, dict(user_message='当前已对准且手已移开，允许执行一轮', message_id='actual-prior-reply'))
    auth = owner / 'authorization.json'
    save(auth, dict(version=1, authorization_id='continuation-fixture',
        scope='observe_return_until_physical_grasp_success', issued_ns=1_000_000_000,
        confirmation_source=dict(source='actual_user_message', text=authority.REQUEST_TEXT,
            platform_message_id=None, local_record_id='local-message', received_at='2026-10-22',
            received_at_kind='locally_recorded_receipt_time'), original_hands_clear_receipt=reference(hands)))
    control = owner / 'control.json'
    save(control, dict(version=1, authorization_id='continuation-fixture', state='active', revision=1))
    preflight = session / 'preflight.json'
    save(preflight, dict(receipt_ns=2_000_000_000, image_stamp_ns=1_980_000_000,
        epoch_ns=100, topics={'/alicia_d/accepted_joint_states':dict(stamp_ns=1_990_000_000,
            frame_id='sdk_measured', names=['Joint%d' % i for i in range(1,7)], positions=[0.] * 6)}))
    ready = session / 'readiness.json'
    save(ready, dict(passed=True, deployment_processes_unchanged=True,
        source_matches_operational_release=True, motion_commands_sent=False, temperature_queried=False,
        sample_count=20, epoch_ns=100, free_bytes=1000, min_free_bytes=100, recorded_ns=2_010_000_000))
    image = session / 'scene.png';image.write_bytes(b'current image bytes')
    observation = session / 'operator_observation.json'
    save(observation, dict(version=1, observation_id='current-image', new_user_confirmation_claimed=False,
        same_target_observed=True, target_visible=True,
        hands_clear_basis=dict(prior_explicit_user_reply_id='actual-prior-reply',
            current_image_review_no_conflicting_hand_in_motion_area=True),
        image=dict(reference(image), source_stamp_ns=1_980_000_000),
        preflight=reference(preflight), readiness=reference(ready),
        captured_ns=2_000_000_000, assessed_ns=2_020_000_000, valid_until_ns=3_000_000_000))

    def attempt():
        return authority.ContinuationAttempt(auth, record, session,
            authority.REQUEST_TEXT, 2_030_000_000, storage=tmp_path)

    return SimpleNamespace(root=tmp_path, owner=owner, session=session, record=record,
        hands=hands, auth=auth, control=control, preflight=preflight, ready=ready,
        image=image, observation=observation, attempt=attempt, save=save)


def test_historical_bytes_read_once_and_current_inputs_rechecked(scene, monkeypatch):
    original = Path.read_bytes
    reads = []

    def counted(path):
        reads.append(path.resolve())
        return original(path)

    monkeypatch.setattr(Path, 'read_bytes', counted)
    with scene.attempt() as attempt:
        for now in (2_040_000_000, 2_050_000_000):
            assert attempt.check_active(now)['state'] == 'active'
        assert attempt.authorization_fields()['new_user_confirmation_claimed'] is False
        assert reads.count(scene.hands) == 1
        for current in (scene.auth, scene.observation, scene.preflight, scene.ready, scene.image):
            assert reads.count(current) >= 3
        assert attempt._history_transaction().stats['read_attempts'] == 1


def test_historical_mutation_is_snapshot_only_and_next_transaction_rejects(scene):
    attempt = scene.attempt()
    original = dict(attempt.hands)
    scene.hands.write_text('{"user_message": "changed historical file"}')
    assert attempt.check_active(2_040_000_000)['state'] == 'active'
    assert attempt.hands == original
    attempt.close()
    with pytest.raises(ValueError, match='HASH_MISMATCH'):
        attempt.check_active(2_050_000_000)
    with pytest.raises(ValueError, match='HASH_MISMATCH'):
        scene.attempt()


@pytest.mark.parametrize('which', ['auth', 'observation', 'preflight', 'ready', 'image', 'source', 'config'])
def test_current_files_never_use_history_cache_even_same_mtime(scene, which):
    attempt = scene.attempt()
    if which in ('source', 'config'):
        path = scene.root / (which + '.txt')
        path.write_bytes(b'original bytes')
        attempt.pinned[path] = authority.digest(path)
        attempt.check_active(2_035_000_000)
    else:
        path = getattr(scene, which)
    before = path.stat()
    raw = path.read_bytes()
    path.write_bytes(bytes([raw[0] ^ 1]) + raw[1:])
    os.utime(path, ns=(before.st_atime_ns, before.st_mtime_ns))
    with pytest.raises(ValueError, match='PINNED_EVIDENCE_CHANGED'):
        attempt.check_active(2_040_000_000)
    attempt.close()


@pytest.mark.parametrize('change', ['paused', 'revoked', 'wrong_id'])
def test_live_control_read_on_every_boundary(scene, change):
    attempt = scene.attempt()
    control = authority.read(scene.control)
    control['authorization_id' if change == 'wrong_id' else 'state'] = change
    scene.save(scene.control, control)
    with pytest.raises(ValueError, match='PAUSED_REVOKED_OR_COMPLETED'):
        attempt.check_active(2_040_000_000)
    attempt.close()


def test_historical_cache_cannot_extend_current_observation(scene):
    attempt = scene.attempt()
    with pytest.raises(ValueError, match='OPERATOR_OBSERVATION_EXPIRED'):
        attempt.check_active(3_000_000_000)
    attempt.close()


def test_current_aliases_read_once_and_conflicting_pin_rejected(scene, monkeypatch):
    attempt = scene.attempt()
    attempt.pinned[str(scene.auth)] = attempt.auth_sha
    original = Path.read_bytes
    reads = []

    def counted(path):
        reads.append(path.resolve())
        return original(path)

    monkeypatch.setattr(Path, 'read_bytes', counted)
    attempt.check_active(2_040_000_000)
    assert reads.count(scene.auth) == 1
    reads.clear()
    attempt.pinned[str(scene.auth)] = '0' * 64
    with pytest.raises(ValueError, match='EVIDENCE_PATH_CONFLICT'):
        attempt.check_active(2_050_000_000)
    assert not reads
    attempt.close()


def test_current_pin_cannot_alias_cached_history(scene):
    attempt = scene.attempt()
    attempt.pinned[str(scene.hands)] = authority.digest(scene.hands)
    with pytest.raises(ValueError, match='CURRENT_INPUT_CANNOT_BE_CACHED'):
        attempt.check_active(2_040_000_000)
    attempt.close()
