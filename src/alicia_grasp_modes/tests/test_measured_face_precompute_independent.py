"""Independent PRE-only cache equivalence and lifetime checks, no ROS actions."""
from types import SimpleNamespace
from dataclasses import FrozenInstanceError
import json

import numpy as np
import pytest

from alicia_grasp_modes import observation_measured_face as face
from alicia_grasp_modes import arrival_dense_identity as dense
from alicia_grasp_modes import arrival_train_only_proposal as train
from test_measured_face_independent import flat_scene, freeze


class OwnerChanged(ValueError):
    pass


def fixture():
    scene = flat_scene()
    reference = dense.freeze_reference(scene['rgb'], scene['whole'], scene['valid'], 100)
    return scene, reference


def affine(scene, reference, cache=None, check=lambda: None):
    return face._affine_proposal(reference, scene['rgb'], scene['whole'], 101, check,
                                frozen_affine=cache)


def refine(scene, reference, cache=None, check=lambda: None):
    initial = dense.propose_homography(reference, 101, np.eye(3))
    return train.refine_train_only_proposal(reference, scene['rgb'], scene['whole'],
        scene['valid'], 101, initial, check=check, frozen_training=cache)


def test_affine_cache_has_exact_features_homography_and_checkpoint_count():
    scene, reference = fixture()
    cache = face.FrozenAffineSource(reference)
    points, descriptors = face._affine_features(reference.bgr, reference.mask)
    np.testing.assert_array_equal(cache.points, points)
    np.testing.assert_array_equal(cache.descriptors, descriptors)
    calls = [[], []]
    original = affine(scene, reference, check=lambda: calls[0].append(1))
    cached = affine(scene, reference, cache, check=lambda: calls[1].append(1))
    np.testing.assert_array_equal(original.matrix, cached.matrix)
    assert len(calls[0]) == len(calls[1]) == 5


def test_training_cache_has_exact_pool_parameters_support_and_checkpoint_count():
    scene, reference = fixture()
    cache = train.FrozenTrainingSource(reference)
    for frozen, ordinary in zip(cache.samples, train._samples(reference)):
        np.testing.assert_array_equal(frozen, ordinary)
    calls = [[], []]
    original, audit = refine(scene, reference, check=lambda: calls[0].append(1))
    cached, cached_audit = refine(scene, reference, cache, check=lambda: calls[1].append(1))
    np.testing.assert_array_equal(original.matrix, cached.matrix)
    np.testing.assert_array_equal(original.optimizer_current_support, cached.optimizer_current_support)
    assert audit == cached_audit
    assert len(calls[0]) == len(calls[1]) and len(calls[0]) > 10


@pytest.mark.parametrize('kind', ['affine', 'train'])
def test_cache_owns_immutable_arrays_and_cannot_replace_its_reference(kind):
    scene, reference = fixture()
    cache = face.FrozenAffineSource(reference) if kind == 'affine' else train.FrozenTrainingSource(reference)
    arrays = (cache.points, cache.descriptors) if kind == 'affine' else cache.samples
    copies = [x.copy() for x in arrays]
    scene['rgb'][:] = 0; scene['whole'][:] = 0; scene['valid'][:] = False
    for value, expected in zip(arrays, copies):
        np.testing.assert_array_equal(value, expected)
        with pytest.raises(ValueError):
            value.setflags(write=True)
        with pytest.raises(ValueError):
            value.flat[0] = 0
    with pytest.raises((AttributeError, FrozenInstanceError)):
        cache.reference = None
    with pytest.raises((AttributeError, FrozenInstanceError)):
        cache.reference_digest = '0'*64


@pytest.mark.parametrize('kind', ['affine', 'train'])
@pytest.mark.parametrize('wrong', ['equal_copy', 'digest', 'type'])
def test_frozen_source_never_accepts_another_owner_or_wrong_binding(kind, wrong):
    scene, reference = fixture()
    make = face.FrozenAffineSource if kind == 'affine' else train.FrozenTrainingSource
    cache = make(reference)
    if wrong == 'equal_copy':
        reference = dense.freeze_reference(scene['rgb'], scene['whole'], scene['valid'], 100)
        assert reference.digest == cache.reference_digest and cache.reference is not reference
    elif wrong == 'digest':
        # Fault injection bypasses frozen assignment only to exercise the consumer guard.
        object.__setattr__(cache, 'reference_digest', '0'*64)
    else:
        cache = SimpleNamespace(reference=reference, reference_digest=reference.digest)
    with pytest.raises(ValueError, match='(?:affine|frozen)_source_binding_invalid'):
        (affine if kind == 'affine' else refine)(scene, reference, cache)


@pytest.mark.parametrize('kind', ['affine', 'train'])
@pytest.mark.parametrize('checkpoint', [1, 2, 3])
def test_owner_revocation_at_or_after_cache_lookup_preserves_exact_exception(kind, checkpoint):
    scene, reference = fixture()
    cache = face.FrozenAffineSource(reference) if kind == 'affine' else train.FrozenTrainingSource(reference)
    error = OwnerChanged('manual/selection/lease owner revoked')
    calls = []
    def check():
        calls.append(1)
        if len(calls) == checkpoint:
            raise error
    with pytest.raises(OwnerChanged) as caught:
        (affine if kind == 'affine' else refine)(scene, reference, cache, check=check)
    assert caught.value is error and len(calls) == checkpoint


def test_cached_affine_still_computes_current_features_and_rejects_missing_current_texture(monkeypatch):
    scene, reference = fixture()
    cache = face.FrozenAffineSource(reference)
    original = face._affine_features
    seen = []
    def current_only(image, mask):
        assert image is not reference.bgr
        seen.append(image.copy())
        return original(image, mask)
    monkeypatch.setattr(face, '_affine_features', current_only)
    affine(scene, reference, cache)
    scene['rgb'][:] = 80
    with pytest.raises(face.MeasuredFaceUnavailable, match='descriptors_unavailable'):
        affine(scene, reference, cache)
    assert len(seen) == 2 and not np.array_equal(seen[0], seen[1])


@pytest.mark.parametrize('changed', ['invalid_rgb', 'narrow_mask'])
def test_cached_train_still_recomputes_current_domain_without_rerunning_source_pool(monkeypatch, changed):
    scene, reference = fixture()
    cache = train.FrozenTrainingSource(reference)
    monkeypatch.setattr(train, '_samples', lambda _: pytest.fail('PRE samples recomputed'))
    refine(scene, reference, cache)
    if changed == 'invalid_rgb':
        scene['valid'][:] = False
        expected = 'MASK_CONSUMES_INVALID_RGB'
    else:
        scene['whole'][:] = 0
        scene['whole'][90:100, 100:110] = 1
        expected = 'insufficient_fixed_train_domain'
    with pytest.raises(ValueError, match=expected):
        refine(scene, reference, cache)


@pytest.mark.parametrize('hook', ['affine', 'training'])
@pytest.mark.parametrize('invalidate', ['owner', 'deadline'])
def test_precomputation_cost_remains_inside_original_source_lifetime(monkeypatch, hook, invalidate):
    scene = flat_scene()
    now = [0.]
    error = OwnerChanged('original PRE source or owner revoked')
    revoked = [False]
    name = 'FrozenAffineSource' if hook == 'affine' else 'FrozenTrainingSource'
    original = getattr(face, name)
    completed = []
    def do_then_expire(*args):
        result = original(*args)
        completed.append(True)
        now[0] = 1.
        revoked[0] = True
        return result
    monkeypatch.setattr(face, name, do_then_expire)
    def check():
        if revoked[0] and invalidate == 'owner':
            raise error
    with pytest.raises(ValueError) as caught:
        freeze(scene, check=check, deadline=1., clock=lambda: now[0])
    assert completed == [True]
    if invalidate == 'owner':
        assert caught.value is error
    else:
        assert str(caught.value) == 'observation_appearance_deadline_exceeded'


def test_real_freeze_binds_precomputed_sources_to_exact_dense_reference():
    frozen = freeze(flat_scene())
    assert frozen.affine_reference.reference is frozen.reference
    assert frozen.training_reference.reference is frozen.reference
    assert frozen.affine_reference.reference_digest == frozen.reference.digest
    assert frozen.training_reference.reference_digest == frozen.reference.digest
    audit = json.loads(frozen.audit_json)
    assert audit['immutable_source_features_precomputed']
    assert audit['source_training_samples_precomputed']


def test_textureless_source_preserves_unavailable_optional_proof():
    scene = flat_scene()
    scene['rgb'][scene['whole'] > 0] = 70
    frozen = freeze(scene)
    assert frozen.training_reference is None
    assert not json.loads(frozen.audit_json)['source_training_samples_precomputed']
    with pytest.raises(face.MeasuredFaceUnavailable, match='descriptors_unavailable'):
        affine(scene, frozen.reference, frozen.affine_reference)
