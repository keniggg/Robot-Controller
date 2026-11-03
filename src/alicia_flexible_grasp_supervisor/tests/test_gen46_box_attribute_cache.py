"""Field decoding retains the JSON model's ownership and numeric semantics."""
from copy import deepcopy
from dataclasses import asdict, fields, replace
import json
import pickle

import numpy as np
import pytest

from alicia_flexible_grasp.vision import regular_box_inference as subject
from test_gen26_regular_box_inference_independent import qualify, ray_scene


ARRAY_FIELDS = ('center_base', 'axes_base', 'size_xyz_m',
                'support_normal_base', 'registration_transform')


@pytest.fixture(scope='module')
def model():
    result = qualify(ray_scene())
    assert result.ok, result.audit
    return result.model


@pytest.mark.parametrize('key', sorted(subject._FIELDS))
def test_each_field_matches_original_full_document_decode(model, key):
    expected = json.loads(model._serialized)[key]
    observed = getattr(model, key)
    if key in ARRAY_FIELDS:
        np.testing.assert_array_equal(observed, np.asarray(expected, dtype=float))
        with pytest.raises(ValueError):
            observed.setflags(write=True)
    else:
        assert observed == expected and type(observed) is type(expected)


def test_returned_nested_containers_cannot_modify_future_reads(model):
    original = model.to_evidence_dict()
    source = model.source
    source['target_identity']['track_id'] = 'changed'
    qualification = model.qualification
    qualification['corner_coverage'][0] = -100.
    evidence = model.to_evidence_dict()
    evidence['qualification']['corner_coverage'][1] = -200.
    assert model.to_evidence_dict() == original
    assert model.source == original['source']
    assert model.qualification == original['qualification']


def test_array_metadata_changes_do_not_change_a_subsequent_field_read(model):
    rotation = model.axes_base
    rotation.shape = (9,)
    assert model.axes_base.shape == (3, 3)
    np.testing.assert_array_equal(model.axes_base, json.loads(model._serialized)['axes_base'])


def test_cache_has_only_immutable_strings_and_keeps_original_dataclass_value(model):
    backing, pairs = model._attribute_json_cache
    assert backing is model._serialized and type(pairs) is tuple
    assert all(type(pair) is tuple and len(pair) == 2 and
               all(type(value) is str for value in pair) for pair in pairs)
    assert [field.name for field in fields(model)] == ['_serialized']
    assert asdict(model) == {'_serialized': model._serialized}
    with pytest.raises(AttributeError):
        model._attribute_json_cache = ()


@pytest.mark.parametrize('copy_kind', ['deepcopy', 'pickle', 'replace'])
def test_copy_and_serialization_keep_value_and_owned_returns(model, copy_kind):
    clone = (deepcopy(model) if copy_kind == 'deepcopy' else
             pickle.loads(pickle.dumps(model)) if copy_kind == 'pickle' else replace(model))
    assert clone == model and hash(clone) == hash(model)
    assert clone.to_evidence_dict() == model.to_evidence_dict()
    assert clone.qualification == model.qualification
    clone.source['stamp_ns'] = 1
    assert clone.source == model.source
    with pytest.raises(ValueError):
        clone.center_base.setflags(write=True)


def test_legacy_serialized_instance_without_cache_preserves_the_original_api(model):
    legacy = object.__new__(subject.RegularBoxInference)
    object.__setattr__(legacy, '_serialized', model._serialized)
    assert legacy.to_evidence_dict() == model.to_evidence_dict()
    assert legacy.source == model.source
    np.testing.assert_array_equal(legacy.center_base, model.center_base)
    with pytest.raises(AttributeError):
        getattr(legacy, 'not_a_model_field')


def test_a_detached_cache_cannot_override_reconstructed_backing_evidence(model):
    transform = np.eye(4)
    transform[:3, 3] = [.01, -.02, .03]
    other = subject.transform_inferred_box(model, transform)
    rebuilt = deepcopy(model)
    object.__setattr__(rebuilt, '_serialized', other._serialized)
    assert rebuilt.model_sha256 == other.model_sha256
    np.testing.assert_array_equal(rebuilt.center_base, other.center_base)
    assert rebuilt.to_evidence_dict() == other.to_evidence_dict()


def test_repeated_small_fields_do_not_decode_unrelated_evidence(model, monkeypatch):
    original_loads = json.loads
    decoded_strings = []

    def record(value, *args, **kwargs):
        decoded_strings.append(value)
        return original_loads(value, *args, **kwargs)

    monkeypatch.setattr(subject.json, 'loads', record)
    for _ in range(20):
        assert model.schema_version in (1, 2, 3)
        assert model.model_sha256
        assert model.size_xyz_m.shape == (3,)
    assert len(decoded_strings) == 60
    assert all(len(value) < len(model._serialized) for value in decoded_strings)
    assert model._serialized not in decoded_strings


def test_unknown_field_preserves_attribute_error(model):
    with pytest.raises(AttributeError, match='not_a_model_field'):
        getattr(model, 'not_a_model_field')
