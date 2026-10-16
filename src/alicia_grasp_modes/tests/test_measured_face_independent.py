"""Independent synthetic counterexamples, not recorded grasp success evidence.

Real dense photometry and source-local face extraction execute without ROS.
The examples deliberately retain histogram similarity while breaking texture,
or retain plausible texture while breaking ownership and source contracts.
"""
from copy import deepcopy
from dataclasses import replace
import hashlib

import cv2
import numpy as np
import pytest

from alicia_grasp_modes import arrival_dense_identity as dense
from alicia_grasp_modes.tabletop import Config


def image_scene(*, constant=False):
    shape = (240, 320)
    whole = np.zeros(shape, np.uint8)
    whole[35:210, 55:265] = 1
    mask = np.zeros(shape, np.uint8)
    mask[35:190, 55:265] = 1
    rng = np.random.RandomState(8047)
    gray = rng.randint(16, 104, shape).astype(np.uint8)
    if constant:
        gray[:] = 65
    rgb = np.repeat(gray[..., None], 3, axis=2)
    rgb[whole == 0] = (80, 110, 180)
    rgb[190:210, 55:265] = (15, 25, 160)
    depth = np.full(shape, .4, np.float32)
    depth[mask > 0] = .35
    depth[(whole > 0) & (mask == 0)] = .365
    return dict(rgb=rgb, whole=whole, mask=mask, depth=depth,
                valid=np.ones(shape, bool), intrinsics=[400., 400., 160., 120.],
                transform=np.eye(4), plane=np.array([-1., 0., 0., .4]),
                config=Config())


def evaluate(scene, *, current=None, mask=None, matrix=None, source=101,
             require_dense=True, clock=lambda: 0., deadline=1., proposal=None):
    ref = dense.freeze_reference(scene['rgb'], scene['mask'], scene['valid'], 100)
    if proposal is None:
        proposal = dense.propose_homography(ref, source, np.eye(3) if matrix is None else matrix)
    return dense.evaluate_appearance(ref, scene['rgb'] if current is None else current,
        scene['mask'] if mask is None else mask, scene['valid'], source, proposal,
        deadline_monotonic=deadline, clock=clock, require_dense=require_dense)


def test_complete_same_face_requires_real_distributed_dense_proof():
    scene = image_scene()
    result = evaluate(scene)
    assert result.appearance_match and result.code == 'DENSE_TEXTURE_CORROBORATED'
    assert result.metrics['colour_distance'] == pytest.approx(0.)
    assert result.metrics['holdout_ncc'] == pytest.approx(1.)
    assert result.metrics['good_test_quadrants'] == 4
    assert result.metrics['good_test_cells'] >= result.metrics['required_good_test_cells']
    assert not result.motion_authorized and not result.metric_geometry_authority
    assert not result.reference_refreshed


def test_uniform_same_colour_is_not_a_dense_identity_proof():
    scene = image_scene(constant=True)
    ordinary = evaluate(scene, require_dense=False)
    mandatory = evaluate(scene)
    assert ordinary.appearance_match and ordinary.code == 'ORIGINAL_HISTOGRAM_PASS'
    assert not mandatory.appearance_match
    assert mandatory.code == 'INSUFFICIENT_REFERENCE_TEXTURE'


def test_identical_histogram_with_permuted_real_pixels_cannot_short_circuit_dense():
    scene = image_scene()
    current = scene['rgb'].copy()
    values = current[scene['mask'] > 0].copy()
    current[scene['mask'] > 0] = values[np.random.RandomState(901).permutation(len(values))]
    assert evaluate(scene, current=current, require_dense=False).appearance_match
    result = evaluate(scene, current=current)
    assert result.metrics['colour_distance'] == pytest.approx(0.)
    assert not result.appearance_match
    # Independent shuffled pixels can also invalidate the fitted gain first.
    assert result.code in ('AFFINE_GAIN_OUT_OF_BOUNDS',
        'DISTRIBUTED_TEXTURE_NOT_CONFIRMED', 'HOLDOUT_TEXTURE_MISMATCH')


@pytest.mark.parametrize('factor', [.4, 2.2])
def test_mandatory_dense_keeps_original_gain_bounds(factor):
    scene = image_scene()
    current = scene['rgb'].copy()
    current[scene['mask'] > 0] = np.rint(
        current[scene['mask'] > 0].astype(float) * factor).astype(np.uint8)
    result = evaluate(scene, current=current)
    assert not result.appearance_match and result.code == 'AFFINE_GAIN_OUT_OF_BOUNDS'
    assert result.metrics['holdout_ncc'] > .99


@pytest.mark.parametrize('kind', ['partial', 'hole'])
def test_complete_face_mask_cannot_be_replaced_by_a_matching_subset(kind):
    scene = image_scene()
    partial = scene['mask'].copy()
    if kind == 'partial':
        partial[:, :145] = 0
    else:
        partial[65:160, 95:225] = 0
    result = evaluate(scene, mask=partial)
    assert not result.appearance_match
    assert result.code in ('WHOLE_MASK_INCONSISTENT', 'ORIGINAL_SHAPE_MISMATCH')


@pytest.mark.parametrize('source', [99, 100])
def test_common_face_source_must_strictly_follow_frozen_source(source):
    result = evaluate(image_scene(), source=source)
    assert not result.appearance_match and result.code == 'CURRENT_SOURCE_NOT_AFTER_REFERENCE'


@pytest.mark.parametrize('mutation', ['digest', 'source'])
def test_low_colour_distance_does_not_bypass_homography_binding(mutation):
    scene = image_scene()
    ref = dense.freeze_reference(scene['rgb'], scene['mask'], scene['valid'], 100)
    proposal = dense.propose_homography(ref, 101, np.eye(3))
    if mutation == 'digest':
        proposal = replace(proposal, reference_digest='0' * 64)
    else:
        proposal = replace(proposal, current_source_stamp_ns=102)
    result = evaluate(scene, proposal=proposal)
    assert not result.appearance_match and result.code == 'HOMOGRAPHY_SOURCE_BINDING_MISMATCH'


@pytest.mark.parametrize('offset', [-1e-9, 0., 1e-9])
def test_mandatory_dense_deadline_is_absolute_and_not_renewed(offset):
    now = 1. + offset
    result = evaluate(image_scene(), clock=lambda: now, deadline=1.)
    assert result.appearance_match is (offset < 0.)
    if offset >= 0.:
        assert result.code == 'DEADLINE_EXCEEDED'


def test_clock_crossing_after_geometry_cannot_return_dense_success(monkeypatch):
    scene = image_scene()
    now = [0.]
    original = dense._mask_geometry
    def crossed(*args):
        result = original(*args)
        now[0] = 1.
        return result
    monkeypatch.setattr(dense, '_mask_geometry', crossed)
    result = evaluate(scene, clock=lambda: now[0])
    assert not result.appearance_match and result.code == 'DEADLINE_EXCEEDED'


def test_dense_reference_inputs_cannot_roll_with_later_array_mutation():
    scene = image_scene()
    ref = dense.freeze_reference(scene['rgb'], scene['mask'], scene['valid'], 100)
    digest = ref.digest
    rgb, mask = ref.bgr.copy(), ref.mask.copy()
    scene['rgb'][:] = 0
    scene['mask'][:] = 0
    scene['valid'][:] = False
    assert ref.digest == digest
    np.testing.assert_array_equal(ref.bgr, rgb)
    np.testing.assert_array_equal(ref.mask, mask)
    for value in [ref.bgr, ref.mask, ref.valid, ref.gray, ref.histogram, ref.contour]:
        with pytest.raises(ValueError):
            value.setflags(write=True)


def face_module():
    from alicia_grasp_modes import observation_measured_face
    return observation_measured_face


def flat_scene():
    scene = image_scene()
    scene['whole'] = scene['mask'].copy()
    scene['rgb'][scene['whole'] == 0] = (80, 110, 180)
    scene['depth'][scene['whole'] == 0] = .4
    return scene


def extract(scene, check=lambda: None):
    return face_module().extract_measured_face(scene['rgb'], scene['whole'],
        scene['valid'], scene['depth'], scene['intrinsics'], scene['transform'],
        scene['plane'], scene['config'], check)


def face_source(scene, encoding='32FC1'):
    raw = scene['depth'].copy() if encoding == '32FC1' else np.rint(scene['depth'] / .001).astype(np.uint16)
    return dict(raw_depth=raw, encoding=encoding, depth_scale=1. if encoding == '32FC1' else .001,
        intrinsics=list(scene['intrinsics']), base_from_camera=scene['transform'].copy(),
        support_plane=scene['plane'].copy(), config=deepcopy(scene['config']),
        source_ns=100, source_id='independent-original-premotion-source',
        rgb_sha256=hashlib.sha256(scene['rgb'].tobytes()).hexdigest(),
        whole_mask_sha256=hashlib.sha256(scene['whole'].tobytes()).hexdigest())


def freeze(scene, source=None, check=lambda: None, deadline=1., clock=lambda: 0.):
    module = face_module()
    source = face_source(scene) if source is None else source
    if isinstance(source, dict):
        source = module.FrozenMeasuredFaceSource(**source)
    return module.freeze_measured_face(source,
        scene['rgb'], scene['whole'], scene['valid'], check_current=check,
        deadline_monotonic=deadline, clock=clock)


def test_actual_source_local_extraction_never_changes_current_depth():
    scene = flat_scene()
    original = {k: v.copy() for k, v in scene.items() if isinstance(v, np.ndarray)}
    result = extract(scene)
    assert result is not None
    for name, value in original.items():
        np.testing.assert_array_equal(scene[name], value)


@pytest.mark.parametrize('encoding', ['32FC1', '16UC1', 'mono16'])
def test_frozen_face_owns_original_depth_and_complete_reference_arrays(encoding):
    scene = flat_scene()
    source = face_source(scene, encoding)
    raw = source['raw_depth'].copy()
    frozen = freeze(scene, source)
    original_rgb = frozen.reference.bgr.copy()
    original_mask = frozen.reference.mask.copy()
    source['raw_depth'][:] = 0
    source['base_from_camera'][0, 3] = 1
    scene['rgb'][:] = 0
    scene['depth'][:] = 0
    scene['whole'][:] = 0
    np.testing.assert_array_equal(frozen.source.raw_depth, raw)
    np.testing.assert_array_equal(frozen.reference.bgr, original_rgb)
    np.testing.assert_array_equal(frozen.reference.mask, original_mask)
    assert frozen.source.source_ns == frozen.reference.source_stamp_ns == 100
    for arr in [frozen.source.raw_depth, frozen.source.intrinsics,
                frozen.source.transform, frozen.source.plane, frozen.points,
                frozen.normal, frozen.centre, frozen.reference.mask]:
        with pytest.raises(ValueError):
            arr.setflags(write=True)
    with pytest.raises(AttributeError):
        frozen.source.source_ns = 101
    with pytest.raises(AttributeError):
        frozen.reference = None


def test_disconnected_second_measured_face_cannot_hide_below_fifteen_percent():
    scene = flat_scene()
    # A second coplanar component has independently reliable measured core.
    # The larger component exceeds 85% globally, so aggregate ownership alone
    # cannot establish that one complete face owns this entire source mask.
    small = np.zeros(scene['whole'].shape, bool)
    small[195:225, 100:150] = True
    scene['whole'][small] = 1
    scene['depth'][small] = .35
    scene['rgb'][small] = (35, 185, 30)
    with pytest.raises(face_module().MeasuredFaceUnavailable):
        extract(scene)


def test_source_local_selection_is_invariant_to_reference_freeze_history():
    scene = flat_scene()
    mask1, points1, normal1, centre1, audit1 = extract(scene)
    freeze(flat_scene())
    mask2, points2, normal2, centre2, audit2 = extract(scene)
    for a, b in [(mask1, mask2), (points1, points2), (normal1, normal2), (centre1, centre2)]:
        np.testing.assert_array_equal(a, b)
    assert audit1 == audit2


def test_unknown_depth_is_not_promoted_to_measured_face_geometry():
    scene = flat_scene()
    scene['depth'][85:105, 125:145] = 0.
    before = scene['depth'].copy()
    mask, points, _normal, _centre, audit = extract(scene)
    np.testing.assert_array_equal(scene['depth'], before)
    assert audit['raw_missing_depth_filled'] is False
    assert audit['measured_planar_pixels'] == len(points)
    assert len(points) < np.count_nonzero(mask)
    assert np.all(np.isfinite(points)) and np.all(points[:, 0] > .03)


@pytest.mark.parametrize('fault', ['rotation_reflection', 'plane_unnormalized', 'nan_intrinsics', 'negative_fx'])
def test_bad_calibration_or_support_cannot_supply_face_geometry(fault):
    scene = flat_scene()
    if fault == 'rotation_reflection':
        scene['transform'][1, 1] = -1
    elif fault == 'plane_unnormalized':
        scene['plane'] *= 2
    elif fault == 'nan_intrinsics':
        scene['intrinsics'][0] = float('nan')
    else:
        scene['intrinsics'][0] = -400.
    with pytest.raises(face_module().MeasuredFaceUnavailable):
        extract(scene)


def test_expiry_during_reference_feature_freeze_never_returns_a_template(monkeypatch):
    scene = flat_scene()
    module = face_module()
    original = module.freeze_reference
    now = [0.]
    def delayed(*args):
        result = original(*args)
        now[0] = 1.
        return result
    monkeypatch.setattr(module, 'freeze_reference', delayed)
    with pytest.raises(ValueError, match='observation_appearance_deadline_exceeded'):
        freeze(scene, clock=lambda: now[0])


def corroborate(frozen, scene, *, source=101, check=lambda: None,
                deadline=1., clock=lambda: 0.):
    frame = {name: scene[name] for name in ['depth', 'intrinsics', 'transform', 'plane', 'config']}
    return face_module().corroborate_measured_face(frozen, scene['rgb'], scene['whole'],
        scene['valid'], frame, source_ns=source, deadline_monotonic=deadline,
        check=check, clock=clock)


def test_actual_independent_extraction_affine_and_dense_same_face_positive():
    scene = flat_scene()
    frozen = freeze(scene)
    before = scene['depth'].copy()
    evidence, audit = corroborate(frozen, scene)
    assert evidence.appearance_match and evidence.code == 'DENSE_TEXTURE_CORROBORATED'
    assert audit['required_dense'] is True
    assert audit['raw_missing_depth_filled'] is False
    assert audit['dense_metrics']['good_test_quadrants'] == 4
    assert not evidence.motion_authorized and not evidence.metric_geometry_authority
    np.testing.assert_array_equal(scene['depth'], before)


@pytest.mark.parametrize('change', ['config', 'plane_height'])
def test_same_real_texture_does_not_override_current_geometry_change(change):
    scene = flat_scene()
    frozen = freeze(scene)
    if change == 'config':
        scene['config'].support_threshold_m *= 2
    else:
        scene['depth'][scene['whole'] > 0] -= .02
    with pytest.raises(face_module().MeasuredFaceUnavailable):
        corroborate(frozen, scene)


@pytest.mark.parametrize('when', ['entry', 'after_extract', 'after_affine', 'after_dense'])
def test_current_face_owner_revocation_preserves_exception_identity(monkeypatch, when):
    scene = flat_scene()
    frozen = freeze(scene)
    module = face_module()
    class OwnerRevoked(ValueError):
        pass
    failure = OwnerRevoked('selection/epoch/manual revoked')
    invalid = [when == 'entry']
    if when != 'entry':
        name = {'after_extract': 'extract_measured_face', 'after_affine': '_affine_proposal',
                'after_dense': 'evaluate_appearance'}[when]
        original = getattr(module, name)
        def injected(*args, **kwargs):
            result = original(*args, **kwargs)
            invalid[0] = True
            return result
        monkeypatch.setattr(module, name, injected)
    def check():
        if invalid[0]:
            raise failure
    with pytest.raises(OwnerRevoked) as caught:
        corroborate(frozen, scene, check=check)
    assert caught.value is failure


@pytest.mark.parametrize('kind', ['source', 'capture', 'registered_station', 'session_phase'])
@pytest.mark.parametrize('when', ['before_extract', 'after_extract', 'after_affine', 'after_dense'])
def test_explicit_owner_deadline_checks_are_not_hidden_as_optional_evidence(monkeypatch, kind, when):
    """Contract injection tests; no claim to reconstruct 5s/60s from a short bag.

    The real call-local owner check supplies the absolute limit; this test
    proves each typed denial reaches the caller unchanged through new work.
    """
    scene = flat_scene()
    frozen = freeze(scene)
    module = face_module()
    # Fixed absolute deadlines represent existing owner contracts, not renewed
    # budgets or extra allowances added by the measured-face helper.
    deadlines = dict(source=.8, capture=3., registered_station=5., session_phase=60.)
    deadline = deadlines[kind]
    now = [deadline - .001]
    class OriginalDeadline(ValueError):
        pass
    failure = OriginalDeadline(kind + ' existing absolute deadline')
    if when == 'before_extract':
        now[0] = deadline
    else:
        name = {'after_extract': 'extract_measured_face', 'after_affine': '_affine_proposal',
                'after_dense': 'evaluate_appearance'}[when]
        original = getattr(module, name)
        def reached(*args, **kwargs):
            result = original(*args, **kwargs)
            now[0] = deadline
            return result
        monkeypatch.setattr(module, name, reached)
    def check():
        if now[0] >= deadline:
            raise failure
    with pytest.raises(OriginalDeadline) as caught:
        corroborate(frozen, scene, check=check, deadline=deadline,
                    clock=lambda: now[0])
    assert caught.value is failure
    assert now[0] == deadline


def appearance_case():
    from alicia_grasp_modes import observation_appearance as appearance
    from alicia_grasp_modes.rgb_observation_handoff import _appearance
    scene = flat_scene()
    face = freeze(scene)
    hist, contour = _appearance(scene['rgb'], scene['whole'])
    reference = appearance.FrozenObservationAppearance(scene['rgb'], scene['whole'],
        scene['valid'], 100, hist, contour, measured_face=face)
    current = scene['rgb'].copy()
    current[scene['whole'] > 0] = np.rint(current[scene['whole'] > 0].astype(float) * .5).astype(np.uint8)
    frame = {k: scene[k] for k in ['depth', 'intrinsics', 'transform', 'plane', 'config']}
    context = dict(reference=reference, source_ns=101, deadline_monotonic=1.,
        clock=lambda: 0., check_current=lambda: None, measured_face_cache={},
        measured_face_frame=frame)
    def call():
        return appearance.match_observation_appearance(current, scene['whole'], hist, contour,
            rgb_valid=scene['valid'], appearance_context=context)
    return scene, current, context, call


def test_call_local_exact_input_cache_retains_real_dense_and_copies_audit(monkeypatch):
    _scene, _current, context, call = appearance_case()
    module = face_module()
    original = module.corroborate_measured_face
    calls = []
    def counted(*args, **kwargs):
        calls.append(True)
        return original(*args, **kwargs)
    monkeypatch.setattr(module, 'corroborate_measured_face', counted)
    first = call()
    assert first['appearance_match'], first
    assert first['code'] == 'COMMON_MEASURED_FACE_DENSE_CORROBORATED'
    assert first['colour_distance'] > .4
    assert not first['common_measured_face']['same_frame_evidence_reused']
    assert len(context['measured_face_cache']) == 1
    first['common_measured_face']['dense_metrics']['holdout_ncc'] = -100.
    second = call()
    assert second['appearance_match'] and len(calls) == 1
    assert second['common_measured_face']['same_frame_evidence_reused']
    assert second['common_measured_face']['dense_metrics']['holdout_ncc'] > .99
    # A second observe call owns a new cache and must independently compute.
    context['measured_face_cache'] = {}
    assert call()['appearance_match'] and len(calls) == 2


@pytest.mark.parametrize('changed', ['rgb', 'depth', 'valid', 'mask', 'transform',
    'intrinsics', 'plane', 'config', 'source', 'deadline', 'reference'])
def test_changed_evidence_byte_or_owner_key_never_reuses_cached_face(monkeypatch, changed):
    from alicia_grasp_modes import observation_appearance as appearance
    from alicia_grasp_modes.rgb_observation_handoff import _appearance
    scene, current, context, call = appearance_case()
    assert call()['appearance_match']
    if changed == 'rgb':
        current[70, 80, 0] ^= 1
    elif changed == 'depth':
        scene['depth'].view(np.uint8)[70, 80 * 4] ^= 1
    elif changed == 'valid':
        scene['valid'][0, 0] = False
    elif changed == 'mask':
        scene['whole'][35, 55] = 0
    elif changed == 'transform':
        scene['transform'][1, 3] = .000001
    elif changed == 'intrinsics':
        scene['intrinsics'][2] += .001
    elif changed == 'plane':
        scene['plane'][3] += .000001
    elif changed == 'config':
        scene['config'].association_distance_m += .000001
    elif changed == 'source':
        context['source_ns'] += 1
    elif changed == 'deadline':
        context['deadline_monotonic'] += .01
    else:
        original_scene = flat_scene()
        hist, contour = _appearance(original_scene['rgb'], original_scene['whole'])
        context['reference'] = appearance.FrozenObservationAppearance(original_scene['rgb'],
            original_scene['whole'], original_scene['valid'], 100, hist, contour,
            measured_face=freeze(original_scene))
    class RecomputeRequired(ValueError):
        pass
    failure = RecomputeRequired('fresh complete evidence computation reached')
    def stop(*args, **kwargs):
        raise failure
    monkeypatch.setattr(face_module(), 'corroborate_measured_face', stop)
    with pytest.raises(RecomputeRequired) as caught:
        call()
    assert caught.value is failure


@pytest.mark.parametrize('boundary', ['entry', 'lookup_before', 'lookup_after'])
@pytest.mark.parametrize('reason', ['manual', 'epoch', 'source_expiry'])
def test_owner_check_on_cached_evidence_never_reuses_revoked_result(boundary, reason):
    _scene, _current, context, call = appearance_case()
    assert call()['appearance_match']
    class Revoked(ValueError):
        pass
    failure = Revoked(reason)
    checks = []
    cutoff = {'entry': 1, 'lookup_before': 2, 'lookup_after': 3}[boundary]
    def check():
        checks.append(True)
        if len(checks) == cutoff:
            raise failure
    context['check_current'] = check
    with pytest.raises(Revoked) as caught:
        call()
    assert caught.value is failure


def test_cached_evidence_does_not_extend_original_absolute_deadline():
    _scene, _current, context, call = appearance_case()
    assert call()['appearance_match']
    context['clock'] = lambda: 1.
    with pytest.raises(ValueError, match='observation_appearance_deadline_exceeded'):
        call()
    assert context['deadline_monotonic'] == 1.


@pytest.mark.parametrize('value', [None, 1, 'true', np.bool_(True)])
def test_dense_requirement_flag_cannot_be_coerced_or_disable_validation(value):
    result = evaluate(image_scene(), require_dense=value)
    assert not result.appearance_match and result.code == 'INVALID_DENSE_REQUIREMENT'


def test_cached_negative_face_evidence_never_becomes_common_face_success(monkeypatch):
    """Verdict-routing test; real positive/negative photometry is tested above."""
    _scene, _current, context, call = appearance_case()
    module = face_module()
    original = module.corroborate_measured_face
    calls = []
    def failed_member(*args, **kwargs):
        evidence, audit = original(*args, **kwargs)
        assert evidence.appearance_match
        calls.append(True)
        return replace(evidence, appearance_match=False, code='INDEPENDENT_REQUIRED_MEMBER_FAILED'), audit
    monkeypatch.setattr(module, 'corroborate_measured_face', failed_member)
    def old_path_failed(reference, image, mask, valid, source, proposal, **kwargs):
        return dense.AppearanceEvidence(False, True, 'INDEPENDENT_OLD_PATH_REJECTED',
            reference.digest, reference.source_stamp_ns, source, {})
    monkeypatch.setattr(dense, 'evaluate_appearance', old_path_failed)
    first, second = call(), call()
    assert len(calls) == 1 and len(context['measured_face_cache']) == 1
    assert not first['appearance_match'] and not second['appearance_match']
    assert first['code'] == second['code'] == 'INDEPENDENT_OLD_PATH_REJECTED'
    assert second['common_measured_face']['same_frame_evidence_reused']


@pytest.mark.parametrize('fault', ['all_depth_missing', 'whole_rgb_invalid', 'no_foreground'])
def test_missing_measurements_never_produce_a_face(fault):
    scene = image_scene()
    if fault == 'all_depth_missing':
        scene['depth'][scene['whole'] > 0] = np.nan
    elif fault == 'whole_rgb_invalid':
        scene['valid'][scene['whole'] > 0] = False
    else:
        scene['depth'][:] = .4
    with pytest.raises(face_module().MeasuredFaceUnavailable):
        extract(scene)


@pytest.mark.parametrize('where', ['extract', 'freeze'])
def test_typed_owner_revocation_is_not_relabelled_as_optional_face_unavailable(where):
    scene = image_scene()
    class Revoked(ValueError):
        pass
    failure = Revoked('independent owner changed')
    def check():
        raise failure
    with pytest.raises(Revoked) as caught:
        (extract(scene, check) if where == 'extract' else freeze(scene, check=check))
    assert caught.value is failure


@pytest.mark.parametrize('fault', ['rgb', 'whole_mask', 'future_source', 'bool_source', 'nan_scale', 'zero_scale'])
def test_premotion_reference_cannot_freeze_changed_or_invalid_source(fault):
    scene = image_scene()
    source = face_source(scene)
    if fault == 'rgb':
        scene['rgb'][70, 80, 0] ^= 255
    elif fault == 'whole_mask':
        scene['whole'][70, 80] = 0
    elif fault == 'future_source':
        source['source_ns'] = -1
    elif fault == 'bool_source':
        source['source_ns'] = True
    else:
        source['encoding'] = '16UC1'
        source['raw_depth'] = np.rint(scene['depth'] / .001).astype(np.uint16)
        source['depth_scale'] = float('nan') if fault == 'nan_scale' else 0.
    with pytest.raises(ValueError):
        freeze(scene, source)
