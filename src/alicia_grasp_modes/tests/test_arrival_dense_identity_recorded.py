"""Offline GEN22 recorded-adjacent-frame checks and explicit synthetic challenges."""
from dataclasses import asdict
import json
from pathlib import Path
import time

import cv2
import numpy as np
import pytest

from alicia_grasp_modes import arrival_dense_identity as candidate


BASE = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen22_analysis')
if not (BASE/"replay_inputs/manifest.json").exists():
    pytest.skip("GEN22 external recorded inputs are unavailable", allow_module_level=True)
MANIFEST = json.loads((BASE/'replay_inputs/manifest.json').read_text())
FILES = {row['source_stamp_ns']: row for row in MANIFEST['files']}
LAST = next(row for row in MANIFEST['files'] if row['role'] == 'last_pre_motion')
REFERENCE_INPUT = dict(np.load(LAST['path']))
H_ROWS = [row for row in json.loads((BASE/'orb_texture_correspondence_diagnostic.json').read_text())['results'] if row['variant'] == 'recorded_object']
SOURCES = [row['source_ns'] for row in H_ROWS]
H_MAP = {row['source_ns']: np.array(row['homography']) for row in H_ROWS}
EXPECTED_REJECT = {1791638841807271480, 1791638843276834011}


@pytest.fixture(scope='module')
def reference():
    return candidate.freeze_reference(REFERENCE_INPUT['color'], REFERENCE_INPUT['recorded_refined_mask'],
                                      REFERENCE_INPUT['rgb_valid'], LAST['source_stamp_ns'])


def load_frame(source):
    frame = dict(np.load(FILES[source]['path']))
    frame['mask'] = cv2.imread(str(BASE/('crop_ab_production_'+str(source)+'_mask.png')), 0)
    return frame


def run(reference, source, frame, *, matrix=None, clock=lambda: 0., deadline=1.):
    proposal = candidate.propose_homography(reference, source, H_MAP[source] if matrix is None else matrix)
    return candidate.evaluate_appearance(reference, frame['color'], frame['mask'], frame['rgb_valid'],
                                         source, proposal, deadline_monotonic=deadline, clock=clock)


def challenge(frame, name):
    image = frame['color'].copy()
    mask = frame['mask'].copy()
    x, y, w, h = cv2.boundingRect(mask)
    selected = mask > 0
    if name == 'central_logo_peripheral_permutation':
        yy, xx = np.indices(mask.shape)
        central = selected & (xx >= x+.25*w) & (xx < x+.75*w) & (yy >= y+.25*h) & (yy < y+.75*h)
        outside = selected & ~central
        pixels = image[outside].copy()
        image[outside] = pixels[np.random.RandomState(1729).permutation(len(pixels))]
        # Exact BGR tuple multiset, not merely independent per-channel bins.
        before = np.ascontiguousarray(frame['color'][selected]).view('V3').ravel()
        after = np.ascontiguousarray(image[selected]).view('V3').ravel()
        assert np.array_equal(np.sort(before), np.sort(after))
    elif name == 'different_lettering_same_palette':
        paper = np.empty((h, w, 3), np.uint8)
        paper[:] = np.median(image[selected], axis=0).astype(np.uint8)
        ink = np.percentile(image[selected], 15, axis=0).astype(np.uint8)
        for i, text in enumerate(('OTHER', 'ABCD', '2026')):
            cv2.putText(paper, text, (3, 25+i*32), cv2.FONT_HERSHEY_SIMPLEX, .65, tuple(map(int, ink)), 2, cv2.LINE_AA)
        image[y:y+h, x:x+w][selected[y:y+h, x:x+w]] = paper[selected[y:y+h, x:x+w]]
    elif name == 'hue_shift':
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv[:, :, 0] = (hsv[:, :, 0].astype(int)+60) % 180
        image[selected] = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[selected]
    elif name == 'brightness_inversion':
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        lab[:, :, 0] = 255-lab[:, :, 0]
        image[selected] = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)[selected]
    elif name == 'textureless':
        image[selected] = np.median(image[selected], axis=0).astype(np.uint8)
    elif name == 'mask_intrusion':
        mask[max(0, y-30):y+h, max(0, x-30):x+w] = 255
    elif name == 'empty_mask':
        mask[:] = 0
    else:
        raise AssertionError(name)
    return dict(color=image, mask=mask, rgb_valid=frame['rgb_valid'])


@pytest.mark.parametrize('source', SOURCES)
def test_real_recorded_adjacent_sources_keep_two_unqualified(reference, source):
    result = run(reference, source, load_frame(source))
    assert result.appearance_match == (source not in EXPECTED_REJECT)
    assert result.fallback_attempted
    assert not result.motion_authorized and not result.metric_geometry_authority and not result.reference_refreshed
    assert result.metrics['source_train_test_overlap_pixels'] == 0
    assert result.metrics['current_train_test_overlap_pixels'] == 0
    assert result.metrics['reference_preselected_test_cells'] == 48


@pytest.mark.parametrize('source', SOURCES)
@pytest.mark.parametrize('name', ['central_logo_peripheral_permutation', 'different_lettering_same_palette',
                                  'hue_shift', 'brightness_inversion', 'textureless', 'mask_intrusion', 'empty_mask'])
def test_actual_h_proposal_cannot_admit_challenged_content(reference, source, name):
    result = run(reference, source, challenge(load_frame(source), name))
    assert not result.appearance_match, (name, result.code, result.metrics)


@pytest.mark.parametrize('source', SOURCES)
def test_independent_reference_texture_permutation_rejected(reference, source):
    image = REFERENCE_INPUT['color'].copy()
    selected = REFERENCE_INPUT['recorded_refined_mask'] > 0
    image[selected] = image[selected][np.random.RandomState(901).permutation(np.count_nonzero(selected))]
    other = candidate.freeze_reference(image, REFERENCE_INPUT['recorded_refined_mask'],
                                       REFERENCE_INPUT['rgb_valid'], LAST['source_stamp_ns'])
    assert other.digest != reference.digest
    assert not run(other, source, load_frame(source)).appearance_match


@pytest.mark.parametrize('source', SOURCES)
def test_wrong_homography_rejected(reference, source):
    wrong = H_MAP[source].copy()
    shift = np.eye(3); shift[0, 2] = 30.
    result = run(reference, source, load_frame(source), matrix=shift @ wrong)
    assert not result.appearance_match
    assert result.code == 'WHOLE_MASK_INCONSISTENT'


def test_histogram_pass_preserves_original_path_and_ignores_fallback_proposal(reference):
    result = candidate.evaluate_appearance(reference, reference.bgr, reference.mask, reference.valid,
                                           reference.source_stamp_ns+1, None,
                                           deadline_monotonic=1., clock=lambda: 0.)
    assert result.appearance_match and not result.fallback_attempted
    assert result.code == 'ORIGINAL_HISTOGRAM_PASS'
    assert 'holdout_ncc' not in result.metrics


def test_reference_is_immutable_and_detached():
    image = REFERENCE_INPUT['color'].copy()
    ref = candidate.freeze_reference(image, REFERENCE_INPUT['recorded_refined_mask'], REFERENCE_INPUT['rgb_valid'], LAST['source_stamp_ns'])
    original = ref.bgr.copy()
    image[:] = 0
    assert np.array_equal(ref.bgr, original)
    with pytest.raises(ValueError):
        ref.bgr.setflags(write=True)


@pytest.mark.parametrize('kind', ['missing', 'wrong_reference', 'wrong_source'])
def test_proposal_binding_rejected(reference, kind):
    source = SOURCES[1]; frame = load_frame(source)
    proposal = candidate.propose_homography(reference, source, H_MAP[source])
    if kind == 'missing': proposal = None
    if kind == 'wrong_reference': proposal = candidate.HomographyProposal('0'*64, source, proposal.matrix)
    if kind == 'wrong_source': proposal = candidate.HomographyProposal(reference.digest, source-1, proposal.matrix)
    result = candidate.evaluate_appearance(reference, frame['color'], frame['mask'], frame['rgb_valid'], source,
                                           proposal, deadline_monotonic=1., clock=lambda: 0.)
    assert not result.appearance_match and result.code == 'HOMOGRAPHY_SOURCE_BINDING_MISMATCH'


@pytest.mark.parametrize('stage', ['entry', 'before_dense', 'inside_fit', 'histogram_return'])
def test_absolute_budget_never_accepts_late_result(reference, monkeypatch, stage):
    source = SOURCES[1]; frame = load_frame(source)
    now = [0.]
    def expire_after(function):
        def wrapped(*args, **kwargs):
            value = function(*args, **kwargs)
            now[0] = 2.
            return value
        return wrapped
    if stage == 'entry': now[0] = 2.
    if stage == 'before_dense':
        monkeypatch.setattr(candidate, '_mask_geometry', expire_after(candidate._mask_geometry))
    if stage == 'inside_fit':
        monkeypatch.setattr(candidate.np.linalg, 'lstsq', expire_after(candidate.np.linalg.lstsq))
    if stage == 'histogram_return':
        frame = dict(color=reference.bgr, mask=reference.mask, rgb_valid=reference.valid)
        monkeypatch.setattr(candidate.cv2, 'matchShapes', expire_after(candidate.cv2.matchShapes))
    result = run(reference, source, frame, clock=lambda: now[0])
    assert not result.appearance_match and result.code == 'DEADLINE_EXCEEDED'


def test_invalid_current_mask_never_consumes_rgb(reference):
    source = SOURCES[1]; frame = load_frame(source)
    yy, xx = np.nonzero(frame['mask'])
    frame['rgb_valid'][yy[0], xx[0]] = False
    result = run(reference, source, frame)
    assert not result.appearance_match and result.code == 'MASK_CONSUMES_INVALID_RGB'


@pytest.mark.parametrize('matrix', [np.zeros((3, 3)), np.array([[-1.,0,840],[0,1,0],[0,0,1]]), np.array([[1.,0,0],[0,1,0],[.1,0,-40.]])])
def test_singular_mirror_or_horizon_rejected(reference, matrix):
    result = run(reference, SOURCES[1], load_frame(SOURCES[1]), matrix=matrix)
    assert not result.appearance_match


def test_identical_histogram_different_texture_original_path_limit_is_explicit(reference):
    image = reference.bgr.copy()
    selected = reference.mask > 0
    image[selected] = image[selected][np.random.RandomState(901).permutation(np.count_nonzero(selected))]
    result = candidate.evaluate_appearance(reference, image, reference.mask, reference.valid,
                                           reference.source_stamp_ns+1, None,
                                           deadline_monotonic=1., clock=lambda: 0.)
    # Existing histogram pathway remains unchanged; this candidate must not be
    # advertised as a universal rejection of all same-colour alternate objects.
    assert result.appearance_match and result.code == 'ORIGINAL_HISTOGRAM_PASS'


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_dense_correlation_cannot_pass(reference, monkeypatch, value):
    monkeypatch.setattr(candidate, '_ncc', lambda x, y: value)
    result = run(reference, SOURCES[1], load_frame(SOURCES[1]))
    assert not result.appearance_match and result.code == 'NONFINITE_OR_DEGENERATE_DENSE_EVIDENCE'


def test_good_texture_must_cover_three_quadrants(reference, monkeypatch):
    from collections import Counter
    from itertools import combinations
    source = SOURCES[2]; frame = load_frame(source)
    prior = run(reference, source, frame)
    rows = prior.metrics['test_cell_metrics']
    counts = Counter(tuple(row['quadrant']) for row in rows)
    selected_pair = max(combinations(counts, 2), key=lambda pair: sum(counts[q] for q in pair))
    values = iter([prior.metrics['holdout_ncc']] + [.9 if tuple(row['quadrant']) in selected_pair else .6 for row in rows])
    monkeypatch.setattr(candidate, '_ncc', lambda x, y: next(values))
    result = run(reference, source, frame)
    assert result.metrics['observed_test_quadrants'] == 4
    assert result.metrics['good_test_quadrants'] == 2
    assert result.metrics['good_test_cells'] >= result.metrics['required_good_test_cells']
    assert not result.appearance_match and result.code == 'DISTRIBUTED_TEXTURE_NOT_CONFIRMED'


@pytest.mark.parametrize('policy,expected', [
    (candidate.HandoffPolicy(max_colour_distance=.5), 'ORIGINAL_HISTOGRAM_PASS'),
    (candidate.HandoffPolicy(max_colour_distance=.1), 'CHROMA_MISMATCH'),
    (candidate.HandoffPolicy(max_contour_distance=.01), 'ORIGINAL_SHAPE_MISMATCH'),
])
def test_uses_existing_handoff_policy(reference, policy, expected):
    source = SOURCES[1]; frame = load_frame(source)
    proposal = candidate.propose_homography(reference, source, H_MAP[source])
    result = candidate.evaluate_appearance(reference, frame['color'], frame['mask'], frame['rgb_valid'], source,
                                           proposal, deadline_monotonic=1., clock=lambda: 0., policy=policy)
    assert result.code == expected
