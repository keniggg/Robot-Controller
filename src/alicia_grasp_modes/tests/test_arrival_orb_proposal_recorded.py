import cv2
import numpy as np
import pytest

from alicia_grasp_modes import arrival_orb_proposal as helper
from alicia_grasp_modes import arrival_dense_identity as dense
from test_arrival_dense_identity_recorded import reference, SOURCES, H_MAP, load_frame


@pytest.mark.parametrize('source', SOURCES)
def test_real_fixed_proposal_reproduces_saved_h_without_identity_authority(reference, source):
    frame = load_frame(source)
    proposal, evidence = helper.propose_orb_homography(reference, frame['color'], frame['mask'], frame['rgb_valid'], source,
                                                      deadline_monotonic=1., clock=lambda: 0.)
    assert evidence.code == 'H_PROPOSED_ONLY'
    assert not evidence.appearance_match and not evidence.motion_authorized and not evidence.metric_geometry_authority
    assert proposal.reference_digest == reference.digest and proposal.current_source_stamp_ns == source
    assert np.allclose(proposal.matrix/proposal.matrix[2, 2], H_MAP[source]/H_MAP[source][2, 2], rtol=0., atol=1e-10)
    result = dense.evaluate_appearance(reference, frame['color'], frame['mask'], frame['rgb_valid'], source,
                                       proposal, deadline_monotonic=1., clock=lambda: 0.)
    assert result.appearance_match == (source not in {1791638841807271480, 1791638843276834011})


def test_flat_content_has_no_authorized_proposal(reference):
    source = SOURCES[1]; frame = load_frame(source)
    image = np.full_like(frame['color'], 180)
    proposal, evidence = helper.propose_orb_homography(reference, image, frame['mask'], frame['rgb_valid'], source,
                                                      deadline_monotonic=1., clock=lambda: 0.)
    assert proposal is None and not evidence.appearance_match


@pytest.mark.parametrize('stage', ['entry', 'ransac_return'])
def test_proposal_budget_counts_actual_estimation_work(reference, monkeypatch, stage):
    source = SOURCES[1]; frame = load_frame(source)
    now = [2. if stage == 'entry' else 0.]
    if stage == 'ransac_return':
        original = helper.cv2.findHomography
        def delayed(*args, **kwargs):
            value = original(*args, **kwargs)
            now[0] = 2.
            return value
        monkeypatch.setattr(helper.cv2, 'findHomography', delayed)
    proposal, evidence = helper.propose_orb_homography(reference, frame['color'], frame['mask'], frame['rgb_valid'], source,
                                                      deadline_monotonic=1., clock=lambda: now[0])
    assert proposal is None and evidence.code == 'DEADLINE_EXCEEDED'


def test_invalid_rgb_mask_does_not_produce_h(reference):
    source = SOURCES[1]; frame = load_frame(source)
    yy, xx = np.nonzero(frame['mask'])
    frame['rgb_valid'][yy[0], xx[0]] = False
    proposal, evidence = helper.propose_orb_homography(reference, frame['color'], frame['mask'], frame['rgb_valid'], source,
                                                      deadline_monotonic=1., clock=lambda: 0.)
    assert proposal is None and evidence.code == 'INVALID_PROPOSAL_INPUT'
