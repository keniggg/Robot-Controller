"""A station template is the acknowledged third image, never rolling geometry."""
from copy import deepcopy
import hashlib

import numpy as np
import pytest

from test_observation_measurement_unknown import frozen, arrival, next_frame, send, ack
from test_arrival_fragment_group import completed_fragment_arrival, fragment_frame
from test_locked_rgb_hold import module, latest_status
from alicia_grasp_modes.observation_appearance import FrozenObservationAppearance, match_observation_appearance


def test_only_real_qualified_receipt_freezes_third_measurement_and_preserves_original_three_frame_identity(monkeypatch):
    n, clock, _, joint, params, request = frozen(monkeypatch)
    premotion = n._observation_measurement['confirmation'].appearance_reference
    request = arrival(n, clock, joint, request)
    for index in range(3):
        assert n._observation_measurement['confirmation'].appearance_reference is premotion
        next_frame(n, clock, joint)
        assert (n._locked_rgb_hold_reference or {}).get('observation_arrival_retry') is None
    sources = ack(n)['source_stamps_ns']
    third = n._locked_rgb_hold_reference
    clock[0] += .01
    qualified = dict(request, state='qualified', request_id='completed',
        stamp_ns=module.rospy.Time.now().to_nsec(), source_stamps_ns=sources)
    send(n, qualified)
    assert ack(n)['accepted']
    marker = third['observation_arrival_retry']
    template = marker['station_appearance']
    assert template.source_ns == sources[-1] == third['source_stamp_ns']
    assert marker['fragment_appearance_reference'] is premotion
    provenance = marker['station_appearance_provenance']
    assert provenance['source_stamp_ns'] == sources[-1]
    assert provenance['premotion_source_stamp_ns'] == premotion.source_ns
    assert provenance['qualified_source_stamps_ns'] == sources
    assert provenance['measurement_binding'] == marker['measurement_binding']
    assert provenance['source_rgb_sha256'] == template.rgb_sha256 == third['source_rgb_sha256']
    assert provenance['refined_mask_sha256'] == third['refined_mask_sha256']
    assert provenance['normalized_mask_sha256'] == template.mask_sha256
    assert template is not premotion
    np.testing.assert_array_equal(template.bgr, third['bgr'])
    np.testing.assert_array_equal(template.mask > 0, third['mask'] > 0)
    np.testing.assert_array_equal(template.histogram, third['histogram'])
    for array in (template.bgr, template.mask, template.histogram, template.contour):
        assert not array.flags.writeable
        with pytest.raises(ValueError): array.flags.writeable = True
    with pytest.raises(AttributeError): template.source_ns += 1


def test_three_normal_fragment_frames_keep_one_frozen_template_but_publish_only_current_measured_geometry(monkeypatch):
    n, clock, _, joint, _, marker = completed_fragment_arrival(monkeypatch)
    template = marker['station_appearance']
    reference_digest = hashlib.sha256(template.bgr.tobytes()+template.mask.tobytes()).hexdigest()
    original_deadline = marker['session_deadline_ns']
    original_provenance = deepcopy(marker['station_appearance_provenance'])
    sources = []
    for _ in range(3):
        pair = fragment_frame(n, clock, joint)
        sources.append(pair[0].header.stamp.to_nsec())
        status = latest_status(n)
        assert status['state'] == 'ready', status
        hold = n._locked_rgb_hold_reference
        assert hold['source_stamp_ns'] == sources[-1] > template.source_ns
        assert hold['observation_arrival_retry'] is marker
        assert marker['station_appearance'] is template
        assert marker['session_deadline_ns'] == original_deadline
        assert marker['station_appearance_provenance'] == original_provenance
        audit = n.tracker.last.metrics['observation_fragment_group']['appearance_identity']
        assert audit['reference_source_stamp_ns'] == template.source_ns
        assert audit['reference_role'] == 'qualified_station_identity_template'
        assert audit['same_station_mask_iou'] >= .85
        assert audit['metric_geometry_authority'] is False and audit['motion_authorized'] is False
    assert sources == sorted(set(sources))
    assert hashlib.sha256(template.bgr.tobytes()+template.mask.tobytes()).hexdigest() == reference_digest


@pytest.mark.parametrize('fault', ['current_source_template', 'invalid_type', 'qualified_source_changed',
                                   'same_source_wrong_rgb', 'same_source_wrong_mask'])
def test_wrong_template_cannot_be_substituted_by_current_frame(monkeypatch, fault):
    n, clock, _, joint, _, marker = completed_fragment_arrival(monkeypatch)
    template = marker['station_appearance']
    if fault == 'current_source_template':
        marker['station_appearance'] = FrozenObservationAppearance(template.bgr, template.mask,
            template.valid, template.source_ns+1, template.histogram, template.contour)
    elif fault in ('same_source_wrong_rgb', 'same_source_wrong_mask'):
        rgb, mask = template.bgr.copy(), template.mask.copy()
        if fault == 'same_source_wrong_rgb': rgb[0, 0, 0] ^= 1
        else:
            yy, xx = np.nonzero(mask)
            mask[yy[0], xx[0]] = 0
        marker['station_appearance'] = FrozenObservationAppearance(rgb, mask,
            template.valid, template.source_ns, template.histogram, template.contour)
    elif fault == 'invalid_type': marker['station_appearance'] = object()
    else: marker['qualified_source_stamps_ns'][-1] += 1
    old = n.last_good_source_stamp_ns
    fragment_frame(n, clock, joint)
    assert n.last_good_source_stamp_ns == old
    assert not n.pubs['object'].publish.call_args[0][0].detected
    assert 'appearance_binding_changed' in latest_status(n)['detail']


def test_same_station_complete_outline_is_additional_gate_not_a_colour_bypass(monkeypatch):
    n, clock, _, joint, _, marker = completed_fragment_arrival(monkeypatch)
    template = marker['station_appearance']
    calls = []
    context = dict(reference=template, source_ns=template.source_ns+1,
        same_station_mask_iou_minimum=.85, reference_role='qualified_station_identity_template',
        check_current=lambda: calls.append('checked'))
    partial = template.mask.copy()
    yy, xx = np.nonzero(partial)
    partial[yy[len(yy)//2:], xx[len(xx)//2:]] = 0
    result = match_observation_appearance(template.bgr, partial, template.histogram,
        template.contour, appearance_context=context)
    assert result['appearance_match'] is False
    assert result['code'] == 'SAME_STATION_COMPLETE_OUTLINE_MISMATCH'
    assert result['same_station_mask_iou'] < .85 and calls
    bad = dict(context, same_station_mask_iou_minimum=.5)
    with pytest.raises(ValueError, match='context_invalid'):
        match_observation_appearance(template.bgr, template.mask, template.histogram,
            template.contour, appearance_context=bad)
