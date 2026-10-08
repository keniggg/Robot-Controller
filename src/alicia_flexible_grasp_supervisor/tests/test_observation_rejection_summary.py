"""Mixed view failures must not be reported as one preferred collision row."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import pytest

from test_observation_distance_search import remote


def test_information_rejections_are_not_counted_as_support_collisions():
    # This captures the gen9 failure shape: a few close poses fail CAD;
    # other poses clear CAD but fail to expose the missing measured side.
    rows = [dict(failure_code='OBSERVATION_SUPPORT_COLLISION',
                 minimum_support_clearance_m=.00185),
            dict(failure_code='OBSERVATION_SUPPORT_COLLISION',
                 minimum_support_clearance_m=.00222)]
    rows += [dict(failure_code='OBSERVE_RETURN_MISSING_SIDE_NOT_EXPOSED')
             for _ in range(88)]
    before = deepcopy(rows)
    result = remote.summarize_observation_view_rejections(rows)
    assert result['rejected_count'] == 90
    assert result['failure_code_counts'] == {
        'OBSERVATION_SUPPORT_COLLISION': 2,
        'OBSERVE_RETURN_MISSING_SIDE_NOT_EXPOSED': 88,
    }
    assert result['rejection_stage_counts'] == {
        'endpoint_geometry': 2, 'information_gain': 88,
    }
    assert result['clearance_evidence_count'] == 2
    assert result['clearance_missing_count'] == 88
    assert result['clearance_range_m'] == [.00185, .00222]
    assert result['clearance_scope'] == 'only_rejected_rows_with_computed_clearance'
    assert result['ranking_performed'] is False
    assert result['motion_planning_performed'] is False
    assert result['contact_or_motion_authorized'] is False
    assert rows == before
    assert not any('best' in key for key in result)


def test_later_rejections_retain_passing_endpoint_evidence_without_authorizing_it():
    result = remote.summarize_observation_view_rejections([
        dict(failure_code='OBSERVE_RETURN_MISSING_SIDE_NOT_EXPOSED',
             rejection_stage='information_gain',
             observation_envelope=dict(ok=True, minimum_support_clearance_m=.0893)),
        dict(failure_code='OBSERVATION_WORKING_VOLUME_INVALID'),
        dict(reason='OBSERVE_RETURN_VIEW_BASELINE_INSUFFICIENT'),
    ])
    assert result['rejection_stage_counts'] == {
        'information_gain': 1, 'working_volume': 1, 'view_baseline': 1,
    }
    assert result['failure_code_counts']['OBSERVE_RETURN_VIEW_BASELINE_INSUFFICIENT'] == 1
    assert result['clearance_evidence_count'] == 1
    assert result['clearance_range_m'] == [.0893, .0893]
    assert 'OBSERVATION_SUPPORT_COLLISION' not in result['failure_code_counts']
    assert result['contact_or_motion_authorized'] is False


@pytest.mark.parametrize('clearance', [None, -1e6, float('nan'), float('inf'),
                                     True, '0.002', complex(1., 2.)])
def test_missing_invalid_and_sentinel_clearance_never_becomes_a_measurement(clearance):
    result = remote.summarize_observation_view_rejections([
        dict(failure_code='OBSERVATION_ENVELOPE_INVALID',
             minimum_support_clearance_m=clearance),
    ])
    assert result['clearance_evidence_count'] == 0
    assert result['clearance_missing_count'] == 1
    assert result['clearance_range_m'] is None
    json.dumps(result, allow_nan=False)


def test_empty_or_unspecified_rejections_do_not_invent_collision_evidence():
    assert remote.summarize_observation_view_rejections(())['rejected_count'] == 0
    result = remote.summarize_observation_view_rejections([{}, None])
    assert result['failure_code_counts'] == {'OBSERVATION_REJECTION_UNSPECIFIED': 2}
    assert result['rejection_stage_counts'] == {'unspecified': 2}
    assert result['clearance_range_m'] is None


def test_rejection_audit_requires_same_prepared_source_and_returns_independent_copy():
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    source = NS(ticket=NS(request_id=3, generation=9, target_epoch=26))
    families = [dict(track_id=7, variant_index=0, summary=dict(rejected_count=90))]
    assert node._observation_rejection_audit_for(source) == []
    node._observation_rejection_audit = (source, families)
    report = node._observation_rejection_audit_for(source)
    assert report == families
    report[0]['summary']['rejected_count'] = 1
    assert families[0]['summary']['rejected_count'] == 90
    # Equal-looking ticket fields cannot bind different snapshot/geometry.
    assert node._observation_rejection_audit_for(deepcopy(source)) == []
    assert node._observation_rejection_audit_for(NS(ticket=NS(request_id=4))) == []
