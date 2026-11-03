"""One new stationary input window, never a retry of rejected motion.

Only a complete, zero-path search with exclusively missing surface evidence
may use this receipt. Geometry contradictions and path failures stay terminal.
The caller owns original context checks and the physical worker completion.
"""
from copy import deepcopy

from .station_compute_reacquisition import validate_contract, validate_registration

CODE = 'STATION_CONTACT_NEW_INPUT_PENDING'
SOURCE = 'station_contact_new_input_pending'
FAILURE = 'NEAR_FIELD_SURFACE_VIEW_REQUIRED'
POLICY = 'one_stationary_input_window_after_incomplete_surface'
BOX_CODES = frozenset(('BOX_TOP_CONTOUR_INCOMPLETE',
                       'BOX_INFERENCE_PRECISION_VIEW_REQUIRED'))
LOCAL_CODES = frozenset(('BILATERAL_SURFACE_EVIDENCE_MISSING',
                         'GRASPNET_STAGE_PROFILE_UNAVAILABLE'))


def completed_missing_evidence(report, prepared, selection):
    """An outer missing-side code cannot hide a conflicting model or collision."""
    try:
        ticket = prepared.ticket
        if (report['grasp_mode_selection'] != selection
                or report['request_id'] != ticket.request_id
                or report['generation'] != ticket.generation
                or report['target_epoch'] != ticket.target_epoch
                or report['snapshot_stamp_ns'] != prepared.snapshot.stamp_ns
                or report['selected'] is not None
                or report['outcome']['code'] != FAILURE
                or report['outcome']['valid_plan'] is not False
                or report['outcome']['preview_valid'] is not False
                or getattr(prepared, 'remote_failure_code', '')):
            return False
        search = report['moveit_selection']
        if (search['terminated_early'] is not False
                or any(type(search[k]) is not int or search[k] != 0
                       for k in ('checked_count', 'shortlist_count', 'reachable_count'))):
            return False
        funnel = report['pipeline_funnel']
        counts = funnel['stage_counts']['locally_valid']
        if (any(type(counts[k]) is not int for k in ('entered', 'passed', 'rejected'))
                or counts['passed'] != 0 or counts['entered'] <= 0
                or counts['entered'] != counts['rejected']):
            return False
        failures = funnel['rejection_counts']
        if (not failures or not set(failures) <= LOCAL_CODES
                or sum(failures.values()) != counts['rejected']
                or any(type(v) is not int or v <= 0 for v in failures.values())):
            return False
        geometry = report['remote_diagnostics']['tabletop_geometry']
        if (geometry['failure_code'] != 'BILATERAL_SURFACE_EVIDENCE_MISSING'
                or geometry['plan_phase'] != 'CONTACT_EXECUTION_PLAN'
                or geometry['contact_execution_gate_deferred'] is not False
                or geometry['materialized_count'] != 0
                or geometry['regular_box_inference']['code'] not in BOX_CODES
                or not set(geometry['rejection_counts']) <= {'BILATERAL_SURFACE_EVIDENCE_MISSING'}):
            return False
        return True
    except (KeyError, TypeError, ValueError, AttributeError):
        return False


def validate_receipt(value, contract, *, now_ns, header_ns, known_source):
    """Join independently registered sources; this does not refresh any clock."""
    try:
        fields = {'version', 'policy', 'code', 'execution_authority',
            'metric_geometry_authority', 'binding', 'phase_deadline_ns',
            'session_deadline_ns', 'request', 'emitted_ns', 'sources', 'current',
            'audit_sha256', 'maximum_new_windows'}
        if (not isinstance(value, dict) or set(value) != fields
                or type(value['version']) is not int or value['version'] != 1
                or value['policy'] != POLICY or value['code'] != CODE
                or value['execution_authority'] is not False
                or value['metric_geometry_authority'] is not False
                or type(value['maximum_new_windows']) is not int
                or value['maximum_new_windows'] != 1
                or value['binding'] != contract['binding']):
            raise ValueError('station_new_input_binding_changed')
        for key in ('phase_deadline_ns', 'session_deadline_ns'):
            if type(value[key]) is not int or value[key] != contract[key]:
                raise ValueError('station_new_input_deadline_changed')
        emitted = value['emitted_ns']
        if (any(type(v) is not int for v in (emitted, now_ns, header_ns))
                or emitted != header_ns
                or not contract['binding']['phase_started_ns'] < emitted <= now_ns
                    < min(contract['phase_deadline_ns'], contract['session_deadline_ns'])):
            raise ValueError('station_new_input_outside_phase')
        request = value['request']
        if (not isinstance(request, dict) or set(request) !=
                {'request_id', 'generation', 'target_epoch', 'snapshot_stamp_ns'}
                or any(type(v) is not int or v <= 0 for v in request.values())
                or request['target_epoch'] != contract['binding']['target_epoch']):
            raise ValueError('station_new_input_request_changed')
        digest = value['audit_sha256']
        if (not isinstance(digest, str) or len(digest) != 64
                or any(c not in '0123456789abcdef' for c in digest)):
            raise ValueError('station_new_input_audit_missing')
        sources, current = value['sources'], value['current']
        if not isinstance(sources, list) or len(sources) != 3:
            raise ValueError('station_new_input_requires_three_sources')
        for row in sources + [current]:
            source = row['contract']['source_ns']
            validate_registration(row['contract'], binding=value['binding'], now_ns=source)
            if row != known_source(source):
                raise ValueError('station_new_input_source_not_independently_known')
        stamps = [row['contract']['source_ns'] for row in sources]
        if (not all(a < b for a, b in zip(stamps, stamps[1:]))
                or not contract['binding']['phase_reference_source_ns'] <= stamps[0]
                or stamps[-1] != request['snapshot_stamp_ns']
                or not stamps[-1] <= current['contract']['source_ns'] <= emitted):
            raise ValueError('station_new_input_source_window_changed')
        validate_contract(current['contract'], binding=value['binding'], now_ns=emitted)
        return deepcopy(value)
    except (KeyError, TypeError, AttributeError, OverflowError) as exc:
        raise ValueError('station_new_input_receipt_malformed') from exc
