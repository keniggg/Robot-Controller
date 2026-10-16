"""Supplemental observation provenance only; no geometry or motion authority.

The original RGB-D timestamps, final plan and registration are never rewritten.
Consumers must match these witnesses to their own previously registered inputs
and independently check their current, unexpired station before freezing.
"""
from copy import deepcopy
import math

from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_flexible_grasp.grasp.rich_plan_integrity import (
    compute_plan_id, validate_plan_header_binding)
from .station_compute_reacquisition import validate_registration

KEY = 'station_supplemental_observation_computation'


def build_receipt(plan, *, ticket, source_witnesses, current_witness,
                  issued_ns, snapshot_policy):
    value = dict(version=1, scope='registered_station_supplemental_observation_provenance',
        motion_authorized=False, metric_geometry_authority=False,
        plan_id=plan.plan_id, plan_sha256=committed_plan_digest(plan),
        plan_source_ns=plan.header.stamp.to_nsec(), ticket=deepcopy(ticket),
        sources=deepcopy(source_witnesses), current=deepcopy(current_witness),
        issued_ns=int(issued_ns), snapshot_policy=deepcopy(snapshot_policy))
    validate_receipt(value, plan, now_ns=issued_ns)
    return value


def validate_receipt(value, plan, *, now_ns):
    """Check wire integrity, not current geometry or execution permission."""
    try:
        validate_plan_header_binding(plan)
        if compute_plan_id(plan) != plan.plan_id:
            raise ValueError('station_supplemental_receipt_noncanonical_plan')
        if (set(value) != {'version','scope','motion_authorized','metric_geometry_authority',
                'plan_id','plan_sha256','plan_source_ns','ticket','sources','current',
                'issued_ns','snapshot_policy'}
                or type(value['version']) is not int or value['version'] != 1
                or value['scope'] != 'registered_station_supplemental_observation_provenance'
                or value['motion_authorized'] is not False
                or value['metric_geometry_authority'] is not False
                or plan.valid is not True or plan.diagnostic != 'FAR_FIELD_OBSERVATION_PLAN'
                or plan.model_choice != 'unknown_tabletop'
                or value['plan_id'] != plan.plan_id
                or value['plan_sha256'] != committed_plan_digest(plan)
                or type(value['plan_source_ns']) is not int
                or value['plan_source_ns'] != plan.header.stamp.to_nsec()
                or type(value['issued_ns']) is not int or not 0 < value['issued_ns'] <= now_ns):
            raise ValueError('station_supplemental_receipt_plan_changed')
        policy = value['snapshot_policy']
        if (policy.get('purpose') != 'supplementary_observation' or policy.get('phase') != 'observation_prefix'
                or type(policy.get('frame_count')) is not int or not 1 <= policy['frame_count'] <= 64):
            raise ValueError('station_supplemental_receipt_snapshot_policy_changed')
        if (set(policy) != {'purpose', 'phase', 'frame_count', 'maximum_span_sec'}
                or type(policy['maximum_span_sec']) not in (int, float)
                or not math.isfinite(policy['maximum_span_sec']) or policy['maximum_span_sec'] <= 0):
            raise ValueError('station_supplemental_receipt_snapshot_policy_changed')
        sources=value['sources']; current=value['current']; ticket=value['ticket']
        if not isinstance(sources, list) or len(sources) != policy['frame_count']:
            raise ValueError('station_supplemental_receipt_configured_sources_required')
        binding=current['contract']['binding']
        for row in sources + [current]:
            if (set(row) != {'contract','object_sha256','mask'}
                    or not isinstance(row['object_sha256'], str) or len(row['object_sha256']) != 64
                    or any(c not in '0123456789abcdef' for c in row['object_sha256'])
                    or row['contract']['binding'] != binding):
                raise ValueError('station_supplemental_receipt_source_binding_changed')
            validate_registration(row['contract'], binding=binding,
                now_ns=row['contract']['source_ns'])
        stamps=[row['contract']['source_ns'] for row in sources]
        if (not all(a < b for a,b in zip(stamps,stamps[1:]))
                or stamps[-1] != value['plan_source_ns']
                or stamps[-1] > current['contract']['source_ns']
                or stamps[-1]-stamps[0] > round(float(policy['maximum_span_sec'])*1e9)
                or set(ticket) != {'request_id','generation','target_epoch'}
                or any(type(ticket[k]) is not int or ticket[k] <= 0
                    for k in ('request_id', 'generation'))
                or type(ticket['target_epoch']) is not int or ticket['target_epoch'] < 0
                or ticket['target_epoch'] != binding['target_epoch']
                or plan.target_track_id != binding['target_track_id']):
            raise ValueError('station_supplemental_receipt_snapshot_changed')
        # This is an issuance witness only. It never renews this source's
        # clocks; a later consumer must independently have a live registration.
        validate_registration(current['contract'], binding=binding, now_ns=value['issued_ns'])
        if now_ns >= min(current['contract']['phase_deadline_ns'], current['contract']['session_deadline_ns']):
            raise ValueError('station_supplemental_receipt_phase_expired')
        return deepcopy(binding)
    except (KeyError, TypeError, AttributeError, OverflowError) as exc:
        raise ValueError('station_supplemental_receipt_malformed') from exc
