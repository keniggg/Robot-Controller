#!/usr/bin/env python3
"""Stratify the bounded conditional shadow sample without estimating success rate."""
import argparse
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('regional_same_read_analysis',
                                             HERE/'analyze_regional_shadow_20261005.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)


def center_benefit(value):
    return bool(value.get('read_complete') is True
        and value.get('shadow_audit_available') is True
        and value.get('global_active') is False and value.get('center_active') is True)


def current_stratum(value):
    if value.get('read_complete') is not True or value.get('shadow_audit_available') is not True:
        return 'current_condition_unavailable'
    if center_benefit(value):
        return 'actual_center_benefit_condition'
    if value.get('global_active') is True and value.get('center_active') is False:
        return 'actual_reverse_condition'
    if value.get('global_active') is True and value.get('center_active') is True:
        return 'both_active'
    if value.get('global_active') is False and value.get('center_active') is False:
        return 'both_inactive'
    return 'current_condition_unavailable'


def condition_audit(info):
    kind = info.get('selection_kind')
    previous, actual = info.get('previous_condition'), info.get('actual_condition', {})
    reasons = []
    if kind not in ('uniform_baseline', 'previous_global_false_center_true'):
        reasons.append('unknown_selection_kind')
    if (info.get('selection_uses_previous_completed_read_only') is not True
            or info.get('current_condition_used_to_reject_sample') is not False):
        reasons.append('selection_scope_not_recorded')
    if kind == 'previous_global_false_center_true' and not center_benefit(previous or {}):
        reasons.append('conditional_sample_previous_condition_not_qualified')
    diag = info.get('production_read_diagnostics', {})
    complete = info.get('complete') is True and diag.get('result') == 'complete'
    if actual.get('read_complete') is not complete:
        reasons.append('current_completion_differs')
    audit = info.get('regional_shadow')
    if complete and isinstance(audit, dict):
        center = next((r for r in audit.get('regions', []) if r.get('name') == 'center'), None)
        center_active = (center.get('qualified') is True and center.get('filter_applied') is True) if center else None
        global_active = info.get('runtime_profile', {}).get('stationary_temporal', {}).get('active') is True
        raw = diag.get('raw_frames', {}).get('color')
        if (actual.get('shadow_audit_available') is not True
                or actual.get('global_active') is not global_active
                or actual.get('center_active') is not center_active
                or actual.get('sdk_color_source') != raw
                or info.get('shadow_current_sdk_color_source') != raw
                or info.get('published_ros_image_stamp_available_at_tap') is not False):
            reasons.append('current_condition_does_not_bind_actual_tap')
        if kind == 'previous_global_false_center_true' and isinstance(previous, dict):
            old = previous.get('sdk_color_source') or {}
            try:
                if not (old['frame_number'] < raw['frame_number']
                        and old['device_timestamp_ms'] < raw['device_timestamp_ms']
                        and old['timestamp_domain'] == raw['timestamp_domain']):
                    raise ValueError('previous source ordering')
            except (KeyError, TypeError, ValueError):
                reasons.append('previous_condition_source_not_before_current_source')
    return dict(valid=not reasons, contradictions=reasons, selection_kind=kind,
        current_stratum=current_stratum(actual), actual_condition=actual,
        previous_condition=previous,
        current_condition_never_substituted_with_previous=True)


def counters_audit(summary):
    counts = summary.get('conditioned_read_counts', {})
    tables = summary.get('global_center_cross_table', {})
    regions = summary.get('region_qualification_reason_counts', {})
    reasons = []
    outcomes = ('complete', 'rejected', 'original_error')
    try:
        if (any(type(value) is not int or value < 0 for value in counts.values())
                or sum(counts[k] for k in outcomes) != counts['reads']
                or counts['audit_unavailable'] > counts['reads']
                or counts['baseline_selected'] > 2 or counts['conditional_selected'] > 6
                or counts['baseline_selected']+counts['conditional_selected'] != summary['counts']['selected']
                or summary.get('baseline_samples') != 2 or summary.get('conditional_samples_maximum') != 6
                or summary.get('current_condition_never_filters_selected_sample') is not True):
            raise ValueError('conditioned aggregate contract')
        for outcome in outcomes:
            table = tables[outcome]
            if (sum(table.values()) != counts[outcome]
                    or any(type(v) is not int or v < 0 for v in table.values())):
                raise ValueError('cross table count')
            for reasons_ in regions[outcome].values():
                if (sum(reasons_.values()) > counts[outcome]
                        or any(type(v) is not int or v < 0 for v in reasons_.values())):
                    raise ValueError('regional reason count')
    except (KeyError, TypeError, ValueError):
        reasons.append('recorded_conditional_counters_inconsistent')
    return dict(valid=not reasons, contradictions=reasons, counts=counts,
        full_read_cross_table=tables, full_read_region_reason_counts=regions,
        scope='Recorded full-read gate counts only; no support or geometry pass rate can be inferred for unsaved reads.')


def analyze(capture, config):
    report = base.analyze(capture, config)
    folder = Path(report['capture'])
    summary = json.loads((folder/'summary.json').read_text())
    report['conditional_full_read_audit'] = counters_audit(summary)
    report['conditional_analysis_source_sha256'] = base.support.sha(__file__)
    report['conditional_sampling_estimates_population_success_rate'] = False
    for row in report['rows']:
        info, _ = base.support.read_sample(folder, 'sample_%02d' % row['index'])
        row['conditional_sampling'] = condition_audit(info)
    strata = {
        'uniform_baseline': lambda c:c['selection_kind'] == 'uniform_baseline',
        'condition_triggered_all_current_outcomes': lambda c:c['selection_kind'] == 'previous_global_false_center_true',
        'actual_center_benefit_condition': lambda c:c['current_stratum'] == 'actual_center_benefit_condition',
        'actual_reverse_condition': lambda c:c['current_stratum'] == 'actual_reverse_condition',
        'both_active': lambda c:c['current_stratum'] == 'both_active',
        'both_inactive': lambda c:c['current_stratum'] == 'both_inactive',
        'current_condition_unavailable': lambda c:c['current_stratum'] == 'current_condition_unavailable',
    }
    report['sample_strata'] = {}
    for name, accepts in strata.items():
        rows = [r for r in report['rows'] if r['conditional_sampling']['valid'] and accepts(r['conditional_sampling'])]
        report['sample_strata'][name] = dict(indices=[r['index'] for r in rows], comparisons=base.summaries(rows))
    report['conditional_sample_contradictions'] = [dict(index=r['index'], reasons=r['conditional_sampling']['contradictions'])
                                                  for r in report['rows'] if not r['conditional_sampling']['valid']]
    report['limitations'].extend([
        'Two uniform baseline samples and up to six previous-read-triggered samples are intentionally stratified. Their combined support/geometry pass counts are not an unbiased estimate of full-stream performance.',
        'The trigger describes the previous completed read. Actual current qualification is audited separately; a sampled current failure is retained and never silently excluded as an unsuccessful trigger.',
        'Actual center-benefit condition means only global inactive and center active. It does not assert that regional support or geometry passed or improved.',
        'Reverse-condition counts can exist in the full-read cross table without a saved depth pair. No geometry result is imputed to such unsaved reads.',
        'All sample strata retain the base analyzer strict TF/feedback failures. Conditional selection grants no new source, target, geometry or motion authority.'])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not base.support.MOUNT.is_mount() or base.support.EXT not in args.output.resolve().parents:
        raise ValueError('analysis output requires mounted SanStand; no fallback')
    if args.output.exists():
        raise ValueError('new output path required')
    report = analyze(args.capture, args.config)
    with args.output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(output=str(args.output), sha256=base.support.sha(args.output),
                         sample_strata=report['sample_strata'],
                         full_read_audit=report['conditional_full_read_audit']), ensure_ascii=False))


if __name__ == '__main__':
    main()
