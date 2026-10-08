"""Read-only validation of derived perception receipts back to a fixed index.

No ROS imports, writes, authority mutation or image reads. All filesystem
access and source/context contracts are injected so counterexamples stay pure.
"""
import copy
import math
from pathlib import Path


NO_EFFECTS = ('planning_requested', 'runner_started', 'motion_commands_sent',
              'torque_commands_sent', 'new_user_confirmation_received')


def require(value, reason):
    if not value:
        raise ValueError(reason)


def validate_diagnostic_chain(receipt_path, indexed_receipt_path, indexed_selection,
                              *, read_json, digest_file, resolve_path, parse_selection,
                              digest_context, validate_activation_only,
                              validate_capture_summary, frozen_scripts,
                              confirmation_id, confirmation_source, confirmed_ns,
                              pose_tolerance_rad=.004, max_depth=8):
    """Return immutable-file bindings and oldest-first nodes; never repair a chain.

    The indexed receipt is an explicit termination anchor, not automatically
    the immediately preceding receipt. Each receipt binds the selection *before*
    its own selection report, while expected_selected binds the report's result.
    """
    require(type(max_depth) is int and max_depth > 0, 'invalid_chain_depth_bound')
    require(math.isfinite(pose_tolerance_rad) and pose_tolerance_rad >= 0,
            'invalid_pose_tolerance')
    anchor = resolve_path(indexed_receipt_path)
    indexed_selection = parse_selection(indexed_selection)
    files, visited = {}, set()

    def load(path, expected=None):
        path = resolve_path(path)
        actual = digest_file(path)
        if expected is not None:
            require(actual == expected, 'chain_artifact_hash_changed:' + str(path))
        if str(path) in files:
            require(files[str(path)] == actual, 'chain_artifact_changed_while_reading')
        value = read_json(path)
        require(digest_file(path) == actual, 'chain_artifact_changed_while_reading')
        files[str(path)] = actual
        return path, value

    def feedback(value):
        require(isinstance(value, dict) and value.get('frame_id') == 'sdk_measured',
                'chain_feedback_not_measured')
        q = value.get('positions')
        require(isinstance(q, list) and len(q) == 6 and
                all(type(x) in (int, float) and math.isfinite(x) for x in q),
                'chain_feedback_positions_invalid')
        stamp = value.get('stamp_ns')
        require((type(stamp) is int and stamp > 0) or
                (type(stamp) is str and stamp.isascii() and stamp.isdecimal()
                 and str(int(stamp)) == stamp and int(stamp) > 0),
                'chain_feedback_stamp_invalid')
        return q, int(stamp)

    def visit(path, depth):
        path = resolve_path(path)
        require(path not in visited, 'chain_cycle')
        require(depth < max_depth, 'chain_depth_exceeded')
        visited.add(path)
        path, receipt = load(path)
        event_path, event = load(receipt['event_file'], receipt['event_file_sha256'])
        require(event_path.parent == path.parent, 'chain_event_outside_receipt_directory')
        selection_path, selection = load(path.parent / 'new_target_selection.json')
        require(receipt.get('kind') == 'explicit_user_target_alignment'
                and receipt.get('target_aligned') is True, 'chain_alignment_invalid')
        require(receipt.get('confirmation_id') == confirmation_id
                and receipt.get('confirmed_ns') == confirmed_ns
                and receipt.get('confirmation_source') == confirmation_source,
                'chain_original_confirmation_changed')
        require(event.get('confirmation_source') == confirmation_source
                and event.get('confirmed_ns') == confirmed_ns
                and event.get('target_aligned') is True
                and event.get('robot_commands_sent') is False
                and event.get('torque_commands_sent') is False,
                'chain_event_confirmation_or_effect_changed')
        require(event.get('bound_feedback') == receipt.get('bound_feedback')
                and event.get('derivation') == receipt.get('derivation'),
                'chain_event_receipt_binding_changed')
        feedback(receipt['bound_feedback'])
        before = parse_selection(receipt['selection'])
        context = event['source_context']
        require(parse_selection(context['selection']) == before
                and int(context['epoch_ns']) == int(receipt['epoch_ns'])
                and digest_context(context) == receipt['source_context_sha256'],
                'chain_source_context_binding_changed')
        require(resolve_path(selection['alignment_receipt_file']) == path
                and selection.get('alignment_receipt_sha256') == files[str(path)]
                and selection.get('alignment_receipt') == receipt,
                'chain_selection_receipt_binding_changed')
        require(selection.get('response', {}).get('success') is True
                and not selection.get('failure'), 'chain_selection_failed')
        selected = parse_selection(selection['new_selection'])
        require(parse_selection(selection['previous_selection']) == before
                and before['mode'] == selected['mode'] == 'unknown'
                and before['strategy'] == selected['strategy'] == 'observe_return'
                and selected['generation'] == before['generation'] + 1
                and selected['stamp_ns'] > before['stamp_ns'],
                'chain_selection_edge_invalid')
        node = dict(receipt_path=path, receipt=receipt, event_path=event_path,
                    event=event, selection_path=selection_path,
                    selection_report=selection, selected=selected)
        if path == anchor:
            require(selected == indexed_selection, 'chain_indexed_selection_changed')
            return [node]

        derivation = receipt['derivation']
        require(derivation.get('new_user_confirmation_received') is False,
                'chain_derivative_reissued_confirmation')
        parent_path, _ = load(derivation['parent_receipt'], derivation['parent_receipt_sha256'])
        parents = visit(parent_path, depth + 1)
        parent = parents[-1]
        require(before == parent['selected'], 'chain_parent_selection_changed')
        require(all(receipt[k] == parent['receipt'][k] for k in
                    ('confirmation_id', 'confirmation_source', 'confirmed_ns', 'epoch_ns')),
                'chain_parent_confirmation_or_epoch_changed')
        parent_q, parent_stamp = feedback(parent['receipt']['bound_feedback'])
        child_q, child_stamp = feedback(receipt['bound_feedback'])
        require(child_stamp >= parent_stamp and
                max(abs(a - b) for a, b in zip(child_q, parent_q)) <= pose_tolerance_rad,
                'chain_measured_alignment_changed')
        previous_context = copy.deepcopy(parent['event']['source_context'])
        previous_context['selection'] = before
        activations = validate_activation_only(previous_context, context)
        require(tuple(derivation.get(key) for key in
                    ('old_activation_stamp_ns', 'new_activation_stamp_ns')) == activations,
                'chain_camera_activation_binding_changed')
        sp, _ = load(derivation['prior_selection_record'], derivation['prior_selection_sha256'])
        require(sp == parent['selection_path'], 'chain_prior_selection_path_changed')

        run_path, run = load(path.parent / 'run.json')
        summary_path, summary = load(path.parent / 'raw_support/summary.json')
        node.update(run_path=run_path, run=run, summary_path=summary_path, summary=summary)
        require(run.get('scope') == 'gen%d_to_gen%d_perception_only' %
                    (before['generation'], selected['generation'])
                and run.get('complete') is True and run.get('source_unchanged') is True
                and run.get('capture_returncode') == 0 and not run.get('error'),
                'chain_diagnostic_not_complete')
        for value in (run, selection):
            require(value.get('preview_only') is True
                    and all(value.get(key) is False for key in NO_EFFECTS),
                    'chain_diagnostic_effect_scope_changed')
        script, expected_hash = frozen_scripts[selected['generation']]
        require(run.get('source_sha256', {}).get(script) == expected_hash,
                'chain_diagnostic_script_changed')
        require(resolve_path(run['alignment_receipt']) == path
                and run.get('selection_report') == selection
                and parse_selection(run['new_selection']) == selected
                and run.get('confirmation_id') == confirmation_id
                and run.get('original_confirmed_ns') == confirmed_ns
                and run.get('capture_summary') == summary,
                'chain_diagnostic_binding_changed')
        validate_capture_summary(summary)
        require(summary.get('initial_generation') == before['generation'],
                'chain_capture_initial_generation_changed')
        dp, deployment = load(run['deployment'], run['deployment_sha256'])
        require(resolve_path(derivation['deployment']) == dp
                and derivation['deployment_sha256'] == files[str(dp)],
                'chain_derivation_deployment_changed')
        require(deployment.get('camera_loaded') is True
                and deployment.get('static_validation_passed') is True
                and not deployment.get('error')
                and deployment.get('events', [])[-1]['event'] == 'finished'
                and all(deployment.get(k) is False for k in
                    ('robot_commands_sent', 'torque_commands_sent', 'selection_changed',
                     'alignment_confirmation_reissued', 'new_motion_authority_created')),
                'chain_deployment_effect_or_status_changed')
        require(parse_selection(deployment['current_selection']) == before
                and resolve_path(deployment['alignment_receipt_path']) == parent_path
                and deployment['alignment_receipt_sha256'] == files[str(parent_path)],
                'chain_deployment_parent_changed')
        if 'run' in parent:
            require(resolve_path(run['prior_diagnostic']) == parent_path.parent
                    and run['prior_run_sha256'] == files[str(parent['run_path'])],
                    'chain_prior_run_binding_changed')
            pp, _ = load(derivation['prior_diagnostic_run'], derivation['prior_diagnostic_run_sha256'])
            ps, _ = load(derivation['prior_diagnostic_summary'], derivation['prior_diagnostic_summary_sha256'])
            require(pp == parent['run_path'] and ps == parent['summary_path'],
                    'chain_derivation_prior_diagnostic_changed')
        node['deployment_path'] = dp
        return parents + [node]

    nodes = visit(receipt_path, 0)
    # No accumulation of tolerated per-edge pose changes beyond the original
    # fixed anchor; the live runner separately checks the newest receipt too.
    anchor_q, _ = feedback(nodes[0]['receipt']['bound_feedback'])
    for node in nodes[1:]:
        q, _ = feedback(node['receipt']['bound_feedback'])
        require(max(abs(a - b) for a, b in zip(q, anchor_q)) <= pose_tolerance_rad,
                'chain_accumulated_alignment_changed')
    return dict(nodes=nodes, artifact_hashes=files)
