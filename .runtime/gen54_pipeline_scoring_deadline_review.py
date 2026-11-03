"""Summarize frozen queue lineage and historical RPC timing; no robot access."""
import hashlib
import json
import os
from pathlib import Path
import statistics

os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
import numpy as np

W=Path(__file__).resolve().parents[1]
C=W/'.runtime/gen54_release_candidate_r1'
V=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
OUT=V/'gen54_geometry_pipeline_r1/normalized_scoring_r3'


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def rotation(q):
    x,y,z,w=np.asarray(q,dtype=float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    normalized_path=OUT/'result.json'
    registered_path=V/'gen54_collision_geometry_r1/registered_materialized_geometry.json'
    materializer_path=V/'gen54_geometry_pipeline_r1/materializer_probe_r3/result.json'
    heights_path=V/'gen54_collision_geometry_r1/height_joint_window_actual_api.json'
    history_path=V/'gen53_actual_handoff_monitor_r1/final_review_r1.json'
    n=json.loads(normalized_path.read_text());r=json.loads(registered_path.read_text())
    p=json.loads(materializer_path.read_text());h=json.loads(heights_path.read_text())
    history=json.loads(history_path.read_text());matches=[]
    for rank,item in enumerate(n['ranked_queue'],start=1):
        transforms={}
        for stage in item['sequence']['stages']:
            t=np.eye(4);t[:3,:3]=rotation(stage['quaternion_xyzw']);t[:3,3]=stage['position_m']
            transforms[stage['stage']]=t
        for registered in r['rows']:
            if not registered['source_placement_ok']:continue
            original=next(y for y in p['outputs'][1]['rows'] if (y['source_index'],y['variant_index'])==
                (registered['source_index'],registered['variant_index']))
            errors=[]
            for name,mat in original['stage_matrices'].items():
                matrix=np.array(mat);matrix[:3,3]+=registered['final_source_placement_delta_m']
                errors.append(float(np.max(np.abs(matrix-transforms[name]))))
            error=max(errors)
            if error<1e-10:
                height=next((z for z in h['rows'] if (z['source_index'],z['variant_index'])==
                    (registered['source_index'],registered['variant_index'])),None)
                matches.append(dict(fallback_rank=rank,source_index=item['source_index'],
                    materialized_variant_index=item['materialized_variant_index'],
                    recheck_variant_index=item['recheck_variant_index'],
                    independent_source_index=registered['source_index'],
                    independent_variant_index=registered['variant_index'],
                    four_source_stage_max_abs_difference=error,
                    original_height_proposal_count=len(height.get('actual_production_proposals',())) if height else None,
                    height_deltas_m=[x['audit']['normal_translation_m'] for x in height.get('actual_production_proposals',())] if height else [],
                    known_both_approach_to_grasp_curves_pass=bool(height and height.get('both_nominal_and_command_continuous_pass')),
                    independent_height_source_sha256=sha(heights_path)))
    assert len(matches)==12 and all(x['four_source_stage_max_abs_difference']<1e-14 for x in matches)
    assert matches[3]['known_both_approach_to_grasp_curves_pass']
    timings=[]
    for delivery in history['strict_completed_deliveries']:
        evidence=delivery['evidence'];path=Path(evidence['path'])
        with path.open('rb') as source:
            source.seek(evidence['offset']);raw=source.read(evidence['bytes'])
        assert hashlib.sha256(raw).hexdigest()==evidence['sha256']
        text=raw.decode();event=json.loads(text[text.index('{'):].strip())
        response=event['response']
        timings.append(dict(plan_id=delivery['plan_id'],
            rpc_wall_elapsed_sec=(response['received_monotonic_ns']-event['request_monotonic_ns'])/1e9,
            request_wall_sec=event['request_wall_ns']/1e9,
            response_wall_sec=response['received_wall_ns']/1e9,
            unchanged_selection_deadline_sec=event['deadline_sec'],
            remaining_at_rpc_entry_sec=event['deadline_sec']-event['request_wall_ns']/1e9,
            failure_code=delivery['failure_code'],failed_stage=delivery['failed_stage'],
            reason=delivery['reason'],exact_log_reference=evidence))
    collision=[x['rpc_wall_elapsed_sec'] for x in timings if x['failed_stage']=='grasp']
    before=matches[:4]
    per_candidate=[1+x['original_height_proposal_count'] for x in before]
    calls=sum(per_candidate)
    report=dict(kind='GEN54_frozen_normalization_lineage_and_conditional_deadline_review',
        diagnostic_only=True,execution_authorized=False,physical_grasp_success=False,
        source_root=str(C),full_regression=False,source_or_runtime_mutated=False,
        normalized_summary={k:n[k] for k in ('materialized_count','normalized_count','stable_count',
            'scored_count','deduplicated_count','full_mode_generation_same_32_matrices')},
        configuration_limits=n['limitations'],
        phase_mapping=dict(archived_audit_phase='observation_contact',runtime_phase='observing_contact',
            source='observe_return_remote.py audit serializer'),
        queue_matches=matches,
        historical_gen53_rpc_timings=timings,
        historical_collision_rpc_statistics_sec=dict(count=len(collision),minimum=min(collision),
            maximum=max(collision),mean=statistics.mean(collision),median=statistics.median(collision)),
        conditional_time_scenario=dict(
            assumed_rank_order='Fallback order with no newly obtained endpoint IK hints',
            initial_and_height_rpc_counts_before_first_known_passing_height=per_candidate,
            total_calls_if_every_proposal_passes_local_gates_and_reaches_rpc=calls,
            gate_early_rejections_not_yet_executed_in_this_report=True,
            estimated_rpc_time_using_historical_mean_sec=calls*statistics.mean(collision),
            estimated_rpc_range_using_historical_extremes_sec=[calls*min(collision),calls*max(collision)],
            historical_budget_at_first_rpc_sec=timings[0]['remaining_at_rpc_entry_sec'],
            future_elapsed_time_measured=False,actual_rpc_count_predicted=False,
            candidate_four_height_four_guaranteed_before_deadline=False,
            excludes=['preliminary nominal MoveIt/resolver/registration work',
                'new endpoint IK hint order', 'early full-geometry/goal rejection',
                'future feedback, scene and IK branch changes', 'publication revalidation']),
        static_call_order=[
            '_evaluate_local_candidates → _single_request_stable_candidates → _recheck_and_score_stable → mode exact-family dedupe',
            '_direct_contact_endpoint_priorities (bounded hints only) → contact_direction_priorities → bounded_moveit_select',
            '_direct_near_field_comparison_search sets selection deadline to shared phase deadline minus unchanged 2 s publication reserve',
            '_check_direct_registered_candidate → early necessary nominal goal check → original preliminary MoveIt/resolver → registered rich-plan build',
            '_strict_check_final_near_field_plan → final nominal necessary goal check → full contact-sequence RPC',
            'Only exact current-plan CONTACT_SEQUENCE_PRECHECK_FAILED / success False / failed_stage grasp / predicted_command continuous geometry: target OBB collision at ... starts height retry',
            '_contact_height_window_retry visits original three proposals then optional fourth upper interior; each repeats registration/source geometry, final evidence, plan hash, unchanged height margin, nominal necessary goal and full contact-sequence RPC',
            'Every retry shares the same selection deadline; current-ticket and deadline checks repeat before and after service; no automatic time extension',
            'Complete strict success preserves selected geometry recipe; publication rebuilds current evidence and runs complete strict validation again'],
        existing_early_rejection_locations=[
            dict(file='scripts/remote_grasp6d_node.py',method='_early_contact_goal_rejection',
                scope='Nominal pose-family necessary overlap only. Nonidentity registration/support restoration is intentionally deferred.'),
            dict(file='scripts/remote_grasp6d_node.py',method='_reject_inevitable_contact_goal',
                scope='Already called after final registration and for every height proposal. Does not model raw compensated command FK.'),
            dict(file='robot/contact_sequence_precheck.py',method='validate_contact_prediction',
                scope='Already checks exact same-branch nominal and raw-command terminal before continuous interval proof, after the complete nominal contact trajectory is available.'),
            dict(file='scripts/remote_grasp6d_node.py',method='_direct_contact_endpoint_priorities',
                scope='Existing endpoint IK hints can reorder but never remove candidates; current hint does not evaluate frozen-bias command geometry.')],
        next_step_if_runtime_repeats_same_blocker=dict(
            recommendation='Measure which exact stage consumes the new attempt before designing an earlier check.',
            no_safe_existing_drop_in_raw_command_hard_veto=True,
            reason='A preliminary endpoint IK hint can use a different branch from final registered held-state planning. Its collision cannot reject all branches or authorize a path.',
            possible_followup='Reuse exact chosen endpoint/bias/scene/opening identity for a necessary same-branch terminal screen before expensive repeated held-path work, or use preliminary clear/colliding hints solely for ordering; preserve all full validators and unchanged shared deadline.',
            implementation_performed=False),
        evidence_hashes={str(x):sha(x) for x in (normalized_path,registered_path,materializer_path,heights_path,history_path)},
        helper_sha256=sha(__file__))
    path=OUT/'lineage_deadline_review.json'
    assert not path.exists()
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(path=str(path),sha256=sha(path),statistics=report['historical_collision_rpc_statistics_sec'],
        conditional=report['conditional_time_scenario']),ensure_ascii=False))


if __name__=='__main__':main()
