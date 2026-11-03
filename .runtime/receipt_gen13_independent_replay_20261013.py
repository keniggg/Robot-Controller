#!/usr/bin/env python3
"""Seal independent offline replay evidence; never changes original reports."""
import hashlib,json
from pathlib import Path
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
V=N/'verification'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,d):
    assert not p.exists(),str(p)
    p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
def main():
    original=V/'gen13_full_qualified_station_process/result.json';op=json.loads(original.read_text())
    assert not op['passed'] and len(op['rows'])==5
    assert all(r['state']=='ready' and r['detected'] and r['source_age_at_completion_sec']<.8 and r['last_good_ns']==r['source_ns']for r in op['rows'][:4])
    o=op['rows'][-1];assert o['state']=='processing_deadline_pending' and not o['detected'] and o['source_age_at_completion_sec']>=.8 and o['last_good_ns']==op['rows'][-2]['source_ns'] and o['first_loss']is None
    correction=dict(passed=True,scope='Correct classification of the preserved earlier candidate report, not validation of current candidate bytes',original_report_path=str(original),original_report_sha256=sha(original),original_passed_field=False,explanation='The earlier harness incorrectly required every forced-FIFO frame to be READY. The fifth frame exceeded the unchanged 0.8 second source limit and correctly became processing_deadline_pending, without refreshing last_good or creating first_loss. Its original false report is preserved unchanged.',fresh_ready_count=4,expected_expired_current_frame_pending_count=1,source_sha256=op['source_sha256'],no_new_execution=True,not_live_latest_only_simulation=True)
    write(original.parent/'classification_receipt.json',correction)
    pp=V/'gen13_full_qualified_station_process_v2/result.json';np=V/'gen13_no_callback_bad_negative/result.json';p=json.loads(pp.read_text());n=json.loads(np.read_text());sources=p['source_sha256']
    assert p['passed'] and p['source_unchanged'] and p['full_saved_sequence_replayed'] and n['no_qualified_callback_no_station_template']
    assert sources==n['candidate_binding']['source_sha256'] and all(sha(Path(path))==digest for path,digest in sources.items())
    assert len(n['rows'])==2 and all(not r['detected'] and r['last_good_ns']==1791854972104432344 and r['first_loss']is not None and r['following_actual_saved_frame']['first_loss_preserved'] and not r['following_actual_saved_frame']['detected']for r in n['rows'])
    assert [r['detail']for r in n['rows']]==['perception_failure: source_exceeds_existing_frame_limit','perception_failure: metric_ambiguity_no_unique_complete_rgb_identity']
    provenance=p['rows'][0]['template_provenance'];assert all(r['template_provenance']==provenance and r['template_object_unchanged'] and r['marker_object_unchanged']for r in p['rows'])
    dependencies=[pp,np,original,original.parent/'classification_receipt.json',W/'.runtime/replay_gen13_qualified_station_process_v2_20261013.py',W/'.runtime/replay_gen13_no_callback_bad_negative_20261013.py',W/'.runtime/replay_attempt1_full_process_negative_20261013.py',W/'.runtime/replay_attempt1_first_phase_sources_20261013.py',N/'readonly_full_frame_probe_002/context.json',N/'full_restart_acceptance/result.json',V/'attempt1_first_phase_with_qualified_source/fixture.json',V/'attempt1_first_phase_with_qualified_source/qualified_appearance_seed_receipt.json',Path(__file__)]
    dependencies+=sorted((V/'attempt1_first_phase_with_qualified_source').glob('*_source.npz'))
    dependencies+=sorted((V/'attempt1_first_phase_independent_v2').glob('*_source.npz'))
    receipt=dict(passed=True,source_unchanged=True,source_sha256=sources,scope='Independent actual qualified callback, measured computation clock, original saved RGBD/TF/SDK five-frame forced FIFO, unchanged original source timestamps, fixed third-qualified-frame identity template; full process legacy/no-callback negative',fresh_current_geometry_count=sum(r['state']=='ready'for r in p['rows']),expected_expired_current_frame_pending_count=sum(r['state']=='processing_deadline_pending'for r in p['rows']),full_process_no_qualified_callback_expected_rejects=2,selector_lineage_tamper_expected_rejects=len(p['selector_lineage_negatives']),positive_result=str(pp),negative_result=str(np),qualified_callback_timing=p['qualified_callback_timing'],qualified_source_ns=p['qualified_source_ns'],station_template_provenance=provenance,phase_ns=p['phase_ns'],phase_schedule_explicitly_counterfactual=True,all_new_computation_elapsed_charged=True,source_restamped=False,old_metric_geometry_reused=False,first_two_qualification_frames_pixels_missing=True,successful_first_three_qualification_receipt_seeded=True,live_stream_completeness_claimed=False,limits=['All five actually saved post-qualification frames were fed in timestamp order; no convenient frame selection. Forced FIFO is not the production latest-only queue.','Original camera frames absent from sealed keyframes cannot be fabricated. No latest-only or full live stream success is inferred.','First two qualification pixels are absent: independently accepted three-frame state is explicitly seeded; the third frame raw RGB/mask and qualified callback are real.','Early phase follows both original peer ACK availability and the newly measured candidate qualified callback completion; the schedule is a stated fix counterfactual.','No real task success or physical grasp success is asserted by offline replay.'],dependencies_sha256={str(x):sha(x)for x in dependencies},no_ros_master=True,motion_authorized=False)
    output=pp.parent/'validation_receipt.json';write(output,receipt);print(output);print(sha(output))
if __name__=='__main__':main()
