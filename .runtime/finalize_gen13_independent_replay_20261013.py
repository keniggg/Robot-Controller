#!/usr/bin/env python3
"""Finalize SHA-bound offline evidence; preserve every failed and prior report."""
import argparse,hashlib,json
from pathlib import Path
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
V=N/'verification'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    a=argparse.ArgumentParser();a.add_argument('--positive',required=True);a.add_argument('--negative',required=True);a.add_argument('--deadline',required=True);args=a.parse_args()
    pp=V/args.positive/'result.json';np=V/args.negative/'result.json';dp=V/args.deadline/'result.json'
    p,n,d=[json.loads(path.read_text())for path in (pp,np,dp)];sources=p['source_sha256']
    assert p['passed'] and p['source_unchanged'] and p['full_saved_sequence_replayed'] and len(p['rows'])==5
    assert d['passed'] and d['source_unchanged'] and len(d['deadline_injection'])==1
    assert n['no_qualified_callback_no_station_template']
    assert sources==n['candidate_binding']['source_sha256']==d['source_sha256']
    assert all(sha(Path(path))==value for path,value in sources.items())
    assert len(n['rows'])==2 and all(not r['detected'] and r['last_good_ns']==1791854972104432344 and r['first_loss']is not None and r['following_actual_saved_frame']['first_loss_preserved'] and not r['following_actual_saved_frame']['detected']for r in n['rows'])
    assert [r['detail']for r in n['rows']]==['perception_failure: source_exceeds_existing_frame_limit','perception_failure: metric_ambiguity_no_unique_complete_rgb_identity']
    assert d['rows'][0]['state']=='ready' and d['rows'][1]['state']=='processing_deadline_pending' and not d['rows'][1]['detected'] and d['rows'][1]['first_loss']is None and d['rows'][1]['last_good_ns']==d['rows'][0]['source_ns']
    provenance=p['rows'][0]['template_provenance'];assert all(r['template_provenance']==provenance and r['template_object_unchanged'] and r['marker_object_unchanged']for r in p['rows'])
    assert len(p['selector_lineage_negatives'])==9
    old=[V/'gen13_full_qualified_station_process/result.json',V/'gen13_full_qualified_station_process/classification_receipt.json',V/'gen13_full_qualified_station_process_v2/result.json',V/'gen13_full_qualified_station_process_v3/result.json',V/'gen13_full_qualified_station_process_v4/result.json',V/'gen13_post_sdk_deadline_before_fix/result.json']
    dependencies=[pp,np,dp,*old,W/'.runtime/replay_gen13_qualified_station_process_v4_20261013.py',W/'.runtime/replay_gen13_post_sdk_deadline_negative_v2_20261013.py',W/'.runtime/replay_gen13_no_callback_bad_negative_v2_20261013.py',W/'.runtime/replay_attempt1_full_process_negative_20261013.py',W/'.runtime/replay_attempt1_first_phase_sources_20261013.py',N/'readonly_full_frame_probe_002/context.json',N/'full_restart_acceptance/result.json',V/'attempt1_first_phase_with_qualified_source/fixture.json',V/'attempt1_first_phase_with_qualified_source/qualified_appearance_seed_receipt.json',Path(__file__)]
    dependencies+=sorted((V/'attempt1_first_phase_with_qualified_source').glob('*_source.npz'))
    dependencies+=sorted((V/'attempt1_first_phase_independent_v2').glob('*_source.npz'))
    receipt=dict(passed=True,source_unchanged=True,source_sha256=sources,scope='Independent actual qualified callback and full producer process, original saved RGBD/TF/SDK five-frame forced FIFO; fixed qualified identity template; full process no-callback negatives and deterministic post-SDK current-source deadline negative',fresh_current_geometry_count=sum(r['state']=='ready'for r in p['rows']),natural_fifo_expired_pending_count=sum(r['state']=='processing_deadline_pending'for r in p['rows']),deterministic_post_sdk_deadline_expected_pending_count=1,full_process_no_qualified_callback_expected_rejects=2,selector_lineage_tamper_expected_rejects=len(p['selector_lineage_negatives']),positive_result=str(pp),negative_result=str(np),deadline_negative_result=str(dp),qualified_callback_timing=p['qualified_callback_timing'],qualified_source_ns=p['qualified_source_ns'],station_template_provenance=provenance,phase_ns=p['phase_ns'],phase_schedule_explicitly_counterfactual=True,all_new_computation_elapsed_charged=True,source_restamped=False,old_metric_geometry_reused=False,limits=['All five actually saved post-qualification frames were fed in timestamp order; forced FIFO is not the production latest-only queue. No convenient saved-frame selection.','Original camera frames absent from sealed keyframes cannot be reconstructed. This evidence cannot establish a full-rate live stream result.','First two qualification pixels are absent: original independently accepted three-frame state is explicitly seeded; the third frame raw RGB/mask and qualified callback are real.','Early phase follows both original peer ACK availability and newly measured candidate qualified callback completion; it is a stated fixed-schedule counterfactual.','The post-SDK deadline test explicitly advances diagnostic time after a real SDK bracket result; it is not a timing-performance claim.','The original all-READY misclassification report and the subsequent real deadline defect reports remain intact; the fixed result is separately bound to new candidate SHA.','No real grasp success is asserted by offline replay.'],dependencies_sha256={str(x):sha(x)for x in dependencies},no_ros_master=True,motion_authorized=False)
    out=pp.parent/'validation_receipt.json';assert not out.exists();out.write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n');print(out);print(sha(out))
if __name__=='__main__':main()
