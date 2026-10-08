import copy,hashlib,importlib.util,json
from pathlib import Path
import pytest
P=Path(__file__).with_name('analyze_passive_load_round_20261007.py')
spec=importlib.util.spec_from_file_location('passive_analysis',P);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def sdk(t,ns=None):
 return dict(monotonic=t,source_ns=ns if ns is not None else round(t*1e9),result='complete',
  host_wait=dict(enter=dict(monotonic_sec=t-.02),exit=dict(monotonic_sec=t-.01),elapsed_sec=.01,thread_cpu_elapsed_sec=.001),
  stages_sec={'filter':.005},raw_frames={k:dict(present=True,device_timestamp_ms=t*1000,frame_number=round(t*30),metadata_raw={'frame_timestamp':round(t*1e6),'actual_fps':30000}) for k in ['color','depth']})
def disk():
 return dict(phases=[dict(name='A1_without_test_writes',start=1,end=2),dict(name='B_bounded_buffered_disk_write',start=2,end=3),dict(name='A2_without_test_writes',start=3,end=4)],sdk_reads=[sdk(t) for t in [.9,1.1,1.2,2.1,2.2,3.1,3.2]],samples=[],feedback={'accepted':[],'heartbeat':[]},error=None,write_timings=[dict(monotonic=2.1,seconds=.01,bytes=10)],bytes_written=10)
def raw():
 d=disk();d.pop('write_timings');d.pop('bytes_written');d.pop('error');d.update(experiment='raw_exact_rgbd_recording_load_ABA',errors=[],recorder_returncode=0,owned_recorder_still_running=False)
 d['phases'][1]['name']='B_raw_rgbd_recording'
 d['recorder_outcome']=dict(capture_start_monotonic=2,capture_end_monotonic=3,duration_requested_sec=1,completed_monotonic=3.1,capture_valid=True,bag_closed=True,errors=[],pair_count=1,bag_bytes=500,raw_pixel_bytes=1000,stop_reason='duration_elapsed',receive={'counts':{}},pairs=[dict(source_ns=7,write_completed_monotonic=2.4,images={k:dict(source_ns=7,receipt_monotonic=t) for k,t in [('color',2.2),('depth',2.3)]})])
 return d

def test_half_open_crossings_and_pre_phase_history():
 d=m.analyze(disk());v=d['phase_summaries'][0]['sdk_source_interval_ms'];assert v['fully_inside']['count']==1;assert len(v['cross_boundary_excluded'])==2;assert d['sdk_history_outside_all_phases']['before']==1

def test_missing_interval_is_barrier_not_bridge():
 rows=[dict(monotonic=x,source_ns=v) for x,v in [(1,1),(2,None),(3,3)]];r=m.interval_stats(rows,0,4,scale=1);assert not r['fully_inside']['available'];assert len(r['invalid_intervals'])==2

def test_zero_sdk_does_not_mean_healthy():
 d=disk();d['sdk_reads']=[];r=m.analyze(d);assert not r['sdk_evidence_available_in_all_main_phases'];assert r['phase_summaries'][0]['sdk_wait_ms']['maximum'] is None;assert r['phase_summaries'][0]['feedback']['accepted']['absence_is_arm_poweroff_not_stall']

def test_missing_native_metadata_explicit():
 d=disk();d['sdk_reads'][1]['raw_frames']=None;r=m.analyze(d);v=r['phase_summaries'][0]['native']['color'];assert v['missing_frames']==1;assert v['device_interval_ms']['invalid_intervals']

def test_negative_native_counter_explicit():
 d=disk();d['sdk_reads'][2]['raw_frames']['color']['frame_number']=-3;r=m.analyze(d);assert r['phase_summaries'][0]['native']['color']['frame_number_step']['invalid_intervals'][0]['reason']=='negative_value_delta'

def test_wait_crossing_boundary_excluded():
 d=disk();d['sdk_reads'][1]['host_wait']['enter']['monotonic_sec']=.99;r=m.analyze(d);v=r['phase_summaries'][0];assert v['sdk_wait_ms']['count']==1;assert any(i['reason']=='cross_boundary_wait_excluded' for i in v['sdk_wait_issues'])

def test_missing_wait_is_not_zero_duration():
 d=disk();d['sdk_reads'][1]['host_wait']=None;r=m.analyze(d);v=r['phase_summaries'][0];assert v['sdk_wait_ms']['count']==1;assert v['sdk_wait_issues'][0]['reason']=='missing_or_invalid_wait_bounds'

def test_pure_write_count_mismatch_invalid_contrast():
 d=disk();d['bytes_written']=11;r=m.analyze(d);assert not r['complete_phase_contrast'];assert not r['write_accounting_valid']

def test_write_crossing_does_not_enter_phase_throughput():
 d=disk();d['write_timings']=[dict(monotonic=2.99,seconds=.1,bytes=10)];r=m.analyze(d);assert r['phase_summaries'][1]['actual_write']['bytes']==0;assert len(r['phase_summaries'][1]['actual_write']['cross_boundary_excluded'])==1

def test_raw_pair_queue_latency_is_not_write_duration():
 r=m.analyze(raw());assert r['recorder']['valid_artifact'];assert r['recorder']['latest_receipt_to_write_end_ms']['maximum']==pytest.approx(100);assert r['recorder']['compressed_bag_MiB_s']==500/2**20

@pytest.mark.parametrize('failure',['wrong_source','zero_pairs','outside_capture','duplicate','short'])
def test_raw_bad_pair_or_window_not_valid(failure):
 d=raw();o=d['recorder_outcome']
 if failure=='wrong_source':o['pairs'][0]['images']['color']['source_ns']=8
 elif failure=='zero_pairs':o['pairs']=[];o['pair_count']=0
 elif failure=='outside_capture':o['pairs'][0]['images']['color']['receipt_monotonic']=1.9
 elif failure=='duplicate':o['pairs']*=2;o['pair_count']=2
 else:o['capture_end_monotonic']=2.5
 r=m.analyze(d);assert not r['recorder']['valid_artifact'];assert not r['complete_phase_contrast']

def test_system_missing_counters_not_healthy():
 r=m.system_summary([dict(monotonic=1),dict(monotonic=2)],0,3);assert r['errors'];assert r['psi']=={};assert r['diskstats']=={}

def test_system_counter_decrease_not_negative_pressure():
 rows=[dict(monotonic=t,psi_cpu='some total=%d'%v,psi_memory='some total=0',psi_io='some total=0',vmstat={'pswpin':v},diskstats=[]) for t,v in [(1,10),(2,1)]]
 r=m.system_summary(rows,0,3);assert any('counter_decreased' in e for e in r['errors']);assert 'cpu' not in r['psi']

def test_only_overlap_aggregate_not_main_availability():
 d=disk();d['phases']=[dict(name='B_owned_process_lifetime',start=1,end=4)];r=m.analyze(d);assert r['phase_summaries'][0]['aggregate_overlaps_other_phases'];assert not r['sdk_evidence_available_in_all_main_phases'];assert not r['complete_phase_contrast']

def test_frozen_hash_mismatch_rejected_before_output(tmp_path,monkeypatch):
 monkeypatch.setattr(m,'EXT',tmp_path);p=tmp_path/'capture';p.mkdir();(p/'observations.json').write_text(json.dumps(disk()));(p/'SHA256SUMS').write_text('0'*64+'  observations.json\n')
 with pytest.raises(ValueError,match='frozen_hash'):m.run(p)
 assert not (p/'analysis').exists()

def test_no_output_overwrite(tmp_path,monkeypatch):
 monkeypatch.setattr(m,'EXT',tmp_path);p=tmp_path/'capture';p.mkdir();b=json.dumps(disk()).encode();(p/'observations.json').write_bytes(b);(p/'SHA256SUMS').write_text(hashlib.sha256(b).hexdigest()+'  observations.json\n');(p/'analysis').mkdir()
 with pytest.raises(FileExistsError):m.run(p)

def test_unknown_schema_rejected():
 with pytest.raises(ValueError,match='unsupported'):m.analyze({})
