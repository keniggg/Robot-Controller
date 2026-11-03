from pathlib import Path
s=Path('.runtime/gen24_fragment_ownership_replay_20261011.py').read_text();exec(s[:s.index('captured={}')])
start=time.monotonic();a=tracker.anchor;last=tracker.last;frozen=tracker.plane_base.copy()
try:
 confirmed,candidate,staged,evidence=helper.observe(seg,f['color'],depth,K,T,row['source_stamp_ns'],rgb_valid=f['rgb_valid'],appearance_deadline_monotonic=start+.8,appearance_check_current=lambda:None)
 reason=evidence['reason'];metrics=candidate.metrics
except ValueError as e:
 confirmed=False;reason=str(e);metrics={};evidence={}
report=dict(kind='actual_production_chain_staged_sources',source_ns=row['source_stamp_ns'],reference_source_ns=rr['source_stamp_ns'],reason=reason,confirmed=confirmed,metrics=metrics,evidence=evidence,wall_sec=time.monotonic()-start,anchor_unchanged=tracker.anchor is a,last_unchanged=tracker.last is last,plane_unchanged=np.array_equal(frozen,tracker.plane_base),source_sha256=before,source_after_sha256={m.__file__:sha(m.__file__)for m in mods},input_manifest_sha256=sha(INP/'manifest.json'),script_sha256=sha(__file__),limitations=['Only this first online failure has exact saved RGBD.','No ROS timing/owner/motion is replayed.','Original 4-connected segmentation and current support result unchanged; recovery runs inside original matching-fragment helper.'])
def enc(x):
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 raise TypeError(type(x).__name__)
(OUT/'final_owner_replay_r2.json').write_text(json.dumps(report,indent=2,default=enc)+'\n')
print(json.dumps(dict(reason=reason,confirmed=confirmed,wall_sec=report['wall_sec'],audit=metrics.get('observation_fragment_group'))))
