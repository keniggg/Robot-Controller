from pathlib import Path
import inspect
# Reuse exact saved-input reconstruction only; do not run its output path.
src=Path('.runtime/gen24_fragment_ownership_replay_20261011.py').read_text();exec(src[:src.index('captured={}')])
original=t._depth_components;source=inspect.getsource(original)
assert 'connectivity=4' in source and 'flags=4 |' in source
alternative=source.replace('connectivity=4','connectivity=8').replace('flags=4 |','flags=8 |')
ns={};exec(alternative,t.__dict__,ns);fn=ns['_depth_components'];t._depth_components=fn
try:
 alt=t.segment_with_support_fallback(depth,K,T,cfg,anchor=tracker.anchor,last=tracker.last,reference_plane_base=tracker.plane_base,prefer_current_support=True)
finally:t._depth_components=original
matches=t._matching_candidates(alt.candidates,tracker.anchor,tracker.last,cfg)
start=time.monotonic()
try:
 confirmed,candidate,staged,evidence=helper.observe(alt,f['color'],depth,K,T,row['source_stamp_ns'],rgb_valid=f['rgb_valid'],appearance_deadline_monotonic=start+.8,appearance_check_current=lambda:None)
 error=None
except ValueError as e: error=str(e);candidate=None;evidence={}
report=dict(kind='offline_hypothesis_only_no_production_change',old_candidates=len(seg.candidates),new_candidates=len(alt.candidates),new_matching=len(matches),reason=error or evidence['reason'],candidate_metrics=candidate.metrics if candidate is not None else None,appearance=helper.last_appearance_evidence,original_function_sha256=hashlib.sha256(source.encode()).hexdigest(),alternative_function_sha256=hashlib.sha256(alternative.encode()).hexdigest(),preserved_depth_edge_m=cfg.depth_edge_m,exact_source_ns=row['source_stamp_ns'],script_sha256=sha(__file__),source_sha256=before,source_after_sha256={m.__file__:sha(m.__file__)for m in mods})
if candidate is not None:cv2.imwrite(str(OUT/'diagonal_proposal_mask.png'),candidate.mask)
(OUT/'diagonal_connectivity_probe.json').write_text(json.dumps(report,indent=2,default=lambda x:x.tolist()if isinstance(x,np.ndarray)else x.item())+'\n')
print(json.dumps({k:report[k]for k in ['old_candidates','new_candidates','new_matching','reason','appearance']}))
