"""Read-only timing of exact GEN8 proposals; no robot communication."""
from pathlib import Path
import runpy,cProfile,pstats,json,hashlib,time
helper=runpy.run_path(str(Path(__file__).with_name('gen8_fk_seed_experiments_20261012.py')))
args=helper['load'](); module=helper['module']; output=helper['NEW']/'gen8_analysis/fk_constraint_profile.json'
prof=cProfile.Profile();started=time.monotonic();prof.enable()
rows,audit=module.generate_reachable_observation_views(**args)
prof.disable();elapsed=time.monotonic()-started
stats=pstats.Stats(prof);items=[]
for (filename,line,name),(primitive,total,selfsec,cumsec,callers) in stats.stats.items():
 items.append(dict(path=filename,line=line,function=name,primitive_calls=primitive,total_calls=total,self_sec=selfsec,cumulative_sec=cumsec))
items.sort(key=lambda x:-x['cumulative_sec'])
report=dict(source_path=str(helper['INPUT']/'source.json'),source_sha256=hashlib.sha256((helper['INPUT']/'source.json').read_bytes()).hexdigest(),opening_width_m=args['opening_width_m'],elapsed_sec=elapsed,candidate_count=len(rows),audit=audit,top_cumulative=items[:45],physical_actions=False,limitations=['cProfile adds runtime overhead and may exhaust the same 2 second budget earlier than uninstrumented execution','Cumulative timings overlap; do not sum them'])
assert not output.exists();output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(path=str(output),elapsed_sec=elapsed,candidate_count=len(rows),status=audit['status'],top=[{k:r[k]for k in ('function','total_calls','self_sec','cumulative_sec')}for r in items[:18]]),indent=2))
