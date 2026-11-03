"""Offline proposal experiment; every final gate uses the unchanged frozen OBB."""
from pathlib import Path
import inspect, json, runpy, time

W=Path(__file__).resolve().parents[1]
context=runpy.run_path(str(W/'.runtime/replay_supplement_collision.py'))
node,remote,geometry,snapshot,prepared=[context[k] for k in ('node','remote','geometry','snapshot','prepared')]
np,plain,OUT=[context[k] for k in ('np','plain','OUT')]
original=remote.generate_tabletop_proposals
source=inspect.getsource(original)
assert source.count('    contact_center = center\n')==1
namespace=dict(original.__globals__)
namespace['diagnostic_center_offset']=np.zeros(3)
exec(compile(source.replace('    contact_center = center\n',
    '    contact_center = center + diagnostic_center_offset\n'),'<offline_contact_center_proposal>','exec'),namespace)
remote.generate_tabletop_proposals=namespace['generate_tabletop_proposals']
class CapturedGate(Exception):
    def __init__(self,gate):self.gate=gate
def capture(gate):raise CapturedGate(gate)
node._record_geometry_gate_result=capture
rows=[];started=time.monotonic()
for long_fraction in (0.,-.15,.15,-.30,.30):
    for short_fraction in (0.,-.1,.1):
        offset=geometry.axes_base@np.array([long_fraction*geometry.size_xyz_m[0],short_fraction*geometry.size_xyz_m[1],0.])
        namespace['diagnostic_center_offset']=offset
        candidates,diag=node._generate_tabletop_candidates(geometry,snapshot)
        results=[]
        for c in candidates:
            try:
                node._normalize_tabletop_candidate(prepared,c)
                raise AssertionError('normalization must reach unchanged geometry gate')
            except CapturedGate as exc:
                results.append(dict(gate=exc.gate,T_base_tool0=c.T_base_tool0,contact_center_base=c.contact_center_base,
                    source_index=c.source_index,variant_index=c.variant_index,audit=dict(c.audit)))
            except remote.CandidateContractError as exc:
                results.append(dict(failure=str(exc),code=exc.code))
        passed=sum(r.get('gate') is not None and r['gate'].ok for r in results)
        rows.append(dict(offset_base_m=offset,offset_fractions=[long_fraction,short_fraction],
            candidates=len(candidates),passed=passed,diagnostics=diag,results=results))
        print(json.dumps(dict(offset_fractions=[long_fraction,short_fraction],candidates=len(candidates),passed=passed)),flush=True)
out=dict(scope='diagnostic candidate-center translation only; original fused points, full OBB, support plane and final gates unchanged',
    historical_pipeline_modified=False,execution_authorized=False,complete_path_checked=False,
    rows=rows,wall_sec=time.monotonic()-started,passed=sum(r['passed'] for r in rows),
    limitations=['Alternative proposals are a counterfactual search, not original candidate IDs or physical success.'])
(OUT/'center_scan.json').write_text(json.dumps(plain(out),ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(passed=out['passed'],wall_sec=out['wall_sec'],output=str(OUT/'center_scan.json'))),flush=True)
