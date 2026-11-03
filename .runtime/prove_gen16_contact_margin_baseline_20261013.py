import ast,hashlib,json,sys,textwrap
from pathlib import Path
W=Path(__file__).resolve().parents[1];C=W/'.runtime/gen16_contact_margin_recenter_20261013';N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
sys.path[:0]=[str(C/'src/alicia_flexible_grasp_supervisor/tests'),str(C/'src/alicia_flexible_grasp_supervisor/src')]
from test_final_registered_contact_height_recenter import real_fixture, remote
base=W/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
tree=ast.parse(base.read_text());klass=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='RemoteGrasp6DNode');method=next(x for x in klass.body if isinstance(x,ast.FunctionDef) and x.name=='_prepare_final_registered_contact_plan')
namespace=dict(remote.__dict__);exec(compile(ast.Module(body=[method],type_ignores=[]),str(base),'exec'),namespace)
node,p,r,plan=real_fixture();original=namespace[method.name].__get__(node,type(node));failure=None
try:original(plan,p,r)
except remote.CandidateContractError as exc:failure=dict(code=exc.code,reason=str(exc))
assert failure and failure['code']=='CONTACT_HEIGHT_REGISTRATION_MARGIN'
node,p,r,plan=real_fixture();node._prepare_final_registered_contact_plan(plan,p,r)
assert r['final_contact_height_margin']['ok'] and r['final_contact_height_recenter']['applied']
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
out=N/'verification/gen16_contact_recenter_baseline_r1';out.mkdir(exist_ok=False)
result=dict(passed=True,baseline_reproduced=True,baseline_failure=failure,candidate_final_margin=r['final_contact_height_margin'],candidate_recenter=r['final_contact_height_recenter'],base_path=str(base),base_sha256=sha(base),candidate_path=remote.__file__,candidate_sha256=sha(remote.__file__),script=str(Path(__file__).resolve()),script_sha256=sha(__file__),scope='Same genuine synthetic measured-surface fixture and original production method versus candidate production method; no planner RPC or motion. Separate R4 fixture reconstructs saved analytic boundaries, not unavailable fused raw cloud.',physical_grasp_verified=False,motion_commands_sent=False)
(out/'result.json').write_text(json.dumps(result,indent=2,sort_keys=True)+'\n');print(out/'result.json');print(sha(out/'result.json'))
