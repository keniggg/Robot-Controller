import ast,hashlib,json,os,subprocess,sys,time,xml.etree.ElementTree as ET
from pathlib import Path
W=Path(__file__).resolve().parent.parent;C=W/'.runtime/gen16_contact_budget_checkpoint_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification'/sys.argv[1];O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
baseline=json.loads((C/'baseline_sha256.json').read_text());assert all(sha(W/f)==h for f,h in baseline.items())
relative='src/alicia_grasp_modes/src/alicia_grasp_modes/contact_search_yield.py'
old=next(n for n in ast.parse((W/relative).read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='contact_search_yield_proof_valid')
new=next(n for n in ast.parse((C/relative).read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='_legacy_contact_search_yield_proof_valid')
assert ast.dump(ast.Module(body=old.body,type_ignores=[]))==ast.dump(ast.Module(body=new.body,type_ignores=[]))
before={str(p):sha(p) for p in sorted(C.rglob('*.py'))};before[str(Path(__file__).resolve())]=sha(__file__)
T=C/'src/alicia_grasp_modes/tests';original=W/'src/alicia_grasp_modes/tests'
selected=[str(p) for p in sorted(T.glob('test_*.py'))]+[str(original/name) for name in ('test_contact_search_yield.py','test_gen16_recorded_contact_search_yield.py','test_observe_return_supplement_remote.py','test_supplement_preview_audit_handoff.py')]
env=os.environ.copy();env.update(PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1')
env['PYTHONPATH']=':'.join(map(str,[T,original,W/'src/alicia_flexible_grasp_supervisor/tests',W/'src/alicia_grasp_modes/src',W/'src/alicia_flexible_grasp_supervisor/src',W/'devel/lib/python3/dist-packages']))+':'+env.get('PYTHONPATH','')
env['ROS_PACKAGE_PATH']=str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
bootstrap='''import json,pathlib,sys,pytest
import alicia_grasp_modes
alicia_grasp_modes.__path__.insert(0,str(pathlib.Path(sys.argv[2])/'src/alicia_grasp_modes/src/alicia_grasp_modes'))
result=pytest.main(json.loads(sys.argv[1]))
from alicia_grasp_modes import observe_return_remote,contact_search_yield
loaded={m.__name__:str(pathlib.Path(m.__file__).resolve()) for m in (observe_return_remote,contact_search_yield)}
pathlib.Path(sys.argv[3]).write_text(json.dumps(loaded,indent=2)+'\\n')
assert all(pathlib.Path(sys.argv[2]).resolve() in pathlib.Path(p).parents for p in loaded.values()),loaded
sys.exit(result)
'''
args=['-q','-p','no:cacheprovider',*selected,'--basetemp='+str(O/'tmp'),'--junitxml='+str(O/'pytest.xml')]
started=time.monotonic()
with (O/'pytest.log').open('w') as out:
 child=subprocess.run([sys.executable,'-c',bootstrap,json.dumps(args),str(C),str(O/'loaded.json')],cwd=W,env=env,stdout=out,stderr=subprocess.STDOUT)
summary={}
if (O/'pytest.xml').exists():
 xml=ET.parse(O/'pytest.xml').getroot();summary={k:sum(int(s.attrib.get(k,0)) for s in xml.findall('testsuite')) for k in ('tests','failures','errors','skipped')}
after={f:sha(f) for f in before};unchanged=all(sha(W/f)==h for f,h in baseline.items())
report=dict(passed=child.returncode==0 and before==after and unchanged,returncode=child.returncode,summary=summary,elapsed_sec=time.monotonic()-started,selected=selected,candidate_before=before,candidate_after=after,actual_baseline=baseline,actual_unchanged=unchanged,legacy_v1_validator_body_AST_identical=True,loaded=json.loads((O/'loaded.json').read_text()) if (O/'loaded.json').exists() else None,log_sha256=sha(O/'pytest.log'),no_ros_master=True,no_physical_action=True,temperature_queried=False,scope='Real selection and mode height dispatch plus typed-proof consumers; synthetic strict transport; legacy recorded proofs retained; no hardware or full geometry success claim.')
(O/'result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),passed=report['passed'],summary=summary)))
