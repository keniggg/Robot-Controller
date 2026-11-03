import ast,hashlib,json,os,subprocess,sys,time,xml.etree.ElementTree as ET
from pathlib import Path
W=Path(__file__).resolve().parent.parent;C=W/'.runtime/gen16_height_recipe_policy_test_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification'/sys.argv[1];O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
prior=N/'verification/gen16_success_first_actual_integrated/result.json';original=json.loads(prior.read_text());baseline=json.loads((C/'baseline_sha256.json').read_text());rel=next(iter(baseline));assert original['source_sha256'][rel]==baseline[rel] and sha(W/rel)==baseline[rel]
oldtree=ast.parse((W/rel).read_text());newtree=ast.parse((C/rel).read_text())
old_name='test_observe_return_override_forwards_options_only_after_original_yield_policy';new_names={'test_observe_return_override_keeps_height_search_and_forwards_options','test_observe_return_override_runs_real_height_search_and_fresh_publication_check'}
def unaffected(tree,names):return ast.dump(ast.Module(body=[n for n in tree.body if not(isinstance(n,ast.FunctionDef) and n.name in names)],type_ignores=[]))
assert unaffected(oldtree,{old_name})==unaffected(newtree,new_names)
actual_before={r:sha(W/r) for r in original['source_sha256']}
candidate_before=sha(C/rel)
T=C/'src/alicia_flexible_grasp_supervisor/tests'
env=os.environ.copy();env.update(PYTHONDONTWRITEBYTECODE='1',PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1')
env['PYTHONPATH']=':'.join(map(str,[T,W/'src/alicia_flexible_grasp_supervisor/tests',W/'src/alicia_grasp_modes/tests',W/'src/alicia_grasp_modes/src',W/'src/alicia_flexible_grasp_supervisor/src',W/'devel/lib/python3/dist-packages']))+':'+env.get('PYTHONPATH','')
env['ROS_PACKAGE_PATH']=str(W/'src')+':'+env.get('ROS_PACKAGE_PATH','')
selected=[str(C/rel),str(W/'src/alicia_flexible_grasp_supervisor/tests/test_contact_height_window_proposals.py')]
args=['-q','-p','no:cacheprovider',*selected,'--basetemp='+str(O/'tmp'),'--junitxml='+str(O/'pytest.xml')]
bootstrap='''import pathlib,json,pytest,sys
result=pytest.main(json.loads(sys.argv[1]))
import test_selected_contact_height_recipe as suite
from alicia_grasp_modes import observe_return_remote
loaded={'test':str(pathlib.Path(suite.__file__).resolve()),'base_remote':str(pathlib.Path(suite.remote.__file__).resolve()),'mode_mixin':str(pathlib.Path(observe_return_remote.__file__).resolve())}
assert loaded['test']==sys.argv[2],loaded
pathlib.Path(sys.argv[3]).write_text(json.dumps(loaded,indent=2)+'\\n')
sys.exit(result)
'''
started=time.monotonic()
with (O/'pytest.log').open('w') as f:
 child=subprocess.run([sys.executable,'-c',bootstrap,json.dumps(args),str(C/rel),str(O/'loaded.json')],cwd=W,env=env,stdout=f,stderr=subprocess.STDOUT)
summary={}
if (O/'pytest.xml').exists():
 x=ET.parse(O/'pytest.xml').getroot();summary={k:sum(int(s.attrib.get(k,0)) for s in x.findall('testsuite')) for k in ('tests','failures','errors','skipped')}
actual_after={r:sha(W/r) for r in actual_before};candidate_after=sha(C/rel)
report=dict(passed=child.returncode==0 and actual_before==actual_after and candidate_before==candidate_after,summary=summary,returncode=child.returncode,elapsed_sec=time.monotonic()-started,candidate=dict(path=str(C/rel),before_sha256=candidate_before,after_sha256=candidate_after,baseline_sha256=baseline[rel]),original_failure=dict(path=str(prior),sha256=sha(prior),original_summary=original['summary'],affected_tests='Four measured-opportunity=True cases demanded obsolete height deferral.'),unchanged_other_tests_AST=True,actual_sources_before=actual_before,actual_sources_after=actual_after,actual_unchanged=actual_before==actual_after,actual_count=len(actual_before),loaded=json.loads((O/'loaded.json').read_text()) if (O/'loaded.json').exists() else None,log_sha256=sha(O/'pytest.log'),xml_sha256=sha(O/'pytest.xml'),no_ros_master=True,no_motion_commands=True,temperature_queried=False,scope='Test-only policy expectation update. All original recipe/strict transport/deadline checks retained; real mode+base height method chain exercised with synthetic strict transport.')
(O/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(dict(result=str(O/'result.json'),sha256=sha(O/'result.json'),passed=report['passed'],summary=summary)))
