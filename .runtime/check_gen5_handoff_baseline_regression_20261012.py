"""Compare broad-suite failures with the exact pre-patch source tree."""
import hashlib,json,os,subprocess,time,xml.etree.ElementTree as ET
from pathlib import Path
work=Path(__file__).resolve().parents[1]
base=work/'.runtime/gen5_handoff_baseline_20261012'
root=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_frozen_no_skin_20261012T021050Z/verification')
assert Path('/media/zhuyupei/SanStand').is_mount()
out=root/'gen5_handoff_baseline_comparison'
out.mkdir(exist_ok=False)
expected=json.loads((base/'baseline_sha256.json').read_text())
def hashes():return {p:hashlib.sha256((base/p).read_bytes()).hexdigest() for p in expected}
assert hashes()==expected
current=json.loads((root/'gen5_handoff_integrated/result.json').read_text())
scope=[p for p in current['test_scope'] if (base/p.split('::')[0]).exists()]
env=os.environ.copy()
env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:1',TMPDIR='/tmp')
env['ROS_PACKAGE_PATH']=str(base/'src')+':'+env.get('ROS_PACKAGE_PATH','')
env['PYTHONPATH']=':'.join(str(base/p) for p in ('src/alicia_flexible_grasp_supervisor/src','src/alicia_grasp_modes/src','src/alicia_flexible_grasp_supervisor/tests','src/alicia_grasp_modes/tests'))+':'+env.get('PYTHONPATH','')
cmd=['/usr/bin/python3','-m','pytest','-q',*scope,'-p','no:cacheprovider','--junitxml='+str(out/'pytest.xml')]
start=time.monotonic()
with (out/'pytest.log').open('x') as log:
    result=subprocess.run(cmd,cwd=base,env=env,stdout=log,stderr=subprocess.STDOUT)
def failures(path):
    return {t.attrib['classname']+'::'+t.attrib['name']:{'message':t.find('failure').attrib.get('message',''),'trace':t.find('failure').text}
        for t in ET.parse(path).iter('testcase') if t.find('failure') is not None}
actual_failures=failures(root/'gen5_handoff_integrated/pytest.xml')
baseline_failures=failures(out/'pytest.xml')
after=hashes()
summary={}
for suite in ET.parse(out/'pytest.xml').getroot().iter('testsuite'):
    for key in ('tests','failures','errors','skipped'):summary[key]=summary.get(key,0)+int(suite.attrib.get(key,0))
receipt=dict(returncode=result.returncode,summary=summary,elapsed_sec=time.monotonic()-start,
    command=cmd,test_scope=scope,source_sha256=expected,source_unchanged=after==expected,
    actual_failures=actual_failures,baseline_failures=baseline_failures,
    new_failure_ids=sorted(set(actual_failures)-set(baseline_failures)),
    resolved_failure_ids=sorted(set(baseline_failures)-set(actual_failures)),
    matching_failure_ids=set(actual_failures)==set(baseline_failures),
    excluded_new_test_files=[p for p in current['test_scope'] if p not in scope],
    motion_commands_sent=False,temperature_queried=False)
(out/'result.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps({k:receipt[k] for k in ('summary','elapsed_sec','source_unchanged','matching_failure_ids','new_failure_ids','resolved_failure_ids')}),flush=True)
print('\n'.join((out/'pytest.log').read_text().splitlines()[-5:]),flush=True)
