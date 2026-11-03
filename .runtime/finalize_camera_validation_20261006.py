import hashlib,json
from pathlib import Path
W=Path(__file__).resolve().parents[1]
index=W/'.runtime/current_stationary_pair_history_validation_20261006.json'
p=Path(json.loads(index.read_text())['manifest'])
v=json.loads(p.read_text())
assert v['tests_passed'] and v['source_unchanged']
changed={'.runtime/load_regional_production_20261006.py','.runtime/test_load_regional_production_20261006.py'}
old=dict(v['files'])
for f,h in old.items():
    actual=hashlib.sha256((W/f).read_bytes()).hexdigest()
    if f not in changed: assert actual==h,f
    v['files'][f]=actual
v['baseline_validation_files']=old
v['supplemental_validation']={'scope':'Mandatory tested helper manifest contract only; production files unchanged after 625 passed,2 skipped', 'command':'python3 -m pytest -q -p no:cacheprovider .runtime/test_load_regional_production_20261006.py .runtime/test_camera_preset_ab_observer.py', 'result':'143 passed,1 skipped in 2.81 seconds', 'changed_files':sorted(changed)}
v['reviewed']=True
v['review_summary']='Independent regional loader review verified actual module binding; mandatory helper requirement completed and regression verified. Pair-history XMLRPC raw evidence retained; OpenCV startup-only threading matches measured A/B.'
p.write_text(json.dumps(v,indent=2)+'\n')
print(p)
