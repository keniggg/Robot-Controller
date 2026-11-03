"""Seal reporting-only ACK/qualification timeout classification and evidence."""
import difflib
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

W=Path(__file__).resolve().parents[1]
STAGE=W/'.runtime/gen14_measurement_timeout_reason_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
V=N/'verification';OUT=V/'gen14_timeout_reporting_delivery_seal'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_text())
record=lambda p:dict(path=str(p),sha256=sha(p))
assert Path('/media/zhuyupei/SanStand').is_mount() and not OUT.exists()
assert not (STAGE/'delivery_manifest.json').exists()
prior_path=V/'gen13_identity_integrated_install/result.json';prior=read(prior_path)
assert prior['passed'] and len(prior['source_sha256'])==890
assert all(sha(W/p)==h for p,h in prior['source_sha256'].items())
evidence={}
for target,failures in [('baseline',4),('candidate',0)]:
    folder=V/('gen14_reporting_independent_'+target+'_r1');data=read(folder/'result.json')
    counts={k:sum(int(s.attrib.get(k,0)) for s in ET.parse(folder/'pytest.xml').iter('testsuite'))
            for k in ('tests','failures','errors','skipped')}
    assert counts==data['counts']==dict(tests=81,failures=failures,errors=0,skipped=0)
    assert data['source_unchanged'] and data['pytest_xml_sha256']==sha(folder/'pytest.xml')
    assert data['passed']==(target=='candidate')
    assert all(sha(Path(p) if Path(p).is_absolute() else W/p)==h for p,h in data['source_sha256'].items())
    if target=='baseline':
        assert len(data['failed_tests'])==4
        assert sum(n.startswith('test_accepted_transport_without_three_frame_completion_reports_evidence_timeout') for n in data['failed_tests'])==3
        assert 'test_ros_clock_stall_cannot_renew_original_monotonic_capture_budget' in data['failed_tests']
    evidence[target]=dict(counts=counts,files={p.name:record(p) for p in folder.iterdir() if p.is_file()})
original_test=W/'.runtime/gen14_timeout_independent_20261013/test_measurement_timeout_independent.py'
installed_test=STAGE/'src/alicia_grasp_modes/tests/test_measurement_timeout_independent.py'
assert sha(original_test)==sha(installed_test)
production='src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_task.py'
scope=['src/alicia_grasp_modes/tests/test_measurement_timeout_reason.py',
       'src/alicia_grasp_modes/tests/test_measurement_timeout_independent.py',
       'src/alicia_grasp_modes/tests/test_observe_return_measurement.py']
files=[]
for p in sorted((STAGE/'src').rglob('*.py')):
    if '__pycache__' in p.parts:continue
    relative=str(p.relative_to(STAGE));base=W/relative
    old=sha(base) if base.exists() else None
    if old!=sha(p):files.append(dict(path=relative,base_sha256=old,candidate_sha256=sha(p)))
assert len(files)==3 and {r['path'] for r in files}=={production,*scope[:2]}
old=(W/production).read_text();new=(STAGE/production).read_text()
assert new.replace('        last_validated_acks = None\n','').replace('            last_validated_acks = acks\n','').replace(
    "        if qualified and last_validated_acks is not None:\n            pending = [p for p, ack in zip(PARTICIPANTS, last_validated_acks)\n                       if ack['qualified'] is not True]\n            raise ValueError('OBSERVATION_MEASUREMENT_QUALIFICATION_TIMEOUT: pending=' + ','.join(pending))\n",'')==old
patch=''.join(''.join(difflib.unified_diff((W/r['path']).read_text().splitlines(True) if (W/r['path']).exists() else [],
    (STAGE/r['path']).read_text().splitlines(True),fromfile='a/'+r['path'],tofile='b/'+r['path'])) for r in files)
(STAGE/'delivery.patch').write_text(patch)
limits=[
 'Reporting-only change: no identity/vision recovery or physical grasp success is claimed.',
 'Qualification timeout means that the last fully validated ACK pair for this exact request contained pending qualification. It is not a renewed authority or a claim that the final asynchronous ACK snapshot remains valid.',
 'Missing/unvalidated ACK remains ACK_TIMEOUT. Original live-owner, ACK authority/binding, source proof and early ROS expiry exceptions remain unchanged and terminal.',
 'The original absolute monotonic budget, ROS capture/session deadline, 5-second transport cap, request publication count, qualification/source gates and return values are unchanged.',
 '81 tests include 13 author, 19 independent and 49 existing lifecycle tests; counts are not additive with separately named subscopes.',
 'The initial author 11-pass/2-fail run is preserved: a floating-clock fixture rounded ROS to the cutoff before monotonic reached it, correctly triggering original NOT_QUALIFIED; only fixture step rounding was corrected.',
 'Offline tests use original publish/ACK/live methods with inert ROS transport and deterministic clocks; actual installed combined regression remains required before deployment.'
]
manifest=dict(schema=1,kind='gen14_measurement_qualification_timeout_reporting',passed=True,
 recorded_ns=time.time_ns(),candidate_root=str(STAGE),stage=str(STAGE),files=files,test_scope=scope,
 source_sha256={r['path']:r['candidate_sha256'] for r in files},prior_release=record(prior_path),
 current_baseline_sources=890,validation=evidence,
 historical_author_fixture_failure=record(V/'gen14_timeout_reporting_candidate/result.json'),
 independent_test_provenance=record(original_test),patch=record(STAGE/'delivery.patch'),
 runner=record(W/'.runtime/gen14_timeout_independent_20261013/run.py'),
 installer=record(W/'.runtime/reviewed_grasp_patch_20261012.py'),limits=limits,
 production_installed=False,physical_grasp_verified=False,motion_commands_sent=False,temperature_queried=False)
OUT.mkdir();payload=json.dumps(manifest,ensure_ascii=False,indent=2,sort_keys=True)+'\n'
(STAGE/'delivery_manifest.json').write_text(payload);(OUT/'delivery_manifest.json').write_text(payload)
review=dict(schema=1,passed=True,recorded_ns=time.time_ns(),delivery=str(STAGE/'delivery_manifest.json'),
 delivery_sha256=sha(STAGE/'delivery_manifest.json'),test_scope=scope,source_sha256=manifest['source_sha256'],
 validation=evidence,limits=limits,
 source_review=[
  'Exact production diff only initializes the last validated pair, saves it after both original ACK validations, and chooses a qualified-only terminal label at existing loop expiry.',
  'No deadline/cutoff calculation, loop predicate, wait duration, publication, existing validator or source-equality check changed. There is no string-specific ACK_TIMEOUT retry branch in the task call chain.',
  'Baseline fails exactly the three pending-participant labels and one frozen-ROS-clock label; candidate passes all 81. Epoch/session changes use the actual live validator, while manual/stop faults enter through the unchanged upstream live guard.',
  'Early ROS capture/session expiry remains NOT_QUALIFIED/EXPIRED before monotonic cutoff; fully qualified but invalid or divergent source triples remain immediate rejection. Accepted transport cannot promote geometry or motion authority.',
  'Pending participant order follows fixed PARTICIPANTS unknown,remote; no observed validated pair cannot produce qualification timeout. Earlier accepted ACKs for another request do not satisfy transport for a new request.'
 ],sealing_script=record(Path(__file__).resolve()),production_sources_unchanged=True,
 production_installed=False,physical_grasp_verified=False,motion_commands_sent=False,temperature_queried=False)
assert all(sha(W/p)==h for p,h in prior['source_sha256'].items())
(OUT/'independent_review.json').write_text(json.dumps(review,ensure_ascii=False,indent=2,sort_keys=True)+'\n')
result=dict(passed=True,delivery=record(STAGE/'delivery_manifest.json'),review=record(OUT/'independent_review.json'),
 changed_files=3,test_scope_files=3,tests=dict(tests=81,failures=0,errors=0,skipped=0),
 baseline=dict(tests=81,failures=4,errors=0,skipped=0),production_sources_unchanged=True,
 production_installed=False,physical_grasp_verified=False,motion_commands_sent=False,temperature_queried=False)
(OUT/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');print(json.dumps(result,ensure_ascii=False))
