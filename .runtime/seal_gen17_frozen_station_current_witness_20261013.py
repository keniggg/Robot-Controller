import json,hashlib,shutil
from pathlib import Path
W=Path(__file__).resolve().parents[1]
C=W/'.runtime/gen17_frozen_station_retirement_20261013'
D=W/'.runtime/gen17_frozen_station_retirement_r2_20261013'
N=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
O=N/'verification/gen17_frozen_station_retirement_current_witness_r1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
m=C/'delivery_manifest.json';assert sha(m)=='689f81ca233770ef9c4ded62a59225219b9dea1cbf4698a89f599d55445103aa'
r=json.loads((O/'result.json').read_text());assert r['passed'] and r['summary']['tests']==4 and r['actual_unchanged'] and r['source_unchanged']
production={}
for name in ['contact_station_remote.py','station_compute_wait_remote.py']:
 rel='src/alicia_grasp_modes/src/alicia_grasp_modes/'+name
 assert sha(C/rel)==sha(D/rel)==r['source_sha256'][rel]
 production[rel]=sha(C/rel)
actual=json.loads((N/'verification/gen16_success_first_release_manifest.json').read_text())['source_sha256']
assert all(sha(W/p)==h for p,h in actual.items())
for p in [D/'src/alicia_grasp_modes/tests/test_frozen_station_retirement_history.py',W/'.runtime/run_gen17_frozen_station_retirement_current_witness_20261013.py',Path(__file__)]:shutil.copyfile(p,O/p.name)
review=dict(passed=True,production_unchanged=True,production_sha256=production,
 sealed_manifest=str(m),sealed_manifest_sha256=sha(m),result_sha256=sha(O/'result.json'),summary=r['summary'],
 supplemental_test_sha256=sha(O/'test_frozen_station_retirement_history.py'),actual_sources=len(actual),actual_unchanged=True,
 conclusion='Four real freeze/clock-only/MOVE cases establish that identical latest-current object/mask repeats preserve exact retirement, while nonempty changed mask and changed detected object irreversibly revoke history. Current source was asserted outside the three original computation inputs. No KeyError exists: callbacks reload the live reference before object/mask indexing and ready_history performs contradiction checks.',
 deployment_manifest_unchanged=True,no_new_production_patch=True,physical_actions=False,
 scope='Offline original protocol, immutable report fixture, synthetic RGBD and inert action sentinel only.')
(O/'supplemental_receipt.json').write_text(json.dumps(review,indent=2)+'\n')
print(json.dumps(dict(receipt=str(O/'supplemental_receipt.json'),sha256=sha(O/'supplemental_receipt.json'),production_sha256=production)))
