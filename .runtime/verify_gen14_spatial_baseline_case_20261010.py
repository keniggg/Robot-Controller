"""Reproduce one unchanged old assertion using pre-spatial-ROI rgb_instance."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tarfile
import time
import xml.etree.ElementTree as ET

WORK = Path(__file__).resolve().parents[1]
ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_restart_20261010T002347Z/gen14_analysis/contact_station_retirement')
OUT = ROOT / 'baseline_spatial_old_assertion'
ARCHIVE = WORK / '.runtime/gen14_contact_station_retirement_20261010/source_before_spatial_roi.tar'
RELATIVE = 'src/alicia_grasp_modes/src/alicia_grasp_modes/rgb_instance.py'
TEST = 'src/alicia_grasp_modes/tests/test_arrival_fragment_support_guards.py'
NODE = TEST + '::test_arrival_spatial_proposal_cannot_relax_original_raw_uniqueness[two_original_matches]'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


assert Path('/media/zhuyupei/SanStand').is_mount()
OUT.mkdir(exist_ok=False)
baseline_source = OUT / 'baseline_rgb_instance.py'
with tarfile.open(ARCHIVE) as archive:
    baseline_source.write_bytes(archive.extractfile(RELATIVE).read())
import alicia_grasp_modes
spec = importlib.util.spec_from_file_location('alicia_grasp_modes.rgb_instance', baseline_source)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
alicia_grasp_modes.rgb_instance = module
import pytest
with (OUT / 'pytest.log').open('w') as log:
    original_stdout, original_stderr = sys.stdout, sys.stderr
    sys.stdout = sys.stderr = log
    try:
        exit_code = int(pytest.main(['-q', '-p', 'no:cacheprovider', NODE,
                                    '--basetemp=' + str(OUT / 'pytest_tmp'),
                                    '--junitxml=' + str(OUT / 'pytest.xml')]))
    finally:
        sys.stdout, sys.stderr = original_stdout, original_stderr
old_case = ET.parse(OUT / 'pytest.xml').getroot().find('.//testcase')
new_cases = ET.parse(ROOT / 'spatial_combined_r1.xml').getroot().findall('.//testcase')
new_case = next(case for case in new_cases if case.attrib['name'] == old_case.attrib['name']
                and case.attrib['classname'] == old_case.attrib['classname'])
old_failure = old_case.find('failure')
new_failure = new_case.find('failure')
actual = 'matching_fragment_independent_geometry_required'
expected = 'arrival_unique_original_identity_required'
same = (exit_code == 1 and old_failure is not None and new_failure is not None
        and old_failure.attrib['message'] == new_failure.attrib['message']
        and actual in old_failure.attrib['message'] and expected in old_failure.attrib['message'])
arrival = sys.modules['alicia_grasp_modes.observation_arrival_confirmation']
report = {
    'baseline_failure_confirmed': same, 'same_nodeid_and_exact_failure_message': same,
    'recorded_ns': time.time_ns(), 'offline_only': True,
    'archive': {'path': str(ARCHIVE), 'sha256': sha(ARCHIVE)},
    'nodeid': NODE, 'exit_code': exit_code,
    'baseline_rgb_instance': {'path': module.__file__, 'sha256': sha(module.__file__)},
    'unchanged_arrival_module': {'path': arrival.__file__, 'sha256': sha(arrival.__file__)},
    'unchanged_test': {'path': str(WORK / TEST), 'sha256': sha(WORK / TEST)},
    'current_suite': {'path': str(ROOT / 'spatial_combined_r1.xml'), 'sha256': sha(ROOT / 'spatial_combined_r1.xml')},
    'baseline_xml': {'path': str(OUT / 'pytest.xml'), 'sha256': sha(OUT / 'pytest.xml')},
    'baseline_failure_message': old_failure.attrib['message'] if old_failure is not None else None,
    'current_failure_message': new_failure.attrib['message'] if new_failure is not None else None,
    'conclusion': 'Both versions reject the ambiguous raw candidates before RGB refinement. The unchanged test expects a later rejection reason; no acceptance or assertion was relaxed.' if same else 'Comparison did not prove a pre-existing identical failure.',
}
path = OUT / 'result.json'
path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print(json.dumps({'baseline_failure_confirmed': same, 'report': str(path), 'sha256': sha(path)}))
raise SystemExit(0 if same else 2)
