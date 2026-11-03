#!/usr/bin/env python3
"""Bind independent original-pixel registered-BAD negatives to staged GEN13."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
S=W/'.runtime/gen13_station_fragment_identity_20261012'
HARNESS=W/'.runtime/replay_attempt1_full_process_negative_20261013.py'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);mod=importlib.util.module_from_spec(spec);sys.modules[name]=mod;spec.loader.exec_module(mod);return mod
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    paths=[S/'src/alicia_grasp_modes/src/alicia_grasp_modes/station_compute_wait_producer.py',S/'src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py',S/'src/alicia_grasp_modes/src/alicia_grasp_modes/observation_appearance.py',S/'src/alicia_grasp_modes/src/alicia_grasp_modes/arrival_fragment_group.py']
    before={str(path):sha(path)for path in paths}
    source=HARNESS.read_text();old="('original_phase_original_clock','early_phase_original_clock','early_phase_timely_identity_diagnostic')";new="('early_phase_original_clock','early_phase_timely_identity_diagnostic')";assert source.count(old)==1
    assertion="            assert 'station_appearance' not in marker and 'station_appearance_provenance' not in marker\n"
    needle="            reference['observation_arrival_retry']=marker;node._locked_rgb_hold_reference=reference\n"
    assert source.count(needle)==1;source=source.replace(needle,assertion+needle)
    namespace={'__name__':'bound_candidate_replay','__file__':str(HARNESS)}
    exec(compile(source.replace(old,new),str(HARNESS),'exec'),namespace)
    producer=load('alicia_grasp_modes.station_compute_wait_producer',paths[0])
    candidate=load('staged_unknown_bound_bad_negative',paths[1])
    assert producer.StationComputeWaitProducerMixin in candidate.UnknownTabletopPerception.__mro__
    namespace['module']=candidate
    namespace['main']()
    after={str(path):sha(path)for path in paths};assert before==after,'candidate changed during offline replay'
    output=Path(args.output);result_path=output/'result.json';result=json.loads(result_path.read_text())
    result['no_qualified_callback_no_station_template']=True
    result['candidate_binding']={'source_sha256':before,'source_hashes_unchanged_during_replay':True,'actual_unknown_module_file':candidate.__file__,'actual_producer_module_file':producer.__file__,'excluded_original_late_phase_case':'GEN13 cancels its old owner; covered by separate phase/checkpoint tests, not falsely expected to publish original failure','harness_sha256':sha(HARNESS),'wrapper_sha256':sha(Path(__file__))}
    result['source_sha256']={str(HARNESS):sha(HARNESS),str(Path(__file__)):sha(Path(__file__)),**before}
    result_path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result['candidate_binding'],ensure_ascii=False))
if __name__=='__main__':main()
